"""Compile production firmware sources against a small native hardware boundary."""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

SUPPORT = Path(__file__).with_name("firmware_support")
DEFAULT_FIRMWARE = Path(__file__).resolve().parents[2] / "kinisi-motor-controller-firmware"
SOURCES = (
    "src/command_handler.c", "src/controllers_manager.c", "src/odometry_manager.c",
    "src/odometry_integrator.c", "src/encoder_odometry.c", "src/platform.c",
    "src/platform_common.c", "src/platform_differential.c", "src/platform_omni.c",
    "src/platform_mecanum.c", "src/platform_position.c",
    "lib/libcontrollers/src/pid_controller.c", "lib/libcontrollers/src/position_controller.c",
    "lib/libcontrollers/src/loop_frequency.c", "lib/connection/connection.c",
    "lib/protocol/protocol.c", "lib/time_sync/time_sync.c",
    "lib/initialization/initialization.c", "lib/command_requirements/command_requirements.c",
)
INCLUDES = ("lib/hardware", "lib/libcontrollers/src", "lib/connection", "lib/protocol",
            "lib/time_sync", "lib/initialization", "lib/command_requirements", "lib/libutils/src")


def find_compiler(compiler=None):
    candidate = compiler or os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
    if not candidate and os.name == "nt":
        candidate = "C:/msys64/ucrt64/bin/gcc.exe"
    resolved = shutil.which(str(candidate)) if candidate else None
    if not resolved:
        raise ValueError("Native firmware needs a host GCC/Clang compiler; set CC or --compiler to its executable")
    return str(Path(resolved).resolve())


def _hardware_header(profile):
    motors = profile["motors"]["channels"]
    counts = {"MOTORS": len(motors), "ENCODERS": len(profile["encoders"]),
              "GPIO_PINS": len(profile["gpio"]), "STATUS_LEDS": len(profile["status_leds"]),
              "TEMPERATURE_SENSORS": len(profile.get("temperature_sensors", [])),
              "IMU_SENSORS": len(profile.get("imu_sensors", [])),
              "CURRENT_SENSORS": sum("current_sense" in motor for motor in motors)}
    if counts["MOTORS"] != 4 or counts["ENCODERS"] != 4 or counts["GPIO_PINS"] > 10:
        raise ValueError("Native hardware boundary currently supports the four-channel v0.3/v0.4 boards")
    driver = profile["motors"]["control_model"] == "enable_direction"
    lines = ["#pragma once", '#include "board_revision.h"',
             "#define MOTOR_CONTROL_MODEL_PWM 0", "#define MOTOR_CONTROL_MODEL_ENABLE_DIRECTION 1",
             f"#define MOTOR_CONTROL_MODEL {int(driver)}", f"#define SIM_HAS_DRIVER {int(driver)}"]
    lines += [f"#define NUMBER_{name} {count}" for name, count in counts.items()]
    # Model driver capabilities from the board file, never from an invented limit table.
    limits = [m.get("hardware_current_limit", {}) for m in motors]
    lines += ["static const unsigned sim_limit_default[4] = {" + ",".join(str(l.get("threshold_ma", 0)) for l in limits) + "};",
              "static const unsigned sim_limit_enabled[4] = {" + ",".join(str(int(l.get("enabled", False))) for l in limits) + "};",
              "static const unsigned sim_limit_can_disable[4] = {" + ",".join(str(int(l.get("can_disable", False))) for l in limits) + "};",
              "static const unsigned sim_limit_count[4] = {" + ",".join(str(len(l.get("supported_thresholds_ma", []))) for l in limits) + "};"]
    rows = ["{" + ",".join(map(str, l.get("supported_thresholds_ma", []) or [0])) + "}" for l in limits]
    if any(len(l.get("supported_thresholds_ma", [])) > 4 for l in limits):
        raise ValueError("Native driver supports at most four current-limit thresholds")
    lines += ["static const unsigned sim_limit_values[4][4] = {" + ",".join(rows) + "};"]
    frequencies = profile["motors"].get("supported_pwm_frequencies_hz", [100000])
    lines += [f'#define SIM_PWM_DEFAULT {profile["motors"].get("pwm_frequency_hz", 100000)}',
              "static const unsigned sim_pwm_frequencies[] = {" + ",".join(map(str, frequencies)) + "};"]
    return "\n".join(lines) + "\n"


def build_firmware(schema, firmware=DEFAULT_FIRMWARE, revision="0.4.0", compiler=None):
    """Return a content-addressed library plus provenance. Never modify the checkout."""
    firmware = Path(firmware).resolve()
    if revision not in ("0.3.0", "0.4.0"):
        raise ValueError("Native firmware supports revisions 0.3.0 and 0.4.0")
    canonical = firmware / "commands.json"
    if schema.document != json.loads(canonical.read_text(encoding="utf-8-sig")):
        raise ValueError("--commands must match the selected firmware checkout's commands.json")
    board_file = firmware / "boards" / f"v{revision[:-2]}.json"
    profile = json.loads(board_file.read_text())
    header = _hardware_header(profile)
    compiler = find_compiler(compiler)
    identity = subprocess.run([compiler, "--version"], check=True, capture_output=True, text=True).stdout.splitlines()[0]
    inputs = {p: (firmware / p).read_bytes() for p in SOURCES}
    for directory in ("include", *INCLUDES, "tools/commands_generator"):
        for path in sorted((firmware / directory).rglob("*")):
            if path.suffix in (".h", ".py", ".jinja", ".j2"):
                inputs[str(path.relative_to(firmware)).replace("\\", "/")] = path.read_bytes()
    for path in sorted(SUPPORT.iterdir()):
        if path.suffix in (".c", ".h"):
            inputs[f"shim/{path.name}"] = path.read_bytes()
    inputs["commands.json"] = canonical.read_bytes()
    inputs["board.json"] = board_file.read_bytes()
    inputs["build.py"] = Path(__file__).read_bytes()
    digest = hashlib.sha256((compiler + identity + sys.platform + header).encode())
    for name, content in sorted(inputs.items()):
        digest.update(name.encode() + b"\0" + content + b"\0")
    build_id = digest.hexdigest()
    cache = Path(tempfile.gettempdir()) / "kinisi-native-firmware" / build_id
    cache.mkdir(parents=True, exist_ok=True)
    library = cache / ("firmware.dll" if os.name == "nt" else "firmware.so")
    manifest = {"build_id": build_id, "compiler": identity, "compiler_path": compiler,
                "firmware": str(firmware), "board": str(board_file),
                "sources": list(SOURCES), "schema_sha256": schema.sha256}
    handlers = inputs["src/command_handler.c"].decode() + inputs["lib/connection/connection.c"].decode()
    dispatch = set(re.findall(r"\bcase\s+([A-Z][A-Z_0-9]+)\s*:", handlers))
    dispatch.update(re.findall(r"(?:commandType|data\[0\])\s*==\s*([A-Z][A-Z_0-9]+)", handlers))
    manifest["unsupported_commands"] = sorted(c["command"] for c in schema.document["commands"]
        if c["direction"] == "client_to_controller" and c["command"] not in dispatch)
    if not library.is_file():
        with tempfile.TemporaryDirectory(prefix="build-", dir=cache) as folder:
            stage = Path(folder)
            include = stage / "include"
            shutil.copytree(firmware / "include", include)
            (include / "hw_config.h").write_text(header)
            (include / "commands.h").unlink(missing_ok=True)
            command = [sys.executable, str(firmware / "tools/commands_generator/generator.py"),
                       str(canonical), str(include / "commands.h"), "--language", "c"]
            generated = subprocess.run(command, capture_output=True, text=True)
            if generated.returncode or not (include / "commands.h").exists():
                raise RuntimeError(f"Firmware schema generation failed:\n{generated.stdout}{generated.stderr}")
            output = stage / library.name
            command = [compiler, "-std=gnu11", "-shared", "-O2", "-Wall", "-Wextra", "-Werror",
                       "-Wno-unused-parameter", "-Wno-unused-variable", "-Wno-error=absolute-value",
                       f"-D{profile['board']['build_define']}", "-I" + str(SUPPORT), "-I" + str(include)]
            if os.name != "nt":
                command += ["-fPIC"]
            else:
                command += ["-static-libgcc"]
            command += ["-I" + str(firmware / inc) for inc in INCLUDES]
            command += [str(firmware / source) for source in SOURCES]
            command += [str(SUPPORT / "runtime.c"), "-lm", "-o", str(output)]
            environment = dict(os.environ, PATH=str(Path(compiler).parent) + os.pathsep + os.environ.get("PATH", ""))
            result = subprocess.run(command, capture_output=True, text=True, env=environment)
            if result.returncode:
                raise RuntimeError(f"Native firmware compilation failed (exit {result.returncode}):\n{result.stdout}{result.stderr}")
            manifest["compiler_warnings"] = result.stderr
            (cache / "manifest.json").write_text(json.dumps(manifest, indent=2))
            os.replace(output, library)
    return library, manifest
