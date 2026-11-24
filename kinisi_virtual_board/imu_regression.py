"""Run fixed public motion cases and score the firmware's wire observations."""
from pathlib import Path
import subprocess

from .bmi088 import MODEL_ID
from .firmware_build import build_firmware
from .imu_regression_capture import PROFILES, capture_case
from .imu_regression_score import LIMITS, score_case
from .schema import Schema


def checkout_info(directory):
    """A source archive can still be identified by its native build fingerprint."""
    try:
        def git(*args):
            return subprocess.run(["git", "-C", str(directory), *args], check=True,
                                  capture_output=True, text=True).stdout.strip()
        if Path(git("rev-parse", "--show-toplevel")).resolve() == directory:
            return {"revision": git("rev-parse", "HEAD"), "dirty": bool(git("status", "--porcelain"))}
    except (OSError, subprocess.CalledProcessError):
        pass
    return {"revision": None, "dirty": None}


def run(firmware, seed, compiler=None):
    firmware = Path(firmware).resolve()
    candidate = checkout_info(firmware)
    results = []
    report = {"seed": seed, "model": MODEL_ID, "board": "0.4.0", "firmware": candidate,
              "limits": LIMITS, "cases": results}
    errors = (OSError, ValueError, TypeError, RuntimeError, LookupError, subprocess.SubprocessError)
    try:
        if not 0 <= seed < 2**32:
            raise ValueError("seed must be an unsigned 32-bit integer")
        schema = Schema(firmware / "commands.json")
        required = {"SET_IMU_TRACKING", "GET_IMU_TRACKING_CONFIGURATION",
                    "GET_IMU_TRACKING_STATUS", "GET_IMU_TRACKING_VECTOR"}
        missing = required.difference(schema.commands)
        if missing:
            raise ValueError("Firmware schema lacks " + ", ".join(sorted(missing)))
        _, build = build_firmware(schema, firmware, compiler=compiler)
        candidate.update(build_id=build["build_id"], compiler=build["compiler"],
                         protocol=".".join(map(str, schema.version)), schema_sha256=schema.sha256)
    except errors as error:
        report["error"] = str(error)
    else:
        for profile in PROFILES:
            try:
                capture = capture_case(firmware, profile=profile, seed=seed, compiler=compiler)
                metadata = capture["metadata"]
                if "build_id" not in metadata:
                    raise RuntimeError(metadata.get("error", "Native board did not return a build ID"))
                if metadata["build_id"] != candidate["build_id"]:
                    raise RuntimeError("Firmware build changed during capture")
                result = score_case(capture)
            except errors as error:
                result = {"outcome": "INCOMPLETE", "failures": [], "incomplete": [str(error)]}
            case = {"profile": profile, "outcome": result["outcome"],
                    "samples": result.get("counts", {}).get("samples", 0),
                    "polls": result.get("counts", {}).get("polls", 0),
                    "peaks": {metric: peak["value"] for metric, peak in result.get("peaks", {}).items()}}
            if result["failures"]:
                case["failure"] = result["failures"][0]
            if result["incomplete"]:
                case["incomplete"] = result["incomplete"][0]
            results.append(case)
    failed = sum("failure" in row for row in results)
    incomplete = sum("incomplete" in row for row in results) + len(PROFILES) - len(results)
    passed = sum(row["outcome"] == "PASS" for row in results)
    report["counts"] = {"planned": len(PROFILES), "passed": passed, "product_failed": failed, "incomplete": incomplete}
    report["outcome"] = "FAIL" if failed else "INCOMPLETE" if incomplete else "PASS"
    return report, {"PASS": 0, "FAIL": 1, "INCOMPLETE": 2}[report["outcome"]]
