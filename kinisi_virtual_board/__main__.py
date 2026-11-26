"""Serve a virtual board or run the public native firmware IMU regression."""
import argparse
import json
from pathlib import Path

from . import Schema, TCPServer, VirtualBoard, FirmwareBoard
from .native import PTYEndpoint, SerialEndpoint, serve_native
from .imu_mount import IDENTITY, Mount, board_center_to_imu_m


def main():
    sibling = Path(__file__).resolve().parents[2] / "kinisi-motor-controller-firmware"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=["firmware", "python"], default="firmware", help="Native production C firmware (default) or Python behavioral model")
    parser.add_argument("--firmware", type=Path, default=sibling, help="Firmware source checkout for the native backend")
    parser.add_argument("--compiler", help="Host GCC/Clang executable; defaults to CC or auto-detection")
    parser.add_argument("--commands", type=Path, help="Shared commands.json; defaults to --firmware/commands.json")
    parser.add_argument("--board-profile", type=Path, help="Optional firmware boards/v0.x.json")
    position = parser.add_mutually_exclusive_group()
    position.add_argument("--imu-board-center-m", metavar="X,Y,Z", help="v0.4 PCB underside center in chassis metres")
    position.add_argument("--imu-offset-m", metavar="X,Y,Z", help="Legacy physical chip position in chassis metres")
    parser.add_argument("--imu-board-to-chassis", metavar="R00,...,R22",
                        help="Physical board-to-chassis rotation, nine row-major values")
    parser.add_argument("--revision", default="0.4.0", choices=["0.3.0", "0.3.1", "0.4.0"])
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766, help="0 chooses an available port")
    transport = parser.add_mutually_exclusive_group()
    transport.add_argument("--serial", metavar="DEVICE", help="Emulator end of an existing virtual COM/TTY pair (no proxy)")
    transport.add_argument("--pty", action="store_true", help="Create a Linux/POSIX virtual serial device for the SDK (no driver install)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Native serial baud rate (default: 115200)")
    parser.add_argument("--check", action="store_true", help="Print schema identity/handler coverage and exit; fail on unsupported commands")
    commands = parser.add_subparsers(dest="command")
    regression = commands.add_parser(
        "imu-regression", help="Check native v0.4 firmware against public BMI088 trajectories")
    regression.add_argument("--firmware", type=Path, required=True, help="Firmware source checkout to test")
    regression.add_argument("--compiler", default=argparse.SUPPRESS, help="Host GCC/Clang executable (or set CC)")
    regression.add_argument("--seed", type=int, default=8101, help="Reproducible BMI088 unit and noise seed (default: 8101)")
    qualification = commands.add_parser("imu-qualify", help="Run all 144 public v0.4 chassis IMU qualification cases")
    qualification.add_argument("--firmware", type=Path, required=True, help="Diagnostic firmware checkout to test")
    qualification.add_argument("--compiler", default=argparse.SUPPRESS, help="Host C compiler (or set CC)")
    qualification.add_argument("--out", type=Path, help="Fresh artifact directory (default: build/imu-qualification/<timestamp>)")
    source = qualification.add_mutually_exclusive_group()
    source.add_argument("--motion-firmware", type=Path, help="Independent native motion producer checkout (defaults to --firmware)")
    source.add_argument("--replay", type=Path, help="Replay an archived run's plan and original motion sources")
    selector = commands.add_parser("selector-regression", help="Run eight public native feedback source selector scenarios")
    selector.add_argument("--firmware", type=Path, required=True, help="Clean firmware source checkout to test")
    selector.add_argument("--out", type=Path, required=True, help="New output directory for raw packets and SDK examples")
    selector.add_argument("--compiler", default=argparse.SUPPRESS, help="Host GCC/Clang executable (or set CC)")
    from .feedback_selector.runner import CASES
    selector.add_argument("--case", choices=CASES, action="append", dest="cases", help="Run only this case; repeat to select several")
    selector.add_argument("--development", action="store_true", help="Allow and fingerprint uncommitted firmware/simulator changes")
    health = commands.add_parser("tracking-health", help="Qualify native v0.4 dual-source tracking health")
    health.add_argument("--firmware", type=Path, required=True)
    health.add_argument("--out", type=Path, required=True, help="New directory for the summary and wire traces")
    health.add_argument("--compiler", default=argparse.SUPPRESS, help="Host GCC/Clang executable (or set CC)")
    mounting = commands.add_parser("mounting-acceptance", help="Check physical IMU mounting against native origin tracking")
    mounting.add_argument("--firmware", type=Path, required=True)
    mounting.add_argument("--out", type=Path, required=True, help="New directory for frozen plan, wire traces and scores")
    mounting.add_argument("--compiler", default=argparse.SUPPRESS)
    mounting.add_argument("--reference", choices=("chip", "board-center"), default="chip",
                          help="Use chip coordinates (legacy default) or the v0.4 PCB underside center")
    mounting.add_argument("--development", action="store_true", help="Fingerprint uncommitted sources")
    mounting.add_argument("--case", action="append", dest="cases", help="Select a declared case; repeat for several")
    args = parser.parse_args()
    if args.command == "mounting-acceptance":
        from .mounting_acceptance import run
        try:
            return run(args)
        except (OSError, ValueError, RuntimeError) as error:
            parser.exit(2, f"Mounting acceptance incomplete: {error}\n")
    if args.command == "tracking-health":
        from .feedback_selector.health import run
        try:
            return run(args)
        except (OSError, ValueError, RuntimeError) as error:
            parser.exit(2, f"Tracking health qualification incomplete: {error}\n")
    if args.command == "selector-regression":
        from .feedback_selector.runner import run
        try:
            return run(args)
        except (OSError, ValueError, RuntimeError) as error:
            parser.exit(2, f"Selector regression incomplete: {error}\n")
    if args.command == "imu-qualify":
        from .imu_qualification import execute
        try:
            return execute(args)
        except (OSError, ValueError, RuntimeError, LookupError) as error:
            parser.exit(2, f"IMU qualification incomplete: {error}\n")
    if args.command == "imu-regression":
        from .imu_regression import run
        report, status = run(args.firmware, args.seed, compiler=args.compiler)
        print(json.dumps(report, indent=2, allow_nan=False))
        return status
    try:
        schema = Schema(args.commands or args.firmware / "commands.json")
        mount = None
        if any(value is not None for value in (args.imu_offset_m, args.imu_board_center_m, args.imu_board_to_chassis)):
            offset = args.imu_offset_m.split(",") if args.imu_offset_m is not None else (0., 0., 0.)
            matrix = IDENTITY
            if args.imu_board_to_chassis is not None:
                values = args.imu_board_to_chassis.split(",")
                if len(values) != 9:
                    raise ValueError("--imu-board-to-chassis requires nine row-major values")
                matrix = (values[:3], values[3:6], values[6:])
            if args.imu_board_center_m is None:
                mount = Mount(offset, matrix)
            else:
                profile_root = schema.path.parent if args.backend == "python" else args.firmware
                profile_path = args.board_profile or profile_root / "boards" / "v0.4.json"
                if not args.board_profile and args.revision != "0.4.0":
                    raise ValueError("Board-center mounting requires v0.4 hardware with a BMI088")
                profile = json.loads(profile_path.read_text(encoding="utf8"))
                mount = Mount.from_board_center(args.imu_board_center_m.split(","), matrix,
                                                center_to_imu_m=board_center_to_imu_m(profile))
        if args.backend == "firmware" and args.board_profile:
            raise ValueError("The firmware backend uses --firmware/boards and --revision; --board-profile is only for the Python model")
        def factory():
            if args.backend == "firmware":
                return FirmwareBoard(schema, firmware=args.firmware, revision=args.revision,
                                     compiler=args.compiler, imu_mount=mount)
            return VirtualBoard(schema, args.board_profile, revision=args.revision, imu_mount=mount)
        board = factory()
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        parser.error(str(error))
    report = {"backend": args.backend, "protocol": ".".join(map(str, schema.version)), "commands": str(schema.path),
              "sha256": schema.sha256, "board_revision": ".".join(map(str, board.revision)),
              "unsupported_commands": board.unsupported_commands}
    if args.backend == "firmware":
        report["native_build"] = board.build
    if mount is not None:
        report["physical_imu_mount"] = dict(offset_m=mount.offset_m, board_to_chassis=mount.board_to_chassis)
        if args.imu_board_center_m is not None:
            report["physical_imu_mount"]["board_center_m"] = tuple(map(float, args.imu_board_center_m.split(",")))
    if args.check:
        print(json.dumps(report, indent=2))
        if hasattr(board, "close"):
            board.close()
        return int(bool(board.unsupported_commands))
    if args.serial or args.pty:
        try:
            endpoint = SerialEndpoint(args.serial, args.baudrate) if args.serial else PTYEndpoint()
            with endpoint:
                print(json.dumps(dict(report, transport="serial" if args.serial else "pty",
                                      serial_port=endpoint.port, baudrate=args.baudrate)), flush=True)
                serve_native(board, endpoint)
        except KeyboardInterrupt:
            pass
        except (OSError, ValueError, RuntimeError) as error:
            parser.exit(1, f"Native serial error: {error}\n")
        finally:
            if hasattr(board, "close"):
                board.close()
        return 0
    if hasattr(board, "close"):
        board.close()
    with TCPServer((args.host, args.port), factory) as server:
        print(json.dumps(dict(report, listening=f"{server.server_address[0]}:{server.server_address[1]}")), flush=True)
        try:
            server.serve_forever(poll_interval=0.1)
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
