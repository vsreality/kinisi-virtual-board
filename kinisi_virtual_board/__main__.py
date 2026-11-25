"""Serve a virtual board or run the public native firmware IMU regression."""
import argparse
import json
from pathlib import Path

from . import Schema, TCPServer, VirtualBoard, FirmwareBoard
from .native import PTYEndpoint, SerialEndpoint, serve_native


def main():
    sibling = Path(__file__).resolve().parents[2] / "kinisi-motor-controller-firmware"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=["firmware", "python"], default="firmware", help="Native production C firmware (default) or Python behavioral model")
    parser.add_argument("--firmware", type=Path, default=sibling, help="Firmware source checkout for the native backend")
    parser.add_argument("--compiler", help="Host GCC/Clang executable; defaults to CC or auto-detection")
    parser.add_argument("--commands", type=Path, help="Shared commands.json; defaults to --firmware/commands.json")
    parser.add_argument("--board-profile", type=Path, help="Optional firmware boards/v0.x.json")
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
    args = parser.parse_args()
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
        if args.backend == "firmware" and args.board_profile:
            raise ValueError("The firmware backend uses --firmware/boards and --revision; --board-profile is only for the Python model")
        def factory():
            if args.backend == "firmware":
                return FirmwareBoard(schema, firmware=args.firmware, revision=args.revision, compiler=args.compiler)
            return VirtualBoard(schema, args.board_profile, revision=args.revision)
        board = factory()
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        parser.error(str(error))
    report = {"backend": args.backend, "protocol": ".".join(map(str, schema.version)), "commands": str(schema.path),
              "sha256": schema.sha256, "board_revision": ".".join(map(str, board.revision)),
              "unsupported_commands": board.unsupported_commands}
    if args.backend == "firmware":
        report["native_build"] = board.build
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
