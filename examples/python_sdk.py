"""Exercise the real Python SDK without a serial port or wall-clock sleeps."""
import argparse
import importlib
import json
from pathlib import Path
import sys
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kinisi_virtual_board import FirmwareBoard, VirtualSerial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk", type=Path, default=ROOT.parent / "pykinisi")
    parser.add_argument("--firmware", type=Path, default=ROOT.parent / "kinisi-motor-controller-firmware")
    parser.add_argument("--commands", type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str(args.sdk))
    from pykinisi import KinisiController
    module = importlib.import_module("pykinisi.KinisiController")
    board = FirmwareBoard(args.commands, firmware=args.firmware)
    port = VirtualSerial(board, realtime=False, fragment_size=1)
    client = KinisiController(wall_clock=False, heartbeat_timeout_ms=None)
    try:
        # Replace only the serial-port factory. All session logic and bytes use the SDK.
        with patch.object(module.serial, "Serial", return_value=port):
            if not client.connect("virtual"):
                raise RuntimeError(client.last_error)
        client.set_controller_frequency(100)
        client.initialize_motor_controller(0, False, 0, False, 10000, 10, 20, 0)
        client.initialize_motor_position_controller(0, 2, 2, .01)
        client.set_motor_position(0, 1)
        port.advance(6)
        position = client.get_motor_position(0)
        if abs(position - 1) > .015:
            raise AssertionError(f"Unexpected position: {position}")
        print(json.dumps({"backend": "firmware", "build_id": board.build["build_id"],
                          "ready": client.ready, "protocol": list(board.schema.version),
                          "simulated_seconds": board.now_us / 1e6, "target_radians": 1,
                          "measured_radians": position}, indent=2))
    finally:
        client.disconnect()
        board.close()


if __name__ == "__main__":
    main()
