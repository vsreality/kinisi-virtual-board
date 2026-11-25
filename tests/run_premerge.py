"""Run the hardware-free simulator suites against the configured public firmware."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

SUITES = (
    "test_board",
    "test_firmware.FirmwareTests",
    "test_native.NativeLoopTests",
    "test_standalone.StandaloneTests",
)
if sys.platform.startswith("linux"):
    SUITES += ("test_native.LinuxPTYTests.test_native_serial_endpoint_uses_real_pyserial",)


def main():
    runner = unittest.TextTestRunner(verbosity=2)
    for name in SUITES:
        result = runner.run(unittest.defaultTestLoader.loadTestsFromName(name))
        if not result.wasSuccessful():
            return 1
        if result.testsRun <= len(result.skipped):
            print(f"No tests executed in {name}", file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
