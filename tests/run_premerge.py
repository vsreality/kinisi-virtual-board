"""Run release unit and integration tests, and reject missing platform coverage."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

LINUX_PTY = {
    "test_native.LinuxPTYTests.test_cli_creates_native_port_without_socket",
    "test_native.LinuxPTYTests.test_native_serial_endpoint_uses_real_pyserial",
    "test_native.LinuxPTYTests.test_unmodified_sdk_through_real_pty",
    "test_standalone.StandaloneSerialTests.test_real_sdk_on_single_native_port",
}
WINDOWS_DIAGNOSTIC = "test_native.NativeLoopTests.test_pty_on_windows_explains_serial_alternative"
WINDOWS_PAIR = "test_native.WindowsCOMTests.test_unmodified_sdk_through_virtual_com_pair"


class CoverageResult(unittest.TextTestResult):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.passed = set()

    def addSuccess(self, test):
        super().addSuccess(test)
        self.passed.add(test.id())


def run(directory, minimum, allowed_skips=(), required_passes=()):
    suite = unittest.TestLoader().discover(str(directory))
    collected = suite.countTestCases()
    if collected < minimum:
        print(f"{directory.name}: collected {collected}, expected at least {minimum}", file=sys.stderr)
        return False
    result = unittest.TextTestRunner(verbosity=2, resultclass=CoverageResult).run(suite)
    skipped = {test.id() for test, _ in result.skipped}
    unexpected = skipped.difference(allowed_skips)
    missing = set(required_passes) - result.passed
    if unexpected or missing or len(result.passed) + len(skipped) != collected:
        print(f"{directory.name}: {len(result.passed)} distinct passes, {len(skipped)} skips, "
              f"{collected} collected; unexpected skips={sorted(unexpected)}, "
              f"missing required passes={sorted(missing)}", file=sys.stderr)
        return False
    return result.wasSuccessful()


def main():
    if sys.platform == "win32":
        # Local hardware-free rehearsal: Linux PTYs and optional COM/driver endpoints.
        allowed = LINUX_PTY | {WINDOWS_PAIR}
        required = {WINDOWS_DIAGNOSTIC}
    else:
        allowed = {WINDOWS_PAIR, WINDOWS_DIAGNOSTIC}
        required = LINUX_PTY
    # Release inventory floors guard against silently losing tests during discovery.
    common = run(ROOT / "tests", 220, allowed, required)
    comparator = run(ROOT / "tools" / "fusion_comparison", 18)
    return 0 if common and comparator else 1


if __name__ == "__main__":
    sys.exit(main())
