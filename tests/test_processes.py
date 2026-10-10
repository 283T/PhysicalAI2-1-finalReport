import signal
import subprocess
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from processes import stop_process


class ProcessCleanupTest(unittest.TestCase):
    def test_unresponsive_process_group_is_killed_and_reaped(self):
        code = (
            "import signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); "
            'print("ready", flush=True); time.sleep(60)'
        )
        process = subprocess.Popen(
            [sys.executable, "-u", "-c", code],
            stdout=subprocess.PIPE,
            text=True,
            start_new_session=True,
        )
        try:
            self.assertEqual(process.stdout.readline().strip(), "ready")
            stop_process(process, group=True, timeout=0.1)
            self.assertEqual(process.returncode, -signal.SIGKILL)
            stop_process(process, group=True)  # Repeated cleanup is safe.
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
            process.stdout.close()
