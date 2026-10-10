"""Child-process cleanup shared by simulation checks and perception."""

import os
import signal
import subprocess


def stop_process(process, *, group=False, timeout=5):
    """Reap a child, escalating if needed. Groups require start_new_session=True."""
    if process.poll() is not None:
        process.wait()
        return

    def send(sig):
        try:
            if group:
                os.killpg(process.pid, sig)
            else:
                process.send_signal(sig)
        except ProcessLookupError:
            pass

    send(signal.SIGTERM)
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        send(signal.SIGKILL)
        process.wait()
