"""Local dashboard: select an image, read it locally, then show Gazebo execution."""

import argparse
import json
import subprocess
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from project_paths import ROOT
from read_drawing import read_drawing


class Demo:
    def __init__(self):
        self.lock = threading.Lock()
        self.state = {"phase": "idle", "message": "図面を選んで実行してください", "variant": "A"}
        self.running = False
        self.process = None

    def update(self, **values):
        with self.lock:
            self.state.update(values)

    def start(self, variant):
        with self.lock:
            if self.running:
                return False
            self.running = True
            self.state = {
                "phase": "reading",
                "message": "ローカルAIが図面を読解中",
                "variant": variant,
            }
        threading.Thread(target=self.run, args=(variant,), daemon=True).start()
        return True

    def run(self, variant):
        try:
            started = time.monotonic()
            task = read_drawing(
                ROOT / f"drawings/assembly_{variant.lower()}.png",
                ROOT / "outputs/dashboard_task.json",
            )
            self.update(
                task=task,
                reading_seconds=round(time.monotonic() - started, 1),
                phase="preparing",
                message="読解結果を検証済み。Gazeboを準備中",
            )
            # Restart only the dedicated demo desktop, clearing the previous assembly.
            if self.process and self.process.poll() is None:
                self.process.terminate()
                self.process.wait(timeout=10)
            subprocess.run(
                ["docker", "compose", "--profile", "gui", "up", "-d", "gui"],
                cwd=ROOT,
                check=True,
                capture_output=True,
                text=True,
            )
            subprocess.run(
                ["docker", "compose", "--profile", "gui", "restart", "gui"],
                cwd=ROOT,
                check=True,
                capture_output=True,
                text=True,
            )
            deadline = time.monotonic() + 20
            while True:
                try:
                    with urllib.request.urlopen(
                        "http://127.0.0.1:6080/vnc.html", timeout=2
                    ) as response:
                        if response.status == 200:
                            break
                except OSError:
                    if time.monotonic() >= deadline:
                        raise RuntimeError("Gazebo表示サービスの起動を確認できません")
                    time.sleep(0.2)
            command = [
                "docker",
                "compose",
                "--profile",
                "gui",
                "exec",
                "-T",
                "gui",
                "/ros_entrypoint.sh",
                "python3",
                "scripts/run_insertion.py",
                "--task",
                "outputs/dashboard_task.json",
                "--gui",
                "--start-delay",
                "10",
                "--keep-open",
            ]
            self.process = subprocess.Popen(
                command,
                cwd=ROOT,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            self.update(phase="moving", message="Gazebo起動中。表示接続後に動作を開始します")
            labels = [
                "供給トレイへ接近",
                "プラグを持ち上げ",
                "接続パネル上へ搬送",
                "挿入箇所へ接近",
                "プラグを接続",
                "解放・退避",
            ]
            with (ROOT / "outputs/dashboard.log").open("w") as log:
                for line in self.process.stdout:
                    log.write(line)
                    log.flush()
                    for index, label in enumerate(labels, 1):
                        if line.startswith(f"{index}."):
                            self.update(step=index, message=label)
                    if line.startswith("Demo complete."):
                        result = json.loads((ROOT / "outputs/insertion_result.json").read_text())
                        if not result.get("success") or result.get("drawing_id") != variant:
                            raise RuntimeError("挿入結果が選択した図面と一致しません")
                        self.update(
                            phase="complete", message="接続・解放・退避が完了", result=result
                        )
                        # Stop reading here; Gazebo stays open until the next run or shutdown.
                        return
            raise RuntimeError(
                "ロボット実行に失敗しました。outputs/dashboard.logを確認してください"
            )
        except Exception as error:
            self.update(phase="error", message=str(error))
        finally:
            with self.lock:
                self.running = False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    demo = Demo()

    class Handler(BaseHTTPRequestHandler):
        def respond(self, code, data, content_type="application/json"):
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == "/status":
                with demo.lock:
                    data = json.dumps(demo.state, ensure_ascii=False).encode()
                self.respond(200, data)
                return
            allowed = {
                "/": (ROOT / "web/index.html", "text/html; charset=utf-8"),
                "/assembly_a.png": (ROOT / "drawings/assembly_a.png", "image/png"),
                "/assembly_b.png": (ROOT / "drawings/assembly_b.png", "image/png"),
            }
            if self.path not in allowed:
                self.respond(404, b"{}")
                return
            path, content_type = allowed[self.path]
            self.respond(200, path.read_bytes(), content_type)

        def do_POST(self):
            # Require same-origin requests. Do not allow other sites to start local processes.
            if self.headers.get("Origin") != f"http://127.0.0.1:{args.port}":
                self.respond(403, b"{}")
                return
            if self.path not in ("/run/A", "/run/B"):
                self.respond(404, b"{}")
                return
            accepted = demo.start(self.path[-1])
            self.respond(202 if accepted else 409, json.dumps({"accepted": accepted}).encode())

        def log_message(self, format, *values):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(f"Dashboard: http://127.0.0.1:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if demo.process and demo.process.poll() is None:
            demo.process.terminate()


if __name__ == "__main__":
    main()
