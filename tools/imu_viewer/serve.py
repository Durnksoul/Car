"""Serve the offline IMU viewer on localhost, using Python's standard library."""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import socket
import subprocess
from urllib.request import urlopen

class ViewerServer(ThreadingHTTPServer):
    allow_reuse_address = False

    def server_bind(self):
        # Windows SO_REUSEADDR can allow two live HTTP servers on one port.
        # Keep this listener exclusive so repeated launches reuse it instead.
        if os.name == "nt":
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()

def open_supported_browser(url):
    candidates = []
    for relative, name in (("Google/Chrome/Application/chrome.exe", "Chrome"),
                           ("Microsoft/Edge/Application/msedge.exe", "Edge")):
        for variable in ("PROGRAMFILES", "PROGRAMFILES(X86)", "LOCALAPPDATA"):
            base = os.environ.get(variable)
            if base:
                candidates.append((Path(base) / relative, name))
    for executable, name in candidates:
        if executable.is_file():
            subprocess.Popen([str(executable), url],
                             creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
            return {"ok": True, "browser": name}
    return {"ok": False, "error": "未找到独立 Chrome 或 Edge，请手动在浏览器中打开本地地址"}

class Handler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-IMU-Viewer", "1")
        super().end_headers()

    def send_json(self, value, status=200):
        data = json.dumps(value, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/viewer-info":
            self.send_json({"app": "imu-viewer"})
        else:
            super().do_GET()

    def do_POST(self):
        if self.path != "/open-browser":
            self.send_json({"error": "Not found"}, 404)
            return
        port = self.server.server_port
        allowed_origins = {f"http://127.0.0.1:{port}", f"http://localhost:{port}"}
        if (self.headers.get("Origin") not in allowed_origins or
                self.headers.get("Content-Type") != "application/json"):
            self.send_json({"error": "Only the local viewer can open its browser"}, 403)
            return
        # Fixed application URL and known browser executables, no client commands.
        try:
            self.send_json(open_supported_browser(f"http://127.0.0.1:{port}"))
        except OSError as error:
            self.send_json({"ok": False, "error": str(error)}, 500)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    url = f"http://127.0.0.1:{args.port}"
    root = Path(__file__).resolve().parent / "dist"
    handler = partial(Handler, directory=str(root))
    try:
        server = ViewerServer(("127.0.0.1", args.port), handler)
    except OSError as error:
        # Reuse this viewer's existing server when the launcher is double-clicked.
        try:
            with urlopen(url + "/viewer-info", timeout=1) as response:
                existing = json.load(response)
            if existing.get("app") == "imu-viewer":
                print(f"IMU viewer already running: {url}", flush=True)
                if not args.no_browser:
                    print(open_supported_browser(url), flush=True)
                return 0
        except (OSError, ValueError):
            pass
        print(f"Cannot start local viewer: {error}\nTry: python serve.py --port 8766", flush=True)
        return 1
    print(f"IMU viewer: {url}\nUse Chrome or Edge. Press Ctrl+C to stop.", flush=True)
    if not args.no_browser:
        try:
            print(open_supported_browser(url), flush=True)
        except OSError as error:
            print(f"Cannot open browser: {error}. Open {url} in Chrome or Edge.", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
