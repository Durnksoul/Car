"""Verify local browser-launch routing with mocks; never launch a real browser."""
from functools import partial
from pathlib import Path
import threading
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen
import json

import serve

class ServerTests(unittest.TestCase):
    def setUp(self):
        self.server = serve.ViewerServer(("127.0.0.1", 0),
            partial(serve.Handler, directory=str(Path(__file__).parent / "dist")))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = f"http://127.0.0.1:{self.server.server_port}"

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def post(self, origin):
        return urlopen(Request(self.url + "/open-browser", data=b"{}",
            headers={"Content-Type": "application/json", "Origin": origin}), timeout=2)

    def test_page_and_identity(self):
        with urlopen(self.url, timeout=2) as response:
            self.assertEqual(response.status, 200)
            self.assertIn(b'open-browser', response.read())
        with urlopen(self.url + "/viewer-info", timeout=2) as response:
            self.assertEqual(json.load(response), {"app": "imu-viewer"})

    def test_second_listener_is_rejected(self):
        with self.assertRaises(OSError):
            serve.ViewerServer(("127.0.0.1", self.server.server_port), serve.Handler)

    @patch.object(serve, "open_supported_browser", return_value={"ok": True, "browser": "Chrome"})
    def test_local_request(self, launch):
        with self.post(self.url) as response:
            self.assertEqual(json.load(response), {"ok": True, "browser": "Chrome"})
        launch.assert_called_once_with(self.url)

    @patch.object(serve, "open_supported_browser")
    def test_foreign_origin_does_not_launch(self, launch):
        with self.assertRaises(HTTPError) as caught:
            self.post("https://example.com")
        self.assertEqual(caught.exception.code, 403)
        launch.assert_not_called()

    @patch.object(serve.subprocess, "Popen")
    @patch.object(serve.Path, "is_file", return_value=True)
    @patch.dict(serve.os.environ, {"PROGRAMFILES": "C:/Program Files"})
    def test_explicit_browser_executable(self, exists, launch):
        self.assertEqual(serve.open_supported_browser(self.url), {"ok": True, "browser": "Chrome"})
        command = launch.call_args.args[0]
        self.assertTrue(command[0].replace("\\", "/").endswith("Google/Chrome/Application/chrome.exe"))
        self.assertEqual(command[1], self.url)

if __name__ == "__main__":
    unittest.main()
