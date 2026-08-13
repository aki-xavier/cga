"""渲染服务自检: health / 正常渲染 / 解析错误 400。"""

import io
import threading
import urllib.request
from http.server import HTTPServer

from PIL import Image

from cga.scene_lang.render_server import RenderServer
from tests.checks import Checks


class TestRenderServer(Checks):
    """常驻渲染 HTTP 服务 (编辑器预览后端) 的行为断言。"""

    @staticmethod
    def post(url: str, body: str) -> tuple[int, bytes]:
        req = urllib.request.Request(url, data=body.encode("utf-8"), method="POST")
        try:
            with urllib.request.urlopen(req) as res:
                return res.status, res.read()
        except urllib.error.HTTPError as e:  # type: ignore
            return e.code, e.read()

    def test_render_roundtrip_and_error(self):
        server = HTTPServer(("127.0.0.1", 0), RenderServer.handler())
        port = server.server_address[1]
        threading.Thread(target=server.serve_forever, daemon=True).start()
        try:
            with urllib.request.urlopen(f"http://127.0.0.1:{port}/health") as res:
                assert res.read() == b"ok"

            status, body = self.post(
                f"http://127.0.0.1:{port}/render?w=64&h=48&aa=1",
                "sphere(r=1);",
            )
            assert status == 200
            img = Image.open(io.BytesIO(body))
            assert img.size == (64, 48)  # 合法 PNG 且尺寸正确

            status, body = self.post(f"http://127.0.0.1:{port}/render", "blah();")
            assert status == 400
            assert "未知语句" in body.decode("utf-8")
        finally:
            server.shutdown()
