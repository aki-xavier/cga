"""CGS 渲染服务: 常驻 HTTP, 供编辑器实时预览 (避免进程冷启动)。

POST /render?w=640&h=480&aa=1  body = CGS 文本 → PNG 字节;
CGS 解析/渲染错误 → 400 + 纯文本错误信息。GET /health → "ok"。

运行: uv run python -m cga.scene_lang.render_server [端口=8123]
单线程串行: 编辑器防抖后请求稀疏, 且避免 GPU 渲染交叠。
"""

import sys
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs, urlparse

from cga.engine import Renderer
from cga.scene_lang import SceneLoader


class RenderServer:
    """CGS → PNG 的常驻渲染 HTTP 服务。"""

    @staticmethod
    def render_png(text: str, w: int, h: int, aa: int) -> bytes:
        """CGS 文本 → PNG 字节 (SceneLoader 解析 + Renderer 渲染 + PIL 编码)。"""
        import io

        from PIL import Image

        scene, camera = SceneLoader.load(text)
        camera.aspect = w / h
        img = Renderer(w, h, aa=aa).render(scene, camera)
        buf = io.BytesIO()
        Image.frombytes("RGBA", (w, h), Renderer.frame_to_bytes(img)).save(buf, "PNG")
        return buf.getvalue()

    @staticmethod
    def handler() -> type[BaseHTTPRequestHandler]:
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                if urlparse(self.path).path == "/health":
                    self.send_response(200)
                    self.end_headers()
                    self.wfile.write(b"ok")
                else:
                    self.send_response(404)
                    self.end_headers()

            def do_POST(self) -> None:
                url = urlparse(self.path)
                if url.path != "/render":
                    self.send_response(404)
                    self.end_headers()
                    return
                q = parse_qs(url.query)
                w = int(q.get("w", ["640"])[0])
                h = int(q.get("h", ["480"])[0])
                aa = int(q.get("aa", ["1"])[0])
                text = self.rfile.read(int(self.headers["content-length"]))
                try:
                    png = RenderServer.render_png(text.decode("utf-8"), w, h, aa)
                except (ValueError, KeyError) as e:
                    self.send_response(400)
                    self.end_headers()
                    self.wfile.write(str(e).encode("utf-8"))
                    return
                self.send_response(200, "OK")
                self.send_header("content-type", "image/png")
                self.end_headers()
                self.wfile.write(png)

            def log_message(self, *args) -> None:
                pass

        return Handler

    @staticmethod
    def main() -> None:
        port = int(sys.argv[1]) if len(sys.argv) > 1 else 8123
        server = HTTPServer(("127.0.0.1", port), RenderServer.handler())
        print(f"CGS render server on http://127.0.0.1:{port}")
        server.serve_forever()


if __name__ == "__main__":
    RenderServer.main()
