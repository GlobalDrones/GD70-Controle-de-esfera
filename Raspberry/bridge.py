"""Ponte TCP (porta 9999) + Servidor Web/SSE (porta 8080) embutidos no app de visão.

O app de visão é o DONO da serial do STM; esta ponte expõe o mesmo fluxo por:
  1. TCP puro (porta 9999) para o twin Python e o wizard.py no PC;
  2. HTTP + Server-Sent Events (porta 8080) para o supervisorio_esfera.html
     no navegador (/api/stream, /api/telemetria e /api/cmd).

Import-safe no PC (sem gpiozero/cv2) — testável headless.
"""

import json
import os
import socket
import socketserver
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

DEFAULT_TCP_PORT = 9999
DEFAULT_HTTP_PORT = 8080

# Raiz do repositório (procura supervisorio_esfera.html em esfera/ ou na pasta pai)
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_HERE)
_PARENT_ROOT = os.path.dirname(_REPO_ROOT)


class _ClientHandler(socketserver.BaseRequestHandler):
    def handle(self):
        print(f"[BRIDGE] cliente TCP conectado: {self.client_address}")
        bridge = self.server.bridge
        with bridge._lock:
            bridge._clients.add(self.request)
        try:
            for raw in iter(lambda: self.request.recv(256), b""):
                for piece in raw.split(b"\n"):
                    piece = piece.strip()
                    if piece and bridge.on_client_line is not None:
                        try:
                            bridge.on_client_line(piece.decode("ascii", errors="replace"))
                        except Exception as exc:
                            print(f"[BRIDGE] erro ao encaminhar comando TCP: {exc}")
        except (ConnectionError, OSError):
            pass
        finally:
            with bridge._lock:
                bridge._clients.discard(self.request)
            print(f"[BRIDGE] cliente TCP saiu: {self.client_address}")


class _ThreadedServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


class _ThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def _make_http_handler(bridge):
    serve_dir = _REPO_ROOT
    if not os.path.exists(os.path.join(_REPO_ROOT, "supervisorio_esfera.html")):
        if os.path.exists(os.path.join(_PARENT_ROOT, "supervisorio_esfera.html")):
            serve_dir = _PARENT_ROOT

    class _HTTPHandler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=serve_dir, **kwargs)

        def log_message(self, format, *args):
            pass

        def do_GET(self):
            parsed = urlparse(self.path)

            if parsed.path == "/api/stream":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                with bridge._sse_lock:
                    bridge._sse_clients.append(self.wfile)
                try:
                    while bridge._http_server is not None:
                        time.sleep(0.5)
                except Exception:
                    pass
                finally:
                    with bridge._sse_lock:
                        if self.wfile in bridge._sse_clients:
                            bridge._sse_clients.remove(self.wfile)
                return

            if parsed.path == "/api/telemetria":
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                payload = json.dumps({"line": bridge.last_line, "connected": bool(bridge.last_line)})
                self.wfile.write(payload.encode("utf-8"))
                return

            if parsed.path == "/api/cmd":
                params = parse_qs(parsed.query)
                cmd = params.get("c", [""])[0].strip()
                ok = False
                if cmd and bridge.on_client_line is not None:
                    try:
                        bridge.on_client_line(cmd)
                        ok = True
                    except Exception as exc:
                        print(f"[BRIDGE] erro ao encaminhar comando HTTP: {exc}")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(json.dumps({"success": ok, "cmd": cmd}).encode("utf-8"))
                return

            if parsed.path in ("/", "/index.html"):
                self.path = "/supervisorio_esfera.html"

            return super().do_GET()

    return _HTTPHandler


class Bridge:
    """Servidor TCP (porta 9999) + HTTP/SSE (porta 8080): broadcast de linhas
    do STM; comandos dos clientes viram chamadas a ``on_client_line``."""

    def __init__(
        self,
        on_client_line=None,
        port: int = DEFAULT_TCP_PORT,
        http_port: int = DEFAULT_HTTP_PORT,
    ):
        self.on_client_line = on_client_line
        self.port = port
        self.http_port = http_port
        self._clients = set()
        self._lock = threading.Lock()
        self._server = None

        self._sse_clients = []
        self._sse_lock = threading.Lock()
        self._http_server = None
        self.last_line = ""

    def broadcast(self, line: str) -> None:
        """Envia uma linha do STM a todos os clientes TCP e navegadores SSE."""
        if not line:
            return
        self.last_line = line

        if self._server is not None:
            data = (line + "\n").encode("ascii", errors="replace")
            with self._lock:
                dead = []
                for sock in self._clients:
                    try:
                        sock.sendall(data)
                    except OSError:
                        dead.append(sock)
                for sock in dead:
                    self._clients.discard(sock)

        if self._http_server is not None:
            sse_data = f"data: {line}\n\n".encode("utf-8", errors="replace")
            with self._sse_lock:
                dead_sse = []
                for wfile in self._sse_clients:
                    try:
                        wfile.write(sse_data)
                        wfile.flush()
                    except Exception:
                        dead_sse.append(wfile)
                for wfile in dead_sse:
                    self._sse_clients.remove(wfile)

    def start(self) -> None:
        if self._server is None:
            self._server = _ThreadedServer(("0.0.0.0", self.port), _ClientHandler)
            self._server.bridge = self
            threading.Thread(
                target=self._server.serve_forever, daemon=True, name="bridge-tcp"
            ).start()
            print(f"[BRIDGE] TCP 0.0.0.0:{self.port} — twin/wizard podem conectar")

        if self._http_server is None and self.http_port:
            try:
                handler_cls = _make_http_handler(self)
                self._http_server = _ThreadingHTTPServer(("0.0.0.0", self.http_port), handler_cls)
                threading.Thread(
                    target=self._http_server.serve_forever, daemon=True, name="bridge-http"
                ).start()
                print(
                    f"[BRIDGE] Web/SSE http://0.0.0.0:{self.http_port}/supervisorio_esfera.html ativo"
                )
            except OSError as exc:
                print(f"[BRIDGE] Aviso: porta HTTP {self.http_port} ocupada ({exc})")

    def stop(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server = None
        if self._http_server is not None:
            self._http_server.shutdown()
            self._http_server = None