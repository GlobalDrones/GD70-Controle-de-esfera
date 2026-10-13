"""Ponte TCP embutida no app de visão (absorve o antigo rasp/agent.py).

O app de visão é o DONO da serial do STM; esta ponte expõe o mesmo fluxo por
TCP (porta 9999) para o twin e o wizard no PC:

    STM (USB) ── serial_communication ──┬── parser TEL → telemetria_dados/HUD
                                         └── bridge.broadcast(linha) → clientes TCP
    clientes TCP → bridge → on_client_line → escrita na serial

Import-safe no PC (sem gpiozero/cv2) — testável headless.
"""

import socket
import socketserver
import threading

DEFAULT_TCP_PORT = 9999


class _ClientHandler(socketserver.BaseRequestHandler):
    def handle(self):
        print(f"[BRIDGE] cliente conectado: {self.client_address}")
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
                        except Exception as exc:  # serial caiu? não derrubar o servidor
                            print(f"[BRIDGE] erro ao encaminhar comando: {exc}")
        except (ConnectionError, OSError):
            pass
        finally:
            with bridge._lock:
                bridge._clients.discard(self.request)
            print(f"[BRIDGE] cliente saiu: {self.client_address}")


class _ThreadedServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


class Bridge:
    """Servidor TCP: broadcast de linhas do STM; comandos dos clientes viram
    chamadas a ``on_client_line`` (escrita na serial do STM)."""

    def __init__(self, on_client_line=None, port: int = DEFAULT_TCP_PORT):
        self.on_client_line = on_client_line
        self.port = port
        self._clients = set()
        self._lock = threading.Lock()
        self._server = None

    def broadcast(self, line: str) -> None:
        """Envia uma linha do STM a todos os clientes conectados."""
        if self._server is None or not line:
            return
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

    def start(self) -> None:
        if self._server is not None:
            return
        self._server = _ThreadedServer(("0.0.0.0", self.port), _ClientHandler)
        self._server.bridge = self
        threading.Thread(
            target=self._server.serve_forever, daemon=True, name="bridge-tcp"
        ).start()
        print(f"[BRIDGE] TCP 0.0.0.0:{self.port} — twin/wizard podem conectar")

    def stop(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server = None
