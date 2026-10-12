"""Transportes do twin: como o PC conversa com o STM.

- :class:`SerialTransport` — STM plugado direto no PC via USB (bancada,
  zero rede).
- :class:`TcpTransport` — agente na Raspberry Pi (``rasp/agent.py``) servindo
  a Serial2 do STM por TCP.
- :class:`SimTransport` — STM simulado (:class:`twin.simstm.SimSTM`) no mesmo
  processo, para usar/validar o twin sem hardware.
"""

from __future__ import annotations

import queue
import socket
from typing import Optional


class Transport:
    """Interface de linha: write_line envia comando; read_line recebe uma
    linha (TEL/GAIN/...) ou None se nada disponível."""

    def read_line(self, timeout: Optional[float] = None) -> Optional[str]:
        raise NotImplementedError

    def write_line(self, line: str) -> None:
        raise NotImplementedError

    def close(self) -> None:
        pass

    @property
    def description(self) -> str:
        return type(self).__name__


class SerialTransport(Transport):
    def __init__(self, port: str, baud: int = 115200):
        import serial  # pyserial

        self._ser = serial.Serial(port, baud, timeout=0.01)
        self._port = port

    def read_line(self, timeout: Optional[float] = None) -> Optional[str]:
        line = self._ser.readline()
        if not line:
            return None
        return line.decode("ascii", errors="replace").strip()

    def write_line(self, line: str) -> None:
        self._ser.write((line + "\n").encode("ascii"))

    def close(self) -> None:
        self._ser.close()

    @property
    def description(self) -> str:
        return f"serial {self._port}"


class TcpTransport(Transport):
    def __init__(self, host: str, port: int = 9999, connect_timeout: float = 5.0):
        self._sock = socket.create_connection((host, port), timeout=connect_timeout)
        self._sock.settimeout(0.01)
        self._buf = b""
        self._addr = f"{host}:{port}"

    def read_line(self, timeout: Optional[float] = None) -> Optional[str]:
        try:
            chunk = self._sock.recv(4096)
            if not chunk:
                raise ConnectionError("conexão fechada pelo agente")
            self._buf += chunk
        except socket.timeout:
            pass
        if b"\n" not in self._buf:
            return None
        line, self._buf = self._buf.split(b"\n", 1)
        return line.decode("ascii", errors="replace").strip()

    def write_line(self, line: str) -> None:
        self._sock.sendall((line + "\n").encode("ascii"))

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass

    @property
    def description(self) -> str:
        return f"tcp {self._addr}"


class SimTransport(Transport):
    """Transporte para o STM simulado no mesmo processo.

    O chamador bombeia o tempo do simulador chamando ``sim.advance(dt)``
    (o twin faz isso no timer da janela).
    """

    def __init__(self, sim: "twin.simstm.SimSTM"):  # noqa: F821
        self.sim = sim

    def read_line(self, timeout: Optional[float] = None) -> Optional[str]:
        try:
            return self.sim.out_queue.get_nowait()
        except queue.Empty:
            return None

    def write_line(self, line: str) -> None:
        for piece in line.split("\n"):
            if piece.strip():
                self.sim.handle_command(piece.strip())

    @property
    def description(self) -> str:
        return "STM simulado"
