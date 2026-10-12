#!/usr/bin/env python3
"""Agente da Raspberry Pi: Serial2 do STM ↔ TCP para o twin no PC.

Roda na Rasp (Python 3 + pyserial), lendo a Serial2 do STM e servindo as
linhas por TCP para múltiplos clientes (twin, wizard, netcat...); comandos
enviados pelos clientes vão para a serial do STM.

Uso (na Rasp):

    python3 agent.py                       # /dev/serial0 @115200, TCP 9999
    python3 agent.py --port /dev/ttyAMA0 --tcp 8765

Deploy recomendado: systemd (ver esfera-agent.service ao lado) —

    scp rasp/agent.py rasp/esfera-agent.service pi@<rasp>:~
    ssh pi@<rasp> 'sudo mv esfera-agent.service /etc/systemd/system/ && \
                   sudo systemctl enable --now esfera-agent'

Se o PC não alcançar a Rasp por IP direto, use túnel SSH (no PC):

    ssh -L 9999:localhost:9999 pi@<rasp>
    python twin.py --tcp 127.0.0.1:9999
"""

from __future__ import annotations

import argparse
import socket
import socketserver
import threading

import serial

ser: serial.Serial
clients_lock = threading.Lock()
clients: set = set()


def broadcast(line: str) -> None:
    data = (line + "\n").encode("ascii")
    with clients_lock:
        dead = []
        for sock in clients:
            try:
                sock.sendall(data)
            except OSError:
                dead.append(sock)
        for sock in dead:
            clients.discard(sock)


def serial_reader() -> None:
    while True:
        try:
            line = ser.readline()
            if line:
                broadcast(line.decode("ascii", errors="replace").strip())
        except serial.SerialException as exc:
            print(f"[agent] erro na serial: {exc} — tentando reabrir em 2 s")
            global ser
            try:
                ser.close()
            except Exception:
                pass
            threading.Event().wait(2.0)
            try:
                ser = serial.Serial(ser.port, ser.baudrate, timeout=0.05)
            except serial.SerialException:
                pass


class ClientHandler(socketserver.BaseRequestHandler):
    def handle(self):
        print(f"[agent] cliente conectado: {self.client_address}")
        with clients_lock:
            clients.add(self.request)
        try:
            for raw in iter(lambda: self.request.recv(256), b""):
                for piece in raw.split(b"\n"):
                    piece = piece.strip()
                    if piece:
                        ser.write(piece + b"\n")
        except (ConnectionError, OSError):
            pass
        finally:
            with clients_lock:
                clients.discard(self.request)
            print(f"[agent] cliente saiu: {self.client_address}")

    # threads: cada cliente no seu thread
    def setup(self):
        self.request.settimeout(None)


class ThreadedServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main() -> None:
    global ser
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/serial0", help="porta serial do STM")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--tcp", type=int, default=9999, help="porta TCP do servidor")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    threading.Thread(target=serial_reader, daemon=True).start()

    server = ThreadedServer(("0.0.0.0", args.tcp), ClientHandler)
    print(f"[agent] serial {args.port}@{args.baud} ↔ TCP 0.0.0.0:{args.tcp}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
