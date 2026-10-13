"""Testes da camada Raspberry/ mesclada (headless, sem câmeras/GPIO).

Cobrem o que roda no PC: mapeamento TEL v2 → telemetria_dados (mesma API do
HUD), formato do contrato de visão V<bearing> <D>, ponte TCP (broadcast +
comando reverso) e o bearing com sinal (se houver cv2 instalado).
"""

import os
import socket
import sys
import time

import pytest

RASP_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "Raspberry"
)
sys.path.insert(0, RASP_DIR)

import serial_communication  # noqa: E402  (efeitos no import: sem GPIO/STM no PC → arduino=None)


class FakeSerial:
    def __init__(self):
        self.written = []

    @property
    def is_open(self):
        return True

    def write(self, data):
        self.written.append(data)


@pytest.fixture()
def fake_serial(monkeypatch):
    fake = FakeSerial()
    monkeypatch.setattr(serial_communication, "arduino", fake)
    yield fake


# ------------------------------------------------- TEL v2 → telemetria_dados


def test_tel_line_updates_telemetria_dict():
    line = (
        "TEL,1000,45.00,10.00,35.00,1.20,1.500,-0.500,4.000,-2.000,"
        "1620,1380,1500,2.00,2.50,15"
    )
    serial_communication.processar_linha_telemetria(line)
    d = serial_communication.telemetria_dados
    assert d["yaw"] == pytest.approx(45.0)
    assert d["yaw_des"] == pytest.approx(35.0)
    assert d["rate_z"] == pytest.approx(1.20)
    assert (d["pwm_bf"], d["pwm_as"], d["pwm_tr"]) == (1620, 1380, 1500)
    assert d["sx"] == pytest.approx(1.5)
    assert d["sy"] == pytest.approx(-0.5)
    assert d["theta"] == pytest.approx(1.581, abs=0.01)  # hypot(1.5, 0.5) em graus
    assert d["bearing"] == pytest.approx(10.0)
    assert d["D"] == pytest.approx(2.5)
    assert d["L"] == pytest.approx(2.0)
    assert d["swing_ativo"] == 1  # flags=15 → bits 1|2|4|8
    assert d["yaw_ativo"] == 1
    assert d["erro"] == pytest.approx(10.0)  # wrap(45 − 35)


def test_non_tel_lines_are_passthrough(capsys):
    serial_communication.processar_linha_telemetria(">> [YAW]: ATIVADO")
    assert "YAW" in capsys.readouterr().out


# ------------------------------------------------------ contrato de visão V


def test_send_ang_serial_v_format(fake_serial):
    serial_communication.send_ang_serial(12.3, 2.5)
    assert fake_serial.written[-1] == b"V12.30 2.50\n"
    serial_communication.send_ang_serial(-5.0)  # mono: D = -1
    assert fake_serial.written[-1] == b"V-5.00 -1.00\n"
    # clamp do bearing em ±90°
    serial_communication.send_ang_serial(120.0, 1.0)
    assert fake_serial.written[-1] == b"V90.00 1.00\n"


# ------------------------------------------------------------------- bridge


def test_bridge_broadcast_and_reverse_command():
    from bridge import Bridge

    received = []
    br = Bridge(on_client_line=received.append, port=19999)
    br.start()
    try:
        client = socket.create_connection(("127.0.0.1", 19999), timeout=3)
        client.settimeout(2.0)
        time.sleep(0.3)  # handler registrar o cliente

        br.broadcast("TEL,42")
        data = client.recv(256).decode().strip()
        assert data.startswith("TEL,42")

        client.sendall(b"p\n")
        deadline = time.time() + 2.0
        while not received and time.time() < deadline:
            time.sleep(0.05)
        assert received == ["p"]
        client.close()
    finally:
        br.stop()


# ------------------------------------------------- bearing com sinal (cv2?)


def test_bearing_com_sinal():
    cv2 = pytest.importorskip("cv2")  # módulo da câmera importa cv2
    from camera_and_image_manipulation import bearing_com_sinal

    # linha vertical (para baixo na imagem): 0°
    assert bearing_com_sinal(100, 10, 100, 200) == pytest.approx(0.0)
    # inclinada ~45° para a direita: sinal positivo
    assert bearing_com_sinal(100, 10, 200, 110) == pytest.approx(45.0)
    # inclinada ~45° para a esquerda: sinal negativo
    assert bearing_com_sinal(200, 10, 100, 110) == pytest.approx(-45.0)
    # linha dada "de cabeça para baixo" (pontas trocadas) dá o MESMO bearing
    # (a linha é simétrica — normalização para (-90, 90])
    assert bearing_com_sinal(200, 110, 100, 10) == pytest.approx(45.0, abs=1e-6)
