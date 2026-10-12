"""Testes do digital twin: protocolo, STM simulado e wizard de identificação.

Rodar com:  python -m pytest tests/ -q
"""

import math
import os

import matplotlib
import numpy as np
import pytest

from actuators import ActuatorParams, MotorCalib, ThrusterCluster
from main import build_system
from scenarios import SCENARIOS
from twin import protocol
from twin.protocol import TelemetryFrame
from twin.simstm import SimSTM
from twin.transport import SimTransport


# ------------------------------------------------------------- protocolo


def test_protocol_tel_roundtrip():
    line = ("TEL,12345,90.12,0.00,90.00,1.50,1.234,-0.567,4.500,-2.100,"
            "1620,1380,1500,1.00,2.35,15")
    frame = protocol.parse_line(line)
    assert isinstance(frame, TelemetryFrame)
    assert frame.t_ms == 12345
    assert frame.yaw_deg == pytest.approx(90.12)
    assert frame.pwm == (1620.0, 1380.0, 1500.0)
    assert frame.L_m == pytest.approx(1.0)
    assert frame.D_m == pytest.approx(2.35)
    assert frame.flags == 15
    assert frame.yaw_on and frame.swing_on and frame.vision_fresh
    assert not frame.wizard_active


def test_protocol_gain_and_commands():
    parsed = protocol.parse_line("GAIN,6,head_rate_kp_us,900")
    assert parsed == (6, "head_rate_kp_us", 900.0)
    assert protocol.cmd_gain(6, 912.5) == "G6 912.5"
    assert protocol.cmd_vision(-12.5, 3.2) == "V-12.500 3.200"
    assert protocol.cmd_wizard_pulse(1, -150, 600) == "W1 -150 600"
    assert protocol.parse_line("lixo") is None
    assert protocol.parse_line("") is None


# ------------------------------------------------------------- STM simulado


def make_simstm_perturbed() -> SimSTM:
    """STM simulado com motor b 'montado diferente' (rotacionado, yaw
    invertido, curva diferente) — o que o wizard deve descobrir."""
    plant, cluster, controller = build_system("nonlinear")
    calib_b = MotorCalib(
        label="b",
        thrust_dir=(math.cos(math.radians(80)), math.sin(math.radians(80))),
        yaw_sign=-1.0,
        kf_fwd=cluster.motors[1].kf_fwd * 1.5,
        kf_rev=cluster.motors[1].kf_rev * 1.5,
        deadzone_us=45.0,
    )
    cluster.apply_motor_calib(1, calib_b)
    calib_c = MotorCalib(
        label="c",
        thrust_dir=tuple(cluster.motors[2].thrust_dir),
        yaw_sign=cluster.motors[2].yaw_sign,
        kf_fwd=cluster.motors[2].kf_fwd * 0.7,
        kf_rev=cluster.motors[2].kf_rev * 0.7,
        deadzone_us=cluster.motors[2].deadzone_us,
    )
    cluster.apply_motor_calib(2, calib_c)
    return SimSTM(plant, cluster, controller)


def _drain(transport, n_frames):
    frames = []
    while len(frames) < n_frames:
        line = transport.read_line()
        if line:
            frame = protocol.parse_line(line)
            if isinstance(frame, TelemetryFrame):
                frames.append(frame)
            continue  # linha disponível: consome a fila antes de avançar
        transport.sim.advance(0.02)  # fila vazia: bombeia o simulador
    return frames


def test_simstm_protocol_commands():
    sim = SimSTM()
    transport = SimTransport(sim)

    # telemetria flui e reflete o estado
    sim.advance(0.2)
    frames = _drain(transport, 5)
    assert len(frames) == 5
    assert all(abs(f.pwm[0] - 1500) < 0.5 for f in frames)  # loops off no boot

    # toggles via protocolo
    transport.write_line("y")
    transport.write_line("p")
    sim.advance(0.1)
    frame = _drain(transport, 1)[0]
    assert frame.yaw_on and frame.swing_on

    # degrau relativo de yaw: esfera deve começar a girar
    transport.write_line("90")
    sim.advance(1.0)
    frames = _drain(transport, 10)
    assert any(f.rate_z_dps > 1.0 for f in frames)

    # ganhos ao vivo: R devolve GAIN com o valor aplicado por G
    transport.write_line("G0 12.5")
    transport.write_line("R")
    sim.advance(0.05)
    gains = {}
    while True:
        line = transport.read_line()
        if line is None:
            sim.advance(0.05)
            if len(gains) >= 8:
                break
            continue
        parsed = protocol.parse_line(line)
        if parsed and not isinstance(parsed, TelemetryFrame):
            idx, name, value = parsed
            gains[idx] = (name, value)
    assert gains[0] == ("vel_kp", 12.5)

    # parada geral (drenar frames antigos da fila antes de conferir flags —
    # a telemetria é FIFO e frames pré-'k' ainda estão na fila)
    transport.write_line("k")
    sim.advance(0.2)
    frames = _drain(transport, 60)
    assert not frames[-1].yaw_on and not frames[-1].swing_on


def test_simstm_vision_sets_pointing():
    sim = SimSTM()
    transport = SimTransport(sim)
    transport.write_line("y")  # liga heading
    transport.write_line("V30 2.0")  # alvo 30° à frente do corpo
    sim.advance(0.4)
    frames = _drain(transport, 3)
    assert all(f.vision_fresh for f in frames)
    assert frames[-1].bearing_deg == pytest.approx(30.0, abs=0.01)
    assert frames[-1].D_m == pytest.approx(2.0)
    # com a visão ativa, o bearing É o erro → esfera gira para reduzir
    sim.advance(2.0)
    frames = _drain(transport, 5)
    assert any(f.rate_z_dps > 0.5 for f in frames)


def test_simstm_length_command():
    sim = SimSTM()
    transport = SimTransport(sim)
    transport.write_line("L2.5")
    sim.advance(0.1)
    frame = _drain(transport, 1)[0]
    assert frame.L_m == pytest.approx(2.5)


# ---------------------------------------------------------------- wizard


def test_wizard_recovers_perturbed_calibration():
    """O teste central do twin: com o motor b montado 'diferente' (rotacionado
    ~80°, yaw invertido, curva 1,5×, dz 45 µs) e o c mais fraco (0,7×), o
    wizard mede a resposta REAL e recupera o modelo — nenhum default é
    assumido."""
    from wizard import identify

    sim = make_simstm_perturbed()
    transport = SimTransport(sim)
    calibs = identify(transport, verbose=False)

    true = sim.cluster.motors  # o modelo "físico" perturbado

    for i, (measured, truth) in enumerate(zip(calibs, true)):
        # direção: ângulo entre medido e verdadeiro < 12°
        dot = (measured.thrust_dir[0] * truth.thrust_dir[0]
               + measured.thrust_dir[1] * truth.thrust_dir[1])
        angle = math.degrees(math.acos(min(1.0, max(-1.0, dot))))
        assert angle < 12.0, f"motor {i}: direção errada ({angle:.1f}°)"
        # sinal de yaw: exato
        assert measured.yaw_sign == truth.yaw_sign, f"motor {i}: yaw_sign errado"
        # curva: dentro de ~35% (atraso de 120 ms do atuador vira viés
        # sistemático conhecido — documentado no wizard)
        assert abs(measured.kf_fwd / truth.kf_fwd - 1.0) < 0.35, (
            f"motor {i}: kf_fwd {measured.kf_fwd:.3e} vs {truth.kf_fwd:.3e}"
        )
        assert abs(measured.kf_rev / truth.kf_rev - 1.0) < 0.35, (
            f"motor {i}: kf_rev fora da tolerância"
        )


# ------------------------------------------------------------ TwinWindow


def test_twin_window_smoke_headless():
    """Fumigação end-to-end do twin: janela completa + STM simulado + teclado
    (comandos ao hardware) sem GUI — nada pode levantar exceção."""
    matplotlib.use("Agg", force=True)
    from interactive import LiveSession
    from main import build_system
    from twin.simstm import SimSTM
    from twin.transport import SimTransport
    from twin.window import TwinWindow

    simstm = SimSTM()
    transport = SimTransport(simstm)
    plant, cluster, controller = build_system("nonlinear")
    session = LiveSession(plant, cluster, controller, SCENARIOS["default"])
    win = TwinWindow(session, transport)

    class FakeEvent:
        def __init__(self, key):
            self.key = key

    # liga loops no real (protocolo) e no sim (super), degrau de heading —
    # e deixa o twin "rodar" alguns segundos (tempo de parede simulado em
    # vários ticks: o on_timer limita cada tick a 0,15 s por segurança)
    win.on_press(FakeEvent("2"))  # 'y' no firmware + toggle no sim
    win.on_press(FakeEvent("1"))  # 'p' no firmware + toggle no sim
    win.on_press(FakeEvent("x"))  # heading +15° nos dois lados
    for _ in range(30):
        win._twin_last -= 0.1
        win.on_timer()

    # telemetria real chegou e alimentou fantasma/painéis
    assert win.last_frame is not None
    assert len(win.real["t"]) > 3
    assert win.real_hud.get_text().startswith("REAL [CONECTADO]")
    # o degrau de heading realmente girou o STM simulado (lado "real")
    assert win.last_frame.yaw_on
    # ganho ajustado foi empurrado ao hardware (G0)
    g_before = controller.gains.vel_kp
    win.on_press(FakeEvent("g"))   # seleciona vel_kp
    win.on_press(FakeEvent("+"))   # ajusta +10% e envia G0 <valor>
    # o sim (lado esfera simulada) recebeu o mesmo ajuste via set_gains
    assert session.controller.gains.vel_kp > g_before

    # parada geral: comando 'k' ao real + loops off no sim
    win.on_press(FakeEvent("k"))
    assert session.controller.xy_enabled is False
    for _ in range(8):
        win._twin_last -= 0.1
        win.on_timer()
    os.makedirs("out", exist_ok=True)
    win.fig.savefig("out/_twin_smoke.png", dpi=72)
