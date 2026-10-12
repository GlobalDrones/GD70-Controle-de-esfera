"""Wizard de identificação: mede o modelo de cada motor pela resposta da esfera.

Substitui TODAS as suposições de atuador (direção de empuxo, sinal de yaw,
curva µs→N frente/ré, zona morta) por medição — nada assume que os 3 motores
foram soldados ou montados da mesma forma.

Procedimento (por motor, com loops de controle DESLIGADOS e esfera pendurada):

    para cada nível u ∈ {+240, +170, +100, −100, −170, −240} µs:
        0,5 s de linha de base → pulso de 600 ms (comando W) → 1,2 s de repouso

Medição (telemetria TEL a 25 Hz): resposta das taxas de inclinação NO MUNDO
(wx, wy) e da taxa de yaw durante o pulso, menos a linha de base.

Ajuste:

- direção de empuxo: componente mundial da resposta ao pulso +240, girada
  para o corpo por Rz(−yaw) do instante do pulso;
- yaw_sign: sinal da resposta da taxa de yaw ao pulso +240;
- curva: F = m·L·2·mean(θ̇)/T do pulso em cada nível; sqrt(F) = sqrt(kf)·(u−dz)
  (regressão linear por sentido → kf e dz por motor);
- kf_rev/kf_fwd pela razão das respostas ±u.

Saída: calib.json (mesmo formato do cluster Python) + relatório; ``--push``
envia os parâmetros ao STM (comando M, RAM).

Uso:

    python wizard.py --sim                 # valida no STM simulado
    python wizard.py --serial COM7         # STM no USB do PC
    python wizard.py --tcp 192.168.0.50    # via agente na Rasp
"""

from __future__ import annotations

import argparse
import math
import time
from typing import List, Optional, Tuple

import numpy as np

from actuators import MotorCalib
from main import build_system
from twin import protocol
from twin.protocol import TelemetryFrame
from twin.simstm import SimSTM
from twin.transport import SimTransport, Transport

MASS_KG = 12.0
PULSE_LEVELS = (240, 170, 100)
PULSE_MS = 600
BASELINE_FRAMES = 12   # 0,48 s @ 25 Hz
PULSE_FRAMES = 15      # 0,60 s
SETTLE_FRAMES = 30     # 1,20 s
ACTUATOR_TAU_S = 0.12  # atraso de 1ª ordem ESC+hélice (refinar na bancada)


def lag_mean_factor(T: float, tau: float, omega: float) -> float:
    """g = mean(θ̇ | força rampada por lag τ) / mean(θ̇ | força constante).

    Forma fechada: θ̇(t) = (1/(mL))∫₀ᵗ F(s)·cos(ω(t−s))ds com F(s) =
    F∞·(1−e^{−s/τ}); integrando duas vezes e normalizando pelo caso
    constante. Sem isso, o kf medido sai ~τ-biased ao quadrado (F medido
    0,8× ⇒ kf 0,64×).
    """
    if tau <= 1e-9:
        return 1.0
    k_const = (1.0 - math.cos(omega * T)) / (omega * omega)
    a = 1.0 / tau + 1j * omega
    term = (
        (np.exp(1j * omega * T) - 1.0) / (1j * omega * a)
        - tau * (1.0 - np.exp(-T / tau)) / a
    )
    k_lag = k_const - term.real
    return float(k_lag / k_const)


def next_frame(transport: Transport) -> TelemetryFrame:
    """Próximo frame de telemetria (bomba o STM simulado, se houver)."""
    sim = getattr(transport, "sim", None)
    while True:
        line = transport.read_line()
        if line:
            frame = protocol.parse_line(line)
            if isinstance(frame, TelemetryFrame):
                return frame
        if sim is not None:
            sim.advance(0.02)
        else:
            time.sleep(0.004)


def collect(transport: Transport, n: int) -> List[TelemetryFrame]:
    return [next_frame(transport) for _ in range(n)]


def _fit_leftover_sinusoid(frames, omega):
    """Ajusta o resíduo de oscilação livre no MUNDO: w_j(t) = a_j·sin(ωt) +
    b_j·cos(ωt) + c_j (eixo fixo no mundo; mínimo quadrados com t em s).

    O pêndulo quase não amortece (ζ ≈ 0,02) — a oscilação deixada pelo pulso
    anterior precisa ser modelada e subtraída, senão o sinal do pulso atual
    fica soterrado (a ~16 °/s de sobra vs ~2 °/s de resposta reversa).
    """
    t = np.array([(f.t_ms - frames[0].t_ms) / 1000.0 for f in frames])
    basis = np.stack([np.sin(omega * t), np.cos(omega * t), np.ones_like(t)]).T
    w = np.array([[f.wx_dps, f.wy_dps] for f in frames])  # (n, 2)
    coef, *_ = np.linalg.lstsq(basis, w, rcond=None)  # (3, 2)
    return coef, frames[0].t_ms


def _leftover_at(coef, t_ms0, t_ms, omega):
    """Valor do sinusóide residual no instante t_ms (mesma base do fit)."""
    t = (t_ms - t_ms0) / 1000.0
    row = np.array([math.sin(omega * t), math.cos(omega * t), 1.0])
    return row @ coef  # (2,)


def run_pulse(transport: Transport, motor: int, us: int) -> dict:
    """Um pulso do wizard: linha de base → pulso → repouso.

    Medição em dois passos: (1) o sinusóide de oscilação livre residual é
    ajustado na janela de baseline (frequência √(g/L) conhecida, eixo fixo
    no MUNDO) e subtraído das amostras do pulso; (2) o resíduo limpo é
    transportado para o CORPO por Rz(−yaw(t)) amostra a amostra — o pulso
    gira a esfera ~15° em 600 ms, e transportar antes da média borraria a
    direção.
    """
    baseline = collect(transport, BASELINE_FRAMES)
    base_rz = float(np.mean([f.rate_z_dps for f in baseline]))
    base_L = float(np.mean([f.L_m for f in baseline]))
    omega = math.sqrt(9.80665 / base_L)
    coef, t0_ms = _fit_leftover_sinusoid(baseline, omega)

    transport.write_line(protocol.cmd_wizard_pulse(motor, us, PULSE_MS))
    during = collect(transport, PULSE_FRAMES)
    collect(transport, SETTLE_FRAMES)

    body_resp = []
    ratez = []
    for f in during:
        w_world = np.array([f.wx_dps, f.wy_dps]) - _leftover_at(coef, t0_ms, f.t_ms, omega)
        yaw = math.radians(f.yaw_deg)
        c, s = math.cos(yaw), math.sin(yaw)
        body_resp.append([c * w_world[0] + s * w_world[1], -s * w_world[0] + c * w_world[1]])
        ratez.append(f.rate_z_dps - base_rz)

    return {
        "us": us,
        "w_body": np.mean(body_resp, axis=0),  # °/s médio no corpo
        "rate_z": float(np.mean(ratez)),
        "L": base_L,
    }


def identify(transport: Transport, verbose: bool = True) -> List[MotorCalib]:
    cluster = build_system("nonlinear")[1]
    calibs: List[MotorCalib] = []

    for motor, label in enumerate(cluster.LABELS):
        if verbose:
            print(f"\n=== Motor {label} (índice {motor}) ===")
        pulses = [run_pulse(transport, motor, u) for u in PULSE_LEVELS]
        pulses += [run_pulse(transport, motor, -u) for u in reversed(PULSE_LEVELS)]

        # Direção de empuxo no CORPO a partir do pulso mais forte (+240).
        strongest = pulses[0]
        w_body = np.asarray(strongest["w_body"])
        norm = float(np.linalg.norm(w_body))
        if norm < 1e-3:
            print(f"[aviso] motor {label}: resposta ínfima — defaults mantidos")
            calibs.append(cluster.motors[motor])
            continue
        t_body = w_body / norm

        yaw_sign = 1.0 if strongest["rate_z"] >= 0 else -1.0

        # Curva µs→N: da resposta média do pêndulo ao pulso. Para força
        # constante F durante T partindo do repouso: mean(θ̇) =
        # F·(1−cos(ωT))/(m·L·ω²·T), com ω = √(g/L). O atraso de 1ª ordem do
        # atuador (τ) reduz a resposta por lag_mean_factor — corrigido.
        def force_of(p: dict) -> float:
            along = float(np.asarray(p["w_body"]) @ t_body) * (math.pi / 180.0)
            L = p["L"]
            omega = math.sqrt(9.80665 / L)
            T = PULSE_MS / 1000.0
            g_lag = lag_mean_factor(T, ACTUATOR_TAU_S, omega)
            return (
                along * MASS_KG * L * omega * omega * T
                / ((1.0 - math.cos(omega * T)) * g_lag)
            )

        def fit(levels: List[dict]) -> Tuple[float, float]:
            us = np.array([abs(p["us"]) for p in levels], dtype=float)
            f = np.array([abs(force_of(p)) for p in levels], dtype=float)
            ok = f > 1e-4
            if ok.sum() < 2:
                return 0.0, 0.0
            a, b = np.polyfit(us[ok], np.sqrt(f[ok]), 1)  # sqrt(F)=a·u+b
            if a <= 0:
                return 0.0, 0.0
            return a * a, -b / a  # kf, dz

        fwd = [p for p in pulses if p["us"] > 0]
        rev = [p for p in pulses if p["us"] < 0]
        kf_fwd, dz_fwd = fit(fwd)
        kf_rev, dz_rev = fit(rev)
        default = cluster.motors[motor]
        if kf_fwd <= 0:
            kf_fwd, dz_fwd = default.kf_fwd, default.deadzone_us
        if kf_rev <= 0:
            kf_rev, dz_rev = default.kf_rev, default.deadzone_us
        # dz do fit pode sair sem sentido (negativo/gigante) → limita ao plausível
        dz = float(np.mean([dz_fwd, dz_rev]))
        if not (0.0 <= dz <= 150.0):
            dz = default.deadzone_us

        calib = MotorCalib(
            label=label,
            thrust_dir=(float(t_body[0]), float(t_body[1])),
            yaw_sign=yaw_sign,
            kf_fwd=kf_fwd,
            kf_rev=kf_rev,
            deadzone_us=dz,
        )
        calibs.append(calib)
        if verbose:
            print(f"  empuxo (corpo): ({t_body[0]:+.3f}, {t_body[1]:+.3f})")
            print(f"  yaw_sign: {yaw_sign:+.0f}")
            print(f"  kf_fwd: {kf_fwd:.3e}  kf_rev: {kf_rev:.3e}  dz: {dz:.1f} µs")
            print(f"  (default era ({default.thrust_dir[0]:+.3f}, "
                  f"{default.thrust_dir[1]:+.3f}), yaw {default.yaw_sign:+.0f}, "
                  f"kf {default.kf_fwd:.3e})")
    return calibs


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sim", action="store_true", help="STM simulado (validação)")
    parser.add_argument("--serial", default=None, help="porta serial do STM no PC")
    parser.add_argument("--tcp", default=None, metavar="HOST[:PORTA]",
                        help="agente na Rasp")
    parser.add_argument("--out", default="calib.json", help="arquivo de saída")
    parser.add_argument("--push", action="store_true",
                        help="envia a calibração ao STM (comando M, RAM)")
    args = parser.parse_args(argv)

    if args.sim:
        transport: Transport = SimTransport(SimSTM())
    elif args.serial:
        from twin.transport import SerialTransport

        transport = SerialTransport(args.serial)
    elif args.tcp:
        from twin.transport import TcpTransport

        host, _, port = args.tcp.partition(":")
        transport = TcpTransport(host, int(port or 9999))
    else:
        parser.error("escolha --sim, --serial PORTA ou --tcp HOST[:PORTA]")

    print(f"Wizard de identificação via {transport.description}")
    print("AVISO: esfera deve estar pendurada e LIVRE; os loops serão desligados.")
    calibs = identify(transport)

    # Salva no formato do cluster (com parâmetros default do cluster real).
    cluster = build_system("nonlinear")[1]
    for motor, calib in enumerate(calibs):
        cluster.apply_motor_calib(motor, calib)
    cluster.save_calibration(args.out)
    print(f"\nCalibração salva em {args.out}")

    if args.push:
        for motor, calib in enumerate(calibs):
            transport.write_line(protocol.cmd_motor_calib(
                motor, calib.thrust_dir[0], calib.thrust_dir[1], calib.yaw_sign,
                calib.kf_fwd, calib.kf_rev, calib.deadzone_us,
            ))
        print("Calibração enviada ao STM (RAM).")


if __name__ == "__main__":
    main()
