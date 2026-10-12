"""STM simulado: fala o mesmo protocolo do firmware v2, sem hardware.

Usa a planta, o cluster de atuadores e o controlador do Python (paridade por
construção) para:

- validar twin/wizard end-to-end sem bancada (``twin.py --sim``,
  ``wizard.py --sim``);
- servir de " STM de bolso" para desenvolver na estrada.

Diferenças documentadas vs. firmware real:

- o controlador usa a velocidade verdadeira da planta (o firmware estima
  v̂ = L·θ̇; equivalentes a pequenos ângulos);
- o atraso de atuação (120 ms) está no cluster (o hardware o tem de verdade);
- a fusão do firmware (acc+gyro, gate, compensação de yaw) é substituída
  pela leitura direta do estado — o que o twin exibe é o "sensor perfeito".
"""

from __future__ import annotations

import math
import queue
from dataclasses import replace
from typing import Optional

import numpy as np

from actuators import MotorCalib, ThrusterCluster
from controller import EsferaController, gains_for_inertia
from main import build_system
from twin import protocol

DT_STEP = 0.005          # 200 Hz, como a malha do firmware
TELEMETRY_PERIOD = 0.04  # 25 Hz
VISION_TIMEOUT = 0.5     # s, igual ao firmware


class SimSTM:
    def __init__(self, plant=None, cluster=None, controller=None):
        if plant is None:
            plant, cluster, controller = build_system("nonlinear")
        self.plant = plant
        self.cluster = cluster
        self.controller = controller
        # Paridade com o boot do firmware: loops DESLIGADOS ('y'/'p' ligam).
        controller.set_xy_enabled(False)
        controller.set_heading_enabled(False)

        self.out_queue: queue.Queue = queue.Queue()
        self.t_sim = 0.0
        self._tel_accum = 0.0
        self._wizard: Optional[dict] = None  # {motor, us, remaining}
        self._vision: Optional[dict] = None  # {bearing, D, t}
        self._buf = ""

    # ------------------------------------------------------------- tempo

    def advance(self, dt_real: float) -> None:
        """Avança a simulação por dt_real segundos (chamado pelo twin/timer)."""
        remaining = dt_real
        while remaining > 1e-9:
            step = min(DT_STEP, remaining)
            remaining -= step
            self._step(step)

    def _step(self, dt: float) -> None:
        state = self.plant.read_state()

        if self._wizard is not None:
            # pulso do wizard: loops já desligados, comando bruto por motor
            u = np.zeros(3)
            u[self._wizard["motor"]] = self._wizard["us"]
            act = self.cluster.command_raw(u, state.heading, dt)
            self._wizard["remaining"] -= dt
            if self._wizard["remaining"] <= 0:
                self._wizard = None
                self.cluster.reset()
        else:
            cmd = self.controller.update(state, dt)
            act = self.cluster.realize(cmd.force_world, cmd.yaw_us, state.heading, dt)

        self.plant.step(act.force_world, act.torque_heading, dt)
        self.t_sim += dt

        # visão obsoleta → controlador volta ao setpoint interno
        if self._vision is not None and self.t_sim - self._vision["t"] > VISION_TIMEOUT:
            self._vision = None
            self.controller.clear_vision()

        self._tel_accum += dt
        if self._tel_accum >= TELEMETRY_PERIOD:
            self._tel_accum -= TELEMETRY_PERIOD
            self._emit_telemetry()

    # ---------------------------------------------------------- telemetria

    def _emit_telemetry(self) -> None:
        state = self.plant.read_state()
        sx, sy = self.plant.tilt_angles(state)
        L = self.plant.rope.length
        # taxas de inclinação no mundo ≈ v/L (mesma aproximação do v̂ do firmware)
        wx = state.velocity[0] / L
        wy = state.velocity[1] / L
        vision_fresh = self._vision is not None
        pwm = self.cluster.params.pwm_neutral_us + self.cluster._u_lag
        flags = (
            (1 if self.controller.heading_enabled else 0)
            | (2 if self.controller.xy_enabled else 0)
            | 4  # calib_ok (sensor perfeito)
            | (8 if vision_fresh else 0)
            | (16 if self._wizard is not None else 0)
        )
        yaw_deg = math.degrees(state.heading) % 360.0
        line = (
            f"TEL,{int(self.t_sim * 1000):d},"
            f"{yaw_deg:.2f},"
            f"{self._vision['bearing'] if vision_fresh else 0.0:.2f},"
            f"{math.degrees(self.controller.heading_setpoint) % 360.0:.2f},"
            f"{math.degrees(state.heading_rate):.2f},"
            f"{math.degrees(sx):.3f},{math.degrees(sy):.3f},"
            f"{math.degrees(wx):.3f},{math.degrees(wy):.3f},"
            f"{pwm[0]:.0f},{pwm[1]:.0f},{pwm[2]:.0f},"
            f"{L:.2f},"
            f"{self._vision['D'] if vision_fresh else -1.0:.2f},"
            f"{flags:d}"
        )
        self.out_queue.put(line)

    # ------------------------------------------------------------ comandos

    def handle_command(self, piece: str) -> None:
        """Processa um comando (char único ou linha) — mesmas regras do firmware."""
        piece = piece.strip()
        if not piece:
            return

        if len(piece) == 1 and piece in "wsxadceymbpkzt0":
            self._char_command(piece)
            return

        c0 = piece[0]
        if c0 in "Ll" and len(piece) > 1:
            novo = float(piece[1:])
            if 0.5 <= novo <= 6.0:
                # L muda entre sessões: ajusta ao vivo (RATTLE re-projeta sozinho)
                object.__setattr__(self.plant.rope, "length", novo)
            return
        if c0 == "G" and " " in piece:
            idx_s, val_s = piece[1:].split(" ", 1)
            idx, val = int(idx_s), float(val_s)
            if 0 <= idx < len(protocol.GAIN_NAMES):
                g = replace(self.controller.gains, **{protocol.GAIN_NAMES[idx]: val})
                self.controller.set_gains(g)
            return
        if c0 == "R":
            for i, name in enumerate(protocol.GAIN_NAMES):
                self.out_queue.put(
                    f"GAIN,{i},{name},{getattr(self.controller.gains, name):.6g}"
                )
            return
        if c0 == "V" and " " in piece:
            b_s, d_s = piece[1:].split(" ", 1)
            self._vision = {
                "bearing": float(b_s),
                "D": float(d_s),
                "t": self.t_sim,
            }
            self.controller.set_pointing_error(float(b_s))
            return
        if c0 == "W" and piece.count(" ") == 2:
            m_s, us_s, ms_s = piece[1:].split(" ")
            m, us, ms = int(m_s), int(us_s), float(ms_s)
            if 0 <= m < 3 and abs(us) <= 250 and 0 < ms <= 3000:
                self.controller.set_heading_enabled(False)
                self.controller.set_xy_enabled(False)
                self._wizard = {"motor": m, "us": float(us), "remaining": ms / 1000.0}
            return
        if c0 == "M" and piece.count(" ") == 6:
            parts = piece[1:].split(" ")
            m = int(parts[0])
            tx, ty, ys, kf, kr, dz = (float(v) for v in parts[1:])
            norm = math.hypot(tx, ty)
            if 0 <= m < 3 and norm > 1e-6 and kf > 0 and kr > 0 and dz >= 0:
                self.cluster.apply_motor_calib(
                    m,
                    MotorCalib(
                        label=self.cluster.LABELS[m],
                        thrust_dir=(tx / norm, ty / norm),
                        yaw_sign=1.0 if ys >= 0 else -1.0,
                        kf_fwd=kf,
                        kf_rev=kr,
                        deadzone_us=dz,
                    ),
                )
            return
        # <número>: degrau relativo de yaw (firmware re-zera + aplica)
        try:
            delta = float(piece)
            current_deg = math.degrees(self.plant.read_state().heading)
            self.controller.set_heading(current_deg + delta)
            return
        except ValueError:
            pass

    def _char_command(self, c: str) -> None:
        if c in "yY":
            self.controller.set_heading_enabled(not self.controller.heading_enabled)
        elif c in "bBpP":
            self.controller.set_xy_enabled(not self.controller.xy_enabled)
        elif c in "kK":
            self.controller.set_heading_enabled(False)
            self.controller.set_xy_enabled(False)
            self._wizard = None
            self.cluster.reset()
        elif c in "zZ":
            for pid in (self.controller.pid_vx, self.controller.pid_vy,
                        self.controller.pid_heading_rate):
                pid.reset()
        elif c == "0":
            self.controller.set_heading(
                math.degrees(self.plant.read_state().heading)
            )
