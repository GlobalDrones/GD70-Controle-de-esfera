"""Digital Twin: simulação + esfera REAL em tempo real na mesma tela.

Conecta ao STM (serial USB direta, agente TCP na Rasp, ou STM simulado) e
sobreponhe à janela interativa:

- **esfera fantasma** (cinza, tracejada): estado REAL da telemetria
  (inclinação sx/sy, heading, empuxos dos PWMs reais) e, quando a visão
  enviar, a linha de transmissão a distância D abaixo da esfera;
- **curvas real (tracejado) vs simulada (sólido)**: inclinação, velocidade,
  heading e PWM dos ESCs;
- **comandos vão aos DOIS lados**: teclas 1/2 (loops), Z/X (heading) e
  +/− (ganhos, com push via G) atuam na simulação E no hardware; K = parada
  geral em ambos; setas/Q/E continuam só na simulação (forças externas
  preditivas).

Uso (o pacote ``twin/`` é executável com ``-m``):

    python -m twin --sim                    # STM simulado (sem hardware)
    python -m twin --serial COM7            # STM no USB do PC
    python -m twin --tcp 192.168.0.50       # agente na Rasp (porta 9999)
    python -m twin --tcp 192.168.0.50:9999
"""

from __future__ import annotations

import math
import time
from typing import Optional

import numpy as np

from interactive import InteractiveWindow, LiveSession, _unbind_matplotlib_defaults
from scenarios import SCENARIOS
from twin import protocol
from twin.protocol import TelemetryFrame
from twin.transport import SimTransport, Transport

REAL_MAX_SAMPLES = 1200  # ~48 s @ 25 Hz


class TwinWindow(InteractiveWindow):
    """Janela interativa + camada do hardware real (fantasma e comandos)."""

    def __init__(self, session: LiveSession, transport: Transport):
        super().__init__(session)
        self.transport = transport
        self.connected = True
        self.last_frame: Optional[TelemetryFrame] = None
        self._t0_ms: Optional[float] = None
        self._twin_last = time.perf_counter()
        self.real: dict = {k: [] for k in (
            "t", "sx", "sy", "yaw", "vx", "vy", "pwm_a", "pwm_b", "pwm_c",
        )}

        self._build_ghost_artists()
        self._build_real_panel_lines()

        self.real_hud = self.ax3d.text2D(
            0.02, 0.60, "", transform=self.ax3d.transAxes,
            family="monospace", fontsize=8, va="top", color="dimgray",
            bbox=dict(facecolor="white", alpha=0.75, edgecolor="none", pad=2),
        )
        self.gains_text.set_text(
            self.gains_text.get_text()
            + "\nTWIN: 1/2, Z/X, K e ganhos (+/−) vão TAMBÉM ao hardware"
        )

    # ------------------------------------------------------------ artistas

    def _build_ghost_artists(self) -> None:
        ax = self.ax3d
        self.ghost_rope, = ax.plot(
            [], [], [], color="dimgray", lw=1.5, ls="--", alpha=0.8
        )
        self.ghost_circles = [
            ax.plot([], [], [], color="gray", lw=1.0, ls="--", alpha=0.6)[0]
            for _ in self.circles_body
        ]
        self.ghost_thrust = [
            ax.plot([], [], [], lw=2.5, alpha=0.6)[0]
            for _ in range(self.cluster.N_THRUSTERS)
        ]
        self.d_line, = ax.plot(
            [], [], [], color="saddlebrown", lw=4, alpha=0.85
        )

    def _build_real_panel_lines(self) -> None:
        dash = dict(ls="--", color="black", lw=1.0, alpha=0.8)
        self.real_tilt_x, = self.panel_axes[0].plot([], [], **dash)
        self.real_tilt_y, = self.panel_axes[0].plot([], [], **dash)
        self.real_vx, = self.panel_axes[1].plot([], [], **dash)
        self.real_vy, = self.panel_axes[1].plot([], [], **dash)
        self.real_yaw, = self.panel_axes[2].plot([], [], **dash)
        self.real_pwms = [
            self.panel_axes[3].plot([], [], **dash)[0]
            for _ in range(self.cluster.N_THRUSTERS)
        ]

    # ---------------------------------------------------------- transporte

    def _send(self, line: str) -> None:
        if not self.connected:
            return
        try:
            self.transport.write_line(line)
        except (OSError, ConnectionError):
            self.connected = False

    def _drain_transport(self) -> None:
        if not self.connected:
            return
        try:
            for _ in range(200):
                line = self.transport.read_line()
                if line is None:
                    return
                frame = protocol.parse_line(line)
                if isinstance(frame, TelemetryFrame):
                    self._record(frame)
        except (OSError, ConnectionError):
            self.connected = False

    def _record(self, f: TelemetryFrame) -> None:
        if self._t0_ms is None:
            self._t0_ms = f.t_ms
        self.last_frame = f
        r = self.real
        r["t"].append((f.t_ms - self._t0_ms) / 1000.0)
        r["sx"].append(f.sx_deg)
        r["sy"].append(f.sy_deg)
        yaw_wrapped = ((f.yaw_deg + 180.0) % 360.0) - 180.0
        r["yaw"].append(yaw_wrapped)
        r["vx"].append(f.L_m * math.radians(f.wx_dps))  # v̂ = L·θ̇ (mesma unidade do sim)
        r["vy"].append(f.L_m * math.radians(f.wy_dps))
        r["pwm_a"].append(f.pwm[0])
        r["pwm_b"].append(f.pwm[1])
        r["pwm_c"].append(f.pwm[2])
        drop = len(r["t"]) - REAL_MAX_SAMPLES
        if drop > 0:
            for key in r:
                del r[key][:drop]

    # ------------------------------------------------------------ teclado

    def on_press(self, event) -> None:
        key = "+" if event.key == "=" else event.key
        if key == "1":
            self._send("p")  # toggle do anti-balanço no firmware
        elif key == "2":
            self._send("y")  # toggle do yaw no firmware
        elif key == "z":
            self._send(protocol.cmd_yaw_step(-15.0))
        elif key == "x":
            self._send(protocol.cmd_yaw_step(15.0))
        elif key == "k":
            self._send("k")
            # sim: parada geral equivalente
            self.session.controller.set_xy_enabled(False)
            self.session.controller.set_heading_enabled(False)
            self.session.cluster.reset()
            return
        super().on_press(event)
        if key in ("+", "-") and self.session.selected >= 1:
            idx = self.session.selected - 1  # ajustáveis 1..7 ↔ ganhos 0..7
            _, getter, _, _ = self.session.adjustables[self.session.selected]
            self._send(protocol.cmd_gain(idx, getter()))

    # ----------------------------------------------------------- loop twin

    def on_timer(self) -> None:
        now = time.perf_counter()
        dt_wall = min(now - self._twin_last, 0.15)
        self._twin_last = now

        simstm = getattr(self.transport, "sim", None)
        if simstm is not None and not self.session.paused:
            simstm.advance(dt_wall)

        self._drain_transport()
        super().on_timer()
        self._update_real_overlays()

    # ------------------------------------------------------------ overlays

    def _update_real_overlays(self) -> None:
        f = self.last_frame
        if f is None:
            self.real_hud.set_text("REAL: aguardando telemetria…")
            return

        # fantasma: posição na esfera de raio L a partir das inclinações
        sx = math.radians(f.sx_deg)
        sy = math.radians(f.sy_deg)
        cz = -math.sqrt(max(0.0, 1.0 - math.sin(sx) ** 2 - math.sin(sy) ** 2))
        center = f.L_m * np.array([math.sin(sx), math.sin(sy), cz])
        yaw = math.radians(f.yaw_deg)

        self.ghost_rope.set_data_3d(
            [0, center[0]], [0, center[1]], [0, center[2]]
        )
        rot = self.cluster.rotation_z(yaw)
        for circle, line in zip(self.circles_body, self.ghost_circles):
            pts = center[:, None] + self.sphere_radius * (rot @ circle)
            line.set_data_3d(pts[0], pts[1], pts[2])

        # empuxos reais: PWM medido → força (modelo do cluster do twin)
        u_real = np.array(f.pwm) - self.cluster.params.pwm_neutral_us
        thrusts = self.cluster.force_from_us(u_real)
        mounts, tangents = self.cluster.geometry_world(yaw, center)
        f_max = self.cluster.max_thrust
        for j in range(self.cluster.N_THRUSTERS):
            fj = thrusts[j]
            direction = tangents[:, j] * (1.0 if fj >= 0 else -1.0)
            length = self.sphere_radius * (0.4 + 1.2 * abs(fj) / f_max)
            tip = mounts[:, j] + length * direction
            self.ghost_thrust[j].set_data_3d(
                [mounts[0, j], tip[0]], [mounts[1, j], tip[1]], [mounts[2, j], tip[2]]
            )
            self.ghost_thrust[j].set_color("tab:red" if fj >= 0 else "tab:blue")

        # linha de transmissão a distância D abaixo, orientada pelo heading+bearing
        if f.D_m is not None and f.D_m >= 0:
            ang = math.radians(f.yaw_deg + f.bearing_deg)
            direction = np.array([math.cos(ang), math.sin(ang), 0.0])
            below = center + np.array([0.0, 0.0, -f.D_m])
            p1 = below - 0.75 * direction
            p2 = below + 0.75 * direction
            self.d_line.set_data_3d(
                [p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]]
            )
        else:
            self.d_line.set_data_3d([], [], [])

        # curvas reais (tracejadas) nos painéis
        r = self.real
        t_arr = np.asarray(r["t"])
        if t_arr.size:
            self.real_tilt_x.set_data(t_arr, np.asarray(r["sx"]))
            self.real_tilt_y.set_data(t_arr, np.asarray(r["sy"]))
            self.real_vx.set_data(t_arr, np.asarray(r["vx"]))
            self.real_vy.set_data(t_arr, np.asarray(r["vy"]))
            self.real_yaw.set_data(t_arr, np.asarray(r["yaw"]))
            for line, key in zip(self.real_pwms, ("pwm_a", "pwm_b", "pwm_c")):
                line.set_data(t_arr, np.asarray(r[key]))
            # eixo do tempo acompanha a janela do sim
            t_hi = max(self.session.t, float(t_arr[-1]))
            for a in self.panel_axes:
                a.set_xlim(max(0.0, t_hi - 40.0), max(t_hi, 1.0))

        # HUD do lado real
        conn = "CONECTADO" if self.connected else "DESCONECTADO"
        d_txt = f"{f.D_m:.2f} m" if f.D_m is not None and f.D_m >= 0 else "—"
        self.real_hud.set_text(
            f"REAL [{conn}] via {self.transport.description}\n"
            f"inclinação {f.tilt_mag_deg:6.2f}°   rate_z {f.rate_z_dps:6.2f}°/s\n"
            f"loops: XY {'ON' if f.swing_on else 'OFF'} · "
            f"HEAD {'ON' if f.yaw_on else 'OFF'} · L {f.L_m:.2f} m · D {d_txt}"
        )


def build_transport(args) -> Transport:
    if args.sim:
        from twin.simstm import SimSTM

        return SimTransport(SimSTM())
    if args.serial:
        from twin.transport import SerialTransport

        return SerialTransport(args.serial, args.baud)
    if args.tcp:
        from twin.transport import TcpTransport

        host, _, port = args.tcp.partition(":")
        return TcpTransport(host, int(port or 9999))
    raise SystemExit("escolha --sim, --serial PORTA ou --tcp HOST[:PORTA]")


def main(argv=None) -> None:
    import argparse

    import matplotlib

    if matplotlib.get_backend().lower() == "agg":
        raise SystemExit("O twin precisa de uma janela gráfica (backend headless detectado).")
    import matplotlib.pyplot as plt

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sim", action="store_true", help="STM simulado (sem hardware)")
    parser.add_argument("--serial", default=None, help="porta serial do STM no PC (ex.: COM7)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--tcp", default=None, metavar="HOST[:PORTA]", help="agente na Rasp")
    parser.add_argument("--scenario", default="default", choices=sorted(SCENARIOS))
    args = parser.parse_args(argv)

    _unbind_matplotlib_defaults(plt)
    transport = build_transport(args)

    from main import build_system

    plant, cluster, controller = build_system("nonlinear")
    session = LiveSession(plant, cluster, controller, SCENARIOS[args.scenario])

    print(f"Twin conectado via {transport.description}.")
    print("Teclas: como o modo interativo; 1/2, Z/X, K e ganhos (+/−) também comandam o hardware.")

    win = TwinWindow(session, transport)
    win.show()


if __name__ == "__main__":
    main()
