"""Modo interativo: simulação ao vivo com controle pelo teclado.

Uso:

    python main.py --interactive            # estado inicial do cenário default
    python main.py --interactive --scenario heading

A física, o controlador e os atuadores rodam em tempo real (dt = 0,01 s,
ritmado pelo relógio de parede); o teclado aplica forças externas, liga e
desliga loops e ajusta ganhos enquanto a malha fechada reage.

Layout da janela:

    +------------------- faixa de status (t, XY, HEADING, PAUSADO) --------+
    |                              |  inclinação | velocidade              |
    |        cena 3D               |  -------------------------            |
    |   (HUD alinhado à esquerda)  |  heading    | PWM dos ESCs            |
    +------------------------------+---------------------------------------+
    | CONTROLES (teclas)           | GANHOS (valores vigentes + ajuste)    |
    +---------------------------------------------------------------------+

Teclas:

    (mesmas teclas do supervisório e do twin — tabela única em docs/COMANDOS.md)
    setas        força externa contínua (±magnitude) em X/Y do MUNDO, enquanto seguras
    , / .        torque externo contínuo ±0,05 N·m no heading
    p / y        liga/desliga o anti-balanço (X/Y) / o heading (as letras do STM)
    [ / ]        setpoint de heading −15° / +15°
    G            seleciona o próximo item ajustável (força ext., depois os ganhos)
    + / -        ajusta o item selecionado
    R            reset: esfera volta à condição inicial, integrais zeradas
    espaço       pausa / retoma
    Esc          sai

A lógica ao vivo fica em :class:`LiveSession` (sem dependência de UI —
testável headless); :class:`InteractiveWindow` monta a janela matplotlib em
cima dela.
"""

from __future__ import annotations

import math
import time
from dataclasses import replace
from typing import Callable, List, Tuple

import numpy as np

from actuators import ThrusterCluster
from controller import EsferaController
from scenarios import Scenario
from simulation import PendulumPlant

HELD_KEYS = {"left", "right", "up", "down", ",", "."}
TRAIL_SECONDS = 3.0


class LiveSession:
    """Estado + lógica de um passo do modo interativo (independente de UI)."""

    def __init__(
        self,
        plant: PendulumPlant,
        cluster: ThrusterCluster,
        controller: EsferaController,
        scenario: Scenario,
        dt: float = 0.01,
        history_seconds: float = 40.0,
    ):
        self.plant = plant
        self.cluster = cluster
        self.controller = controller
        self.dt = dt
        self.max_samples = int(history_seconds / dt)

        self.held_keys: set = set()
        self.paused = False
        self.t = 0.0
        self.external_force_magnitude = 15.0  # N, aplicada pelas setas
        self.external_torque_magnitude = 0.05  # N m, aplicada por , / .

        # Último ato de controle (para HUD/painel mesmo sem passo novo).
        self.last_pwm = np.full(cluster.N_THRUSTERS, cluster.params.pwm_neutral_us)
        self.last_thrusts = np.zeros(cluster.N_THRUSTERS)

        # Condição inicial do cenário (usada também pelo reset).
        ic = scenario.initial
        tilt = [math.radians(a) for a in ic.tilt_deg]
        self._initial_state = plant.initial_state(
            *tilt, ic.velocity, math.radians(ic.heading_deg)
        )

        # Itens ajustáveis por G/+/-: (rótulo, getter, setter, passo).
        g = controller.gains
        self.adjustables: List[Tuple[str, Callable, Callable, float]] = [
            (
                "força ext. (N)",
                lambda: self.external_force_magnitude,
                self._set_force_magnitude,
                2.0,
            ),
            ("vel_kp", lambda: self.controller.gains.vel_kp, self._gain_setter("vel_kp"), 0.1 * g.vel_kp),
            ("vel_ki", lambda: self.controller.gains.vel_ki, self._gain_setter("vel_ki"), 0.1 * g.vel_ki),
            ("vel_kd", lambda: self.controller.gains.vel_kd, self._gain_setter("vel_kd"), 0.1 * g.vel_kd),
            ("head_angle_kp", lambda: self.controller.gains.head_angle_kp, self._gain_setter("head_angle_kp"), 0.1 * g.head_angle_kp),
            ("head_rate_limit", lambda: self.controller.gains.head_rate_limit, self._gain_setter("head_rate_limit"), 0.1 * g.head_rate_limit),
            ("head_rate_kp_us", lambda: self.controller.gains.head_rate_kp_us, self._gain_setter("head_rate_kp_us"), 0.1 * g.head_rate_kp_us),
            ("head_rate_ki_us", lambda: self.controller.gains.head_rate_ki_us, self._gain_setter("head_rate_ki_us"), 0.1 * g.head_rate_ki_us),
        ]
        self.selected = 0

        # Histórico (séries do painel + posições para o rastro 3D).
        self.hist: dict = {k: [] for k in (
            "t", "tilt_x", "tilt_y", "vx", "vy", "heading", "psi_des",
            "px", "py", "pz", "pwm_a", "pwm_b", "pwm_c",
        )}
        self.reset()

    # ------------------------------------------------------------------ estado

    def reset(self) -> None:
        """Volta à condição inicial e zera as integrais dos PIDs."""
        self.plant.reset(self._initial_state)
        self.cluster.reset()
        self.controller.pid_vx.reset()
        self.controller.pid_vy.reset()
        self.controller.pid_heading_rate.reset()
        self.t = 0.0
        self.last_pwm = np.full(self.cluster.N_THRUSTERS, self.cluster.params.pwm_neutral_us)
        self.last_thrusts = np.zeros(self.cluster.N_THRUSTERS)
        for key in self.hist:
            self.hist[key].clear()

    def external_force(self) -> np.ndarray:
        """Força externa (mundo, N) das teclas seguras no momento."""
        keys = self.held_keys
        fx = self.external_force_magnitude * (
            ("right" in keys) - ("left" in keys)
        )
        fy = self.external_force_magnitude * (
            ("up" in keys) - ("down" in keys)
        )
        return np.array([fx, fy, 0.0])

    def external_torque(self) -> float:
        return self.external_torque_magnitude * (
            ("." in self.held_keys) - ("," in self.held_keys)
        )

    # ------------------------------------------------------------------- passo

    def step(self) -> None:
        """Um passo de simulação (malha fechada + forças externas)."""
        state = self.plant.read_state()
        cmd = self.controller.update(state, self.dt)
        act = self.cluster.realize(
            cmd.force_world, cmd.yaw_us, state.heading, self.dt
        )
        f_ext = self.external_force()
        self.plant.step(
            act.force_world + f_ext,
            act.torque_heading + self.external_torque(),
            self.dt,
        )

        self.last_thrusts = act.thrusts
        self.last_pwm = self.cluster.thrust_to_pwm_us(act.thrusts)
        tilt_x, tilt_y = self.plant.tilt_angles(state)
        h = self.hist
        h["t"].append(self.t)
        h["tilt_x"].append(tilt_x)
        h["tilt_y"].append(tilt_y)
        h["vx"].append(state.velocity[0])
        h["vy"].append(state.velocity[1])
        h["heading"].append(state.heading)
        h["psi_des"].append(cmd.heading_setpoint)
        h["px"].append(state.position[0])
        h["py"].append(state.position[1])
        h["pz"].append(state.position[2])
        for label, pwm in zip(self.cluster.LABELS, self.last_pwm):
            h[f"pwm_{label}"].append(float(pwm))
        if len(h["t"]) > self.max_samples:
            drop = len(h["t"]) - self.max_samples
            for key in h:
                del h[key][:drop]
        self.t += self.dt

    # ------------------------------------------------------------ ações de UI

    def toggle_xy(self) -> None:
        self.controller.set_xy_enabled(not self.controller.xy_enabled)

    def toggle_heading(self) -> None:
        self.controller.set_heading_enabled(not self.controller.heading_enabled)

    def nudge_heading_setpoint(self, delta_deg: float) -> None:
        new_deg = math.degrees(self.controller.heading_setpoint) + delta_deg
        self.controller.set_heading(new_deg)

    def cycle_selected(self) -> None:
        self.selected = (self.selected + 1) % len(self.adjustables)

    def adjust_selected(self, sign: float) -> None:
        _, getter, setter, step = self.adjustables[self.selected]
        setter(max(0.0, getter() + sign * step))

    def _set_force_magnitude(self, value: float) -> None:
        self.external_force_magnitude = min(value, 60.0)

    def _gain_setter(self, field_name: str) -> Callable:
        def setter(value: float) -> None:
            new_gains = replace(self.controller.gains, **{field_name: value})
            self.controller.set_gains(new_gains)

        return setter


# --------------------------------------------------------------------------
# Janela matplotlib ao vivo
# --------------------------------------------------------------------------


def _unbind_matplotlib_defaults(plt) -> None:
    """Remove atalhos default do matplotlib que conflitam com os nossos."""
    ours = {
        "left", "right", "up", "down", ",", ".", "p", "y", "[", "]",
        "g", "+", "-", "=", "r", " ", "escape", "k", "z", "0",
        "w", "s", "x", "a", "d", "c", "e", "t", "q",
    }
    for key in list(plt.rcParams):
        if key.startswith("keymap."):
            plt.rcParams[key] = [b for b in plt.rcParams[key] if b not in ours]


# Blocos de texto do rodapé: help de teclas (esquerda) e ganhos (direita).
HELP_TEXT = (
    "setas ... força externa X/Y (mundo)\n"
    ", / . ... torque externo no heading\n"
    "[ / ] ... ψ desejado −15° / +15°\n"
    "p / y ... liga/desliga XY / heading\n"
    "G ....... seleciona item   +/− ajusta\n"
    "R reset   espaço pausa   Esc sai"
)


class InteractiveWindow:
    """Janela do modo interativo (cena 3D + painel + teclado).

    Construída em cima de uma :class:`LiveSession`; pode ser instanciada com
    qualquer backend (inclusive Agg, para teste) — :meth:`on_timer` e o
    tratamento de teclado são métodos normais, fumigáveis headless.

    Cada bloco de texto (status, HUD, ajuda, ganhos) vive no seu próprio
    axes com coordenadas controladas — nada de fig.text solto sobreposto.
    """

    def __init__(self, session: LiveSession):
        import matplotlib.pyplot as plt

        _unbind_matplotlib_defaults(plt)

        self.session = session
        self.plant = session.plant
        self.cluster = session.cluster
        self.controller = session.controller

        dt = session.dt
        sphere_radius = self.plant.sphere.radius
        rope_length = self.plant.rope.length
        f_max = self.cluster.max_thrust
        self.trail_samples = int(TRAIL_SECONDS / dt)
        self.accumulator = 0.0
        self.last_wall = time.perf_counter()

        # ----------------------------------------------------------- layout
        fig = plt.figure("Esfera — interativo", figsize=(15, 8))
        gs = fig.add_gridspec(
            3,
            2,
            width_ratios=[1.22, 1.0],
            height_ratios=[0.05, 1.0, 0.20],
            left=0.02,
            right=0.985,
            top=0.97,
            bottom=0.02,
            hspace=0.45,
            wspace=0.18,
        )
        ax_status = fig.add_subplot(gs[0, :])
        ax_status.axis("off")
        ax3d = fig.add_subplot(gs[1, 0], projection="3d")
        self.ax3d = ax3d
        panel = gs[1, 1].subgridspec(2, 2)
        ax_tilt = fig.add_subplot(panel[0, 0])
        ax_v = fig.add_subplot(panel[0, 1])
        ax_psi = fig.add_subplot(panel[1, 0])
        ax_pwm = fig.add_subplot(panel[1, 1])
        ax_help = fig.add_subplot(gs[2, 0])
        ax_help.axis("off")
        ax_gains = fig.add_subplot(gs[2, 1])
        ax_gains.axis("off")
        self.fig = fig

        # ------------------------------------------------- faixa de status
        ax_status.set_xlim(0, 1)
        ax_status.set_ylim(0, 1)
        self.status_time = ax_status.text(
            0.0, 0.5, "", fontsize=11, family="monospace", va="center", ha="left"
        )
        self.status_xy = ax_status.text(
            0.32, 0.5, "", fontsize=11, fontweight="bold", va="center", ha="left"
        )
        self.status_head = ax_status.text(
            0.60, 0.5, "", fontsize=11, fontweight="bold", va="center", ha="left"
        )
        self.status_pause = ax_status.text(
            0.88, 0.5, "", fontsize=11, fontweight="bold", color="darkorange",
            va="center", ha="left",
        )

        # ----------------------------------------------------------- cena 3D
        lim = rope_length * 0.55
        ax3d.set_xlim(-lim, lim)
        ax3d.set_ylim(-lim, lim)
        ax3d.set_zlim(-rope_length * 1.12, rope_length * 0.12)
        ax3d.set_box_aspect((1, 1, 1.3))
        ax3d.set_xlabel("x (m)")
        ax3d.set_ylabel("y (m)")
        ax3d.set_zlabel("z (m)")
        ax3d.view_init(elev=12, azim=-60)
        ax3d.set_title("cena 3D (arraste o mouse para girar)", fontsize=9)
        ax3d.scatter([0], [0], [0], color="black", marker="*", s=90)
        ax3d.plot([0, 0], [0, 0], [0, -rope_length], color="gray", ls=":", lw=1)

        self.rope_line, = ax3d.plot([], [], [], color="saddlebrown", lw=2)
        self.trail_line, = ax3d.plot([], [], [], color="steelblue", lw=1, alpha=0.6)
        theta = np.linspace(0.0, 2.0 * math.pi, 61)
        self.circles_body = [
            np.stack([np.cos(theta), np.sin(theta), np.zeros_like(theta)]),
            np.stack([np.cos(theta), np.zeros_like(theta), np.sin(theta)]),
            np.stack([np.zeros_like(theta), np.cos(theta), np.sin(theta)]),
        ]
        self.circle_lines = [
            ax3d.plot([], [], [], color="darkorange", lw=1.2, alpha=0.85)[0]
            for _ in self.circles_body
        ]
        self.sphere_radius = sphere_radius
        self.f_max = f_max
        self.thrust_lines = [
            ax3d.plot([], [], [], lw=3.0)[0] for _ in range(self.cluster.N_THRUSTERS)
        ]
        self.ext_force_line, = ax3d.plot([], [], [], color="green", lw=3.5)
        self.hud = ax3d.text2D(
            0.02, 0.96, "", transform=ax3d.transAxes, family="monospace",
            fontsize=8.5, va="top", linespacing=1.5,
            bbox=dict(facecolor="white", alpha=0.8, edgecolor="none", pad=3),
        )

        # ------------------------------------------------------------ painel
        ax_tilt.set_title("inclinação", fontsize=9)
        ax_v.set_title("velocidade", fontsize=9)
        ax_psi.set_title("heading", fontsize=9)
        ax_pwm.set_title("PWM dos ESCs", fontsize=9)
        ax_tilt.set_ylabel("θ (°)")
        ax_v.set_ylabel("v (m/s)")
        ax_psi.set_ylabel("ψ (°)")
        ax_pwm.set_ylabel("µs")
        ax_psi.set_xlabel("t (s)")
        ax_pwm.set_xlabel("t (s)")
        ax_pwm.set_ylim(990, 2010)
        ax_pwm.axhline(1500, color="gray", ls=":", lw=0.8)

        self.line_tilt_x, = ax_tilt.plot([], [], label="θx")
        self.line_tilt_y, = ax_tilt.plot([], [], label="θy", color="tab:green")
        self.line_vx, = ax_v.plot([], [], label="vx")
        self.line_vy, = ax_v.plot([], [], label="vy", color="tab:orange")
        self.line_psi, = ax_psi.plot([], [])
        self.line_psi_des, = ax_psi.plot([], [], "k:", label="desejado")
        self.pwm_keys = [f"pwm_{label}" for label in self.cluster.LABELS]
        self.pwm_lines = [
            ax_pwm.plot([], [], label=label)[0] for label in self.cluster.LABELS
        ]
        self.panel_axes = (ax_tilt, ax_v, ax_psi, ax_pwm)
        self.ax_pwm = ax_pwm
        for a in self.panel_axes:
            a.grid(True, alpha=0.3)
            a.legend(fontsize=7, loc="upper left")

        # ------------------------------------------------- rodapé: 2 blocos
        ax_help.text(
            0.0, 1.0, "CONTROLES", fontsize=9, fontweight="bold", va="top"
        )
        ax_help.text(
            0.0, 0.72, HELP_TEXT, fontsize=8, family="monospace", va="top",
            linespacing=1.6,
        )
        self.gains_text = ax_gains.text(
            0.0, 1.0, "", fontsize=8, family="monospace", va="top", linespacing=1.6
        )

        fig.canvas.mpl_connect("key_press_event", self.on_press)
        fig.canvas.mpl_connect("key_release_event", self.on_release)

    # -------------------------------------------------------------- teclado
    def on_press(self, event) -> None:
        session = self.session
        key = "+" if event.key == "=" else event.key
        if key in HELD_KEYS:
            session.held_keys.add(key)
        if key == "p":
            session.toggle_xy()
        elif key == "y":
            session.toggle_heading()
        elif key == "[":
            session.nudge_heading_setpoint(-15.0)
        elif key == "]":
            session.nudge_heading_setpoint(+15.0)
        elif key == "g":
            session.cycle_selected()
        elif key == "+":
            session.adjust_selected(+1.0)
        elif key == "-":
            session.adjust_selected(-1.0)
        elif key == "r":
            session.reset()
        elif key == " ":
            session.paused = not session.paused
        elif key == "escape":
            import matplotlib.pyplot as plt

            plt.close(self.fig)

    def on_release(self, event) -> None:
        self.session.held_keys.discard(event.key)

    # ----------------------------------------------------------- loop ao vivo
    def on_timer(self) -> None:
        session = self.session
        now = time.perf_counter()
        self.accumulator += now - self.last_wall
        self.last_wall = now
        if not session.paused:
            # Avança o tempo simulado em passos de dt, no ritmo do relógio
            # real (cap contra espiral de morte se o render atrasar).
            n_steps = min(int(self.accumulator / session.dt), 20)
            self.accumulator -= n_steps * session.dt
            for _ in range(n_steps):
                session.step()
        self.redraw()
        # Sem isso a tela só atualiza quando algo mais dispara um redraw
        # (ex.: arrastar o mouse na cena 3D) — o timer avança a simulação,
        # mas o canvas não sabe que precisa repintar.
        self.fig.canvas.draw_idle()

    def redraw(self) -> None:
        session = self.session
        state = self.plant.read_state()
        center = np.asarray(state.position)
        heading = state.heading
        t = session.t

        # ------------------------------------------------ faixa de status
        self.status_time.set_text(f"t = {t:7.2f} s")
        if self.controller.xy_enabled:
            self.status_xy.set_text("XY: LIGADO")
            self.status_xy.set_color("green")
        else:
            self.status_xy.set_text("XY: DESLIGADO")
            self.status_xy.set_color("red")
        if self.controller.heading_enabled:
            self.status_head.set_text("HEADING: LIGADO")
            self.status_head.set_color("green")
        else:
            self.status_head.set_text("HEADING: DESLIGADO")
            self.status_head.set_color("red")
        self.status_pause.set_text("PAUSADO" if session.paused else "")

        # ---------------------------------------------------------- cena 3D
        self.rope_line.set_data_3d(
            [0, center[0]], [0, center[1]], [0, center[2]]
        )
        h = session.hist
        n_trail = min(len(h["t"]), self.trail_samples)
        if n_trail:
            self.trail_line.set_data_3d(
                h["px"][-n_trail:], h["py"][-n_trail:], h["pz"][-n_trail:]
            )

        rot = self.cluster.rotation_z(heading)
        for circle, line in zip(self.circles_body, self.circle_lines):
            pts = center[:, None] + self.sphere_radius * (rot @ circle)
            line.set_data_3d(pts[0], pts[1], pts[2])

        thrusts = session.last_thrusts
        mounts, tangents = self.cluster.geometry_world(heading, center)
        for j in range(self.cluster.N_THRUSTERS):
            f = thrusts[j]
            direction = tangents[:, j] * (1.0 if f >= 0 else -1.0)
            length = self.sphere_radius * (0.4 + 1.2 * abs(f) / self.f_max)
            tip = mounts[:, j] + length * direction
            self.thrust_lines[j].set_data_3d(
                [mounts[0, j], tip[0]], [mounts[1, j], tip[1]], [mounts[2, j], tip[2]]
            )
            self.thrust_lines[j].set_color("tab:red" if f >= 0 else "tab:blue")

        # Seta verde: força externa aplicada pelas teclas.
        f_ext = session.external_force()
        f_norm = float(np.linalg.norm(f_ext[:2]))
        if f_norm > 1e-9:
            tip = center + (0.04 * f_norm) * f_ext / np.linalg.norm(f_ext)
            self.ext_force_line.set_data_3d(
                [center[0], tip[0]], [center[1], tip[1]], [center[2], tip[2]]
            )
        else:
            self.ext_force_line.set_data_3d([], [], [])

        # HUD da cena: colunas alinhadas.
        tilt_x, tilt_y = self.plant.tilt_angles(state)
        tilt_deg = math.degrees(math.hypot(tilt_x, tilt_y))
        speed = math.sqrt(float(state.velocity @ state.velocity))
        pwm_txt = "  ".join(
            f"{label} {p:5.0f}"
            for label, p in zip(self.cluster.LABELS, session.last_pwm)
        )
        self.hud.set_text(
            f"inclinação  {tilt_deg:6.2f}°\n"
            f"|v|         {speed:6.3f} m/s\n"
            f"ψ           {math.degrees(heading):6.1f}° (des. {math.degrees(self.controller.heading_setpoint):.1f}°)\n"
            f"força ext   ({f_ext[0]:+5.1f}, {f_ext[1]:+5.1f}) N\n"
            f"PWM (µs)    {pwm_txt}"
        )

        # Painel: janela móvel do histórico.
        if h["t"]:
            t_arr = np.asarray(h["t"])
            self.line_tilt_x.set_data(t_arr, np.degrees(np.asarray(h["tilt_x"])))
            self.line_tilt_y.set_data(t_arr, np.degrees(np.asarray(h["tilt_y"])))
            self.line_vx.set_data(t_arr, np.asarray(h["vx"]))
            self.line_vy.set_data(t_arr, np.asarray(h["vy"]))
            self.line_psi.set_data(t_arr, np.degrees(np.asarray(h["heading"])))
            self.line_psi_des.set_data(t_arr, np.degrees(np.asarray(h["psi_des"])))
            for line, key in zip(self.pwm_lines, self.pwm_keys):
                line.set_data(t_arr, np.asarray(h[key]))
            t_lo = max(0.0, t - session.max_samples * session.dt)
            for a in self.panel_axes:
                a.set_xlim(t_lo, max(t, t_lo + 1.0))
                if a is not self.ax_pwm:
                    a.relim()
                    a.autoscale_view()

        # Rodapé direito: ganhos vigentes + item em ajuste.
        g = self.controller.gains
        label, getter, _, _ = session.adjustables[session.selected]
        self.gains_text.set_text(
            "GANHOS\n"
            f"  XY:  kp {g.vel_kp:6.2f}   ki {g.vel_ki:5.2f}   kd {g.vel_kd:5.2f}\n"
            f"  HEAD: ang {g.head_angle_kp:5.2f}   lim {g.head_rate_limit:5.2f}"
            f"   kp_us {g.head_rate_kp_us:6.1f}   ki_us {g.head_rate_ki_us:6.1f}\n"
            f"AJUSTANDO → {label} = {getter():.3f}    (G troca · +/− ajusta)"
        )

    def show(self) -> None:
        import matplotlib.pyplot as plt

        timer = self.fig.canvas.new_timer(interval=30)
        timer.add_callback(self.on_timer)
        timer.start()
        plt.show()
        timer.stop()


def run(
    plant: PendulumPlant,
    cluster: ThrusterCluster,
    controller: EsferaController,
    scenario: Scenario,
) -> None:
    """Abre a janela interativa (bloqueia até fechar)."""
    import matplotlib

    if matplotlib.get_backend().lower() == "agg":
        raise RuntimeError(
            "O modo interativo precisa de uma janela gráfica (backend agg/headless detectado)."
        )

    print("Modo interativo — teclas (também na janela, bloco CONTROLES):")
    print("  setas: força externa X/Y · , .: torque no heading · [ ]: ψ desejado ±15°")
    print("  p/y: liga/desliga anti-balanço/heading · G: seleciona item · +/−: ajusta valor")
    print("  R: reset · espaço: pausa · Esc: sai   (tabela única: docs/COMANDOS.md)")

    session = LiveSession(plant, cluster, controller, scenario)
    InteractiveWindow(session).show()
