"""Visualização: animação 3D do pêndulo + painel de séries temporais.

A trajetória já vem pré-computada no ``SimulationLog``; a animação apenas
reproduz (suave, ~30 fps, sem custo de simulação no frame).

Janela 1 (3D): pino, corda, referência vertical, esfera (3 círculos máximos
que giram com o heading), 3 setas de empuxo (vermelho = sentido positivo,
azul = negativo, comprimento proporcional ao empuxo), rastro e HUD.

Janela 2 (2×2): inclinações, velocidades, heading vs desejado, empuxos —
com cursor sincronizado com a animação.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

import matplotlib

from actuators import ThrusterCluster
from main import SimulationLog


def run(
    log: SimulationLog,
    cluster: ThrusterCluster,
    sphere_radius: float,
    rope_length: float,
    title: str = "Esfera",
    show: bool = True,
    save_path: Optional[str] = None,
):
    if save_path and not show:
        # Exportação sem janela: backend não-interativo.
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    dt = float(log.t[1] - log.t[0])
    stride = max(1, round((1.0 / 30.0) / dt))
    frame_idx = np.arange(0, len(log.t), stride)
    interval_ms = stride * dt * 1000.0
    f_max = cluster.max_thrust

    # ------------------------------------------------------------------ cena 3D
    fig = plt.figure(title, figsize=(11, 7.5))
    ax = fig.add_subplot(111, projection="3d")

    lim = rope_length * 0.55
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-rope_length * 1.12, rope_length * 0.12)
    ax.set_box_aspect((1, 1, 1.3))
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.view_init(elev=12, azim=-60)

    # Pino e referência vertical (posição de repouso).
    ax.scatter([0], [0], [0], color="black", marker="*", s=90)
    ax.plot([0, 0], [0, 0], [0, -rope_length], color="gray", linestyle=":", linewidth=1)

    rope_line, = ax.plot([], [], [], color="saddlebrown", linewidth=2)
    trail_line, = ax.plot([], [], [], color="steelblue", linewidth=1, alpha=0.6)

    # Esfera: 3 círculos máximos no referencial do corpo (giram com o heading).
    theta = np.linspace(0.0, 2.0 * math.pi, 61)
    circles_body = [
        np.stack([np.cos(theta), np.sin(theta), np.zeros_like(theta)]),
        np.stack([np.cos(theta), np.zeros_like(theta), np.sin(theta)]),
        np.stack([np.zeros_like(theta), np.cos(theta), np.sin(theta)]),
    ]
    circle_lines = [
        ax.plot([], [], [], color="darkorange", linewidth=1.2, alpha=0.85)[0]
        for _ in circles_body
    ]

    thrust_lines = [
        ax.plot([], [], [], linewidth=3.0)[0] for _ in range(cluster.N_THRUSTERS)
    ]

    hud = ax.text2D(
        0.02,
        0.97,
        "",
        transform=ax.transAxes,
        family="monospace",
        fontsize=9,
        va="top",
        bbox=dict(facecolor="white", alpha=0.75, edgecolor="none"),
    )

    # ------------------------------------------------------- painel 2x2 (t, x)
    fig2, axes2 = plt.subplots(2, 2, figsize=(11, 6), num=f"{title} — séries temporais")
    (ax_tilt, ax_v), (ax_psi, ax_f) = axes2

    t_deg = np.degrees
    ax_tilt.plot(log.t, t_deg(log.tilt_x), label="θx")
    ax_tilt.plot(log.t, t_deg(log.tilt_y), label="θy")
    ax_tilt.set_ylabel("inclinação (°)")
    ax_tilt.legend(fontsize=8)
    ax_v.plot(log.t, log.vx, label="vx")
    ax_v.plot(log.t, log.vy, label="vy")
    ax_v.set_ylabel("v (m/s)")
    ax_v.legend(fontsize=8)
    ax_psi.plot(log.t, t_deg(log.heading), label="ψ")
    ax_psi.plot(log.t, t_deg(log.psi_des), "k:", label="ψ desejado")
    ax_psi.set_ylabel("heading (°)")
    ax_psi.legend(fontsize=8)
    for i in range(cluster.N_THRUSTERS):
        ax_f.plot(log.t, log.pwm[:, i], label=f"ESC {cluster.LABELS[i]}")
    ax_f.set_ylabel("PWM (µs)")
    ax_f.set_xlabel("t (s)")
    ax_f.axhline(1500, color="gray", ls=":", lw=0.8)
    ax_f.legend(fontsize=8)
    for a in axes2.flat:
        a.grid(True, alpha=0.3)
    fig2.tight_layout()

    cursors = [a.axvline(0, color="red", alpha=0.5) for a in axes2.flat]

    # ------------------------------------------------------------------ update
    def update(frame: int):
        i = frame_idx[frame]
        center = np.array([log.x[i], log.y[i], log.z[i]])
        heading = log.heading[i]

        rope_line.set_data_3d([0, center[0]], [0, center[1]], [0, center[2]])
        trail_line.set_data_3d(log.x[: i + 1], log.y[: i + 1], log.z[: i + 1])

        rot = cluster.rotation_z(heading)
        for circle, line in zip(circles_body, circle_lines):
            pts = center[:, None] + sphere_radius * (rot @ circle)
            line.set_data_3d(pts[0], pts[1], pts[2])

        mounts, tangents = cluster.geometry_world(heading, center)
        thrusts = log.thrusts[i]
        for j in range(cluster.N_THRUSTERS):
            f = thrusts[j]
            direction = tangents[:, j] * (1.0 if f >= 0 else -1.0)
            length = sphere_radius * (0.4 + 1.2 * abs(f) / f_max)
            tip = mounts[:, j] + length * direction
            thrust_lines[j].set_data_3d(
                [mounts[0, j], tip[0]], [mounts[1, j], tip[1]], [mounts[2, j], tip[2]]
            )
            thrust_lines[j].set_color("tab:red" if f >= 0 else "tab:blue")

        tilt = log.tilt_mag_deg[i]
        pwm_txt = "  ".join(
            f"{label}{p:5.0f}"
            for label, p in zip(cluster.LABELS, log.pwm[i])
        )
        hud.set_text(
            f"t = {log.t[i]:6.2f} s\n"
            f"inclinação: {tilt:5.2f}°   |v| = {log.speed[i]:5.3f} m/s\n"
            f"ψ = {math.degrees(heading):7.2f}°  desejado {math.degrees(log.psi_des[i]):7.2f}°\n"
            f"PWM (µs): {pwm_txt}"
        )
        for cursor in cursors:
            cursor.set_xdata([log.t[i], log.t[i]])

        artists = [rope_line, trail_line, hud] + circle_lines + thrust_lines + cursors
        return artists

    anim = FuncAnimation(
        fig,
        update,
        frames=len(frame_idx),
        interval=interval_ms,
        blit=False,
        cache_frame_data=False,
        repeat=True,
    )

    if save_path:
        fps = max(1, round(1000.0 / interval_ms))
        anim.save(save_path, writer=PillowWriter(fps=fps), dpi=90)
        print(f"Animação salva em {save_path}")

    if show:
        plt.show()
    return anim
