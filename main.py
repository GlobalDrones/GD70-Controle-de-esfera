"""Execução da simulação Esfera: roda um cenário, registra o log e visualiza.

Uso:

    python main.py                          # cenário padrão, planta não-linear + animação
    python main.py --scenario heading       # outro cenário
    python main.py --plant linear           # planta linearizada
    python main.py --compare                # linear vs não-linear (gráficos + métricas)
    python main.py --no-anim                # só console (rápido, para sintonia)
    python main.py --save animacao.gif      # exporta a animação em GIF
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, fields
from typing import Dict

import numpy as np

from actuators import ActuatorParams, ThrusterCluster
from controller import ControllerGains, EsferaController, gains_for_inertia
from scenarios import SCENARIOS, Scenario
from simulation import (
    LinearizedPendulumPlant,
    RopeParams,
    SphereParams,
    SphericalPendulumPlant,
    wrap_to_pi,
)

PLANTS = {"nonlinear": SphericalPendulumPlant, "linear": LinearizedPendulumPlant}

VELOCITY_TOLERANCE = 0.05  # m/s, critério de acomodação de velocidade
HEADING_TOLERANCE_DEG = 2.0  # graus, critério de acomodação de heading


@dataclass
class SimulationLog:
    """Séries temporais de uma execução (todas com o mesmo comprimento)."""

    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    z: np.ndarray
    vx: np.ndarray
    vy: np.ndarray
    vz: np.ndarray
    heading: np.ndarray
    heading_rate: np.ndarray
    tilt_x: np.ndarray
    tilt_y: np.ndarray
    psi_des: np.ndarray  # setpoint de heading vigente (rad)
    cmd_fx: np.ndarray  # força desejada pelo controlador (N)
    cmd_fy: np.ndarray
    cmd_torque: np.ndarray  # torque desejado (N m)
    act_fx: np.ndarray  # força realizada pelos atuadores (N)
    act_fy: np.ndarray
    act_torque: np.ndarray
    thrusts: np.ndarray  # (n, 3), empuxo de cada hélice (N)
    pwm: np.ndarray  # (n, 3), PWM enviado aos ESCs a/b/c (us)
    ext_fx: np.ndarray  # força externa aplicada à planta (N)
    ext_fy: np.ndarray

    @property
    def speed(self) -> np.ndarray:
        return np.sqrt(self.vx**2 + self.vy**2 + self.vz**2)

    @property
    def tilt_mag_deg(self) -> np.ndarray:
        return np.degrees(np.sqrt(self.tilt_x**2 + self.tilt_y**2))


def build_system(plant_kind: str, mass=None, length=None, max_thrust=None, inertia=None):
    """Cria planta + atuadores + controlador.

    Os parâmetros físicos podem ser sobrescritos (flags CLI); os não
    informados usam os defaults do hardware em ``SphereParams``,
    ``RopeParams`` e ``ActuatorParams``. Os ganhos do loop de taxa do
    heading são rescalados automaticamente pela inércia efetiva (regra
    kp/I, ver ``gains_for_inertia``) — ``--inertia`` com o valor medido no
    hardware mantém a resposta consistente.
    """
    sphere_kwargs = {"mass": mass} if mass is not None else {}
    if inertia is not None:
        sphere_kwargs["inertia_z"] = inertia
    rope_kwargs = {"length": length} if length is not None else {}
    act_kwargs = {"max_thrust": max_thrust} if max_thrust is not None else {}

    sphere = SphereParams(**sphere_kwargs)
    rope = RopeParams(**rope_kwargs)
    plant = PLANTS[plant_kind](sphere, rope)
    cluster = ThrusterCluster(ActuatorParams(**act_kwargs), sphere.radius)
    controller = EsferaController(
        cluster,
        gains=gains_for_inertia(sphere.inertia_z),
        rope_torsional_stiffness=rope.torsional_stiffness,
        rope_torsional_damping=rope.torsional_damping,
    )
    return plant, cluster, controller


def external_force_at(t: float, events) -> tuple:
    """Força externa (fx, fy) em N ativa no instante t (rajadas do cenário)."""
    fx = fy = 0.0
    for t0, t1, ex, ey in events:
        if t0 <= t < t1:
            fx += ex
            fy += ey
    return fx, fy


def run_simulation(
    scenario: Scenario, plant, cluster, controller
) -> SimulationLog:
    """Roda o loop de simulação em malha fechada e devolve o log completo."""
    ic = scenario.initial
    tilt0 = [math.radians(a) for a in ic.tilt_deg]
    plant.reset(
        plant.initial_state(*tilt0, ic.velocity, math.radians(ic.heading_deg))
    )

    schedule = sorted(scenario.heading_schedule_deg)
    schedule_idx = 0
    n_steps = int(round(scenario.duration / scenario.dt))

    records = {f.name: [] for f in fields(SimulationLog)}

    for k in range(n_steps):
        t = k * scenario.dt
        while schedule_idx < len(schedule) and t >= schedule[schedule_idx][0] - 1e-9:
            controller.set_heading(schedule[schedule_idx][1])
            schedule_idx += 1

        state = plant.read_state()
        cmd = controller.update(state, scenario.dt)
        act = cluster.realize(cmd.force_world, cmd.torque_heading, state.heading)
        # Forças externas agem na PLANTA (vento/empurrão); o controlador só
        # as percebe pelo estado medido — mesma malha do sistema real.
        ext_fx, ext_fy = external_force_at(t, scenario.force_events)
        plant.step(
            act.force_world + np.array([ext_fx, ext_fy, 0.0]),
            act.torque_heading,
            scenario.dt,
        )

        tilt_x, tilt_y = plant.tilt_angles(state)
        records["t"].append(t)
        records["x"].append(state.position[0])
        records["y"].append(state.position[1])
        records["z"].append(state.position[2])
        records["vx"].append(state.velocity[0])
        records["vy"].append(state.velocity[1])
        records["vz"].append(state.velocity[2])
        records["heading"].append(state.heading)
        records["heading_rate"].append(state.heading_rate)
        records["tilt_x"].append(tilt_x)
        records["tilt_y"].append(tilt_y)
        records["psi_des"].append(cmd.heading_setpoint)
        records["cmd_fx"].append(cmd.force_world[0])
        records["cmd_fy"].append(cmd.force_world[1])
        records["cmd_torque"].append(cmd.torque_heading)
        records["act_fx"].append(act.force_world[0])
        records["act_fy"].append(act.force_world[1])
        records["act_torque"].append(act.torque_heading)
        records["thrusts"].append(act.thrusts.copy())
        records["ext_fx"].append(ext_fx)
        records["ext_fy"].append(ext_fy)

    arrays = {name: np.asarray(values) for name, values in records.items()}
    # PWM enviado aos ESCs: função linear do empuxo (1500 us = parado).
    arrays["pwm"] = cluster.thrust_to_pwm_us(arrays["thrusts"])
    return SimulationLog(**arrays)


def _settle_time(t: np.ndarray, deviation: np.ndarray, tolerance: float):
    """Primeiro instante a partir do qual |desvio| fica sempre abaixo da tolerância."""
    inside = np.abs(deviation) < tolerance
    for i in range(len(inside)):
        if inside[i:].all():
            return float(t[i])
    return None


def compute_metrics(log: SimulationLog, scenario: Scenario) -> Dict[str, object]:
    metrics = {
        "velocidade máx. (m/s)": float(log.speed.max()),
        "inclinação máx. (°)": float(log.tilt_mag_deg.max()),
    }
    t_settle = _settle_time(log.t, log.speed, VELOCITY_TOLERANCE)
    metrics["acomodação de velocidade (s)"] = (
        f"{t_settle:.2f}" if t_settle is not None else "não acomodou"
    )
    schedule = sorted(scenario.heading_schedule_deg)
    for idx, (t_cmd, heading_deg) in enumerate(schedule):
        t_end = schedule[idx + 1][0] if idx + 1 < len(schedule) else float(log.t[-1]) + 1e-9
        mask = (log.t >= t_cmd) & (log.t < t_end)
        if not mask.any():
            continue
        err_deg = np.degrees(wrap_to_pi(log.heading[mask] - log.psi_des[mask]))
        # Sobressinal consciente da direção: o degrau pode ser dado no sentido
        # curto do wrap (erro inicial de até ±180°); excede o setpoint quando
        # o erro cruza o zero em módulo além do inicial.
        initial_sign = np.sign(err_deg[0]) or 1.0
        beyond = err_deg[initial_sign * err_deg < 0]
        overshoot = float(np.max(np.abs(beyond))) if beyond.size else 0.0
        t_settle = _settle_time(log.t[mask], err_deg, HEADING_TOLERANCE_DEG)
        settle_rel = t_settle - t_cmd if t_settle is not None else None
        metrics[f"heading {heading_deg:.0f}°: acomoda em (s)"] = (
            f"{settle_rel:.2f} (após degrau)" if settle_rel is not None else "não acomodou"
        )
        metrics[f"heading {heading_deg:.0f}°: sobressinal (°)"] = f"{overshoot:.1f}"
    return metrics


def print_metrics(name: str, metrics: Dict[str, object]) -> None:
    print(f"\n--- Métricas: {name} ---")
    for key, value in metrics.items():
        print(f"{key:42s} {value}")


def plot_comparison(logs: Dict[str, SimulationLog], scenario: Scenario, save_path=None):
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    fig.suptitle(f"Comparação de plantas — cenário '{scenario.name}'")
    for name, log in logs.items():
        style = {"linear": "--", "nonlinear": "-"}[name]
        axes[0].plot(log.t, np.degrees(log.tilt_x), style, label=f"θx ({name})")
        axes[0].plot(log.t, np.degrees(log.tilt_y), style, alpha=0.6, label=f"θy ({name})")
        axes[1].plot(log.t, log.speed, style, label=f"|v| ({name})")
        axes[2].plot(log.t, np.degrees(log.heading), style, label=f"ψ ({name})")
    axes[2].plot(
        logs["nonlinear"].t,
        np.degrees(logs["nonlinear"].psi_des),
        "k:",
        label="ψ desejado",
    )
    axes[0].set_ylabel("inclinação (°)")
    axes[1].set_ylabel("|v| (m/s)")
    axes[2].set_ylabel("heading (°)")
    axes[2].set_xlabel("t (s)")
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=130)
        print(f"Gráfico salvo em {save_path}")
    return fig


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="Simulação do pêndulo esférico Esfera")
    parser.add_argument("--scenario", default="default", choices=sorted(SCENARIOS))
    parser.add_argument("--plant", default="nonlinear", choices=sorted(PLANTS))
    parser.add_argument("--mass", type=float, default=None, help="massa total do corpo móvel em kg (default 12)")
    parser.add_argument("--length", type=float, default=None, help="comprimento da corda pino->CG em m (default 1)")
    parser.add_argument("--thrust", type=float, default=None, help="tração máxima por hélice em N (default 18)")
    parser.add_argument("--inertia", type=float, default=None, help="momento de inércia do heading em kg m^2 (default (2/5)mr^2 = 0.588; ganhos rescalados automaticamente)")
    parser.add_argument("--interactive", action="store_true", help="modo interativo ao vivo: forças externas, toggles e ganhos pelo teclado")
    parser.add_argument("--compare", action="store_true", help="roda linear e não-linear com os mesmos ganhos")
    parser.add_argument("--no-anim", action="store_true", help="sem janelas de visualização")
    parser.add_argument("--save", metavar="ARQUIVO", default=None, help="salva visualização (GIF animação / PNG comparação)")
    args = parser.parse_args(argv)

    scenario = SCENARIOS[args.scenario]
    print(f"Cenário '{scenario.name}': {scenario.description}")
    overrides = {
        "massa": args.mass,
        "comprimento": args.length,
        "tração/hélice": args.thrust,
    }
    used = {k: v for k, v in overrides.items() if v is not None}
    defaults = {"massa": "12 kg", "comprimento": "1 m", "tração/hélice": "18 N"}
    shown = {k: f"{v:g}" for k, v in used.items()} or defaults
    print("Parâmetros: " + ", ".join(f"{k}={v}" for k, v in shown.items()))

    def system(plant_kind):
        return build_system(
            plant_kind,
            mass=args.mass,
            length=args.length,
            max_thrust=args.thrust,
            inertia=args.inertia,
        )

    if args.interactive:
        import interactive

        plant, cluster, controller = system(args.plant)
        try:
            interactive.run(plant, cluster, controller, scenario)
        except RuntimeError as exc:
            print(f"Erro: {exc}")
        return

    if args.compare:
        logs = {}
        for kind in ("linear", "nonlinear"):
            plant, cluster, controller = system(kind)
            logs[kind] = run_simulation(scenario, plant, cluster, controller)
            print_metrics(kind, compute_metrics(logs[kind], scenario))
        if not args.no_anim or args.save:
            fig = plot_comparison(logs, scenario, save_path=args.save)
            if not args.no_anim:
                import matplotlib.pyplot as plt

                plt.show()
        return

    plant, cluster, controller = system(args.plant)
    log = run_simulation(scenario, plant, cluster, controller)
    print_metrics(args.plant, compute_metrics(log, scenario))

    if args.no_anim and not args.save:
        return

    import animation

    animation.run(
        log,
        cluster=cluster,
        sphere_radius=plant.sphere.radius,
        rope_length=plant.rope.length,
        title=f"Esfera — {args.plant} — cenário '{scenario.name}'",
        show=not args.no_anim,
        save_path=args.save,
    )


if __name__ == "__main__":
    main()
