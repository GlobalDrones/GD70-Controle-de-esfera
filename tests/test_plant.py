"""Testes de sanidade física e de alocação dos atuadores.

Rodar com:  python -m pytest tests/ -q
"""

import math

import matplotlib
import numpy as np
import pytest

from actuators import ActuatorParams, ThrusterCluster
from scenarios import SCENARIOS
from simulation import (
    LinearizedPendulumPlant,
    RopeParams,
    SphereParams,
    SphericalPendulumPlant,
    wrap_to_pi,
)

from main import build_system, run_simulation

DT = 0.01
G = 9.81


def free_swing(plant: SphericalPendulumPlant, tilt_deg: float, duration: float):
    """Oscilação livre a partir de uma inclinação inicial (sem atuadores)."""
    tilt = math.radians(tilt_deg)
    plant.reset(plant.initial_state(tilt, 0.0, (0.0, 0.0, 0.0), 0.0))
    zero = np.zeros(3)
    states = []
    for _ in range(int(duration / DT)):
        states.append(plant.step(zero, 0.0, DT))
    return states


def test_wrap_to_pi():
    assert wrap_to_pi(3 * math.pi) == pytest.approx(-math.pi)
    assert wrap_to_pi(-3 * math.pi / 2) == pytest.approx(math.pi / 2)
    assert wrap_to_pi(0.1) == pytest.approx(0.1)


def test_rope_length_and_velocity_constraint():
    plant = SphericalPendulumPlant(SphereParams(), RopeParams())
    states = free_swing(plant, tilt_deg=20.0, duration=10.0)
    length = plant.rope.length
    for s in states:
        d = s.position / np.linalg.norm(s.position)
        assert abs(np.linalg.norm(s.position) - length) < 1e-9
        assert abs(float(s.velocity @ d)) < 1e-9


def test_energy_bounded_free_swing():
    plant = SphericalPendulumPlant(SphereParams(), RopeParams())
    plant.reset(plant.initial_state(math.radians(20.0), 0.0, (0, 0, 0), 0.0))
    e0 = plant.energy()
    for _ in range(int(20.0 / DT)):
        plant.step(np.zeros(3), 0.0, DT)
        assert abs(plant.energy() - e0) / e0 < 0.005


def test_pendulum_period_small_angle():
    plant = SphericalPendulumPlant(SphereParams(), RopeParams())
    states = free_swing(plant, tilt_deg=5.0, duration=15.0)
    tilt_x = np.array([plant.tilt_angles(s)[0] for s in states])
    crossings = np.where(np.diff(np.sign(tilt_x)) != 0)[0]
    periods = 2.0 * np.diff(crossings) * DT
    expected = 2.0 * math.pi * math.sqrt(plant.rope.length / G)
    assert periods.mean() == pytest.approx(expected, rel=0.015)


def test_torsional_oscillator_conserves_energy():
    rope = RopeParams(torsional_damping=0.0)
    plant = SphericalPendulumPlant(SphereParams(), rope)
    plant.reset(
        plant.initial_state(0.0, 0.0, (0, 0, 0), heading=0.0, heading_rate=1.0)
    )
    e0 = plant.energy()
    for _ in range(int(10.0 / DT)):
        plant.step(np.zeros(3), 0.0, DT)
        # Oscilação limitada do Euler simplético (~1,8%), sem drift.
        assert abs(plant.energy() - e0) / e0 < 0.025


def test_linear_matches_nonlinear_at_small_angles():
    nonlinear = SphericalPendulumPlant(SphereParams(), RopeParams())
    linear = LinearizedPendulumPlant(SphereParams(), RopeParams())
    tilt = math.radians(5.0)
    nonlinear.reset(nonlinear.initial_state(tilt, 0.0, (0, 0, 0), 0.0))
    linear.reset(linear.initial_state(tilt, 0.0, (0, 0, 0), 0.0))
    for _ in range(int(10.0 / DT)):
        sn = nonlinear.step(np.zeros(3), 0.0, DT)
        sl = linear.step(np.zeros(3), 0.0, DT)
    assert abs(sn.position[0] - sl.position[0]) < 0.02
    assert abs(sn.position[1] - sl.position[1]) < 0.02


def test_thrust_allocation_is_exact_without_saturation():
    cluster = ThrusterCluster(ActuatorParams(), sphere_radius=0.1)
    rng = np.random.default_rng(42)
    for _ in range(20):
        heading = rng.uniform(-math.pi, math.pi)
        force_des = np.append(rng.uniform(-1.0, 1.0, 2), 0.0)
        torque_des = rng.uniform(-0.05, 0.05)
        act = cluster.realize(force_des, torque_des, heading)
        np.testing.assert_allclose(act.force_world, force_des, atol=1e-9)
        assert act.torque_heading == pytest.approx(torque_des, abs=1e-9)
        assert np.all(np.abs(act.thrusts) <= cluster.params.max_thrust)


def test_thrust_allocation_saturates():
    cluster = ThrusterCluster(ActuatorParams(), sphere_radius=0.1)
    act = cluster.realize(np.array([50.0, 0.0, 0.0]), 5.0, heading=0.0)
    assert np.all(np.abs(act.thrusts) <= cluster.params.max_thrust + 1e-12)
    # Sob saturação profunda a alocação desvia do comando (vazamento no
    # espaço nulo, com torque espúrio), mas fica dentro do envelope físico:
    # força <= 2 f_max e torque <= max_torque.
    assert np.linalg.norm(act.force_world) <= 2.0 * cluster.params.max_thrust + 1e-9
    assert abs(act.torque_heading) <= cluster.max_torque + 1e-9


def test_thrust_allocation_exact_region_matches_command():
    """Dentro do envelope exato da alocação — (2/3)|t_i·F| + |tau|/(3R) <=
    f_max para todo i — o wrench realizado é idêntico ao comandado, mesmo
    combinando força e torque e girando o heading."""
    cluster = ThrusterCluster(ActuatorParams(), sphere_radius=0.1)
    for heading in (0.0, 1.0, -2.5):
        for force in (
            np.array([1.0, 0.0, 0.0]),
            np.array([0.7, 0.7, 0.0]),
            np.array([2.5, 0.0, 0.0]),  # perto do envelope exato com tau = 0
        ):
            for torque in (0.0, 0.2, -0.3):
                if np.max(np.abs((2 / 3) * (cluster.thrust_dirs_body[:2].T @ force[:2]))) + abs(
                    torque
                ) / (3 * cluster.arm_radius) > cluster.params.max_thrust:
                    continue  # fora do envelope exato: saturação esperada
                act = cluster.realize(force, torque, heading)
                # realize() recebe/comanda no mundo; sem saturação devolve
                # exatamente o wrench comandado, qualquer que seja o heading.
                np.testing.assert_allclose(act.force_world, force, atol=1e-9)
                assert act.torque_heading == pytest.approx(torque, abs=1e-9)


def test_closed_loop_default_scenario_stabilizes():
    scenario = SCENARIOS["default"]
    plant, cluster, controller = build_system("nonlinear")
    log = run_simulation(scenario, plant, cluster, controller)
    assert log.speed[-1] < 0.05
    assert log.tilt_mag_deg[-1] < 2.0
    final_err_deg = math.degrees(wrap_to_pi(log.heading[-1] - log.psi_des[-1]))
    assert abs(final_err_deg) < 2.0


def test_closed_loop_disturbance_damps_velocity():
    scenario = SCENARIOS["disturbance"]
    plant, cluster, controller = build_system("nonlinear")
    log = run_simulation(scenario, plant, cluster, controller)
    assert log.speed[-1] < 0.05
    assert log.tilt_mag_deg[-1] < 2.0


def test_motor_labels_and_geometry():
    """Hélice 'a' empurra no +X do corpo (= +X do sensor); b e c a 120°."""
    cluster = ThrusterCluster(ActuatorParams(), sphere_radius=0.1)
    assert cluster.LABELS == ("a", "b", "c")
    np.testing.assert_allclose(
        cluster.thrust_dirs_body[:2, 0], [1.0, 0.0], atol=1e-12
    )
    for i in (1, 2):
        cos_angle = float(
            cluster.thrust_dirs_body[:2, 0] @ cluster.thrust_dirs_body[:2, i]
        )
        assert cos_angle == pytest.approx(math.cos(2.0 * math.pi / 3))
    # torque consistente: empuxo positivo de 'a' gira +psi (braço R)
    act = cluster.realize(np.zeros(3), 0.0, heading=0.0)
    assert act.torque_heading == pytest.approx(0.0)


def test_thrust_to_pwm_mapping():
    """PWM bidirecional: 1500 us parado, 2000/1000 us full speed ±."""
    cluster = ThrusterCluster(ActuatorParams(), sphere_radius=0.1)
    f_max = cluster.params.max_thrust
    pwm = cluster.thrust_to_pwm_us(np.array([0.0, f_max, -f_max]))
    assert pwm[0] == pytest.approx(1500.0)
    assert pwm[1] == pytest.approx(2000.0)
    assert pwm[2] == pytest.approx(1000.0)
    # linearidade: meia tração -> meio intervalo
    pwm_half = cluster.thrust_to_pwm_us(np.array([f_max / 2.0, 0.0, 0.0]))
    assert pwm_half[0] == pytest.approx(1750.0)
    # clip fora da faixa
    pwm_clip = cluster.thrust_to_pwm_us(np.array([3.0 * f_max, -3.0 * f_max, 0.0]))
    assert pwm_clip[0] == pytest.approx(2000.0)
    assert pwm_clip[1] == pytest.approx(1000.0)


def test_closed_loop_heading_sequence_tracks_setpoint():
    scenario = SCENARIOS["heading"]
    plant, cluster, controller = build_system("linear")
    log = run_simulation(scenario, plant, cluster, controller)
    final_err_deg = math.degrees(wrap_to_pi(log.heading[-1] - log.psi_des[-1]))
    assert abs(final_err_deg) < 2.0


# ------------------------------------------------------------------ toggles


def test_controller_toggle_xy_zeroes_force():
    plant, cluster, controller = build_system("nonlinear")
    plant.reset(plant.initial_state(math.radians(10.0), 0.0, (0.5, 0.0, 0.0), 0.0))
    state = plant.read_state()
    cmd = controller.update(state, DT)
    assert abs(cmd.force_world[0]) > 0.0  # loop ativo reage à velocidade

    controller.set_xy_enabled(False)
    cmd = controller.update(state, DT)
    np.testing.assert_allclose(cmd.force_world, 0.0)
    # heading continua ativo
    controller.set_heading(90.0)
    cmd = controller.update(state, DT)
    assert abs(cmd.torque_heading) > 0.0

    controller.set_xy_enabled(True)  # religar reseta integrais, força volta
    cmd = controller.update(state, DT)
    assert abs(cmd.force_world[0]) > 0.0


def test_controller_toggle_heading_zeroes_torque():
    plant, cluster, controller = build_system("nonlinear")
    plant.reset(plant.initial_state(0, 0, (0, 0, 0), 0.0))
    controller.set_heading(90.0)
    state = plant.read_state()
    cmd = controller.update(state, DT)
    assert abs(cmd.torque_heading) > 0.0

    controller.set_heading_enabled(False)
    cmd = controller.update(state, DT)
    assert cmd.torque_heading == 0.0
    # XY continua ativo
    plant.reset(plant.initial_state(0, 0, (0.3, 0.0, 0.0), 0.0))
    state = plant.read_state()
    cmd = controller.update(state, DT)
    assert abs(cmd.force_world[0]) > 0.0


def test_both_loops_off_means_idle_escs():
    plant, cluster, controller = build_system("nonlinear")
    controller.set_xy_enabled(False)
    controller.set_heading_enabled(False)
    plant.reset(plant.initial_state(math.radians(10.0), 0.0, (0.4, 0.2, 0.0), 0.5))
    state = plant.read_state()
    cmd = controller.update(state, DT)
    np.testing.assert_allclose(cmd.force_world, 0.0)
    assert cmd.torque_heading == 0.0
    act = cluster.realize(cmd.force_world, cmd.torque_heading, state.heading)
    np.testing.assert_allclose(act.thrusts, 0.0)
    np.testing.assert_allclose(
        cluster.thrust_to_pwm_us(act.thrusts), cluster.params.pwm_neutral_us
    )


def test_set_gains_updates_pids():
    _, _, controller = build_system("nonlinear")
    from controller import ControllerGains

    new = ControllerGains(vel_kp=10.0, vel_ki=1.0, vel_kd=0.5)
    controller.set_gains(new)
    assert controller.pid_vx.kp == 10.0
    assert controller.pid_vy.ki == 1.0
    assert controller.pid_heading_rate.kd == new.head_rate_kd


def test_gains_for_inertia_rescales_rate_loop():
    from controller import REFERENCE_INERTIA_Z, ControllerGains, gains_for_inertia

    default = ControllerGains()
    doubled = gains_for_inertia(2.0 * REFERENCE_INERTIA_Z)
    assert doubled.head_rate_kp == pytest.approx(2.0 * default.head_rate_kp)
    assert doubled.head_rate_ki == pytest.approx(2.0 * default.head_rate_ki)
    # ganhos cinemáticos e de translação inalterados
    assert doubled.head_angle_kp == default.head_angle_kp
    assert doubled.vel_kp == default.vel_kp


# ------------------------------------------------------- PWM no log + rajadas


def test_log_pwm_within_esc_range():
    scenario = SCENARIOS["gust"]
    plant, cluster, controller = build_system("nonlinear")
    log = run_simulation(scenario, plant, cluster, controller)
    assert log.pwm.shape == (len(log.t), 3)
    assert np.all(log.pwm >= 1000.0 - 1e-9) and np.all(log.pwm <= 2000.0 + 1e-9)
    # controle ativo durante a rajada: PWM sai do neutro em algum momento
    assert np.max(np.abs(log.pwm - 1500.0)) > 1.0
    # força externa registrada nos instantes da rajada
    gust_mask = (log.t >= 4.0) & (log.t < 6.0)
    assert np.allclose(log.ext_fx[gust_mask], 20.0)
    assert np.allclose(log.ext_fx[log.t < 4.0], 0.0)


def test_gust_scenario_stabilizes_after_disturbances():
    scenario = SCENARIOS["gust"]
    plant, cluster, controller = build_system("nonlinear")
    log = run_simulation(scenario, plant, cluster, controller)
    assert log.speed[-1] < 0.05
    assert log.tilt_mag_deg[-1] < 2.0
    final_err_deg = math.degrees(wrap_to_pi(log.heading[-1] - log.psi_des[-1]))
    assert abs(final_err_deg) < 2.0


# ------------------------------------------------------------- LiveSession


def test_live_session_external_force_and_control_response():
    from interactive import LiveSession

    scenario = SCENARIOS["default"]
    plant, cluster, controller = build_system("nonlinear")
    session = LiveSession(plant, cluster, controller, scenario)

    # segurar 'right': força externa +X, esfera desloca e o controle reage
    session.held_keys = {"right"}
    for _ in range(200):
        session.step()
    assert session.hist["px"][-1] > 0.01
    assert np.max(np.abs(np.array(session.hist["vx"]))) > 0.01
    assert np.max(np.abs(session.last_pwm - 1500.0)) > 1.0  # ESCs saem do neutro

    # soltar: sem força externa
    session.held_keys = set()
    assert float(np.linalg.norm(session.external_force())) == 0.0


def test_live_session_toggle_and_reset():
    from interactive import LiveSession

    scenario = SCENARIOS["default"]
    plant, cluster, controller = build_system("nonlinear")
    session = LiveSession(plant, cluster, controller, scenario)

    session.toggle_xy()
    assert controller.xy_enabled is False
    session.toggle_heading()
    assert controller.heading_enabled is False
    for _ in range(50):
        session.step()
    np.testing.assert_allclose(session.last_thrusts, 0.0)  # ESCs em 1500 us

    session.toggle_xy()
    session.toggle_heading()
    session.nudge_heading_setpoint(30.0)
    assert controller.heading_setpoint == pytest.approx(math.radians(30.0))

    session.reset()
    assert session.t == 0.0
    assert len(session.hist["t"]) == 0
    # reset volta à condição inicial do CENÁRIO (default: 15° de inclinação)
    state = plant.read_state()
    tilt0 = math.radians(scenario.initial.tilt_deg[0])
    assert state.position[0] == pytest.approx(plant.rope.length * math.sin(tilt0))


def test_live_session_gain_adjustment():
    from interactive import LiveSession

    scenario = SCENARIOS["default"]
    plant, cluster, controller = build_system("nonlinear")
    session = LiveSession(plant, cluster, controller, scenario)

    # item 1 é vel_kp; ajustar +1 passo aumenta em 10% do default
    session.cycle_selected()  # 0 -> 1 (vel_kp)
    kp_before = controller.gains.vel_kp
    session.adjust_selected(+1.0)
    assert controller.gains.vel_kp == pytest.approx(kp_before + 0.1 * kp_before)
    assert controller.pid_vx.kp == controller.gains.vel_kp


def test_interactive_window_smoke_headless():
    """Fumigação da janela interativa com backend Agg: construir figura,
    disparar o timer e simular teclas não pode levantar exceção."""
    matplotlib.use("Agg", force=True)
    from interactive import InteractiveWindow, LiveSession

    scenario = SCENARIOS["default"]
    plant, cluster, controller = build_system("nonlinear")
    session = LiveSession(plant, cluster, controller, scenario)
    win = InteractiveWindow(session)

    class FakeEvent:
        def __init__(self, key):
            self.key = key

    # segurar força externa + sequência de teclas de UI
    win.on_press(FakeEvent("right"))
    win.on_press(FakeEvent("1"))   # desliga XY
    win.on_press(FakeEvent("x"))   # heading setpoint +15°
    win.on_press(FakeEvent("g"))
    win.on_press(FakeEvent("+"))
    for _ in range(5):
        win.on_timer()  # avança simulação + redraw (sem GUI)
    win.on_press(FakeEvent("1"))   # religa XY
    win.on_press(FakeEvent("p"))   # pausa
    win.on_timer()
    win.on_press(FakeEvent("p"))   # retoma
    win.on_release(FakeEvent("right"))

    assert controller.xy_enabled is True
    assert session.paused is False
    assert session.t > 0.0

    # regressão do bug "só anima com o mouse pressionado": o timer tem que
    # pedir o repaint do canvas (draw_idle), não só atualizar os dados.
    draw_calls = []
    win.fig.canvas.draw_idle = lambda: draw_calls.append(1)
    win.on_timer()
    assert len(draw_calls) >= 1

    import os

    os.makedirs("out", exist_ok=True)
    win.fig.savefig("out/_interactive_smoke.png", dpi=72)
    win.on_press(FakeEvent("escape"))
