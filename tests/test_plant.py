"""Testes de sanidade física, atuadores por motor, controle e modo interativo.

Rodar com:  python -m pytest tests/ -q
"""

import math
import os

import matplotlib
import numpy as np
import pytest

from actuators import (
    ActuatorParams,
    MotorCalib,
    ThrusterCluster,
    default_motors,
)
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

# Cluster sem atraso de atuação (lag ~0) para testes de alocação exata.
NO_LAG = ActuatorParams(lag_tau=1e-12)


def free_swing(plant: SphericalPendulumPlant, tilt_deg: float, duration: float):
    """Oscilação livre a partir de uma inclinação inicial (sem atuadores)."""
    tilt = math.radians(tilt_deg)
    plant.reset(plant.initial_state(tilt, 0.0, (0.0, 0.0, 0.0), 0.0))
    zero = np.zeros(3)
    states = []
    for _ in range(int(duration / DT)):
        states.append(plant.step(zero, 0.0, DT))
    return states


# ------------------------------------------------------------- física pura


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
    rope = RopeParams(torsional_stiffness=0.05, torsional_damping=0.0)
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


def test_pivot_accel_disturbs_plant():
    """Aceleração do pivô (fase drone) age como força inercial na planta."""
    fixed = SphericalPendulumPlant(SphereParams(), RopeParams())
    moving = SphericalPendulumPlant(SphereParams(), RopeParams())
    for p in (fixed, moving):
        p.reset(p.initial_state(0, 0, (0, 0, 0), 0.0))
    moving.pivot_accel = np.array([1.0, 0.0, 0.0])
    for _ in range(50):
        fixed.step(np.zeros(3), 0.0, DT)
        moving.step(np.zeros(3), 0.0, DT)
    assert fixed.read_state().position[0] == pytest.approx(0.0, abs=1e-9)
    # a_pivo = +x equivale a gravidade "puxando" o pêndulo para -x
    assert moving.read_state().position[0] < -1e-4


# ------------------------------------------------- atuadores (por motor)


def test_motor_labels_and_geometry():
    """Motor a empurra no +X do corpo; b e c a ±120° (geometria default)."""
    cluster = ThrusterCluster()
    assert cluster.LABELS == ("a", "b", "c")
    np.testing.assert_allclose(cluster.T[:, 0], [1.0, 0.0], atol=1e-12)
    for i in (1, 2):
        cos_angle = float(cluster.T[:, 0] @ cluster.T[:, i])
        assert cos_angle == pytest.approx(-0.5)  # 120° entre pares
    np.testing.assert_allclose(
        cluster.T.sum(axis=1), [0.0, 0.0], atol=1e-12
    )  # distribuição de yaw não gera força líquida


def test_thrust_curve_roundtrip():
    """us_from_force e force_from_us são inversas exatas ACIMA do piso
    anti-travamento (comandos sub-piso viram neutro — nunca habitam a faixa
    de travamento do ESC)."""
    cluster = ThrusterCluster()
    for u in (100.0, 120.0, 240.0, -120.0, -180.0):
        f = cluster.force_from_us(np.full(3, u))
        u_back = cluster.us_from_force(f)
        np.testing.assert_allclose(u_back, u, rtol=1e-9)
    for f in (0.5, 1.0, 3.0, -0.5, -2.0):
        u = cluster.us_from_force(np.full(3, f))
        f_back = cluster.force_from_us(u)
        np.testing.assert_allclose(f_back, f, rtol=1e-9)
    # dentro da zona morta / abaixo do piso: neutro
    assert cluster.force_from_us(np.full(3, 20.0)) == pytest.approx(0.0)
    assert cluster.us_from_force(np.full(3, 0.05)) == pytest.approx(0.0)


def test_command_floor_avoids_latch_band():
    """O menor µs não-nulo comandado já nasce FORA da faixa de travamento
    (latch 60 µs + margem) — o controlador nunca habita a faixa."""
    cluster = ThrusterCluster()
    latch = cluster.motors[0].latch_us
    for f in (0.17, 0.2, 0.3):  # logo acima do piso (~0,165 N)
        u = cluster.us_from_force(np.array([f, 0.0, 0.0]))[0]
        assert abs(u) > latch, f"F={f}: µs {u} dentro da faixa de travamento"
        f_back = cluster.force_from_us(np.array([u, 0.0, 0.0]))[0]
        assert f_back > 0.0  # e o ESC responde


def test_thrust_to_pwm_mapping():
    """Quadrática com zona morta e clamp de segurança ±500 µs; ré menos
    eficiente (60%) pede mais µs para a mesma força."""
    params = ActuatorParams(f_at_500us=18.0, deadzone_us=30.0, reverse_ratio=0.6)
    cluster = ThrusterCluster(params)
    kf = 18.0 / (500.0 - 30.0) ** 2
    # força correspondente a +100 µs de offset -> 1600 µs (frente)
    f100 = kf * (100.0 - 30.0) ** 2
    pwm = cluster.thrust_to_pwm_us(np.array([f100, -f100, 0.0]))
    assert pwm[0] == pytest.approx(1600.0)
    # mesma força no reverso: sqrt(1/0.6) mais µs além da zona morta
    u_rev = 30.0 + 70.0 / math.sqrt(0.6)
    assert pwm[1] == pytest.approx(1500.0 - u_rev)
    assert pwm[2] == pytest.approx(1500.0)
    # acima do clamp: clipa na faixa completa do ESC (2000/1000)
    pwm_clip = cluster.thrust_to_pwm_us(np.array([100.0, -100.0, 0.0]))
    assert pwm_clip[0] == pytest.approx(2000.0)
    assert pwm_clip[1] == pytest.approx(1000.0)


def test_thrust_allocation_exact_region_matches_command():
    """Região exata da alocação = todo componente por motor ACIMA do gate
    anti-travamento — aí a força realizada é a comandada, em qualquer
    heading (componentes sub-gate viram neutro de propósito: nunca habitam
    a faixa de travamento do ESC)."""
    cluster = ThrusterCluster(NO_LAG)
    gate_fwd = min(m.kf_fwd for m in cluster.motors) * (75.0 - 30.0) ** 2
    gate_rev = min(m.kf_rev for m in cluster.motors) * (75.0 - 30.0) ** 2
    for heading in (0.0, 1.0, -2.5):
        for force in (np.array([2.0, 0.0, 0.0]), np.array([1.4, 1.4, 0.0])):
            rot = ThrusterCluster.rotation_z(heading)
            f_body = rot.T @ force
            f_mot = (2.0 / 3.0) * (cluster.T.T @ f_body[:2])
            if any(
                (0 <= f < gate_fwd) or (-gate_rev < f < 0) for f in f_mot
            ):
                continue  # componente sub-gate: zero intencional
            act = cluster.realize(force, 0.0, heading, DT)
            np.testing.assert_allclose(act.force_world, force, atol=1e-9)
            assert np.all(np.abs(act.pwm_us - 1500.0) <= 500.0 + 1e-9)


def test_subgate_component_snaps_to_neutral():
    """Componente por motor abaixo do gate anti-travamento vira neutro — o
    erro de realização fica confinado ao componente descartado (pequeno)."""
    cluster = ThrusterCluster(NO_LAG)
    act = cluster.realize(np.array([1.0, 0.0, 0.0]), 0.0, heading=-2.5, dt=DT)
    err = np.linalg.norm(act.force_world[:2] - np.array([1.0, 0.0]))
    assert err < 0.15  # só o componente sub-gate (~0,08 N) é sacrificado


def test_yaw_distribution_pure_torque():
    """Distribuição de yaw igual nos 3 motores: torque sem força líquida."""
    cluster = ThrusterCluster(NO_LAG)
    act = cluster.realize(np.zeros(3), 100.0, heading=0.7, dt=DT)
    np.testing.assert_allclose(act.force_world[:2], 0.0, atol=1e-9)
    assert act.torque_heading > 0.0
    assert act.torque_heading == pytest.approx(
        cluster.arm_radius * 3.0 * act.thrusts[0], rel=1e-6
    )


def test_thrust_allocation_saturates():
    """Comando enorme: µs clipam na faixa do ESC e a força fica no envelope."""
    cluster = ThrusterCluster(NO_LAG)
    act = cluster.realize(np.array([50.0, 0.0, 0.0]), 400.0, heading=0.0, dt=DT)
    assert np.all(np.abs(act.pwm_us - 1500.0) <= cluster.params.pwm_limit_us + 1e-9)
    assert np.linalg.norm(act.force_world[:2]) <= cluster.max_net_force * 1.5
    assert act.torque_heading <= cluster.max_torque + 1e-9


def test_actuator_lag_first_order():
    """O lag de 1ª ordem suaviza a RESPOSTA (não o comando): pwm_us reporta
    o comandado (salta na hora); o empuxo sobe com constante de tempo tau."""
    params = ActuatorParams(lag_tau=0.12)
    cluster = ThrusterCluster(params)
    u_target = 200.0
    for _ in range(12):  # 0,12 s ≈ 1 tau
        act = cluster.command_raw(np.full(3, u_target), heading=0.0, dt=DT)
    # comando chega inteiro ao ESC
    assert act.pwm_us[0] - 1500.0 == pytest.approx(u_target)
    # empuxo efetivo: recupera o µs pós-lag pela inversa da curva
    u_after = cluster.us_from_force(act.thrusts)[0]
    assert 0.5 * u_target < u_after < 0.8 * u_target


def test_calibration_roundtrip(tmp_path):
    cluster = ThrusterCluster()
    cluster.apply_motor_calib(
        1,
        MotorCalib(label="b", thrust_dir=(0.0, 1.0), yaw_sign=-1.0,
                   kf_fwd=9e-5, kf_rev=5e-5, deadzone_us=25.0, latch_us=45.0),
    )
    path = os.path.join(tmp_path, "calib.json")
    cluster.save_calibration(path)
    loaded = ThrusterCluster.from_calibration(path)
    assert loaded.arm_radius == cluster.arm_radius
    for m_orig, m_load in zip(cluster.motors, loaded.motors):
        assert m_load.label == m_orig.label
        np.testing.assert_allclose(m_load.thrust_dir, m_orig.thrust_dir)
        assert m_load.yaw_sign == m_orig.yaw_sign
        assert m_load.kf_fwd == m_orig.kf_fwd
        assert m_load.latch_us == m_orig.latch_us


# ------------------------------------------- travamento direcional do ESC


def _pulse(cluster, u, dt=0.01, steps=50):
    """Pulsa u por `steps` passos e devolve o último ActuatorOutput."""
    act = None
    for _ in range(steps):
        act = cluster.command_raw(np.array([u, 0.0, 0.0]), heading=0.0, dt=dt)
    return act


def test_esc_latch_blocks_after_small_command():
    """Comando pequeno trava o sentido; comandos GRANDES seguintes não giram."""
    cluster = ThrusterCluster(NO_LAG)
    # pequeno positivo (dentro de 60 µs): trava a frente
    act = _pulse(cluster, 50.0, steps=5)
    assert act.thrusts[0] == pytest.approx(0.0)
    # comando grande na frente: continua travado (não destrava sozinho)
    act = _pulse(cluster, 300.0)
    assert act.pwm_us[0] == pytest.approx(1800.0)  # comando chega ao ESC
    assert act.thrusts[0] == pytest.approx(0.0)     # ...mas o motor não gira


def test_esc_latch_unlocks_by_reversal():
    """Só um comando válido no sentido OPOSTO destrava."""
    cluster = ThrusterCluster(NO_LAG)
    _pulse(cluster, 50.0, steps=5)     # trava frente
    _pulse(cluster, 300.0, steps=5)    # continua travado
    act = _pulse(cluster, -300.0)      # ré válida: destrava a frente
    assert act.thrusts[0] < 0.0        # gira na ré
    act = _pulse(cluster, 300.0)       # frente destravada agora
    assert act.thrusts[0] > 0.0


def test_esc_latch_per_motor_and_disable():
    """latch_us = 0 desativa o modelo no motor; independentes entre motores."""
    cluster = ThrusterCluster(NO_LAG)
    calib = MotorCalib(**{**cluster.motors[1].to_dict(), "latch_us": 0.0})
    cluster.apply_motor_calib(1, calib)
    # motor 1 sem latch: pequeno comando não trava (força segue a curva)
    act = cluster.command_raw(np.array([0.0, 50.0, 0.0]), 0.0, DT)
    assert act.thrusts[1] > 0.0
    # motor 0 com latch trava normalmente
    act = cluster.command_raw(np.array([50.0, 0.0, 0.0]), 0.0, DT)
    assert act.thrusts[0] == pytest.approx(0.0)


def test_esc_latch_neutral_does_not_block():
    """Neutro exato (u=0) não trava nada."""
    cluster = ThrusterCluster(NO_LAG)
    _pulse(cluster, 0.0, steps=10)
    act = _pulse(cluster, 300.0)
    assert act.thrusts[0] > 0.0  # nunca travou


def test_inverted_motor_changes_response():
    """Motor com calibração invertida (ESC/hélice) responde ao contrário —
    base do wizard de identificação."""
    normal = ThrusterCluster(NO_LAG)
    inverted = ThrusterCluster(NO_LAG)
    calib = MotorCalib(**{**normal.motors[0].to_dict(), "yaw_sign": -1.0})
    inverted.apply_motor_calib(0, calib)
    f = np.array([1.0, 0.0, 0.0])
    act_n = normal.realize(f, 0.0, 0.0, DT)
    act_i = inverted.realize(f, 0.0, 0.0, DT)
    # mesmo comando de força: só o torque muda de sinal (distribuição de yaw;
    # 120 µs — acima do piso anti-travamento, sub-piso é zero de propósito)
    act_yaw_n = normal.realize(np.zeros(3), 120.0, 0.0, DT)
    act_yaw_i = inverted.realize(np.zeros(3), 120.0, 0.0, DT)
    assert act_yaw_n.torque_heading > 0
    assert act_yaw_i.torque_heading < act_yaw_n.torque_heading


# ---------------------------------------------------------------- toggles


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
    assert abs(cmd.yaw_us) > 0.0

    controller.set_xy_enabled(True)  # religar reseta integrais, força volta
    cmd = controller.update(state, DT)
    assert abs(cmd.force_world[0]) > 0.0


def test_controller_toggle_heading_zeroes_yaw():
    plant, cluster, controller = build_system("nonlinear")
    plant.reset(plant.initial_state(0, 0, (0, 0, 0), 0.0))
    controller.set_heading(90.0)
    state = plant.read_state()
    cmd = controller.update(state, DT)
    assert abs(cmd.yaw_us) > 0.0

    controller.set_heading_enabled(False)
    cmd = controller.update(state, DT)
    assert cmd.yaw_us == 0.0
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
    assert cmd.yaw_us == 0.0
    act = cluster.realize(cmd.force_world, cmd.yaw_us, state.heading, DT)
    np.testing.assert_allclose(act.thrusts, 0.0, atol=1e-12)
    np.testing.assert_allclose(act.pwm_us, 1500.0)


def test_pointing_error_from_vision():
    """Bearing da visão substitui o setpoint interno como erro direto."""
    plant, cluster, controller = build_system("nonlinear")
    plant.reset(plant.initial_state(0, 0, (0, 0, 0), 0.0))
    controller.set_pointing_error(20.0)  # alvo 20° à frente do corpo
    cmd = controller.update(plant.read_state(), DT)
    assert cmd.yaw_us > 0.0


def test_set_gains_updates_pids():
    from controller import ControllerGains

    _, _, controller = build_system("nonlinear")
    new = ControllerGains(vel_kp=10.0, vel_ki=1.0, vel_kd=0.5)
    controller.set_gains(new)
    assert controller.pid_vx.kp == 10.0
    assert controller.pid_vy.ki == 1.0
    assert controller.pid_heading_rate.kp == new.head_rate_kp_us


def test_gains_for_inertia_rescales_rate_loop():
    from controller import REFERENCE_INERTIA_Z, ControllerGains, gains_for_inertia

    default = ControllerGains()
    doubled = gains_for_inertia(2.0 * REFERENCE_INERTIA_Z)
    assert doubled.head_rate_kp_us == pytest.approx(2.0 * default.head_rate_kp_us)
    assert doubled.head_rate_ki_us == pytest.approx(2.0 * default.head_rate_ki_us)
    # ganhos cinemáticos e de translação inalterados
    assert doubled.head_angle_kp == default.head_angle_kp
    assert doubled.vel_kp == default.vel_kp


# ------------------------------------------------- malha fechada (batch)


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
    assert np.allclose(log.ext_fx[gust_mask], 10.0)
    assert np.allclose(log.ext_fx[log.t < 4.0], 0.0)


def test_gust_scenario_stabilizes_after_disturbances():
    scenario = SCENARIOS["gust"]
    plant, cluster, controller = build_system("nonlinear")
    log = run_simulation(scenario, plant, cluster, controller)
    assert log.speed[-1] < 0.05
    assert log.tilt_mag_deg[-1] < 2.0
    final_err_deg = math.degrees(wrap_to_pi(log.heading[-1] - log.psi_des[-1]))
    assert abs(final_err_deg) < 2.0


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


def test_closed_loop_heading_sequence_tracks_setpoint():
    scenario = SCENARIOS["heading"]
    plant, cluster, controller = build_system("linear")
    log = run_simulation(scenario, plant, cluster, controller)
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
    np.testing.assert_allclose(session.last_thrusts, 0.0, atol=0.05)

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

    os.makedirs("out", exist_ok=True)
    win.fig.savefig("out/_interactive_smoke.png", dpi=72)
    win.on_press(FakeEvent("escape"))
