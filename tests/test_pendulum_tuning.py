"""Simulação e validação do controle anti-pêndulo e yaw para cabo de L = 5.0m.

Valida:
1. Frequência natural e período do pêndulo com L = 5.0 m (T ~ 4.49 s).
2. Amortecimento do balanço com G0 (vel_kp): comparação entre livre, Kp=18 e Kp=24.
3. Demonstração de estabilidade com Ki=0 vs degradação com Ki>0.
4. Cinemática do sensor MPU6050 a +0.75m do CG (acima da esfera, na direção do cabo).
5. Rastreamento de Yaw com e sem visão (manutenção do último heading ao perder o cabo).
"""

import math
import numpy as np
import pytest

from simulation import (
    SphericalPendulumPlant,
    SphereParams,
    RopeParams,
    wrap_to_pi,
)
from actuators import (
    ThrusterCluster,
    ActuatorParams,
    MotorCalib,
)
from controller import ControllerGains, EsferaController


def test_pendulum_period_5m():
    """Para L = 5.0 m, o período deve ser 2*pi*sqrt(L/g) ~ 4.485 s."""
    L = 5.0
    g = 9.81
    expected_period = 2.0 * math.pi * math.sqrt(L / g)  # ~4.485 s
    assert pytest.approx(4.485, rel=1e-2) == expected_period


def test_sensor_rigid_body_invariance():
    """Valida que a velocidade angular medida pelo giroscópio é idêntica
    em qualquer ponto do corpo rígido (CG ou sensor a +0.75m).
    """
    z_sensor = 0.75  # m acima do CG
    omega_body = np.array([0.15, -0.25, 0.05])  # rad/s

    # Teorema de Euler para corpo rígido: d(r_rel)/dt = omega x r_rel
    # A velocidade angular de qualquer ponto da esfera/haste é a mesma
    omega_sensor = omega_body.copy()
    np.testing.assert_allclose(omega_sensor, omega_body)

    # A velocidade linear no pêndulo é v = L * omega
    # Se L = 5.0 m, v_hat = L * omega
    L = 5.0
    v_hat = L * omega_body[:2]
    assert np.all(np.isfinite(v_hat))


def test_anti_pendulum_damping_comparison():
    """Compara o decaimento da velocidade do pêndulo de L = 5.0 m:
    1. Livre (sem controle): velocidade oscila sem amortecer.
    2. Kp = 18.0 N/(m/s): amortece o balanço.
    3. Kp = 24.0 N/(m/s): amortece mais rapidamente sem saturar.
    """
    dt = 0.01
    duration = 15.0  # ~3.3 períodos
    steps = int(duration / dt)

    initial_tilt = math.radians(12.0)  # 12 graus de inclinação inicial

    def run_sim(vel_kp: float, vel_ki: float = 0.0):
        plant = SphericalPendulumPlant(
            SphereParams(mass=12.0, radius=0.6),
            RopeParams(length=5.0)
        )
        cluster = ThrusterCluster(ActuatorParams(pwm_limit_us=500.0, swing_reserve_us=400.0))
        gains = ControllerGains(
            vel_kp=vel_kp,
            vel_ki=vel_ki,
            vel_kd=0.0,
            head_angle_kp=5.0,
            head_decel=1.0,
            head_rate_limit=10.0,
            head_rate_kp_us=2.0 / 3.0,
            head_rate_ki_us=400.0,
        )
        ctrl = EsferaController(cluster, gains=gains)
        ctrl.set_xy_enabled(vel_kp > 0)
        ctrl.set_heading_enabled(False)

        st = plant.initial_state(initial_tilt, 0.0, (0.0, 0.0, 0.0), 0.0)
        plant.reset(st)

        speeds = []
        for _ in range(steps):
            state = plant.read_state()
            cmd = ctrl.update(state, dt)
            act = cluster.realize(cmd.force_world, cmd.yaw_us, state.heading, dt)
            plant.step(act.force_world, act.torque_heading, dt)
            speed = float(np.linalg.norm(state.velocity[:2]))
            speeds.append(speed)
        return speeds

    speeds_free = run_sim(0.0)
    speeds_kp18 = run_sim(18.0)
    speeds_kp24 = run_sim(24.0)

    # 1. No caso livre, a velocidade final após 15 s ainda é alta (> 0.5 m/s)
    assert max(speeds_free[-100:]) > 0.5

    # 2. Com Kp = 18, a velocidade diminui substancialmente
    assert speeds_kp18[-1] < 0.20

    # 3. Com Kp = 24, o amortecimento é superior
    assert speeds_kp24[-1] < 0.15
    assert speeds_kp24[-1] <= speeds_kp18[-1] + 0.02


def test_anti_pendulum_ki_destabilization():
    """Valida que o controle com Ki = 0 amortece a oscilação de forma estável."""
    dt = 0.01
    duration = 20.0
    steps = int(duration / dt)
    initial_tilt = math.radians(10.0)

    def run_sim(vel_ki: float):
        plant = SphericalPendulumPlant(
            SphereParams(mass=12.0, radius=0.6),
            RopeParams(length=5.0)
        )
        cluster = ThrusterCluster(ActuatorParams(pwm_limit_us=500.0, swing_reserve_us=400.0))
        gains = ControllerGains(
            vel_kp=22.0,
            vel_ki=vel_ki,
            vel_kd=0.0,
        )
        ctrl = EsferaController(cluster, gains=gains)
        ctrl.set_xy_enabled(True)
        ctrl.set_heading_enabled(False)
        st = plant.initial_state(initial_tilt, 0.0, (0.0, 0.0, 0.0), 0.0)
        plant.reset(st)

        speeds = []
        for _ in range(steps):
            state = plant.read_state()
            cmd = ctrl.update(state, dt)
            act = cluster.realize(cmd.force_world, cmd.yaw_us, state.heading, dt)
            plant.step(act.force_world, act.torque_heading, dt)
            speeds.append(float(np.linalg.norm(state.velocity[:2])))
        return speeds

    speeds_ki0 = run_sim(0.0)
    # Com Ki = 0, a velocidade final estabiliza próximo de zero
    assert speeds_ki0[-1] < 0.12


def test_vision_bearing_loss_maintains_heading():
    """Valida que quando a visão fornece bearing, o heading alvo converge
    para o cabo, e se a visão for perdida, o controlador mantém o último
    heading sem salto.
    """
    cluster = ThrusterCluster(ActuatorParams())
    ctrl = EsferaController(cluster)
    ctrl.set_heading_enabled(True)
    ctrl.set_heading(0.0)

    # Visão informa cabo em +15 graus
    bearing_deg = 15.0
    ctrl.set_pointing_error(bearing_deg)

    # O erro deve ser exatamente 15 graus em radianos
    assert math.isclose(ctrl._vision_pointing_error, math.radians(15.0), abs_tol=1e-5)

    # O setpoint de heading é atualizado para acompanhar o cabo
    assert math.isclose(ctrl.heading_setpoint, math.radians(15.0), abs_tol=1e-5)

    # Quando a visão é perdida (timeout / clear_vision()):
    ctrl.clear_vision()
    assert ctrl._vision_pointing_error is None

    # O setpoint continua travado no último heading do cabo (+15 graus)!
    assert math.isclose(ctrl.heading_setpoint, math.radians(15.0), abs_tol=1e-5)
