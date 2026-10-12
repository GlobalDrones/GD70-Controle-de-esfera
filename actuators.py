"""Atuadores: 3 motores bidirecionais com hélices tangentes fixas à esfera.

Geometria (referencial do corpo = referencial do sensor: o sensor está
montado no pino, no topo, e gira com a esfera). Os motores são rotulados
``a``, ``b`` e ``c``; a hélice **a empurra no sentido +X do sensor** e as
outras duas ficam a 120° dela. Parametrizando pelo ÂNGULO DE EMPUXO beta
(direção da força, não da montagem):

    beta_i = 0°, 120°, 240°            (a, b, c)
    empuxo  i:  t_i = (cos b_i, sin b_i, 0)
    montagem i: p_i = R (sin b_i, -cos b_i, 0)     [onde a tangente é t_i]

Com hélices tangentes ao raio, cada empuxo positivo f_i produz torque de
heading +R f_i (todas giram a esfera no mesmo sentido com f > 0):

- força no corpo:  F_b = sum_i f_i t_i
- torque vertical: tau = R sum_i f_i

Alocação fechada (matriz inversa analítica): dado F_b desejado e tau desejado,

    f_i = (2/3) (t_i . F_b) + tau / (3 R)

porque sum t_i = 0 e sum t_i t_i^T = (3/2) I.

Interface de hardware: cada motor é acionado por PWM bidirecional —
1500 us = parado, 2000 us = potência máxima em um sentido, 1000 us =
máxima no outro (ver :meth:`ThrusterCluster.thrust_to_pwm_us`).

As hélices são fixas à esfera, então a força efetiva no mundo gira com o
heading: F_world = Rz(psi) F_b — o acoplamento real entre heading e
translação. Envelopes com saturação |f_i| <= f_max:

- envelope EXATO (comando realizado fielmente): |F| <= 1,5 f_max com a
  alocação sem saturação (:attr:`ThrusterCluster.max_net_force`);
- teto físico absoluto sob saturação profunda: 2 f_max, com vazamento no
  espaço nulo (torque espúrio possível).

Aproximação (documentada no README): as hélices empurram sempre no plano
horizontal; o efeito da inclinação da esfera sobre a direção do empuxo é de
segunda ordem em theta e fica para a próxima iteração do digital twin.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class ActuatorParams:
    max_thrust: float = 18.0  # N, empuxo reversível por hélice (±)
    arm_radius: Optional[float] = None  # m; None => usar o raio da esfera
    pwm_neutral_us: float = 1500.0  # us, motor parado
    pwm_span_us: float = 500.0  # us, do neutro ao full speed (±)


@dataclass
class ActuatorOutput:
    """Resultado real dos atuadores após alocação e saturação."""

    force_world: np.ndarray  # (3,), N
    torque_heading: float  # N m, em torno do eixo vertical
    thrusts: np.ndarray  # (3,), N, empuxo individual [a, b, c]


class ThrusterCluster:
    N_THRUSTERS = 3
    LABELS = ("a", "b", "c")  # hélice a empurra no +X do sensor

    def __init__(self, params: ActuatorParams, sphere_radius: float):
        if params.max_thrust <= 0:
            raise ValueError("max_thrust deve ser positivo")
        self.params = params
        self.arm_radius = params.arm_radius or sphere_radius

        betas = 2.0 * math.pi * np.arange(self.N_THRUSTERS) / self.N_THRUSTERS
        # Direções de empuxo e pontos de montagem no referencial do corpo.
        self.thrust_dirs_body = np.stack(
            [np.cos(betas), np.sin(betas), np.zeros(3)]
        )  # (3, N), coluna i = t_i
        self.mounts_body = self.arm_radius * np.stack(
            [np.sin(betas), -np.cos(betas), np.zeros(3)]
        )  # (3, N), coluna i = p_i

    @property
    def max_net_force(self) -> float:
        """Envelope EXATO de força líquida: maior |F| cujo comando é
        realizado fielmente (sem saturação) para qualquer hélice.

        Com a alocação f_i = (2/3)(t_i·F) + tau/(3R), a condição de não
        saturação é (2/3)|t_i·F| + |tau|/(3R) <= f_max para todo i; no caso
        mais favorável (F alinhado a uma tangente, tau = 0) isso dá
        |F| <= (3/2) f_max. Acima disso a saturação redistribui os empuxos e
        o wrench realizado desvia do comandado (teto físico absoluto:
        2 f_max de força, com torque espúrio possível).
        """
        return 1.5 * self.params.max_thrust

    @property
    def max_torque(self) -> float:
        """Máximo torque de heading (todas as hélices no mesmo sentido)."""
        return self.arm_radius * self.N_THRUSTERS * self.params.max_thrust

    @staticmethod
    def rotation_z(psi: float) -> np.ndarray:
        """Matriz de rotação Rz(psi), do referencial do corpo para o mundo."""
        c, s = math.cos(psi), math.sin(psi)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    def allocate(self, force_body_xy, torque_z: float) -> np.ndarray:
        """Empuxos individuais (N) para força no corpo (N, 2) + torque (N m)."""
        force_body_xy = np.asarray(force_body_xy, dtype=float)
        thrusts = (2.0 / 3.0) * (self.thrust_dirs_body[:2, :].T @ force_body_xy)
        thrusts = thrusts + torque_z / (3.0 * self.arm_radius)
        # Saturação por hélice — é isso que o controlador real enfrentará.
        return np.clip(thrusts, -self.params.max_thrust, self.params.max_thrust)

    def realize(
        self, force_world_des, torque_des: float, heading: float
    ) -> ActuatorOutput:
        """Converte comando desejado (mundo) no wrench real após saturação.

        O comando de força é girado para o corpo (que gira com o heading),
        alocado nas hélices, saturado, e devolvido ao referencial do mundo.
        """
        force_world_des = np.asarray(force_world_des, dtype=float)
        rot = self.rotation_z(heading)
        force_body = rot.T @ force_world_des
        thrusts = self.allocate(force_body[:2], torque_des)
        force_world = rot @ (self.thrust_dirs_body @ thrusts)
        torque = self.arm_radius * float(thrusts.sum())
        return ActuatorOutput(force_world=force_world, torque_heading=torque, thrusts=thrusts)

    def thrust_to_pwm_us(self, thrusts) -> np.ndarray:
        """Converte empuxos (N) no PWM bidirecional de cada motor (us).

        f = 0 -> 1500 us (parado); f = +f_max -> 2000 us; f = -f_max ->
        1000 us; linear em entre, clipado na faixa [1000, 2000].
        """
        thrusts = np.asarray(thrusts, dtype=float)
        pwm = self.params.pwm_neutral_us + self.params.pwm_span_us * (
            thrusts / self.params.max_thrust
        )
        lo = self.params.pwm_neutral_us - self.params.pwm_span_us
        hi = self.params.pwm_neutral_us + self.params.pwm_span_us
        return np.clip(pwm, lo, hi)

    def geometry_world(self, heading: float, sphere_center) -> tuple:
        """Geometria para visualização: (montagens, direções) no mundo (3, N)."""
        rot = self.rotation_z(heading)
        mounts = sphere_center[:, None] + rot @ self.mounts_body
        tangents = rot @ self.thrust_dirs_body
        return mounts, tangents
