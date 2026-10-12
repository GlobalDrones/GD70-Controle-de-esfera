"""Controle em malha fechada do sistema Esfera: 3 graus de liberdade.

Loops independentes com objetivos distintos:

- **X e Y (translação):** PIDs sobre a *velocidade* com setpoint 0 —
  amortecimento ativo. A restauração gravitacional do pêndulo cuida da
  posição (levar a esfera de volta ao vertical); o controlador apenas remove
  a energia de oscilação. O termo integral é pequeno: ele age como mola
  extra (a integral do erro de velocidade é o desvio de posição).
- **Heading:** PID sobre o ângulo, com erro envolto para [-pi, pi] e setpoint
  atualizável em runtime (a direção desejada pode mudar durante a operação).

O controlador comanda força desejada no referencial do *mundo* e torque de
heading; a conversão para as hélices (referencial do corpo, que gira com o
heading, com saturação) fica em ``actuators.ThrusterCluster``. Os limites de
saída dos PIDs são derivados da capacidade real do cluster para o anti-windup
refletir a saturação física.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np

from actuators import ThrusterCluster
from pid_controller import PIDController
from simulation import PlantState, wrap_to_pi

# Inércia de referência para a qual os ganhos default foram sintonizados
# (12 kg, raio 0,35 m => (2/5) m r^2). Usada por gains_for_inertia().
REFERENCE_INERTIA_Z = 0.588  # kg m^2


@dataclass(frozen=True)
class ControllerGains:
    """Ganhos sintonizados para o hardware real (m = 12 kg, I = 0,588 kg m²).

    Regra usada no rescale: a resposta em malha fechada depende de kp/m
    (translação) e kp/I (heading); ao mudar a massa/inércia do hardware, os
    ganhos de força/torque escalam junto para manter a mesma dinâmica
    normalizada (ver :func:`gains_for_inertia`). A autoridade relativa é
    limitada (envelope 27 N vs m·g·sin θ): perturbações grandes podem saturar
    — o anti-windup cuida disso.
    """

    # Loops de velocidade (X e Y): dominante-P (amortecimento).
    vel_kp: float = 48.0  # N por (m/s)
    vel_ki: float = 2.4
    vel_kd: float = 1.2
    # Heading em cascata: P no ângulo gera uma TAXA desejada limitada
    # (curvas suaves, sem spin-up pela saturação de torque), e um PID na
    # taxa gera o torque. A mola torcional da corda é cancelada por
    # feedforward (ver EsferaController), então o PID só trata resíduos.
    head_angle_kp: float = 2.5  # (rad/s) por rad de erro angular
    head_rate_limit: float = 3.0  # rad/s, taxa máxima de curva
    head_rate_kp: float = 8.82  # N m por (rad/s)
    head_rate_ki: float = 7.35
    head_rate_kd: float = 0.0


def gains_for_inertia(inertia_z: float) -> ControllerGains:
    """Ganhos default com o loop de taxa do heading rescalado por I/I_ref.

    Mantém a dinâmica normalizada do heading (kp/I constante) quando a
    inércia difere da de referência — ex.: raio/massa diferentes ou valor
    medido no hardware real.
    """
    ratio = inertia_z / REFERENCE_INERTIA_Z
    g = ControllerGains()
    return replace(
        g, head_rate_kp=g.head_rate_kp * ratio, head_rate_ki=g.head_rate_ki * ratio
    )


@dataclass
class ControlCommand:
    force_world: np.ndarray  # (3,), N — desejada
    torque_heading: float  # N m — desejado
    heading_setpoint: float  # rad, para log/visualização


class EsferaController:
    def __init__(
        self,
        cluster: ThrusterCluster,
        gains: ControllerGains = ControllerGains(),
        rope_torsional_stiffness: float = 0.0,
        rope_torsional_damping: float = 0.0,
    ):
        self.gains = gains
        self._heading_setpoint = 0.0
        # Loops podem ser ligados/desligados independentemente (para testar
        # heading e controle pendular separadamente, ou soltar a esfera).
        self.xy_enabled = True
        self.heading_enabled = True
        # Modelo da corda conhecido pelo controlador: usado para cancelar por
        # feedforward o torque da mola torcional (o mesmo que fazer
        # compensação de gravidade em um drone). Sem isso, a integral do PID
        # de taxa teria que reconstruir esse torque a cada degrau, deixando a
        # convergência lenta.
        self._spring_stiffness = rope_torsional_stiffness
        self._spring_damping = rope_torsional_damping
        # Margem para que os dois eixos + torque não saturem o cluster ao
        # mesmo tempo: 0.7 da força máxima por eixo, 0.8 do torque máximo.
        force_axis_limit = 0.7 * cluster.max_net_force
        self.pid_vx = PIDController(
            gains.vel_kp, gains.vel_ki, gains.vel_kd, output_limit=force_axis_limit
        )
        self.pid_vy = PIDController(
            gains.vel_kp, gains.vel_ki, gains.vel_kd, output_limit=force_axis_limit
        )
        self.pid_heading_rate = PIDController(
            gains.head_rate_kp,
            gains.head_rate_ki,
            gains.head_rate_kd,
            output_limit=0.8 * cluster.max_torque,
        )

    def set_heading(self, heading_deg: float) -> None:
        """Define a direção desejada (graus; atualizável em runtime)."""
        self._heading_setpoint = math.radians(heading_deg)
        self.pid_heading_rate.reset()

    @property
    def heading_setpoint(self) -> float:
        return self._heading_setpoint

    def set_xy_enabled(self, enabled: bool) -> None:
        """Liga/desliga o controle pendular (X e Y).

        Ao religar, as integrais são zeradas para não injetar um degrau de
        força acumulado enquanto o loop estava desligado.
        """
        self.xy_enabled = enabled
        if enabled:
            self.pid_vx.reset()
            self.pid_vy.reset()

    def set_heading_enabled(self, enabled: bool) -> None:
        """Liga/desliga o controle de heading (mesma semântica do XY)."""
        self.heading_enabled = enabled
        if enabled:
            self.pid_heading_rate.reset()

    def set_gains(self, gains: ControllerGains) -> None:
        """Troca os ganhos em runtime (ajuste ao vivo na simulação)."""
        self.gains = gains
        for pid, (kp, ki, kd) in (
            (self.pid_vx, (gains.vel_kp, gains.vel_ki, gains.vel_kd)),
            (self.pid_vy, (gains.vel_kp, gains.vel_ki, gains.vel_kd)),
            (
                self.pid_heading_rate,
                (gains.head_rate_kp, gains.head_rate_ki, gains.head_rate_kd),
            ),
        ):
            pid.kp, pid.ki, pid.kd = kp, ki, kd

    def update(self, state: PlantState, dt: float) -> ControlCommand:
        if self.xy_enabled:
            force_x = self.pid_vx.update(state.velocity[0], dt)
            force_y = self.pid_vy.update(state.velocity[1], dt)
        else:
            force_x = force_y = 0.0

        if self.heading_enabled:
            # Cascata do heading: erro angular (com wrap) -> taxa desejada
            # limitada -> PID de taxa -> torque.
            angle_error = wrap_to_pi(self._heading_setpoint - state.heading)
            rate_setpoint = float(
                np.clip(
                    self.gains.head_angle_kp * angle_error,
                    -self.gains.head_rate_limit,
                    self.gains.head_rate_limit,
                )
            )
            self.pid_heading_rate.set_setpoint(rate_setpoint)
            torque = self.pid_heading_rate.update(state.heading_rate, dt)
            # Feedforward: anula mola + amortecimento torcionais conhecidos.
            torque += (
                self._spring_stiffness * state.heading
                + self._spring_damping * state.heading_rate
            )
        else:
            torque = 0.0

        return ControlCommand(
            force_world=np.array([force_x, force_y, 0.0]),
            torque_heading=torque,
            heading_setpoint=self._heading_setpoint,
        )
