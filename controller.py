"""Controle em malha fechada do sistema Esfera: 3 graus de liberdade.

Loops independentes com objetivos distintos (paridade com o firmware):

- **X e Y (translação):** PIDs sobre a *velocidade* com setpoint 0 —
  amortecimento ativo. A restauração gravitacional do pêndulo cuida da
  posição; o controlador remove a energia de oscilação. No hardware o sinal
  é a velocidade estimada ``v̂ = L·θ̇_mundo`` (mesmo significado de ganho).
- **Heading:** cascata apontamento → taxa → **µs por motor**. O erro de
  apontamento vem da VISÃO (bearing do cabo da linha no referencial da
  câmera/corpo — câmeras fixas ao sensor, giram junto) quando disponível,
  ou do setpoint interno vs. heading integrado do gyro (com re-zero
  periódico). A malha de taxa é um PI em µs com anti-windup — o mesmo
  domínio do firmware, para os ganhos terem o mesmo valor nos dois lados.

O controlador comanda força desejada no referencial do MUNDO (a conversão
para o corpo, que gira com o heading, acontece no cluster de atuadores —
corrige o firmware antigo, que aplicava a força direto no corpo).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Optional

import numpy as np

from actuators import ThrusterCluster
from pid_controller import PIDController
from simulation import PlantState, wrap_to_pi

# Inércia de referência para a qual os ganhos default foram sintonizados
# (12 kg, raio 0,60 m => (2/5) m r^2). Usada por gains_for_inertia().
REFERENCE_INERTIA_Z = 1.728  # kg m^2


@dataclass(frozen=True)
class ControllerGains:
    """Ganhos sintonizados para o hardware real (12 kg, I = 1,728 kg·m²).

    A resposta em malha fechada depende de kp/m (translação) e kp·k_u/I
    (heading, com k_u = sensibilidade µs→torque dos atuadores); ao mudar
    massa/inércia, escale junto (ver :func:`gains_for_inertia`).
    """

    # Loops de velocidade (X e Y): dominante-P (amortecimento).
    vel_kp: float = 8.0  # N por (m/s)
    vel_ki: float = 0.4
    vel_kd: float = 0.0
    # Heading em cascata: erro angular -> PERFIL de taxa (com frenagem
    # embutida) -> PI de taxa em µs por motor (domínio do firmware).
    head_angle_kp: float = 2.5  # (rad/s) por rad de erro angular
    head_decel: float = 0.5  # rad/s², desaceleração do perfil (≤ autoridade)
    head_rate_limit: float = 1.5  # rad/s, taxa máxima de curva
    head_rate_kp_us: float = 900.0  # µs por (rad/s)
    head_rate_ki_us: float = 2000.0  # µs por rad


def gains_for_inertia(inertia_z: float) -> ControllerGains:
    """Ganhos default com o loop de taxa do heading rescalado por I/I_ref.

    Mantém a dinâmica normalizada do heading quando a inércia difere da de
    referência (valor medido no hardware, raio/massa diferentes etc.). O
    perfil de frenagem escala ao contrário (a_max = tau_max/I).
    """
    ratio = inertia_z / REFERENCE_INERTIA_Z
    g = ControllerGains()
    return replace(
        g,
        head_rate_kp_us=g.head_rate_kp_us * ratio,
        head_rate_ki_us=g.head_rate_ki_us * ratio,
        head_decel=g.head_decel / ratio,
    )


@dataclass
class ControlCommand:
    force_world: np.ndarray  # (3,), N — desejada
    yaw_us: float  # µs por motor — desejado (distribuição no cluster)
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
        self._cluster = cluster
        self._heading_setpoint = 0.0
        # Loops podem ser ligados/desligados independentemente (para testar
        # heading e controle pendular separadamente, ou soltar a esfera).
        self.xy_enabled = True
        self.heading_enabled = True
        # Erro de apontamento vindo da visão (rad, no corpo): quando ativo,
        # substitui (setpoint − yaw_gyro). None = usar setpoint interno.
        self._vision_pointing_error: Optional[float] = None
        # Feedforward torcional da corda (default 0 — hardware tem heading
        # livre; parâmetro mantido para experimentos).
        self._spring_stiffness = rope_torsional_stiffness
        self._spring_damping = rope_torsional_damping

        force_axis_limit = 0.8 * cluster.max_net_force
        self.pid_vx = PIDController(
            gains.vel_kp, gains.vel_ki, gains.vel_kd, output_limit=force_axis_limit
        )
        self.pid_vy = PIDController(
            gains.vel_kp, gains.vel_ki, gains.vel_kd, output_limit=force_axis_limit
        )
        self.pid_heading_rate = PIDController(
            gains.head_rate_kp_us,
            gains.head_rate_ki_us,
            0.0,
            output_limit=cluster.params.yaw_reserve_us,
        )

    # ------------------------------------------------------------ setpoints

    def set_heading(self, heading_deg: float) -> None:
        """Define a direção desejada (graus; atualizável em runtime)."""
        self._heading_setpoint = math.radians(heading_deg)
        self._vision_pointing_error = None
        self.pid_heading_rate.reset()

    def set_pointing_error(self, bearing_deg: float) -> None:
        """Recebe o bearing da visão (graus, no corpo) como erro direto.

        As câmeras estão fixas ao sensor: o bearing do alvo no referencial da
        câmera É o erro de apontamento (sem depender do yaw integrado, que
        deriva). Enquanto a visão estiver ativa, o setpoint interno é
        continuamente corrigido para manter o log coerente.
        """
        error = math.radians(bearing_deg)
        self._heading_setpoint = wrap_to_pi(self._heading_setpoint + error)
        self._vision_pointing_error = error

    @property
    def heading_setpoint(self) -> float:
        return self._heading_setpoint

    # --------------------------------------------------------------- toggles

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
        """Troca os ganhos em runtime (ajuste ao vivo na simulação/twin)."""
        self.gains = gains
        for pid, (kp, ki, kd) in (
            (self.pid_vx, (gains.vel_kp, gains.vel_ki, gains.vel_kd)),
            (self.pid_vy, (gains.vel_kp, gains.vel_ki, gains.vel_kd)),
            (self.pid_heading_rate, (gains.head_rate_kp_us, gains.head_rate_ki_us, 0.0)),
        ):
            pid.kp, pid.ki, pid.kd = kp, ki, kd

    # ---------------------------------------------------------------- update

    def update(self, state: PlantState, dt: float) -> ControlCommand:
        if self.xy_enabled:
            force_x = self.pid_vx.update(state.velocity[0], dt)
            force_y = self.pid_vy.update(state.velocity[1], dt)
        else:
            force_x = force_y = 0.0

        if self.heading_enabled:
            # Erro de apontamento: da visão se ativa; senão setpoint interno
            # vs. heading integrado do gyro.
            if self._vision_pointing_error is not None:
                angle_error = self._vision_pointing_error
            else:
                angle_error = wrap_to_pi(self._heading_setpoint - state.heading)

            # Cascata: erro angular -> PERFIL de taxa (frenagem embutida:
            # a distância de frenagem sqrt(2·a·|e|) nunca excede o erro
            # restante) -> PI de taxa em µs (domínio do firmware).
            abs_err = abs(angle_error)
            rate_mag = min(
                self.gains.head_angle_kp * abs_err,
                math.sqrt(2.0 * self.gains.head_decel * abs_err),
                self.gains.head_rate_limit,
            )
            rate_setpoint = math.copysign(rate_mag, angle_error)
            self.pid_heading_rate.set_setpoint(rate_setpoint)
            yaw_us = self.pid_heading_rate.update(state.heading_rate, dt)
            # Feedforward torcional (0 no hardware — heading livre).
            if self._spring_stiffness or self._spring_damping:
                k_u = self._torque_per_us()
                if k_u > 1e-9:
                    torque_ff = (
                        self._spring_stiffness * state.heading
                        + self._spring_damping * state.heading_rate
                    )
                    yaw_us += torque_ff / k_u
        else:
            yaw_us = 0.0

        return ControlCommand(
            force_world=np.array([force_x, force_y, 0.0]),
            yaw_us=yaw_us,
            heading_setpoint=self._heading_setpoint,
        )

    def _torque_per_us(self) -> float:
        """Sensibilidade µs→torque perto da reserva de yaw (N·m/µs)."""
        p = self._cluster.params
        kf = min(m.kf_fwd for m in self._cluster.motors)
        u = max(p.yaw_reserve_us - 30.0, 1.0)
        return self._cluster.arm_radius * 3.0 * 2.0 * kf * u
