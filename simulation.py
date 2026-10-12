"""Plantas físicas do sistema Esfera.

Modela um pêndulo esférico: uma esfera suspensa por uma corda rígida de
comprimento ``L``, presa a um pino fixo na origem do referencial do mundo
(+z para cima, esfera pendendo em z < 0). O efeito líquido dos 3 motores com
hélices tangentes (ver ``actuators.py``) chega aqui como uma força no
referencial do mundo e um torque de heading.

Duas plantas com a mesma interface ``step(force, torque, dt)``:

- :class:`SphericalPendulumPlant` — dinâmica não-linear completa (válida para
  grandes ângulos), integrada em coordenadas cartesianas com a tensão da corda
  resolvida por projeção da restrição ``|r| = L``.
- :class:`LinearizedPendulumPlant` — linearização para pequenos ângulos: dois
  pêndulos desacoplados em X e Y + o mesmo heading. Serve para sintonia dos
  controladores.

Hipóteses (detalhadas no README):

- A esfera é um ponto material para a translação; o empuxo das hélices age no
  centro de massa.
- O heading é um GDL separado: rotação em torno do eixo vertical que passa
  pelo pino. A torção da corda exerce mola ``-k_t * psi`` e amortecimento
  ``-b_t * psi_dot`` relativos ao pino (o pino não gira).
- Arrasto do ar e demais perdas: ignorados.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


def wrap_to_pi(angle: float) -> float:
    """Normaliza um ângulo para o intervalo [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class SphereParams:
    """Parâmetros físicos da esfera (valores do hardware real).

    A massa total consolida esfera + estrutura que segura os motores + os
    próprios motores e hélices. A estrutura está majoritariamente ACIMA do
    CG (perto do eixo vertical de rotação do heading) e os motores/hélices,
    no equador, são leves — logo não dominam o momento de inércia.
    """

    mass: float = 12.0  # kg
    radius: float = 0.35  # m
    # Momento de inércia em torno do eixo vertical (kg m^2). Se None, usa a
    # estimativa de esfera maciça (2/5) m r^2. É uma ESTIMATIVA a calibrar
    # por teste de oscilação livre no heading — a distribuição real (massa
    # concentrada perto do eixo acima do CG + motores leves no equador) pode
    # ficar abaixo ou acima desse valor.
    inertia_z: Optional[float] = None

    def __post_init__(self) -> None:
        if self.inertia_z is None:
            object.__setattr__(self, "inertia_z", 0.4 * self.mass * self.radius**2)
        if self.mass <= 0 or self.radius <= 0:
            raise ValueError("massa e raio da esfera devem ser positivos")


@dataclass(frozen=True)
class RopeParams:
    """Parâmetros da corda rígida (tratada como haste sem massa).

    ``length`` é a distância pino -> CG efetivo do corpo móvel; é variável
    no sistema real (default 1 m) e pode ser sobrescrita via CLI/parâmetro.
    """

    length: float = 1.0  # m
    torsional_stiffness: float = 0.05  # N m / rad (estimativa a calibrar)
    torsional_damping: float = 0.01  # N m s / rad (estimativa a calibrar)

    def __post_init__(self) -> None:
        if self.length <= 0:
            raise ValueError("comprimento da corda deve ser positivo")


@dataclass
class PlantState:
    """Estado completo da planta.

    ``position``/``velocity`` são relativos ao pino (origem), no referencial
    do mundo; ``heading`` é a rotação da esfera em torno do eixo vertical.
    """

    position: np.ndarray  # (x, y, z), m
    velocity: np.ndarray  # (vx, vy, vz), m/s
    heading: float  # rad
    heading_rate: float  # rad/s


class PendulumPlant:
    """Interface comum e dinâmica de heading compartilhada."""

    def __init__(self, sphere: SphereParams, rope: RopeParams, gravity: float = 9.81):
        self.sphere = sphere
        self.rope = rope
        self.gravity = gravity
        self._g_vec = np.array([0.0, 0.0, -gravity])
        self._state = self._default_state()

    # -- interface -----------------------------------------------------------

    def step(self, force_world, torque_heading: float, dt: float) -> PlantState:
        raise NotImplementedError

    def read_state(self) -> PlantState:
        return self._state

    def reset(self, state: PlantState) -> None:
        self._state = state

    def tilt_angles(self, state: Optional[PlantState] = None) -> Tuple[float, float]:
        """Inclinações (theta_x, theta_y) em rad, relativas à vertical."""
        raise NotImplementedError

    # -- dinâmica de heading -------------------------------------------------
    # I_psi * psi_ddot = tau - k_t * psi - b_t * psi_dot
    # (a corda age como mola torcional referenciada ao pino)

    def _step_heading(self, torque_heading: float, dt: float) -> None:
        psi = self._state.heading
        omega = self._state.heading_rate
        spring = (
            -self.rope.torsional_stiffness * psi
            - self.rope.torsional_damping * omega
        )
        alpha = (torque_heading + spring) / self.sphere.inertia_z
        omega += alpha * dt
        psi = wrap_to_pi(psi + omega * dt)
        self._state.heading = psi
        self._state.heading_rate = omega

    def _default_state(self) -> PlantState:
        return PlantState(
            position=np.array([0.0, 0.0, -self.rope.length]),
            velocity=np.zeros(3),
            heading=0.0,
            heading_rate=0.0,
        )

    def initial_state(
        self, tilt_x: float, tilt_y: float, velocity, heading: float, heading_rate: float = 0.0
    ) -> PlantState:
        """Constrói um estado inicial a partir das inclinações (rad)."""
        sx, sy = math.sin(tilt_x), math.sin(tilt_y)
        s_sq = sx * sx + sy * sy
        if s_sq >= 1.0:
            raise ValueError("inclinação inicial deve ficar acima da horizontal")
        cz = -math.sqrt(1.0 - s_sq)
        return PlantState(
            position=self.rope.length * np.array([sx, sy, cz]),
            velocity=np.asarray(velocity, dtype=float).copy(),
            heading=wrap_to_pi(heading),
            heading_rate=heading_rate,
        )


class SphericalPendulumPlant(PendulumPlant):
    """Pêndulo esférico não-linear, integrado com RATTLE.

    O centro de massa evolui em coordenadas cartesianas sob gravidade + força
    de controle, com a restrição invariante r·r = L^2 (corda rígida) imposta
    pelo método RATTLE (Leimkuhler & Matthews): chute-meio com o multiplicador
    de velocidade do passo anterior, deriva com correção de posição (SHAKE)
    ao longo do gradiente antigo, chute-meio final e projeção da velocidade
    que zera r·v. O método é simplético: o erro de energia fica limitado e
    oscilante (~0,1% para dt = 0,01 s), sem dissipação numérica — importante
    para não "ajudar" o controlador a estabilizar a esfera.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._velocity_multiplier = 0.0  # multiplicador RATTLE entre passos

    def reset(self, state: PlantState) -> None:
        # Projeta o estado fornecido na restrição da corda.
        d = state.position / np.linalg.norm(state.position)
        position = d * self.rope.length
        velocity = state.velocity - float(state.velocity @ d) * d
        self._state = PlantState(position, velocity, state.heading, state.heading_rate)
        self._velocity_multiplier = 0.0

    def _rattle_step(self, a_ext, dt: float) -> np.ndarray:
        """Um passo RATTLE; devolve (r1, v1) e guarda o multiplicador."""
        r = self._state.position
        v = self._state.velocity
        length_sq = self.rope.length**2

        v_half = v + 0.5 * dt * (a_ext - self._velocity_multiplier * r)
        r_star = r + dt * v_half

        # SHAKE: r1 = r* + alpha * r_n com |r1| = L; raiz de menor módulo.
        b = float(r_star @ r)
        c = float(r_star @ r_star) - length_sq
        disc = b * b - c * length_sq
        alpha = (-b + math.sqrt(max(disc, 0.0))) / length_sq if disc > 0 else -c / (2.0 * b)
        r1 = r_star + alpha * r

        # Chute final + multiplicador de velocidade: zera r1·v1.
        v_star = v_half + 0.5 * dt * a_ext
        mu = float(v_star @ r1) / length_sq
        v1 = v_star - mu * r1
        self._velocity_multiplier = 2.0 * mu / dt
        return r1, v1

    def step(self, force_world, torque_heading: float, dt: float) -> PlantState:
        force = np.asarray(force_world, dtype=float)
        a_ext = self._g_vec + force / self.sphere.mass
        r1, v1 = self._rattle_step(a_ext, dt)
        self._state = PlantState(r1, v1, self._state.heading, self._state.heading_rate)
        self._step_heading(torque_heading, dt)
        return self._state

    def tilt_angles(self, state: Optional[PlantState] = None) -> Tuple[float, float]:
        s = state or self._state
        d = s.position / self.rope.length
        return math.atan2(d[0], -d[2]), math.atan2(d[1], -d[2])

    def energy(self) -> float:
        """Energia total (J): cinética + potencial gravitacional + torcional."""
        s = self._state
        kinetic = 0.5 * self.sphere.mass * float(s.velocity @ s.velocity)
        kinetic += 0.5 * self.sphere.inertia_z * s.heading_rate**2
        gravitational = self.sphere.mass * self.gravity * (s.position[2] + self.rope.length)
        torsional = 0.5 * self.rope.torsional_stiffness * s.heading**2
        return kinetic + gravitational + torsional


class LinearizedPendulumPlant(PendulumPlant):
    """Pêndulo linearizado para pequenos ângulos.

    Cada eixo horizontal vira um pêndulo linear desacoplado:

        x_ddot = -(g/L) x + Fx / m,   y_ddot = -(g/L) y + Fy / m

    com z fixo em -L. O heading segue a mesma dinâmica da planta não-linear.
    """

    def reset(self, state: PlantState) -> None:
        position = np.array([state.position[0], state.position[1], -self.rope.length])
        velocity = np.array([state.velocity[0], state.velocity[1], 0.0])
        self._state = PlantState(position, velocity, state.heading, state.heading_rate)

    def step(self, force_world, torque_heading: float, dt: float) -> PlantState:
        force = np.asarray(force_world, dtype=float)
        state = self._state
        omega_n_sq = self.gravity / self.rope.length

        ax = -omega_n_sq * state.position[0] + force[0] / self.sphere.mass
        ay = -omega_n_sq * state.position[1] + force[1] / self.sphere.mass

        vx = state.velocity[0] + ax * dt
        vy = state.velocity[1] + ay * dt
        position = np.array(
            [state.position[0] + vx * dt, state.position[1] + vy * dt, -self.rope.length]
        )
        self._state = PlantState(
            position, np.array([vx, vy, 0.0]), state.heading, state.heading_rate
        )
        self._step_heading(torque_heading, dt)
        return self._state

    def tilt_angles(self, state: Optional[PlantState] = None) -> Tuple[float, float]:
        s = state or self._state
        return s.position[0] / self.rope.length, s.position[1] / self.rope.length
