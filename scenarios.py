"""Cenários de simulação: condições iniciais + programação de heading.

Cada cenário define a duração, o passo de integração, a condição inicial
(inclinações da corda e velocidades no mundo) e:

- ``heading_schedule_deg``: agenda de heading desejado — lista de
  ``(tempo_s, heading_graus)`` aplicada em runtime;
- ``force_events``: rajadas de força externa — lista de
  ``(t_inicio, t_fim, fx, fy)`` com força em N no referencial do mundo,
  aplicadas diretamente na PLANTA (o controlador só as sente via estado,
  como no sistema real).
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class InitialCondition:
    tilt_deg: tuple = (0.0, 0.0)  # inclinações iniciais (theta_x, theta_y)
    velocity: tuple = (0.0, 0.0, 0.0)  # velocidade inicial no mundo, m/s
    heading_deg: float = 0.0


@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    duration: float = 20.0  # s
    dt: float = 0.01  # s
    initial: InitialCondition = field(default_factory=InitialCondition)
    heading_schedule_deg: tuple = ()  # ((tempo_s, heading_graus), ...)
    force_events: tuple = ()  # ((t_inicio, t_fim, fx_N, fy_N), ...)


SCENARIOS = {
    "default": Scenario(
        name="default",
        description="Inclinação inicial de 15° + velocidade em Y; heading muda para 90° em t=8 s.",
        duration=20.0,
        initial=InitialCondition(tilt_deg=(15.0, 0.0), velocity=(0.0, 0.3, 0.0)),
        heading_schedule_deg=((8.0, 90.0),),
    ),
    "disturbance": Scenario(
        name="disturbance",
        description="Só translação: perturbação forte (20°/-10° + velocidades) sem mudar o heading.",
        duration=20.0,
        initial=InitialCondition(tilt_deg=(20.0, -10.0), velocity=(0.5, -0.4, 0.0)),
        heading_schedule_deg=(),
    ),
    "heading": Scenario(
        name="heading",
        description="Esfera começa em repouso; sequência de heading: 90°, -45°, 180°.",
        duration=24.0,
        initial=InitialCondition(),
        heading_schedule_deg=((2.0, 90.0), (9.0, -45.0), (16.0, 180.0)),
    ),
    "gust": Scenario(
        name="gust",
        description="Rajadas de força externa (20 N em X, depois −15 N/−10 N diagonal); heading 45° em t=6 s.",
        duration=24.0,
        initial=InitialCondition(),
        heading_schedule_deg=((6.0, 45.0),),
        force_events=((4.0, 6.0, 20.0, 0.0), (12.0, 13.0, -15.0, -10.0)),
    ),
}
