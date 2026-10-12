"""Controlador PID com anti-windup, limite de saída e suporte a ângulos.

Melhorias sobre a versão anterior:

- limite de saída com anti-windup por clamping (a integral só cresce quando
  não empurra a saída contra o limite saturado);
- derivada calculada sobre a medição (evita "derivative kick" quando o
  setpoint muda em degrau);
- modo ``angle_wrap`` para variáveis angulares (o erro é normalizado para
  [-pi, pi], então 350°->10° vira +20° e não -340°);
- setpoint atualizável em runtime via :meth:`PIDController.set_setpoint`.
"""

from __future__ import annotations

import math
from typing import Optional


def wrap_to_pi(angle: float) -> float:
    """Normaliza um ângulo para o intervalo [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class PIDController:
    def __init__(
        self,
        kp: float,
        ki: float = 0.0,
        kd: float = 0.0,
        setpoint: float = 0.0,
        output_limit: Optional[float] = None,
        angle_wrap: bool = False,
    ):
        if kp < 0 or ki < 0 or kd < 0:
            raise ValueError("ganhos do PID não podem ser negativos")
        if output_limit is not None and output_limit <= 0:
            raise ValueError("output_limit deve ser positivo")
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.output_limit = output_limit
        self.angle_wrap = angle_wrap

        self.integral = 0.0
        self.error = 0.0
        self.derivative = 0.0
        self.output = 0.0
        self._last_measurement: Optional[float] = None

    def set_setpoint(self, value: float) -> None:
        """Atualiza o setpoint em runtime (a derivada segue sobre a medida)."""
        self.setpoint = value

    def reset(self) -> None:
        self.integral = 0.0
        self.error = 0.0
        self.derivative = 0.0
        self.output = 0.0
        self._last_measurement = None

    def update(self, measurement: float, dt: float) -> float:
        if dt <= 0:
            raise ValueError("dt deve ser positivo")

        error = self.setpoint - measurement
        if self.angle_wrap:
            error = wrap_to_pi(error)

        if self._last_measurement is None:
            derivative = 0.0
        else:
            # Derivada do erro via medição: -d(measurement)/dt (setpoint não
            # entra, evitando pico no instante de um degrau).
            derivative = -(measurement - self._last_measurement) / dt

        p_term = self.kp * error
        d_term = self.kd * derivative
        i_increment = self.ki * error * dt

        # Anti-windup por clamping: integra apenas se a integração não
        # empurra a saída além do limite (ou se puxa de volta para dentro).
        if self.output_limit is None:
            self.integral += error * dt
        else:
            out_before = p_term + d_term + self.ki * self.integral
            saturating = abs(out_before + i_increment) > self.output_limit
            reduces = out_before * i_increment < 0
            if not saturating or reduces:
                self.integral += error * dt

        output = p_term + d_term + self.ki * self.integral
        if self.output_limit is not None:
            output = max(-self.output_limit, min(self.output_limit, output))

        self.error = error
        self.derivative = derivative
        self.output = output
        self._last_measurement = measurement
        return output
