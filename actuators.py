"""Atuadores: 3 motores bidirecionais com hélices tangentes, modelo por motor.

Paridade total com o firmware do STM32 (``firmware/main.cpp``): o mesmo modelo
de atuador roda no simulador, no twin e no firmware, com parâmetros carregados
do MESMO arquivo de calibração JSON (ver :func:`ThrusterCluster.load_calibration`).

Referenciais e convenções:

- **Corpo** = referencial do sensor (MPU/câmeras), que gira com o heading ψ.
- Motor **a** (BF, PA0) empurra no **+X do corpo** (verificado fisicamente);
  b (AS, PA15) e c (TR, PA1) a ±120°: ``t_a=(1,0)``, ``t_b=(−1/2,+√3/2)``,
  ``t_c=(−1/2,−√3/2)``.
- ``yaw_sign`` de cada motor: +1 se empuxo positivo gera torque de heading
  positivo. Default geométrico +1 (montagem tangente coerente) — **a calibrar
  pelo wizard**: ESC soldado invertido ou hélice montada espelhada inverte o
  sinal, e nada no modelo assume que os três foram montados iguais.

Modelo de empuxo por motor (curva quadrática com zona morta, como o firmware):

    F(u) = kf · (u − dz)² · sinal(u)    para |u| > dz;  0 caso contrário
    u(F) = dz + sqrt(|F|/kf)           (inversa exata)

com ``kf`` distinto para frente/ré (hélice de passo fixo tem eficiência
diferente em cada sentido). Os defaults (18 N @ 500 µs, ré 60%, dz 30 µs) são
**PROVISÓRIOS** até o wizard medir por motor na bancada.

Realização (espelha o firmware):

1. força desejada no MUNDO → referencial do corpo (Rz(−ψ)) — corrige o
   firmware antigo, que aplicava a força direto no corpo;
2. alocação de força: ``f_i = (2/3)(t_i·F_b)`` (torque zero — no caso ideal a
   distribuição de yaw dos 3 motores soma zero em força);
3. reescala proporcional λ se algum |f_i| exceder a reserva anti-balanço;
4. conversão N → µs por motor (curva + zona morta próprias);
5. distribuição de yaw: ``u_i += yaw_sign_i · yaw_us`` (yaw_us = µs por
   motor, sinal do controlador);
6. clamp de segurança ±250 µs e **atraso de atuação de 1ª ordem** (~120 ms,
   spin-up de ESC+hélice — o firmware não precisa modelar, o hardware já tem).

O wrench REALizado (que alimenta a planta) é recalculado dos µs efetivos
depois do lag e do clamp — se houver saturação, o controlador sente exatamente
o que o hardware entregaria.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

F_MIN_N = 0.01  # N, limiar anti-chattering (abaixo disso, µs = neutro)
CALIB_VERSION = 1


@dataclass(frozen=True)
class ActuatorParams:
    """Parâmetros do cluster (defaults do hardware; os por-motor vão em MotorCalib).

    O clamp usa a faixa COMPLETA do ESC (±500 µs, calibração de faixa feita)
    — decisão de operação da equipe. Verificar corrente/térmica na bancada na
    primeira sessão com autoridade alta.
    """

    pwm_neutral_us: float = 1500.0
    pwm_limit_us: float = 500.0  # faixa completa do ESC: 1500 ± 500
    swing_reserve_us: float = 490.0  # reserva de µs para o anti-balanço
    yaw_reserve_us: float = 300.0  # reserva de µs para o heading
    f_at_500us: float = 18.0  # N @ ±500 µs — PROVISÓRIO (wizard calibra)
    reverse_ratio: float = 0.60  # kf_rev/kf_fwd — PROVISÓRIO
    deadzone_us: float = 30.0  # zona morta do ESC — PROVISÓRIO
    arm_radius: float = 0.70  # m, eixo central → propulsores
    lag_tau: float = 0.12  # s, atraso de 1ª ordem ESC+hélice


def _geometric_thrust_dirs() -> np.ndarray:
    """Direções de empuxo unitárias no corpo (colunas): a=+X, b/c a ±120°."""
    return np.array(
        [
            [1.0, -0.5, -0.5],
            [0.0, math.sqrt(3.0) / 2.0, -math.sqrt(3.0) / 2.0],
        ]
    )  # (2, 3)


@dataclass
class MotorCalib:
    """Calibração de UM motor — tudo aqui é corrigível pelo wizard."""

    label: str
    thrust_dir: Tuple[float, float]  # vetor unitário no corpo
    yaw_sign: float = 1.0  # +1: empuxo positivo gera torque de heading +
    kf_fwd: float = 0.0  # N/µs² (preenchido pelo default do cluster)
    kf_rev: float = 0.0
    deadzone_us: float = 30.0

    def __post_init__(self) -> None:
        d = np.asarray(self.thrust_dir, dtype=float)
        norm = float(np.linalg.norm(d))
        if norm < 1e-9:
            raise ValueError(f"motor {self.label}: thrust_dir nulo")
        self.thrust_dir = (d / norm).tolist()
        if self.yaw_sign not in (1.0, -1.0):
            raise ValueError(f"motor {self.label}: yaw_sign deve ser ±1")
        if self.kf_fwd <= 0 or self.kf_rev <= 0 or self.deadzone_us < 0:
            raise ValueError(f"motor {self.label}: calibração inválida")

    def to_dict(self) -> dict:
        return {
            "label": self.label,
            "thrust_dir": list(self.thrust_dir),
            "yaw_sign": self.yaw_sign,
            "kf_fwd": self.kf_fwd,
            "kf_rev": self.kf_rev,
            "deadzone_us": self.deadzone_us,
        }

    @staticmethod
    def from_dict(data: dict) -> "MotorCalib":
        return MotorCalib(
            label=str(data["label"]),
            thrust_dir=tuple(float(v) for v in data["thrust_dir"]),
            yaw_sign=float(data["yaw_sign"]),
            kf_fwd=float(data["kf_fwd"]),
            kf_rev=float(data["kf_rev"]),
            deadzone_us=float(data["deadzone_us"]),
        )


def default_motors(params: ActuatorParams) -> List[MotorCalib]:
    """Calibração default (geométrica + curva provisória) para os 3 motores."""
    kf_fwd = params.f_at_500us / (500.0 - params.deadzone_us) ** 2
    kf_rev = kf_fwd * params.reverse_ratio
    dirs = _geometric_thrust_dirs()
    return [
        MotorCalib(
            label=label,
            thrust_dir=tuple(dirs[:, i]),
            yaw_sign=1.0,
            kf_fwd=kf_fwd,
            kf_rev=kf_rev,
            deadzone_us=params.deadzone_us,
        )
        for i, label in enumerate(("a", "b", "c"))
    ]


@dataclass
class ActuatorOutput:
    """Resultado real dos atuadores após alocação, saturação e lag."""

    force_world: np.ndarray  # (3,), N
    torque_heading: float  # N·m
    thrusts: np.ndarray  # (3,), N — empuxo individual [a, b, c]
    pwm_us: np.ndarray  # (3,), µs absolutos EFETIVOS (pós-lag/clamp)


class ThrusterCluster:
    N_THRUSTERS = 3
    LABELS = ("a", "b", "c")

    def __init__(
        self,
        params: Optional[ActuatorParams] = None,
        motors: Optional[List[MotorCalib]] = None,
    ):
        self.params = params or ActuatorParams()
        if motors is None:
            motors = default_motors(self.params)
        if len(motors) != self.N_THRUSTERS:
            raise ValueError("esperados 3 motores")
        self.motors = motors

        self.T = np.array([m.thrust_dir for m in motors]).T  # (2, 3)
        self.yaw_signs = np.array([m.yaw_sign for m in motors])  # (3,)
        self.kf_fwd = np.array([m.kf_fwd for m in motors])
        self.kf_rev = np.array([m.kf_rev for m in motors])
        self.dz = np.array([m.deadzone_us for m in motors])

        # Estado do atraso de atuação (µs de offset em relação ao neutro).
        self._u_lag = np.zeros(self.N_THRUSTERS)

    # ------------------------------------------------------------ calibração

    @classmethod
    def from_calibration(cls, path: str) -> "ThrusterCluster":
        with open(path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
        if data.get("version") != CALIB_VERSION:
            raise ValueError(f"versão de calibração não suportada em {path}")
        params = ActuatorParams(
            pwm_limit_us=float(data["pwm_limit_us"]),
            swing_reserve_us=float(data["swing_reserve_us"]),
            yaw_reserve_us=float(data["yaw_reserve_us"]),
            arm_radius=float(data["arm_radius_m"]),
            lag_tau=float(data["lag_tau_s"]),
        )
        motors = [MotorCalib.from_dict(m) for m in data["motors"]]
        return cls(params, motors)

    def save_calibration(self, path: str) -> None:
        data = {
            "version": CALIB_VERSION,
            "pwm_limit_us": self.params.pwm_limit_us,
            "swing_reserve_us": self.params.swing_reserve_us,
            "yaw_reserve_us": self.params.yaw_reserve_us,
            "arm_radius_m": self.params.arm_radius,
            "lag_tau_s": self.params.lag_tau,
            "motors": [m.to_dict() for m in self.motors],
        }
        os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(data, fh, indent=2)

    def apply_motor_calib(self, index: int, calib: MotorCalib) -> None:
        """Atualiza a calibração de um motor em runtime (wizard)."""
        self.motors[index] = calib
        self.T[:, index] = calib.thrust_dir
        self.yaw_signs[index] = calib.yaw_sign
        self.kf_fwd[index] = calib.kf_fwd
        self.kf_rev[index] = calib.kf_rev
        self.dz[index] = calib.deadzone_us

    def reset(self) -> None:
        """Zera o estado do lag (µs de volta ao neutro)."""
        self._u_lag[:] = 0.0

    # ----------------------------------------------------------- capacidades

    @property
    def arm_radius(self) -> float:
        return self.params.arm_radius

    @property
    def max_thrust(self) -> float:
        """Empuxo máximo por motor (frente, no clamp de segurança)."""
        return float(np.max(self.kf_fwd * (self.params.pwm_limit_us - self.dz) ** 2))

    @property
    def max_swing_force(self) -> float:
        """Maior força por motor dentro da reserva anti-balanço (pior caso)."""
        cap = np.minimum(
            self.kf_fwd * (self.params.swing_reserve_us - self.dz) ** 2,
            self.kf_rev * (self.params.swing_reserve_us - self.dz) ** 2,
        )
        return float(np.min(cap))

    @property
    def max_net_force(self) -> float:
        """Envelope exato de força líquida (alocação sem saturação): 1,5×/motor."""
        return 1.5 * self.max_swing_force

    @property
    def max_torque(self) -> float:
        """Torque de heading máximo (todos os motores no mesmo sentido)."""
        cap = np.minimum(
            self.kf_fwd * (self.params.pwm_limit_us - self.dz) ** 2,
            self.kf_rev * (self.params.pwm_limit_us - self.dz) ** 2,
        )
        return self.arm_radius * float(np.sum(cap))

    # ------------------------------------------------------ conversões µs↔N

    def us_from_force(self, thrusts) -> np.ndarray:
        """Força (N) → offset de µs por motor (inversa exata da curva)."""
        f = np.asarray(thrusts, dtype=float)
        kf = np.where(f >= 0.0, self.kf_fwd, self.kf_rev)
        us = np.zeros_like(f)
        mask = np.abs(f) >= F_MIN_N
        us[mask] = np.sign(f[mask]) * (
            self.dz[mask] + np.sqrt(np.abs(f[mask]) / kf[mask])
        )
        return us

    def force_from_us(self, u_offsets) -> np.ndarray:
        """Offset de µs → força (N) por motor (curva com zona morta)."""
        u = np.asarray(u_offsets, dtype=float)
        kf = np.where(u >= 0.0, self.kf_fwd, self.kf_rev)
        f = np.zeros_like(u)
        mask = np.abs(u) > self.dz
        f[mask] = np.sign(u[mask]) * kf[mask] * (np.abs(u[mask]) - self.dz[mask]) ** 2
        return f

    def thrust_to_pwm_us(self, thrusts) -> np.ndarray:
        """Empuxos (N) → PWM absoluto (µs), clipado na faixa de segurança."""
        u = self.us_from_force(thrusts)
        return self.params.pwm_neutral_us + np.clip(
            u, -self.params.pwm_limit_us, self.params.pwm_limit_us
        )

    @staticmethod
    def rotation_z(psi: float) -> np.ndarray:
        c, s = math.cos(psi), math.sin(psi)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    # ---------------------------------------------------------- realização

    def allocate_swing_us(self, force_body_xy) -> np.ndarray:
        """Força desejada no corpo (N, 2) → offset de µs por motor (reserva)."""
        f_des = (2.0 / 3.0) * (self.T.T @ np.asarray(force_body_xy, dtype=float))
        # Reescala proporcional se exceder a reserva (prioridade: direção).
        cap_fwd = self.kf_fwd * (self.params.swing_reserve_us - self.dz) ** 2
        cap_rev = self.kf_rev * (self.params.swing_reserve_us - self.dz) ** 2
        lam = 1.0
        for i in range(self.N_THRUSTERS):
            cap = cap_fwd[i] if f_des[i] >= 0 else cap_rev[i]
            if abs(f_des[i]) > 1e-9:
                lam = min(lam, cap / abs(f_des[i]))
        return self.us_from_force(f_des * lam)

    def _apply_lag_and_clamp(self, u_cmd: np.ndarray, dt: float) -> np.ndarray:
        """Atraso de 1ª ordem + clamp de segurança por motor."""
        if dt <= 0:
            raise ValueError("dt deve ser positivo")
        alpha = dt / (dt + self.params.lag_tau)
        self._u_lag += alpha * (u_cmd - self._u_lag)
        return np.clip(
            self._u_lag, -self.params.pwm_limit_us, self.params.pwm_limit_us
        )

    def realize(
        self, force_world_des, yaw_us_des: float, heading: float, dt: float
    ) -> ActuatorOutput:
        """Comando desejado (força no mundo + yaw em µs/motor) → wrench real.

        Espelha o caminho do firmware: rotação mundo→corpo, alocação de força
        com reserva, distribuição de yaw com sinais calibrados, clamp, lag.
        O wrench realizado é recalculado dos µs EFETIVOS (pós-lag/clamp).
        """
        force_world_des = np.asarray(force_world_des, dtype=float)
        rot = self.rotation_z(heading)
        force_body = rot.T @ force_world_des

        u_swing = self.allocate_swing_us(force_body[:2])
        u_cmd = u_swing + self.yaw_signs * float(yaw_us_des)
        u_eff = self._apply_lag_and_clamp(u_cmd, dt)

        thrusts = self.force_from_us(u_eff)
        force_body_actual = self.T @ thrusts  # (2,)
        force_world_actual = rot @ np.array(
            [force_body_actual[0], force_body_actual[1], 0.0]
        )
        torque = self.arm_radius * float(self.yaw_signs @ thrusts)
        return ActuatorOutput(
            force_world=force_world_actual,
            torque_heading=torque,
            thrusts=thrusts,
            pwm_us=self.params.pwm_neutral_us + u_eff,
        )

    def command_raw(self, u_offsets, heading: float, dt: float) -> ActuatorOutput:
        """Comando bruto de µs por motor (wizard/testes), sem alocação."""
        u_eff = self._apply_lag_and_clamp(
            np.asarray(u_offsets, dtype=float), dt
        )
        thrusts = self.force_from_us(u_eff)
        rot = self.rotation_z(heading)
        force_body = self.T @ thrusts  # (2,)
        force_world = rot @ np.array([force_body[0], force_body[1], 0.0])
        torque = self.arm_radius * float(self.yaw_signs @ thrusts)
        return ActuatorOutput(
            force_world=force_world,
            torque_heading=torque,
            thrusts=thrusts,
            pwm_us=self.params.pwm_neutral_us + u_eff,
        )

    # ---------------------------------------------------------- visualização

    def geometry_world(self, heading: float, sphere_center) -> tuple:
        """Geometria para visualização: (montagens, direções) no mundo (3, N)."""
        rot = self.rotation_z(heading)
        dirs = np.vstack(
            [self.T, np.zeros((1, self.N_THRUSTERS))]
        )  # (3, N) direções de empuxo
        # Montagem: tangente → braço a 90° "atrás" da direção de empuxo.
        mounts_body = np.vstack(
            [self.arm_radius * dirs[1], -self.arm_radius * dirs[0], np.zeros(self.N_THRUSTERS)]
        )
        center = np.asarray(sphere_center, dtype=float)
        mounts = center[:, None] + rot @ mounts_body
        tangents = rot @ dirs
        return mounts, tangents
