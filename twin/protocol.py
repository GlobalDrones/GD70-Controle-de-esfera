"""Protocolo do firmware v2 (firmware/main.cpp) — parsing/construção compartilhada.

Telemetria (25 Hz, linha CSV):

    TEL,t_ms,yaw,bearing,yaw_des,rate_z,sx,sy,wx,wy,pwm_a,pwm_b,pwm_c,L,D,flags

ângulos em graus, taxas em °/s, PWM em µs, L/D em metros (D = −1 sem visão).
flags (bits): 1=yaw_on 2=swing_on 4=calib_ok 8=visão fresca 16=wizard
              32=MPU em falha 64=repouso no prumo 128=yaw em repouso 256=vel_mode estimador.

TEL v3 (firmware de 14/10/2026) acrescenta 5 campos ao final:
F_cmd_x, F_cmd_y [N, corpo], Tz [N·m], vhat_x, vhat_y [m/s, corpo]. O parser aceita
16 (v2, SimSTM) ou 21 campos. Desde a mesma versão sx/sy/wx/wy são do CORPO com a
convenção sx>0 = esfera deslocada para +X (motor a) — AUDITORIA_CONTROLE.md SC-01.

Ganhos (comando G/R): ``GAIN_NAMES`` abaixo é a semântica do controlador PYTHON
(simulador/twin). O firmware usa ``FIRMWARE_GAIN_NAMES`` (heading em torque, SC-09);
os índices 3..7 NÃO são compatíveis — o firmware recusa valores fora da faixa.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

GAIN_NAMES = (
    "vel_kp", "vel_ki", "vel_kd", "head_angle_kp",
    "head_decel", "head_rate_limit", "head_rate_kp_us", "head_rate_ki_us",
)

FIRMWARE_GAIN_NAMES = (
    "vel_kp", "vel_ki", "vel_kd", "yaw_kp", "yaw_ki",
    "yaw_kd", "yaw_tz_max", "yaw_int_max", "vel_mode", "d_pino_cg",
)

FLAG_YAW_ON = 1
FLAG_SWING_ON = 2
FLAG_CALIB_OK = 4
FLAG_VISION = 8
FLAG_WIZARD = 16
FLAG_MPU_FAIL = 32
FLAG_REST_PLUMB = 64
FLAG_YAW_REST = 128
FLAG_VEL_ESTIMATOR = 256


@dataclass
class TelemetryFrame:
    t_ms: float
    yaw_deg: float
    bearing_deg: float
    yaw_des_deg: float
    rate_z_dps: float
    sx_deg: float
    sy_deg: float
    wx_dps: float
    wy_dps: float
    pwm: tuple  # (a, b, c) µs
    L_m: float
    D_m: float  # −1 = sem visão
    flags: int
    # TEL v3 (opcionais; None quando a linha tem só os 16 campos da v2)
    F_cmd_x: Optional[float] = None
    F_cmd_y: Optional[float] = None
    tz_cmd: Optional[float] = None
    vhat_x: Optional[float] = None
    vhat_y: Optional[float] = None

    @property
    def yaw_on(self) -> bool:
        return bool(self.flags & FLAG_YAW_ON)

    @property
    def swing_on(self) -> bool:
        return bool(self.flags & FLAG_SWING_ON)

    @property
    def vision_fresh(self) -> bool:
        return bool(self.flags & FLAG_VISION)

    @property
    def wizard_active(self) -> bool:
        return bool(self.flags & FLAG_WIZARD)

    @property
    def tilt_mag_deg(self) -> float:
        return math.hypot(self.sx_deg, self.sy_deg)


def parse_line(line: str):
    """Parseia uma linha do STM: TEL → TelemetryFrame; GAIN → (idx, nome, valor)."""
    line = line.strip()
    if not line:
        return None
    if line.startswith("TEL,"):
        parts = line.split(",")
        if len(parts) not in (16, 21):
            return None
        try:
            extra = [float(p) for p in parts[16:21]] if len(parts) == 21 else [None] * 5
            return TelemetryFrame(
                t_ms=float(parts[1]),
                yaw_deg=float(parts[2]),
                bearing_deg=float(parts[3]),
                yaw_des_deg=float(parts[4]),
                rate_z_dps=float(parts[5]),
                sx_deg=float(parts[6]),
                sy_deg=float(parts[7]),
                wx_dps=float(parts[8]),
                wy_dps=float(parts[9]),
                pwm=(float(parts[10]), float(parts[11]), float(parts[12])),
                L_m=float(parts[13]),
                D_m=float(parts[14]),
                flags=int(parts[15]),
                F_cmd_x=extra[0], F_cmd_y=extra[1], tz_cmd=extra[2],
                vhat_x=extra[3], vhat_y=extra[4],
            )
        except ValueError:
            return None
    if line.startswith("GAIN,"):
        parts = line.split(",")
        if len(parts) != 4:
            return None
        try:
            return int(parts[1]), parts[2], float(parts[3])
        except ValueError:
            return None
    return None


# ------------------------------------------------------------- comandos


def cmd_gain(index: int, value: float) -> str:
    return f"G{index} {value:.6g}"


def cmd_vision(bearing_deg: float, D_m: float) -> str:
    return f"V{bearing_deg:.3f} {D_m:.3f}"


def cmd_wizard_pulse(motor: int, us: int, duration_ms: int) -> str:
    return f"W{motor} {us} {duration_ms}"


def cmd_motor_calib(motor: int, tx: float, ty: float, yaw_sign: float,
                    kf_fwd: float, kf_rev: float, dz_us: float) -> str:
    """Linha M (≤ 60 caracteres: o firmware descarta linhas maiores).

    O firmware ignora tx/ty (direção = geometria) e trata kf ≤ 0 / dz < 0 como
    "manter o valor atual".
    """
    return (f"M{motor} {tx:.4g} {ty:.4g} {yaw_sign:+.0f} "
            f"{kf_fwd:.4g} {kf_rev:.4g} {dz_us:.4g}")


def cmd_length(L_m: float) -> str:
    return f"L{L_m:.3f}"


def cmd_yaw_step(delta_deg: float) -> str:
    """Degrau RELATIVO de yaw (o firmware re-zera e aplica o ângulo)."""
    return f"{delta_deg:.3f}"
