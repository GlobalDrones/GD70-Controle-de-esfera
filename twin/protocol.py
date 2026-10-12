"""Protocolo do firmware v2 (firmware/main.cpp) — parsing/construção compartilhada.

Telemetria (25 Hz, linha CSV):

    TEL,t_ms,yaw,bearing,yaw_des,rate_z,sx,sy,wx,wy,pwm_a,pwm_b,pwm_c,L,D,flags

ângulos em graus, taxas em °/s, PWM em µs, L/D em metros (D = −1 sem visão).
flags (bits): 1=yaw_on 2=swing_on 4=calib_ok 8=visão fresca 16=wizard.

Ganhos (comando G/R), mesma ordem no firmware e no twin:

    0 vel_kp, 1 vel_ki, 2 vel_kd, 3 head_angle_kp,
    4 head_decel, 5 head_rate_limit, 6 head_rate_kp_us, 7 head_rate_ki_us
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

GAIN_NAMES = (
    "vel_kp", "vel_ki", "vel_kd", "head_angle_kp",
    "head_decel", "head_rate_limit", "head_rate_kp_us", "head_rate_ki_us",
)

FLAG_YAW_ON = 1
FLAG_SWING_ON = 2
FLAG_CALIB_OK = 4
FLAG_VISION = 8
FLAG_WIZARD = 16


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
        if len(parts) != 16:
            return None
        try:
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
    return (f"M{motor} {tx:.6g} {ty:.6g} {yaw_sign:+.0f} "
            f"{kf_fwd:.6g} {kf_rev:.6g} {dz_us:.6g}")


def cmd_length(L_m: float) -> str:
    return f"L{L_m:.3f}"


def cmd_yaw_step(delta_deg: float) -> str:
    """Degrau RELATIVO de yaw (o firmware re-zera e aplica o ângulo)."""
    return f"{delta_deg:.3f}"
