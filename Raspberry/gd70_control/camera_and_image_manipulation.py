import cv2
import numpy as np
import os
import sys
import time
import threading
from collections import deque
import serial
import subprocess
from serial_communication import *
from gpiozero import LED # Controle seguro do pino de reset na Raspberry Pi 5

# ===========================================================================
# RETIFICACAO
# ===========================================================================
def scale_intrinsics(cmtx, scale):
    m = cmtx.copy().astype(np.float64)
    m[0, 0] *= scale
    m[1, 1] *= scale
    m[0, 2] *= scale
    m[1, 2] *= scale
    return m

def build_rectification(cmtx0, dist0, cmtx1, dist1, R_rel, T_rel, img_size):
    R1r, R2r, P1, P2, Q, _, _ = cv2.stereoRectify(
        cmtx0, dist0, cmtx1, dist1, img_size, R_rel, T_rel, alpha=0
    )
    map1x, map1y = cv2.initUndistortRectifyMap(
        cmtx0, dist0, R1r, P1, img_size, cv2.CV_32FC1
    )
    map2x, map2y = cv2.initUndistortRectifyMap(
        cmtx1, dist1, R2r, P2, img_size, cv2.CV_32FC1
    )
    focal = float(P1[0, 0])
    baseline = abs(float(T_rel[0][0]))
    return map1x, map1y, map2x, map2y, Q, focal, baseline


# ===========================================================================
# DETECCAO DE LINHA POR PROFUNDIDADE
# ===========================================================================
def detectar_linha_mais_proxima(rect_l, disp, focal, baseline, cx, cy, roi_radius):
    gray = cv2.cvtColor(rect_l, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    _, bin_img = cv2.threshold(
        blurred,
        60,
        255,
        cv2.THRESH_BINARY_INV
    )

    # Fecha pequenos buracos
    kernel = np.ones((3, 3), np.uint8)
    bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_CLOSE, kernel)

    bin_img = cv2.dilate(bin_img, kernel, iterations=1)
    # ROI circular
    mask = np.zeros_like(bin_img)
    cv2.circle(mask, (cx, cy), roi_radius, 255, -1)
    roi_bin = cv2.bitwise_and(bin_img, mask)

    contours, _ = cv2.findContours(
        roi_bin,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE
    )

    if len(contours) == 0:
        return None, None, None, roi_bin

    # Descarta contornos pequenos
    contours = [c for c in contours if cv2.contourArea(c) > 100]

    if len(contours) == 0:
        return None, None, None, roi_bin

    # Maior contorno
    contour = max(contours, key=cv2.contourArea)
    debug = cv2.cvtColor(roi_bin, cv2.COLOR_GRAY2BGR)

    for c in contours:
        cv2.drawContours(debug, [c], -1, (0,255,0), 2)

    cv2.imshow("contours", debug)
    # Ajusta reta
    vx, vy, x0, y0 = cv2.fitLine(
        contour,
        cv2.DIST_L2,
        0,
        0.01,
        0.01
    )

    vx, vy, x0, y0 = [v.item() for v in cv2.fitLine(
        contour,
        cv2.DIST_L2,
        0,
        0.01,
        0.01
    )]

    # Gera um segmento suficientemente grande
    comprimento = roi_radius

    x1 = int(x0 - vx * comprimento)
    y1 = int(y0 - vy * comprimento)

    x2 = int(x0 + vx * comprimento)
    y2 = int(y0 + vy * comprimento)

    # Calcula profundidade média ao longo da reta
    n = int(np.hypot(x2 - x1, y2 - y1))

    profundidades = []

    xs = np.linspace(x1, x2, n).astype(int)
    ys = np.linspace(y1, y2, n).astype(int)

    for px, py in zip(xs, ys):

        if (
            px < 0 or
            py < 0 or
            px >= disp.shape[1] or
            py >= disp.shape[0]
        ):
            continue

        d = disp[py, px]

        if d > 0:
            z = focal * baseline / d

            if z <= 3.0:
                profundidades.append(z)

    if len(profundidades) == 0:
        return None, None, None, roi_bin

    distancia = float(np.mean(profundidades))

    angle = np.degrees(np.arctan2(vx, vy))

    if angle < 0:
        angle = 90 + abs(angle)

    return angle, (x1, y1, x2, y2), distancia, roi_bin
# ===========================================================================
# SCALE MANAGER
# ===========================================================================
SCALES = {ord("1"): 1.00, ord("2"): 0.75, ord("3"): 0.50, ord("4"): 0.25}
class ScaleManager:
    def __init__(self, cmtx0, dist0, cmtx1, dist1, R_rel, T_rel, base_w, base_h):
        self.cmtx0 = cmtx0
        self.dist0 = dist0
        self.cmtx1 = cmtx1
        self.dist1 = dist1
        self.R_rel = R_rel
        self.T_rel = T_rel
        self.base_w = base_w
        self.base_h = base_h
        self._cache = {}
        self.current = 1.0
        for s in SCALES.values():
            self._build(s)
        self.current = 1.0
        
    def _build(self, scale):
        if scale in self._cache:
            return
        w = max(2, int(self.base_w * scale))
        h = max(2, int(self.base_h * scale))
        w -= w % 2
        h -= h % 2
        size = (w, h)
        cm0 = scale_intrinsics(self.cmtx0, scale)
        cm1 = scale_intrinsics(self.cmtx1, scale)
        data = build_rectification(
            cm0, self.dist0, cm1, self.dist1, self.R_rel, self.T_rel, size
        )
        self._cache[scale] = {"maps": data, "size": size}

    def set(self, scale):
        self.current = scale
        
    def get(self):
        return self._cache[self.current]


# ===========================================================================
# SGBM
# ===========================================================================
class SGBMParams:
    BLOCKS = [3, 5, 7, 9, 11]

    def __init__(self):
        self.num_disp = 64
        self.min_disp = 0
        self.block_idx = 1

    def reset(self):
        self.num_disp = 64
        self.min_disp = 0
        self.block_idx = 1

    @property
    def block(self):
        return self.BLOCKS[self.block_idx]

    def inc_disp(self):
        self.num_disp = min(self.num_disp + 16, 256)

    def dec_disp(self):
        self.num_disp = max(self.num_disp - 16, 16)

    def inc_min_disp(self):
        self.min_disp += 16

    def dec_min_disp(self):
        self.min_disp -= 16

    def cycle_block(self):
        self.block_idx = (self.block_idx + 1) % len(self.BLOCKS)

    def build(self):
        b = self.block
        left = cv2.StereoSGBM_create(
            minDisparity=self.min_disp,
            numDisparities=self.num_disp,
            blockSize=b,
            P1=8 * 3 * b**2,
            P2=32 * 3 * b**2,
            disp12MaxDiff=1,
            uniquenessRatio=5,
            speckleWindowSize=50,
            speckleRange=16,
            preFilterCap=31,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        right = cv2.ximgproc.createRightMatcher(left)
        wls = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left)
        wls.setLambda(8000)
        wls.setSigmaColor(1.5)
        return left, right, wls


# ===========================================================================
# HUD E VISUALIZACAO (DUAS LINHAS: VISÃO + TELEMETRIA MOTORES)
# ===========================================================================
def draw_mini_hud(frame, fps, stats_str, stats_color):
    h, w = frame.shape[:2]

    # Escala baseada na altura da imagem
    s = h / 720.0

    margem = max(8, int(12 * s))
    linha1 = margem + int(18 * s)
    linha2 = linha1 + int(28 * s)

    altura_hud = linha2 + margem

    # Fundo do HUD
    cv2.rectangle(frame, (0, 0), (min(w, 1100), altura_hud), (0, 0, 0), -1)

    fonte = max(0.35, 0.55 * s)
    espessura = max(1, int(s))

    # Linha 1
    cv2.putText(
        frame,
        f"FPS: {fps:.1f} | {stats_str}",
        (10, linha1),
        cv2.FONT_HERSHEY_SIMPLEX,
        fonte,
        (0, 255, 200),
        espessura,
    )

    # Linha 2
    str_motores = (
        f"STM32 Real -> Yaw: {telemetria_dados['yaw']:+.1f}° | "
        f"GiroZ: {telemetria_dados['rate_z']:+.1f}°/s | "
        f"PWM_BF: {telemetria_dados['pwm_bf']}us | "
        f"PWM_AS: {telemetria_dados['pwm_as']}us | "
        f"Erro PID: {telemetria_dados['erro']:+.1f}°"
    )

    cv2.putText(
        frame,
        str_motores,
        (10, linha2),
        cv2.FONT_HERSHEY_SIMPLEX,
        fonte * 0.95,
        (255, 150, 50),
        espessura,
    )

    return altura_hud
