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
    _, bin_img = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY_INV)
    mask = np.zeros_like(gray)
    cv2.circle(mask, (cx, cy), roi_radius, 255, -1)
    roi_bin = cv2.bitwise_and(bin_img, bin_img, mask=mask)
    edges = cv2.Canny(roi_bin, 50, 150)

    lines = cv2.HoughLinesP(
        edges,
        1,
        np.pi / 180,
        threshold=40,
        minLineLength=roi_radius // 3,
        maxLineGap=20,
    )
    if lines is None:
        return None, None, None, roi_bin
    melhor_linha = None
    menor_distancia_media = float("inf")

    for l in lines:
        x1, y1, x2, y2 = l[0]
        num_points = int(np.hypot(x2 - x1, y2 - y1))
        if num_points == 0:
            continue
        x_coords = np.linspace(x1, x2, num_points).astype(int)
        y_coords = np.linspace(y1, y2, num_points).astype(int)
        profundidades_validas = []
        for px, py in zip(x_coords, y_coords):
            if py >= disp.shape[0] or px >= disp.shape[1] or py < 0 or px < 0:
                continue
            d = disp[py, px]
            if d > 0:
                Z = (focal * baseline) / d
                if Z <= 3.0:
                    profundidades_validas.append(Z)

        if len(profundidades_validas) > (num_points * 0.3):
            distancia_media = np.mean(profundidades_validas)
            if distancia_media < menor_distancia_media:
                menor_distancia_media = distancia_media
                melhor_linha = (x1, y1, x2, y2)

    if melhor_linha is None:
        return None, None, None, roi_bin

    x1, y1, x2, y2 = melhor_linha
    angle = np.degrees(np.arctan2(x2 - x1, y2 - y1))
    if angle < 0:
        angle = 90 + abs(angle)
    return angle, melhor_linha, menor_distancia_media, roi_bin

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
    # Retângulo estendido em altura (85px) para caber os dados da STM32 sem sobreposição
    cv2.rectangle(frame, (0, 0), (min(w, 1100), 85), (0, 0, 0), -1)
    
    # Linha 1: Dados de Visão Estereoscópica e FPS
    cv2.putText(frame, f"FPS: {fps:.1f} | {stats_str}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 200), 1)
                
    # Linha 2: Telemetria da STM32 (Lida assincronamente da Thread paralela)
    str_motores = (f"STM32 Real -> Yaw: {telemetria_dados['yaw']:+.1f}° | "
                   f"GiroZ: {telemetria_dados['rate_z']:+.1f}°/s | "
                   f"PWM_BF: {telemetria_dados['pwm_bf']}us | "
                   f"PWM_AS: {telemetria_dados['pwm_as']}us | "
                   f"Erro PID: {telemetria_dados['erro']:+.1f}°")
                   
    cv2.putText(frame, str_motores, (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 150, 50), 1)
