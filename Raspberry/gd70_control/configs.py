import cv2
import numpy as np
import os
import sys
import time
import threading
from collections import deque
import serial
import subprocess
from gpiozero import LED # Controle seguro do pino de reset na Raspberry Pi 5

# ===========================================================================
# CONFIGURACOES DE IMAGEM E REDE
# ===========================================================================
#CAM0_ID = "rtsp://192.168.144.25:8554/main.264"  # camera esquerda
#CAM1_ID = "rtsp://192.168.144.2:8554/main.264"   # camera direita
CAM0_ID = "/dev/camera_esquerda"
CAM1_ID = "/dev/camera_direita"
FRAME_W = 1280
FRAME_H = 720

cv2.setNumThreads(4)

# ===========================================================================
# MATRIZES HARDCODED (DADOS REAIS DA CALIBRACAO)
# ===========================================================================
def load_params():
    cmtx0 = np.array(
        [[2436.90910, 0, 276.497033], [0, 2484.41881, 257.576427], [0, 0, 1]],
        dtype=np.float64,
    )
    dist0 = np.array([[0.05499633, -0.21484423, 0.0, 0.0, 0.0]], dtype=np.float64)
    cmtx1 = np.array(
        [[2311.62632, 0, 585.375034], [0, 3394.81267, 90.9356098], [0, 0, 1.0]],
        dtype=np.float64,
    )
    dist1 = np.array([[0.18799809, -0.77468388, 0.0, 0.0, 0.0]], dtype=np.float64)
    R_rel = np.array(
        [
            [ 0.99988746, -0.01336806,  0.00680891],
            [ 0.01271275,  0.99598828,  0.08857613],
            [-0.00796569, -0.0884796,   0.99604614],
        ],
        dtype=np.float64,
    )
    T_rel = np.array([[-0.36560792], [-0.01454288], [-0.06727148]], dtype=np.float64)
    baseline_m = abs(float(T_rel[0][0]))
    if baseline_m > 2.0:
        T_rel = T_rel / 100.0
        baseline_m = abs(float(T_rel[0][0]))
    return cmtx0, dist0, cmtx1, dist1, R_rel, T_rel


