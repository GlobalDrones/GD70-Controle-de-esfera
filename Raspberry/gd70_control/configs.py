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
CAM0_ID = "rtsp://192.168.144.25:8554/main.264"  # camera esquerda
CAM1_ID = "rtsp://192.168.144.2:8554/main.264"   # camera direita
FRAME_W = 1280
FRAME_H = 720

cv2.setNumThreads(4)

# ===========================================================================
# MATRIZES HARDCODED (DADOS REAIS DA CALIBRACAO)
# ===========================================================================
def load_params():
    cmtx0 = np.array(
        [[849.09475804, 0, 656.00357451], [0, 827.44435633, 407.89408896], [0, 0, 1]],
        dtype=np.float64,
    )
    dist0 = np.array([[0.29794075, -0.2598218, 0.0, 0.0, 0.0]], dtype=np.float64)
    cmtx1 = np.array(
        [[810.15131758, 0, 669.59465007], [0, 783.52171627, 382.90946542], [0, 0, 1.0]],
        dtype=np.float64,
    )
    dist1 = np.array([[0.28443683, -0.27500358, 0.0, 0.0, 0.0]], dtype=np.float64)
    R_rel = np.array(
        [
            [0.99989951, 0.0131155, 0.00538065],
            [-0.01301987, 0.99976317, -0.01743832],
            [-0.00560808, 0.01736651, 0.99983346],
        ],
        dtype=np.float64,
    )
    T_rel = np.array([[-0.13558734], [0.00144692], [0.01341408]], dtype=np.float64)
    baseline_m = abs(float(T_rel[0][0]))
    if baseline_m > 2.0:
        T_rel = T_rel / 100.0
        baseline_m = abs(float(T_rel[0][0]))
    return cmtx0, dist0, cmtx1, dist1, R_rel, T_rel


