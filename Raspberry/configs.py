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
from collections import defaultdict

# ===========================================================================
# CONFIGURACOES DE IMAGEM E REDE
# ===========================================================================
#CAM0_ID = "rtsp://192.168.144.25:8554/main.264"  # camera esquerda
#CAM1_ID = "rtsp://192.168.144.2:8554/main.264"   # camera direita
CAM0_ID = "/dev/video0"#"/dev/camera_esquerda"
CAM1_ID = "/dev/video2"#"/dev/camera_direita"
FRAME_W = 1280
FRAME_H = 720

# ===========================================================================
# CONFIGURACOES DA JANELA DE VISUALIZACAO (importante p/ acesso via VNC)
# Enviar a janela cheia a todo frame satura o TigerVNC. Ajuste por ambiente:
#   ESFERA_GUI=0          -> sem janela (headless; teclas de comando desativadas)
#   ESFERA_GUI_FPS=10     -> maximo de atualizacoes da janela por segundo
#   ESFERA_GUI_SCALE=0.6  -> escala do painel exibido (so afeta a tela, nao a visao)
# Auto-detecta ambiente grafico: se nao houver $DISPLAY nem $WAYLAND_DISPLAY (ex: terminal SSH),
# ou se o usuario desativar com ESFERA_GUI=0, desativa a janela local para nao crashar o Qt/xcb.
_has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
GUI_ENABLED = (os.environ.get("ESFERA_GUI", "1" if _has_display else "0") != "0") and _has_display
GUI_MAX_FPS = float(os.environ.get("ESFERA_GUI_FPS", "10"))
GUI_SCALE = float(os.environ.get("ESFERA_GUI_SCALE", "0.6"))

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
    


class Profiler:
    def __init__(self, print_every=30):
        self.acc = defaultdict(float)
        self.count = 0
        self.print_every = print_every
        self._t0 = None
        self._label = None

    def start(self, label):
        self._label = label
        self._t0 = time.perf_counter()

    def stop(self):
        self.acc[self._label] += (time.perf_counter() - self._t0) * 1000  # ms

    def tick(self):
        self.count += 1
        if self.count >= self.print_every:
            parts = " | ".join(f"{k}: {v/self.count:.1f}ms" for k, v in self.acc.items())
            total = sum(self.acc.values()) / self.count
            print(f"[PROFILE] {parts} | TOTAL: {total:.1f}ms")
            self.acc.clear()
            self.count = 0


