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
GUI_SCALE = float(os.environ.get("ESFERA_GUI_SCALE", "1"))

cv2.setNumThreads(4)

# ===========================================================================
# MATRIZES HARDCODED (DADOS REAIS DA CALIBRACAO)
# ===========================================================================
def load_params():
    cmtx0 = np.array(
        [
            [1577.07471815, 0.00000000, 700.20379270],
            [0.00000000, 1577.07471815, 486.80694469],
            [0.00000000, 0.00000000, 1.00000000]
        ],
        dtype=np.float64,
    )
    dist0 = np.array(
        [
            [0.02122462, 0.00000000, 0.00000000, 0.00000000, 0.00000000]
        ],
        dtype=np.float64,
    )
    cmtx1 = np.array(
        [
            [1577.07471815, 0.00000000, 611.88354212],
            [0.00000000, 1577.07471815, 436.91619332],
            [0.00000000, 0.00000000, 1.00000000]
        ],
        dtype=np.float64,
    )
    dist1 = np.array(
        [
            [0.03686214, 0.00000000, 0.00000000, 0.00000000, 0.00000000]
        ],
        dtype=np.float64,
    )
    R_rel = np.array(
        [
            [0.99694831, -0.05596877, -0.05442030],
            [0.05651897, 0.99836429, 0.00862310],
            [0.05384866, -0.01167256, 0.99848088]
        ],
        dtype=np.float64,
    )
    T_rel = np.array(
        [
            [0.25091677],
            [0.00954973],
            [0.00804070]
        ],
        dtype=np.float64,
    )
    baseline_m = abs(float(T_rel[0][0]))
    if baseline_m > 2.0:
        T_rel = T_rel / 100.0
        baseline_m = abs(float(T_rel[0][0]))
    return cmtx0, dist0, cmtx1, dist1, R_rel, T_rel



    


def validar_calibracao(cmtx0, dist0, cmtx1, dist1, R_rel, T_rel, frame_w=FRAME_W):
    """Avisa (nao aborta) quando a calibracao tem cara de lixo. Webcam 720p
    normal: f ~ 0.6-1.6 x largura, k1/k2 pequenos, T quase so em X."""
    avisos = []

    for nome, K in (("cam0", cmtx0), ("cam1", cmtx1)):
        fx, fy = K[0, 0], K[1, 1]
        if not (0.4 * frame_w <= fx <= 3.0 * frame_w):
            avisos.append(f"{nome}: fx={fx:.0f}px fora do plausivel "
                          f"({0.4*frame_w:.0f}-{3*frame_w:.0f})")
        if abs(fx - fy) / max(fx, fy) > 0.1:
            avisos.append(f"{nome}: fx={fx:.0f} e fy={fy:.0f} muito diferentes")
        if abs(K[0, 2] - frame_w / 2) > 0.25 * frame_w:
            avisos.append(f"{nome}: ponto principal cx={K[0, 2]:.0f} longe do centro")

    for nome, d in (("cam0", dist0), ("cam1", dist1)):
        k1, k2 = float(d.ravel()[0]), float(d.ravel()[1])
        if abs(k1) > 2.0 or abs(k2) > 5.0:
            avisos.append(f"{nome}: distorcao k1={k1:.2f}, k2={k2:.2f} absurda")

    T = np.asarray(T_rel, dtype=np.float64).ravel()
    nT = float(np.linalg.norm(T))
    if nT > 1e-9 and abs(T[2]) / nT > 0.3:
        avisos.append(f"T={T.round(3)}: componente Z domina (rig lado a lado "
                      "deveria ter T quase so em X)")

    ang = np.degrees(np.arccos(np.clip((np.trace(R_rel) - 1) / 2, -1, 1)))
    if ang > 15:
        avisos.append(f"R_rel gira {ang:.1f} graus entre as cameras")

    for a in avisos:
        print(f"[AVISO CALIB] {a}")
    if avisos:
        print("[AVISO CALIB] Profundidade 3D nao e confiavel com essa "
              "calibracao. Refaca o calib.py.")
    return not avisos


def checar_ordem_cameras():
    """cmtx0/dist0 pertencem a camera0 do calibration_settings.yaml. Se o
    CAM0_ID daqui for outro dispositivo, as matrizes ficam trocadas."""
    try:
        import yaml
        caminho = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "Calibration", "calibration_settings.yaml")
        with open(caminho) as f:
            cfg = yaml.safe_load(f)
    except Exception as e:
        print(f"[AVISO CALIB] Nao consegui ler o yaml de calibracao: {e}")
        return True

    if cfg.get("camera0") != CAM0_ID or cfg.get("camera1") != CAM1_ID:
        print(f"[AVISO CALIB] Ordem das cameras difere: yaml camera0="
              f"{cfg.get('camera0')} camera1={cfg.get('camera1')} | "
              f"configs CAM0_ID={CAM0_ID} CAM1_ID={CAM1_ID}. "
              "As matrizes de calibracao podem estar trocadas.")
        return False
    return True


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

