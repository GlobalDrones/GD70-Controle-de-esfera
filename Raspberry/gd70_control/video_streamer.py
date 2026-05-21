import cv2
import numpy as np
import os
import sys
import time
import threading
from collections import deque
import serial
import subprocess

# ===========================================================================
# FUNÇÃO DO STREAMER (FFMPEG COLETOR)
# ===========================================================================
def iniciar_streamer(width, height, scale):
    largura_atual = int(width * scale)
    altura_atual = int(height * scale)
    # ATENÇÃO: Como o script faz hconcat de TRÊS telas, multiplicamos a largura por 3
    largura_total_stream = largura_atual * 3
    
    command = [
        'ffmpeg',
        '-y',
        '-f', 'rawvideo',
        '-vcodec', 'rawvideo',
        '-pix_fmt', 'bgr24',
        '-s', f"{largura_total_stream}x{altura_atual}", 
        '-r', '20', 
        '-i', '-',
        '-c:v', 'libx264', # Pode trocar por 'h264_v4l2m2m' se quiser usar aceleração via hardware
        '-preset', 'ultrafast',
        '-tune', 'zerolatency',
        '-f', 'rtsp',
        'rtsp://localhost:8554/linha'
    ]
    return subprocess.Popen(command, stdin=subprocess.PIPE)



# ===========================================================================
# CLASSE DE CAPTURA ASSINCRONA
# ===========================================================================
class AsyncCamera:
    def __init__(self, src, name, width, height):
        self.src = src
        self.name = name
        self.width = width
        self.height = height
        self.frame = None
        self.lock = threading.Lock()
        self.running = False
        self._cap = None

    def start(self):
        print(f"[INFO] Conectando {self.name}...")
        self._cap = cv2.VideoCapture(self.src)
        if not self._cap.isOpened():
            print(f"[ERRO] Nao abriu: {self.src}")
            return False
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        for _ in range(5):
            self._cap.read()
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()
        print(f"[OK] {self.name} conectada em background")
        return True

    def _loop(self):
        while self.running:
            if self._cap is not None and self._cap.isOpened():
                ret, frame = self._cap.read()
                if ret and frame is not None and frame.size > 0:
                    with self.lock:
                        self.frame = frame
                else:
                    time.sleep(0.005)
            else:
                time.sleep(0.1)

    def read(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False
        time.sleep(0.1)
        if self._cap is not None:
            self._cap.release()
