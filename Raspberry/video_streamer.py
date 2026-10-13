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
# CLASSE DE CAPTURA ASSÍNCRONA
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
        self._thread = None

        # Controle de reconexão
        self.reconnect_delay = 1.0

    def _open_camera(self):
        print(f"[INFO] Conectando {self.name}...")

        cap = cv2.VideoCapture(self.src)

        if not cap.isOpened():
            cap.release()
            print(f"[ERRO] Nao abriu {self.src}")
            return False

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Descarta alguns frames iniciais
        for _ in range(5):
            ret, frame = cap.read()
            if not ret:
                break

        self._cap = cap

        print(f"[OK] {self.name} conectada em background")
        return True

    def start(self):
        if not self._open_camera():
            return False

        self.running = True

        self._thread = threading.Thread(
            target=self._loop,
            daemon=True
        )
        self._thread.start()

        return True

    def _loop(self):
        while self.running:

            # ---------------------------------------------------------------
            # Verifica se temos uma captura válida
            # ---------------------------------------------------------------
            if self._cap is None or not self._cap.isOpened():

                print(f"[AVISO] {self.name} desconectada.")
                print(f"[INFO] Tentando reconectar {self.name}...")

                if self._cap is not None:
                    self._cap.release()
                    self._cap = None

                time.sleep(self.reconnect_delay)

                if not self.running:
                    break

                if self._open_camera():
                    print(f"[OK] {self.name} reconectada!")
                else:
                    time.sleep(self.reconnect_delay)

                continue

            # ---------------------------------------------------------------
            # Tenta capturar frame
            # ---------------------------------------------------------------
            ret, frame = self._cap.read()

            if ret and frame is not None and frame.size > 0:

                with self.lock:
                    self.frame = frame

            else:

                # -----------------------------------------------------------
                # A câmera provavelmente caiu.
                #
                # Libera o VideoCapture para que uma nova instância possa
                # ser criada quando o dispositivo USB voltar.
                # -----------------------------------------------------------
                print(f"[AVISO] Falha ao ler {self.name}. Reconectando...")

                self._cap.release()
                self._cap = None

                with self.lock:
                    self.frame = None

                time.sleep(self.reconnect_delay)

    def read(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False

        if self._thread is not None:
            self._thread.join(timeout=1.0)

        if self._cap is not None:
            self._cap.release()
            self._cap = None

        with self.lock:
            self.frame = None
