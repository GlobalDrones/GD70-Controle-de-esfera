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
        self.frame_id = 0
        self.new_frame_event = threading.Event()
        self.lock = threading.Lock()

        self.running = False
        self._cap = None
        self._thread = None

        # Controle de reconexão tolerante a ruído (sem reiniciar o processo via os.execv)
        self.reconnect_delay = 1.0
        self.consecutive_read_failures = 0
        self.max_consecutive_failures = 30  # ~1.0-1.5s de falhas contínuas antes de recriar V4L2

    def _open_camera(self):
        print(f"[INFO] Conectando {self.name} ({self.src})...")

        # Dispositivo local (/dev/...) -> V4L2 direto. Sem backend explícito o
        # OpenCV tenta o GStreamer primeiro, que sempre falha aqui.
        is_local = isinstance(self.src, int) or str(self.src).startswith("/dev/")
        backend = cv2.CAP_V4L2 if is_local else cv2.CAP_ANY
        try:
            cap = cv2.VideoCapture(self.src, backend)
        except Exception as e:
            print(f"[ERRO] Excecao ao abrir {self.src}: {e}")
            return False

        if not cap.isOpened():
            cap.release()
            print(f"[ERRO] Nao abriu {self.src}")
            return False

        if is_local:
            # MJPG antes da resolução: sem isso o driver escolhe YUYV sem
            # compressão (1280x720 @ 10 fps, ~18 MB/s por câmera), que satura
            # o USB 2.0 com duas câmeras e acaba travando o stream.
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Descarta alguns frames iniciais e garante que o stream entrega imagem
        got_frame = False
        for _ in range(5):
            ret, frame = cap.read()
            if not ret or frame is None or frame.size == 0:
                time.sleep(0.02)
                continue
            got_frame = True
            break

        if not got_frame:
            cap.release()
            print(f"[ERRO] {self.name} abriu mas nao entregou frames validos")
            return False

        self._cap = cap
        self.consecutive_read_failures = 0
        print(f"[OK] {self.name} conectada em background")
        return True

    def start(self):
        if not self._open_camera():
            return False

        self.running = True
        self._thread = threading.Thread(
            target=self._loop,
            daemon=True,
            name=f"cam-{self.name}"
        )
        self._thread.start()
        return True

    def _loop(self):
        backoff = self.reconnect_delay
        while self.running:
            # ---------------------------------------------------------------
            # Se não temos captura válida, tenta reconectar sem derrubar o processo
            # ---------------------------------------------------------------
            if self._cap is None or not self._cap.isOpened():
                if not self.running:
                    break
                print(f"[INFO] Tentando reconectar {self.name} (espera={backoff:.1f}s)...")
                if self._cap is not None:
                    try:
                        self._cap.release()
                    except Exception:
                        pass
                    self._cap = None

                time.sleep(backoff)
                if not self.running:
                    break

                if self._open_camera():
                    print(f"[OK] {self.name} reconectada com sucesso!")
                    backoff = self.reconnect_delay
                else:
                    backoff = min(backoff * 1.5, 5.0)  # maximo 5s de backoff
                continue

            # ---------------------------------------------------------------
            # Captura o frame
            # ---------------------------------------------------------------
            try:
                ret, frame = self._cap.read()
            except Exception as e:
                ret = False
                frame = None

            if ret and frame is not None and frame.size > 0:
                with self.lock:
                    self.frame = frame
                    self.frame_id += 1
                self.new_frame_event.set()
                self.consecutive_read_failures = 0
            else:
                self.consecutive_read_failures += 1
                # Só reinicia o driver V4L2 se falhar por muitos frames contínuos (~1.5s)
                # Falhas isoladas de 1-2 frames causadas por jitter ou ruído de motor são toleradas.
                if self.consecutive_read_failures >= self.max_consecutive_failures:
                    print(f"[AVISO] Falha persistente ao ler {self.name} ({self.consecutive_read_failures} frames sem sinal). Reabrindo dispositivo...")
                    try:
                        self._cap.release()
                    except Exception:
                        pass
                    self._cap = None
                    with self.lock:
                        self.frame = None
                    self.consecutive_read_failures = 0
                    time.sleep(self.reconnect_delay)
                else:
                    time.sleep(0.005)

    def read(self, wait_new=False, timeout=0.05):
        """Retorna uma cópia do frame. Se wait_new=True, aguarda a chegada de um novo frame físico."""
        if wait_new and self.running:
            self.new_frame_event.wait(timeout=timeout)
        with self.lock:
            if self.frame is None:
                return None
            frame_copy = self.frame.copy()
            self.new_frame_event.clear()
            return frame_copy

    def stop(self):
        self.running = False
        self.new_frame_event.set()

        if self._thread is not None and threading.current_thread() != self._thread:
            self._thread.join(timeout=1.0)

        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

        with self.lock:
            self.frame = None
