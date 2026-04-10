import cv2
import numpy as np
import os
import sys
import time
import threading
from collections import deque  
import serial 

# ===========================================================================
# CONFIGURACOES DE UART (SERIAL) - BUSCA AUTOMATICA
# ===========================================================================
BAUDRATE = 115200
# Lista de portas que o Linux costuma dar para o Arduino
PORTAS_TENTATIVA = [
    '/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2', 
    '/dev/ttyACM0', '/dev/ttyACM1'
]

arduino = None

print("[INFO] Procurando Arduino nas portas USB...")
for porta in PORTAS_TENTATIVA:
    try:
        arduino = serial.Serial(porta, BAUDRATE, timeout=0)
        print(f"[OK] Sucesso! Conectado ao Arduino na porta: {porta}")
        break  # Se conectou, sai do loop de busca
    except Exception as e:
        pass # Ignora o erro e tenta a proxima porta da lista

if arduino is None:
    print("[AVISO] Arduino nao encontrado. O codigo vai rodar, mas sem enviar os dados.")

def send_ang_serial(angulo):
    if arduino is not None and arduino.is_open:
        angulo = max(0, min(90, angulo))
        msg = f"{angulo:.1f}\n"
        try:
            arduino.write(msg.encode('utf-8'))
        except Exception as e:
            pass # Previne que o script quebre se o cabo soltar no meio do uso

# ===========================================================================
# CONFIGURACOES DE IMAGEM E REDE
# ===========================================================================

CAM0_ID    = "rtsp://192.168.144.26:8554/main.264"  # camera esquerda
CAM1_ID    = "rtsp://192.168.144.2:8554/main.264"   # camera direita

FRAME_W    = 1280
FRAME_H    = 720

# Forca o OpenCV a usar todos os nucleos da Raspberry Pi
cv2.setNumThreads(4)

# ===========================================================================
# MATRIZES HARDCODED (DADOS REAIS DA CALIBRACAO)
# ===========================================================================

def load_params():
    cmtx0 = np.array([
        [668.78355047,   0.        , 656.74961609],
        [  0.        , 667.73434135, 341.95018825],
        [  0.        ,   0.        ,   1.        ]
    ], dtype=np.float64)
    
    dist0 = np.array([[ 0.08551549, -0.08088693, 0., 0., 0. ]], dtype=np.float64)

    cmtx1 = np.array([
        [702.3551953 ,   0.        , 660.82123282],
        [  0.        , 703.08238348, 330.96782801],
        [  0.        ,   0.        ,   1.        ]
    ], dtype=np.float64)
    
    dist1 = np.array([[ 0.09119711, -0.10748202, 0., 0., 0. ]], dtype=np.float64)

    R_rel = np.array([
        [ 0.99989951,  0.0131155 ,  0.00538065],
        [-0.01301987,  0.99976317, -0.01743832],
        [-0.00560808,  0.01736651,  0.99983346]
    ], dtype=np.float64)

    T_rel = np.array([
        [-0.13558734],
        [ 0.00144692],
        [ 0.01341408]
    ], dtype=np.float64)

    baseline_m = abs(float(T_rel[0][0]))
    if baseline_m > 2.0:
        T_rel      = T_rel / 100.0
        baseline_m = abs(float(T_rel[0][0]))
    
    return cmtx0, dist0, cmtx1, dist1, R_rel, T_rel


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
        cmtx0, dist0, cmtx1, dist1,
        img_size, R_rel, T_rel, alpha=0
    )
    map1x, map1y = cv2.initUndistortRectifyMap(
        cmtx0, dist0, R1r, P1, img_size, cv2.CV_32FC1)
    map2x, map2y = cv2.initUndistortRectifyMap(
        cmtx1, dist1, R2r, P2, img_size, cv2.CV_32FC1)
    focal    = float(P1[0, 0])
    baseline = abs(float(T_rel[0][0]))
    return map1x, map1y, map2x, map2y, Q, focal, baseline


def detectar_angulo(rect_l, cx, cy, roi_radius):
        gray = cv2.cvtColor(rect_l, cv2.COLOR_BGR2GRAY)
        mask = np.zeros_like(gray)
        cv2.circle(mask, (cx, cy), roi_radius, 255, -1)

        roi = cv2.bitwise_and(gray, gray, mask=mask)
        edges = cv2.Canny(roi, 50, 150)

        lines = cv2.HoughLinesP(edges,
                                1,
                                np.pi / 180,
                                threshold=50,
                                minLineLength=roi_radius // 2,
                                maxLineGap=20)

        if lines is None:
            return None, None

        best_line = None
        best_len = 0

        for l in lines:
            x1, y1, x2, y2 = l[0]
            if ((x1 - cx)**2 + (y1 - cy)**2 > roi_radius**2 and
                (x2 - cx)**2 + (y2 - cy)**2 > roi_radius**2):
                continue

            length = np.hypot(x2 - x1, y2 - y1)

            if length > best_len:
                best_len = length
                best_line = (x1, y1, x2, y2)

        if best_line is None:
            return None, None

        x1, y1, x2, y2 = best_line
        angle = np.degrees(np.arctan2(x2 - x1, y2 - y1))

        if angle < 0:
            angle = 90 + abs(angle)

        return angle, best_line


# ===========================================================================
# SCALE MANAGER
# ===========================================================================

SCALES       = {ord('1'): 1.00, ord('2'): 0.75, ord('3'): 0.50, ord('4'): 0.25}

class ScaleManager:
    def __init__(self, cmtx0, dist0, cmtx1, dist1, R_rel, T_rel, base_w, base_h):
        self.cmtx0  = cmtx0
        self.dist0  = dist0
        self.cmtx1  = cmtx1
        self.dist1  = dist1
        self.R_rel  = R_rel
        self.T_rel  = T_rel
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
        cm0  = scale_intrinsics(self.cmtx0, scale)
        cm1  = scale_intrinsics(self.cmtx1, scale)
        data = build_rectification(cm0, self.dist0, cm1, self.dist1,
                                   self.R_rel, self.T_rel, size)
        self._cache[scale] = {'maps': data, 'size': size}

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
        self.num_disp  = 64
        self.min_disp  = 0
        self.block_idx = 1

    def reset(self):
        self.num_disp  = 64
        self.min_disp  = 0
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
            P1=8  * 3 * b**2,
            P2=32 * 3 * b**2,
            disp12MaxDiff=1,
            uniquenessRatio=5,
            speckleWindowSize=50,
            speckleRange=16,
            preFilterCap=31,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )
        right = cv2.ximgproc.createRightMatcher(left)
        wls   = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left)
        wls.setLambda(8000)
        wls.setSigmaColor(1.5)
        return left, right, wls


# ===========================================================================
# HUD E VISUALIZACAO (SIMPLIFICADO)
# ===========================================================================

def draw_mini_hud(frame, fps, stats_str, stats_color):
    h, w = frame.shape[:2]
    # Retangulo escuro no topo para dar contraste nos textos
    cv2.rectangle(frame, (0, 0), (min(w, 800), 65), (0, 0, 0), -1)
    
    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 200), 1)
                
    cv2.putText(frame, stats_str, (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, stats_color, 2)


# ===========================================================================
# MAIN
# ===========================================================================

def main():
    cmtx0, dist0, cmtx1, dist1, R_rel, T_rel = load_params()

    scale_mgr = ScaleManager(cmtx0, dist0, cmtx1, dist1, R_rel, T_rel, FRAME_W, FRAME_H)
    sgbm = SGBMParams()
    left_m, right_m, wls = sgbm.build()

    cam0 = AsyncCamera(CAM0_ID, "cam0-esq", FRAME_W, FRAME_H)
    cam1 = AsyncCamera(CAM1_ID, "cam1-dir", FRAME_W, FRAME_H)
    
    if not cam0.start() or not cam1.start():
        sys.exit(1)

    print("[INFO] Aguardando o primeiro frame de ambas as cameras...")
    t_wait = time.time() + 15
    while time.time() < t_wait:
        if cam0.read() is not None and cam1.read() is not None:
            break
        time.sleep(0.1)
    else:
        print("[ERRO] Timeout aguardando as cameras. Verifique o RTSP.")
        cam0.stop()
        cam1.stop()
        sys.exit(1)

    print("[OK] Loop de tempo real iniciado!")

    gray_disp = False
    use_wls   = False   
    scale_key = ord('3')  
    scale_mgr.set(SCALES[scale_key])
    
    fps       = 0.0
    t_last    = time.time()
    historico_angulos = deque(maxlen=30)
    
    cv2.namedWindow("Stereo Profundidade", cv2.WINDOW_NORMAL)

    while True:
        f0 = cam0.read()
        f1 = cam1.read()
        
        if f0 is None or f1 is None:
            time.sleep(0.005)
            continue

        data     = scale_mgr.get()
        out_size = data['size']
        map1x, map1y, map2x, map2y, Q, focal, baseline = data['maps']

        if (f0.shape[1], f0.shape[0]) != out_size:
            f0 = cv2.resize(f0, out_size)
        if (f1.shape[1], f1.shape[0]) != out_size:
            f1 = cv2.resize(f1, out_size)

        rect_l = cv2.remap(f0, map1x, map1y, cv2.INTER_LINEAR)
        rect_r = cv2.remap(f1, map2x, map2y, cv2.INTER_LINEAR)

        stats_str = "NUVEM CENTRAL -> Sem dados validos na area"
        stats_color = (255, 255, 0) # Amarelo

        gl = cv2.cvtColor(rect_l, cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(rect_r, cv2.COLOR_BGR2GRAY)
        dl = left_m.compute(gl, gr)
        
        if use_wls:
            dr   = right_m.compute(gr, gl)
            disp = wls.filter(dl, rect_l, None, dr).astype(np.float32) / 16.0
        else:
            disp = dl.astype(np.float32) / 16.0
        
        mask = (disp > sgbm.min_disp)

        valid_disp = disp[mask]
        if len(valid_disp) > 0:
            min_d_clip = np.percentile(valid_disp, 2)
            max_d_clip = np.percentile(valid_disp, 98)
            disp_clipped = np.clip(disp, min_d_clip, max_d_clip)
        else:
            disp_clipped = disp

        disp_norm = cv2.normalize(disp_clipped, None, 0, 255, cv2.NORM_MINMAX)
        
        if gray_disp:
            disp_vis = cv2.cvtColor(np.uint8(disp_norm), cv2.COLOR_GRAY2BGR)
        else:
            disp_vis = cv2.applyColorMap(np.uint8(disp_norm), cv2.COLORMAP_JET)
        
        disp_vis[~mask] = 0

        # --- LOGICA DA NUVEM DE PONTOS E ANGULO ---
        cx, cy = out_size[0] // 2, out_size[1] // 2
        roi_radius = int(80 * SCALES[scale_key])

        angulo, linha = detectar_angulo(rect_l, cx, cy, roi_radius)

        if angulo is not None:
            if angulo > 90:
                angulo = 180 - angulo
            
            historico_angulos.append(angulo)
            media_angulo = int(sum(historico_angulos) / len(historico_angulos))
            
            stats_str += f" | Angulo: {angulo:.1f} deg | Med(30): {media_angulo:.1f} deg"

            x1, y1, x2, y2 = linha
            cv2.line(disp_vis, (x1, y1), (x2, y2), (0, 255, 255), 3)
            
            send_ang_serial(media_angulo)

        # Desenha o circulo delimitador
        cv2.circle(disp_vis, (cx, cy), roi_radius, (255, 255, 255), 1)

        Y, X = np.ogrid[:out_size[1], :out_size[0]]
        dist_from_center = (X - cx)**2 + (Y - cy)**2 <= roi_radius**2
        
        valid_roi_mask = mask & dist_from_center & (disp > 0)
        roi_disps = disp[valid_roi_mask]
        
        if len(roi_disps) > 0:
            roi_depths = (focal * baseline) / roi_disps
            
            p5 = np.percentile(roi_depths, 10)
            p95 = np.percentile(roi_depths, 90)
            filtered_depths = roi_depths[(roi_depths >= p5) & (roi_depths <= p95)]
            
            if len(filtered_depths) > 0:
                d_min = np.min(filtered_depths)
                d_max = np.max(filtered_depths)
                d_mean = np.mean(filtered_depths)
                
                if (d_max - d_min) >= 1.5:
                    if(not angulo):
                        stats_str = f"ALVO ISOLADO-> Distancia: {d_min:.2f}m"
                    else:
                        stats_str = f"ALVO ISOLADO-> Distancia: {d_min:.2f}m | ANG MED: {media_angulo:.1f}°"
                    stats_color = (0, 255, 0) # Verde vibrante
                else:
                    stats_str = f"NUVEM CENTRAL -> Media: {d_mean:.2f}m | Min: {d_min:.2f}m | Max: {d_max:.2f}m"
                    stats_color = (255, 255, 0) # Amarelo padrao

        # Desenha a grade pontual numerica
        step = int(35 * SCALES[scale_key])
        for dy in range(-roi_radius + 15, roi_radius, step):
            for dx in range(-roi_radius + 15, roi_radius, step):
                if dx**2 + dy**2 <= (roi_radius - 10)**2:
                    px, py = cx + dx, cy + dy
                    if mask[py, px] and disp[py, px] > 0:
                        d = (focal * baseline) / disp[py, px]
                        cv2.circle(disp_vis, (px, py), 2, (0, 255, 0), -1)
                        cv2.putText(disp_vis, f"{d:.1f}", (px + 4, py - 4),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        # Montagem da tela final (Apenas o Mapa de Profundidade)
        display = disp_vis

        now    = time.time()
        fps    = 0.9 * fps + 0.1 / max(now - t_last, 1e-6)
        t_last = now

        draw_mini_hud(display, fps, stats_str, stats_color)
        cv2.imshow("Stereo Profundidade", display)

        # Manti os atalhos essenciais apenas para ajuste fino caso vc precise
        k = cv2.waitKey(1) & 0xFF
        if k in [ord('q'), 27]:
            break
        elif k == ord('w'): 
            use_wls = not use_wls
        elif k == ord('d'):
            gray_disp = not gray_disp
        elif k in SCALES:
            scale_key = k
            scale_mgr.set(SCALES[k])
        elif k in (ord('+'), ord('=')):
            sgbm.inc_disp(); left_m, right_m, wls = sgbm.build()
        elif k == ord('-'):
            sgbm.dec_disp(); left_m, right_m, wls = sgbm.build()
        elif k == ord('a'):
            sgbm.inc_min_disp(); left_m, right_m, wls = sgbm.build()
        elif k == ord('s'):
            sgbm.dec_min_disp(); left_m, right_m, wls = sgbm.build()
        elif k == ord('b'):
            sgbm.cycle_block(); left_m, right_m, wls = sgbm.build()
        elif k == ord('r'):
            sgbm.reset(); left_m, right_m, wls = sgbm.build()

    cam0.stop()
    cam1.stop()
    if arduino is not None:
        arduino.close() # Libera a porta USB no final
    cv2.destroyAllWindows()
    print("[OK] Encerrado graciosamente")

if __name__ == '__main__':
    main()
