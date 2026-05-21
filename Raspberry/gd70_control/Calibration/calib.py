# calib.py - Calibracao estereo robusta e ASSINCRONA para Raspberry Pi + RTSP
# Compativel com calibration_settings.yaml
#
# Uso:
#     python3 calib.py calibration_settings.yaml

import cv2 as cv
import glob
import numpy as np
import sys
import time
from scipy import linalg
import yaml
import os
import threading

calibration_settings = {}

# ===========================================================================
# CONFIGURACOES DE ROBUSTEZ E TIMEOUT
# ===========================================================================

WAITKEY_TIMEOUT_MS = 30000  # 30s max esperando tecla (evita travar)


# ===========================================================================
# YAML
# ===========================================================================

def parse_calibration_settings_file(filename):
    global calibration_settings
    if not os.path.exists(filename):
        print('Arquivo nao encontrado:', filename)
        quit()
    with open(filename) as f:
        calibration_settings = yaml.safe_load(f)
    if 'camera0' not in calibration_settings:
        print('Chave camera0 nao encontrada no yaml')
        quit()
    print('Configuracoes carregadas:', filename)


# ===========================================================================
# CLASSE DE CAPTURA ASSINCRONA (Salvacao da Raspberry Pi)
# Evita travamentos, buffer overflow na RAM e garante frames em tempo real
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
        print(f'  [{self.name}] Conectando ao RTSP...')
        self._cap = cv.VideoCapture(self.src)
        if not self._cap.isOpened():
            print(f'  [ERRO] Nao foi possivel abrir a camera {self.name}')
            return False
            
        self._cap.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv.CAP_PROP_BUFFERSIZE, 1)

        # Descarta os primeiros frames para estabilizar
        for _ in range(5):
            self._cap.read()

        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()
        print(f'  [{self.name}] Conectada e rodando em background.')
        return True

    def _loop(self):
        while self.running:
            if self._cap is not None and self._cap.isOpened():
                ret, frame = self._cap.read()
                if ret and frame is not None and frame.size > 0:
                    with self.lock:
                        self.frame = frame
                else:
                    time.sleep(0.01)
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
# CAPTURA MONO  (SPACE = capturar, ESC = sair)
# ===========================================================================

def save_frames_single_camera(camera_name):
    if not os.path.exists('frames'):
        os.mkdir('frames')

    device_id      = calibration_settings[camera_name]
    width          = calibration_settings['frame_width']
    height         = calibration_settings['frame_height']
    number_to_save = calibration_settings['mono_calibration_frames']
    view_resize    = calibration_settings['view_resize']

    cam = AsyncCamera(device_id, camera_name, width, height)
    if not cam.start():
        quit()

    saved_count = 0
    win_name    = f'Calibracao - {camera_name}'
    print(f'\n[{camera_name}] Aguardando video... SPACE = capturar  |  ESC = sair')

    while True:
        frame = cam.read()
        if frame is None:
            time.sleep(0.05)
            continue

        try:
            frame_small = cv.resize(frame, None, fx=1/view_resize, fy=1/view_resize)
        except cv.error as e:
            time.sleep(0.05)
            continue

        cv.putText(frame_small, "SPACE = capturar  |  ESC = sair",
                   (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv.putText(frame_small, f"Salvos: {saved_count}/{number_to_save}",
                   (20, 65), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv.imshow(win_name, frame_small)

        k = cv.waitKey(10) & 0xFF
        if k == 27:
            cam.stop()
            cv.destroyAllWindows()
            quit()
        if k == 32:
            savename = os.path.join('frames', f'{camera_name}_{saved_count}.png')
            if cv.imwrite(savename, frame):
                print(f'  Salvo: {savename}')
                saved_count += 1
            else:
                print(f'  AVISO: falha ao salvar {savename}')

        if saved_count == number_to_save:
            break

    cam.stop()
    cv.destroyAllWindows()
    print(f'  [{camera_name}] {saved_count} frames salvos.')


# ===========================================================================
# CAPTURA STEREO  (SPACE = capturar par, ESC = sair)
# ===========================================================================

def save_frames_two_cams(camera0_name, camera1_name):
    if not os.path.exists('frames_pair'):
        os.mkdir('frames_pair')

    cfg            = calibration_settings
    view_resize    = cfg['view_resize']
    number_to_save = cfg['stereo_calibration_frames']
    width          = cfg['frame_width']
    height         = cfg['frame_height']

    cam0 = AsyncCamera(cfg[camera0_name], camera0_name, width, height)
    cam1 = AsyncCamera(cfg[camera1_name], camera1_name, width, height)
    
    if not cam0.start() or not cam1.start():
        cam0.stop()
        cam1.stop()
        quit()

    saved_count = 0
    print('\n[Stereo] Posicione o tabuleiro visivel nas DUAS cameras.')
    print('         SPACE = capturar par  |  ESC = sair')

    while True:
        frame0 = cam0.read()
        frame1 = cam1.read()

        if frame0 is None or frame1 is None:
            time.sleep(0.05)
            continue

        try:
            f0s = cv.resize(frame0, None, fx=1./view_resize, fy=1./view_resize)
            f1s = cv.resize(frame1, None, fx=1./view_resize, fy=1./view_resize)
        except cv.error as e:
            time.sleep(0.05)
            continue

        for f in (f0s, f1s):
            cv.putText(f, "SPACE = capturar par  |  ESC = sair",
                       (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv.putText(f, f"Salvos: {saved_count}/{number_to_save}",
                       (20, 65), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Mostrar lado a lado para nao sobrecarregar o gerenciador de janelas da Raspberry Pi
        display = np.hstack([f0s, f1s])
        cv.imshow('Visao Stereo Realtime', display)

        # Alivia processador da Pi
        k = cv.waitKey(30) & 0xFF
        
        if k == 27:
            cam0.stop()
            cam1.stop()
            cv.destroyAllWindows()
            quit()

        if k == 32:
            s0 = os.path.join('frames_pair', f'{camera0_name}_{saved_count}.png')
            s1 = os.path.join('frames_pair', f'{camera1_name}_{saved_count}.png')
            ok0 = cv.imwrite(s0, frame0)
            ok1 = cv.imwrite(s1, frame1)
            if ok0 and ok1:
                print(f'  Par {saved_count} salvo sincronizado.')
                saved_count += 1
            else:
                print(f'  AVISO: falha ao salvar par {saved_count}. Tente novamente.')

        if saved_count == number_to_save:
            break

    cam0.stop()
    cam1.stop()
    cv.destroyAllWindows()
    print(f'  {saved_count} pares salvos.')


# ===========================================================================
# CALIBRACAO INTRINSECA
# ===========================================================================

def calibrate_camera_for_intrinsic_parameters(images_prefix):
    images_names = sorted(glob.glob(images_prefix))
    if not images_names:
        print(f'Nenhuma imagem encontrada: {images_prefix}')
        quit()

    images = []
    for n in images_names:
        img = cv.imread(n, 1)
        if img is not None:
            images.append(img)

    if len(images) < 3:
        print('Imagens insuficientes para calibracao.')
        quit()

    criteria      = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.001)
    rows          = calibration_settings['checkerboard_rows']
    columns       = calibration_settings['checkerboard_columns']
    world_scaling = calibration_settings['checkerboard_box_size_scale']

    objp = np.zeros((rows * columns, 3), np.float32)
    objp[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    objp = world_scaling * objp

    width  = images[0].shape[1]
    height = images[0].shape[0]
    imgpoints = []
    objpoints = []

    for i, frame in enumerate(images):
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        ret, corners = cv.findChessboardCorners(gray, (columns, rows), None)
        if ret:
            corners = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            disp = frame.copy()
            cv.drawChessboardCorners(disp, (columns, rows), corners, ret)
            cv.putText(disp, f'Frame {i+1}/{len(images)}  S=pular  outra tecla=aceitar',
                       (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
            
            # Reduz imagem da validacao para n estourar a tela
            disp_small = cv.resize(disp, None, fx=0.6, fy=0.6)
            cv.imshow('Verificacao intrinseca', disp_small)
            
            k = cv.waitKey(WAITKEY_TIMEOUT_MS) & 0xFF
            if k == ord('s'):
                print(f'  Frame {i} pulado.')
                continue
            objpoints.append(objp)
            imgpoints.append(corners)
        else:
            print(f'  Tabuleiro NAO encontrado: {images_names[i]}')

    cv.destroyAllWindows()

    if len(objpoints) < 3:
        print('Poucos frames validos. Capture mais imagens.')
        quit()

    # Trava K3 e distorcoes tangenciais para evitar que a matematica exploda 
    flags = cv.CALIB_FIX_K3 | cv.CALIB_ZERO_TANGENT_DIST

    ret, cmtx, dist, rvecs, tvecs = cv.calibrateCamera(
        objpoints, imgpoints, (width, height), None, None, flags=flags)
        
    print(f'  RMSE: {ret:.4f}  (bom < 1.0, otimo < 0.5)')
    print(f'  Matriz camera:\n{cmtx}')
    print(f'  Distorcao: {dist}')
    return cmtx, dist


# ===========================================================================
# SALVAR INTRINSECOS
# ===========================================================================

def save_camera_intrinsics(camera_matrix, distortion_coefs, camera_name):
    if not os.path.exists('camera_parameters'):
        os.mkdir('camera_parameters')
    out = os.path.join('camera_parameters', camera_name + '_intrinsics.dat')
    with open(out, 'w') as f:
        f.write('intrinsic:\n')
        for row in camera_matrix:
            f.write(' '.join(str(v) for v in row) + '\n')
        f.write('distortion:\n')
        f.write(' '.join(str(v) for v in distortion_coefs.flatten()) + '\n')
    print(f'  Intrinsecos salvos: {out}')


# ===========================================================================
# CALIBRACAO STEREO
# ===========================================================================

def stereo_calibrate(mtx0, dist0, mtx1, dist1, frames_prefix_c0, frames_prefix_c1):
    c0_names = sorted(glob.glob(frames_prefix_c0))
    c1_names = sorted(glob.glob(frames_prefix_c1))

    if not c0_names or not c1_names:
        print('Frames stereo nao encontrados.')
        quit()

    if len(c0_names) != len(c1_names):
        n = min(len(c0_names), len(c1_names))
        print(f'  AVISO: pares desiguais ({len(c0_names)} vs {len(c1_names)}). Usando {n}.')
        c0_names = c0_names[:n]
        c1_names = c1_names[:n]

    criteria      = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.001)
    rows          = calibration_settings['checkerboard_rows']
    columns       = calibration_settings['checkerboard_columns']
    world_scaling = calibration_settings['checkerboard_box_size_scale']

    objp = np.zeros((rows * columns, 3), np.float32)
    objp[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    objp = world_scaling * objp

    img_check = cv.imread(c0_names[0], 1)
    width, height = img_check.shape[1], img_check.shape[0]

    imgpoints_left  = []
    imgpoints_right = []
    objpoints       = []

    for idx, (n0, n1) in enumerate(zip(c0_names, c1_names)):
        frame0 = cv.imread(n0, 1)
        frame1 = cv.imread(n1, 1)
        if frame0 is None or frame1 is None:
            continue

        gray0 = cv.cvtColor(frame0, cv.COLOR_BGR2GRAY)
        gray1 = cv.cvtColor(frame1, cv.COLOR_BGR2GRAY)
        ret0, corners0 = cv.findChessboardCorners(gray0, (columns, rows), None)
        ret1, corners1 = cv.findChessboardCorners(gray1, (columns, rows), None)

        if ret0 and ret1:
            corners0 = cv.cornerSubPix(gray0, corners0, (11, 11), (-1, -1), criteria)
            corners1 = cv.cornerSubPix(gray1, corners1, (11, 11), (-1, -1), criteria)
            disp0 = frame0.copy()
            disp1 = frame1.copy()
            cv.drawChessboardCorners(disp0, (columns, rows), corners0, ret0)
            cv.drawChessboardCorners(disp1, (columns, rows), corners1, ret1)
            cv.putText(disp0, f'Par {idx+1}/{len(c0_names)}  S=pular  outra=aceitar',
                       (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
            
            # Mostrando lado a lado na avaliacao final
            disp_small0 = cv.resize(disp0, None, fx=0.6, fy=0.6)
            disp_small1 = cv.resize(disp1, None, fx=0.6, fy=0.6)
            cv.imshow('Stereo Validation', np.hstack([disp_small0, disp_small1]))
            
            k = cv.waitKey(WAITKEY_TIMEOUT_MS) & 0xFF
            if k == ord('s'):
                print(f'  Par {idx} pulado.')
                continue
            objpoints.append(objp)
            imgpoints_left.append(corners0)
            imgpoints_right.append(corners1)
        else:
            print(f'  Par {idx}: tabuleiro nao detectado. Pulando.')

    cv.destroyAllWindows()

    if len(objpoints) < 3:
        print('Poucos pares validos para calibracao stereo.')
        quit()

    print(f'  Calibrando com {len(objpoints)} pares validos...')
    
    ret, CM1, d0, CM2, d1, R, T, E, F = cv.stereoCalibrate(
        objpoints, imgpoints_left, imgpoints_right,
        mtx0, dist0, mtx1, dist1,
        (width, height),
        criteria=criteria,
        flags=cv.CALIB_FIX_INTRINSIC)

    print(f'  RMSE stereo: {ret:.4f}  (bom < 1.0, otimo < 0.5)')
    print(f'  Baseline: {np.linalg.norm(T):.4f} unidades do tabuleiro')
    print(f'  R:\n{R}')
    print(f'  T: {T.flatten()}')
    return R, T


# ===========================================================================
# UTILITARIOS
# ===========================================================================

def save_extrinsic_calibration_parameters(R0, T0, R1, T1, prefix=''):
    if not os.path.exists('camera_parameters'):
        os.mkdir('camera_parameters')
    def write_rt(filename, R, T):
        with open(filename, 'w') as f:
            f.write('R:\n')
            for row in R:
                f.write(' '.join(str(v) for v in row) + '\n')
            f.write('T:\n')
            for v in T.flatten():
                f.write(str(v) + '\n')
        print(f'  Extrinsecos salvos: {filename}')
    write_rt(os.path.join('camera_parameters', prefix + 'camera0_rot_trans.dat'), R0, T0)
    write_rt(os.path.join('camera_parameters', prefix + 'camera1_rot_trans.dat'), R1, T1)
    return R0, T0, R1, T1


# ===========================================================================
# MAIN
# ===========================================================================

if __name__ == '__main__':

    if len(sys.argv) != 2:
        print('Uso: python3 calib.py calibration_settings.yaml')
        quit()

    parse_calibration_settings_file(sys.argv[1])

    scale = calibration_settings['checkerboard_box_size_scale']
    if scale >= 1.0:
        print(f'\n[AVISO] checkerboard_box_size_scale = {scale}')
        print('        Para profundidade em metros, use o tamanho real do quadrado em metros.')
        print('        Ex: quadrado de 25mm -> 0.025\n')

    print('\n=== PASSO 1: Frames individuais ===')
    save_frames_single_camera('camera0')
    save_frames_single_camera('camera1')

    print('\n=== PASSO 2: Calibracao intrinseca ===')
    cmtx0, dist0 = calibrate_camera_for_intrinsic_parameters(
        os.path.join('frames', 'camera0*'))
    save_camera_intrinsics(cmtx0, dist0, 'camera0')

    cmtx1, dist1 = calibrate_camera_for_intrinsic_parameters(
        os.path.join('frames', 'camera1*'))
    save_camera_intrinsics(cmtx1, dist1, 'camera1')

    print('\n=== PASSO 3: Pares sincronizados ===')
    save_frames_two_cams('camera0', 'camera1')

    print('\n=== PASSO 4: Calibracao stereo ===')
    R, T = stereo_calibrate(
        cmtx0, dist0, cmtx1, dist1,
        os.path.join('frames_pair', 'camera0*'),
        os.path.join('frames_pair', 'camera1*'))

    print('\n=== PASSO 5: Salvar extrinsecos ===')
    R0 = np.eye(3, dtype=np.float32)
    T0 = np.zeros((3, 1), dtype=np.float32)
    save_extrinsic_calibration_parameters(R0, T0, R, T)

    print('\n=== Calibracao concluida! ===')
