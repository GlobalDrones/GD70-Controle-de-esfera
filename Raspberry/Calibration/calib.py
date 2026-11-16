# calib.py - Calibracao estereo robusta e ASSINCRONA para Raspberry Pi + RTSP/V4L2
# Compativel com calibration_settings.yaml
#
# Uso:
#     python3 calib.py calibration_settings.yaml
#     python3 calib.py calibration_settings.yaml --reuse-frames   (pula captura, recalibra com os frames em disco)
#
# Chaves OPCIONAIS novas no yaml (todas com default):
#     fix_principal_point: false   # trava cx,cy no centro (paliativo)
#     fix_aspect_ratio:    false   # trava fx/fy (so se pixel for quadrado)
#     focal_guess_px:      null    # chute inicial de foco (default = frame_width)
#     manual_review:       false   # mostra cada deteccao e espera tecla (S = pular)
#     outlier_factor:      2.0     # descarta frame com erro > fator * mediana
#     outlier_min_px:      0.5     # ...mas nunca abaixo deste erro (px)

import cv2 as cv
import glob
import numpy as np
import sys
import time
import yaml
import os
import re
import threading

calibration_settings = {}

# ===========================================================================
# CONFIGURACOES DE ROBUSTEZ
# ===========================================================================

WAITKEY_TIMEOUT_MS = 30000     # 30s max esperando tecla na revisao manual
SATURATION_MAX_FRAC = 0.05     # >5% de pixels saturados (>=250) na regiao do tabuleiro = reflexo/superexposto
MIN_VIEWS = 8                  # minimo de frames para aceitar rejeicao de outliers
MAX_REJECT_PASSES = 3          # quantas recalibracoes no maximo no loop de outliers

FRAMES_DIR = 'frames'
PAIRS_DIR = 'frames_pair'
PARAMS_DIR = 'camera_parameters'

CRITERIA = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.001)


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


def cfg(key, default=None):
    v = calibration_settings.get(key, default)
    return default if v is None else v


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
        print(f'  [{self.name}] Conectando...')
        self._cap = cv.VideoCapture(self.src)
        if not self._cap.isOpened():
            print(f'  [ERRO] Nao foi possivel abrir a camera {self.name}')
            return False

        self._cap.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv.CAP_PROP_BUFFERSIZE, 1)

        # Descarta os primeiros frames para estabilizar
        last = None
        for _ in range(5):
            ret, f = self._cap.read()
            if ret and f is not None:
                last = f

        # Confere se a resolucao real bate com a configurada
        if last is not None:
            h, w = last.shape[:2]
            if (w, h) != (self.width, self.height):
                print(f'  [AVISO] {self.name}: resolucao real {w}x{h} != configurada '
                      f'{self.width}x{self.height}. As intrinsecas so servem se a '
                      f'resolucao de runtime for IGUAL a da calibracao.')
            self.frame = last

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
# DETECCAO ROBUSTA DO TABULEIRO
# ===========================================================================

def find_corners(gray, columns, rows):
    """Tenta o detector SB (robusto a brilho/ruido, ja devolve subpixel).
    Se falhar, cai no metodo classico + cornerSubPix."""
    flags_sb = cv.CALIB_CB_EXHAUSTIVE | cv.CALIB_CB_ACCURACY
    try:
        ok, corners = cv.findChessboardCornersSB(gray, (columns, rows), flags_sb)
    except cv.error:
        ok, corners = False, None
    if ok:
        return True, corners.astype(np.float32).reshape(-1, 1, 2)

    flags = cv.CALIB_CB_ADAPTIVE_THRESH | cv.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv.findChessboardCorners(gray, (columns, rows), flags)
    if not ok:
        return False, None
    corners = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), CRITERIA)
    return True, corners


def saturation_fraction(gray, corners):
    pts = corners.reshape(-1, 2).astype(np.float32)
    x, y, w, h = cv.boundingRect(pts)
    x0, y0 = max(x, 0), max(y, 0)
    x1, y1 = min(x + w, gray.shape[1]), min(y + h, gray.shape[0])
    region = gray[y0:y1, x0:x1]
    if region.size == 0:
        return 0.0
    return float(np.mean(region >= 250))


def check_frame(frame, columns, rows):
    """Retorna (corners, motivo_da_rejeicao). corners = None se rejeitado."""
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    ok, corners = find_corners(gray, columns, rows)
    if not ok:
        return None, 'tabuleiro nao detectado'
    frac = saturation_fraction(gray, corners)
    if frac > SATURATION_MAX_FRAC:
        return None, f'superexposto/reflexo ({frac * 100:.1f}% saturado)'
    return corners, None


def board_objp():
    rows = cfg('checkerboard_rows')
    columns = cfg('checkerboard_columns')
    scale = cfg('checkerboard_box_size_scale')
    objp = np.zeros((rows * columns, 3), np.float32)
    objp[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    return scale * objp


def order_consistent(c0, c1):
    """Heuristica: a direcao primeiro->ultimo canto deve ser parecida nas duas cameras.
    Se estiver invertida, o tabuleiro foi numerado ao contrario e o par envenena o stereo."""
    v0 = (c0[-1] - c0[0]).reshape(2)
    v1 = (c1[-1] - c1[0]).reshape(2)
    return float(np.dot(v0, v1)) > 0


# ===========================================================================
# ARQUIVOS DE FRAMES
# ===========================================================================

def frames_dict(folder, name):
    """{indice: caminho} dos arquivos <name>_<n>.png"""
    out = {}
    if not os.path.isdir(folder):
        return out
    pat = re.compile(rf'^{re.escape(name)}_(\d+)\.png$')
    for f in os.listdir(folder):
        m = pat.match(f)
        if m:
            out[int(m.group(1))] = os.path.join(folder, f)
    return out


def clear_frames(folder, name):
    old = frames_dict(folder, name)
    for p in old.values():
        os.remove(p)
    if old:
        print(f'  {len(old)} frames antigos de {name} removidos de {folder}/ (evita misturar calibracoes).')


# ===========================================================================
# CAPTURA MONO  (SPACE = capturar, ESC = sair)
# ===========================================================================

def save_frames_single_camera(camera_name):
    os.makedirs(FRAMES_DIR, exist_ok=True)
    clear_frames(FRAMES_DIR, camera_name)

    device_id = calibration_settings[camera_name]
    width = calibration_settings['frame_width']
    height = calibration_settings['frame_height']
    number_to_save = calibration_settings['mono_calibration_frames']
    view_resize = calibration_settings['view_resize']
    rows = calibration_settings['checkerboard_rows']
    columns = calibration_settings['checkerboard_columns']

    cam = AsyncCamera(device_id, camera_name, width, height)
    if not cam.start():
        quit()

    saved_count = 0
    win_name = f'Calibracao - {camera_name}'
    status = ''
    print(f'\n[{camera_name}] SPACE = capturar (so salva se detectar e sem reflexo)  |  ESC = sair')
    print('  Dica: incline o tabuleiro (+-30-45 graus), cubra cantos do frame e varie a distancia.')

    while True:
        frame = cam.read()
        if frame is None:
            time.sleep(0.05)
            continue

        try:
            frame_small = cv.resize(frame, None, fx=1 / view_resize, fy=1 / view_resize)
        except cv.error:
            time.sleep(0.05)
            continue

        cv.putText(frame_small, "SPACE = capturar  |  ESC = sair",
                   (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv.putText(frame_small, f"Salvos: {saved_count}/{number_to_save}",
                   (20, 65), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if status:
            cv.putText(frame_small, status, (20, 100),
                       cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv.imshow(win_name, frame_small)

        k = cv.waitKey(10) & 0xFF
        if k == 27:
            cam.stop()
            cv.destroyAllWindows()
            quit()
        if k == 32:
            corners, reason = check_frame(frame, columns, rows)
            if corners is None:
                status = f'REJEITADO: {reason}'
                print(f'  {status}')
                continue
            savename = os.path.join(FRAMES_DIR, f'{camera_name}_{saved_count}.png')
            if cv.imwrite(savename, frame):
                print(f'  Salvo: {savename}')
                saved_count += 1
                status = ''
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
    os.makedirs(PAIRS_DIR, exist_ok=True)
    clear_frames(PAIRS_DIR, camera0_name)
    clear_frames(PAIRS_DIR, camera1_name)

    view_resize = cfg('view_resize')
    number_to_save = cfg('stereo_calibration_frames')
    width = cfg('frame_width')
    height = cfg('frame_height')
    rows = cfg('checkerboard_rows')
    columns = cfg('checkerboard_columns')

    cam0 = AsyncCamera(calibration_settings[camera0_name], camera0_name, width, height)
    cam1 = AsyncCamera(calibration_settings[camera1_name], camera1_name, width, height)

    if not cam0.start() or not cam1.start():
        cam0.stop()
        cam1.stop()
        quit()

    saved_count = 0
    status = ''
    print('\n[Stereo] Posicione o tabuleiro visivel nas DUAS cameras e mantenha-o PARADO ao apertar SPACE')
    print('         (as duas cameras nao tem trigger sincronizado).')
    print('         SPACE = capturar par  |  ESC = sair')

    while True:
        frame0 = cam0.read()
        frame1 = cam1.read()

        if frame0 is None or frame1 is None:
            time.sleep(0.05)
            continue

        try:
            f0s = cv.resize(frame0, None, fx=1. / view_resize, fy=1. / view_resize)
            f1s = cv.resize(frame1, None, fx=1. / view_resize, fy=1. / view_resize)
        except cv.error:
            time.sleep(0.05)
            continue

        for f in (f0s, f1s):
            cv.putText(f, "SPACE = capturar par  |  ESC = sair",
                       (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv.putText(f, f"Salvos: {saved_count}/{number_to_save}",
                       (20, 65), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if status:
            cv.putText(f0s, status, (20, 100), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        display = np.hstack([f0s, f1s])
        cv.imshow('Visao Stereo Realtime', display)

        k = cv.waitKey(30) & 0xFF

        if k == 27:
            cam0.stop()
            cam1.stop()
            cv.destroyAllWindows()
            quit()

        if k == 32:
            c0, r0 = check_frame(frame0, columns, rows)
            c1, r1 = check_frame(frame1, columns, rows)
            if c0 is None or c1 is None:
                status = f'REJEITADO: cam0={r0 or "ok"} | cam1={r1 or "ok"}'
                print(f'  {status}')
                continue
            if not order_consistent(c0, c1):
                status = 'REJEITADO: ordem dos cantos invertida entre as cameras'
                print(f'  {status}')
                continue

            s0 = os.path.join(PAIRS_DIR, f'{camera0_name}_{saved_count}.png')
            s1 = os.path.join(PAIRS_DIR, f'{camera1_name}_{saved_count}.png')
            ok0 = cv.imwrite(s0, frame0)
            ok1 = cv.imwrite(s1, frame1)
            if ok0 and ok1:
                print(f'  Par {saved_count} salvo.')
                saved_count += 1
                status = ''
            else:
                print(f'  AVISO: falha ao salvar par {saved_count}. Tente novamente.')

        if saved_count == number_to_save:
            break

    cam0.stop()
    cam1.stop()
    cv.destroyAllWindows()
    print(f'  {saved_count} pares salvos.')


# ===========================================================================
# REJEICAO DE OUTLIERS / DIAGNOSTICOS
# ===========================================================================

def select_inliers(errs, labels):
    """Retorna mascara booleana de frames a manter, baseada em erro de reprojecao."""
    errs = np.asarray(errs, dtype=np.float64)
    factor = float(cfg('outlier_factor', 2.0))
    min_px = float(cfg('outlier_min_px', 0.5))
    thr = max(factor * float(np.median(errs)), min_px)
    keep = errs <= thr
    if keep.all():
        return keep
    if keep.sum() < MIN_VIEWS:
        print(f'  [AVISO] Rejeicao cancelada: restariam {int(keep.sum())} frames (< {MIN_VIEWS}). '
              f'Capture mais frames com boa qualidade.')
        return np.ones_like(keep, dtype=bool)
    for i in np.where(~keep)[0]:
        print(f'  Outlier removido: {labels[i]}  erro={errs[i]:.3f}px  (limite {thr:.3f}px)')
    return keep


def per_view_errors(objpoints, imgpoints, rvecs, tvecs, K, dist):
    errs = []
    for op, ip, rv, tv in zip(objpoints, imgpoints, rvecs, tvecs):
        proj, _ = cv.projectPoints(op, rv, tv, K, dist)
        d = ip.reshape(-1, 2) - proj.reshape(-1, 2)
        errs.append(float(np.sqrt(np.mean(np.sum(d * d, axis=1)))))
    return np.array(errs)


def coverage_report(imgpoints, size, name):
    """Quantas das 9 regioes (grid 3x3) do frame tiveram o centro do tabuleiro."""
    w, h = size
    hit = set()
    for ip in imgpoints:
        cx, cy = ip.reshape(-1, 2).mean(axis=0)
        gx = min(int(3 * cx / w), 2)
        gy = min(int(3 * cy / h), 2)
        hit.add((gx, gy))
    msg = f'  [{name}] Cobertura do frame: {len(hit)}/9 regioes.'
    if len(hit) < 6:
        msg += '  <-- POUCA. Mova o tabuleiro para os cantos.'
    print(msg)


def check_intrinsics(K, size, name):
    w, h = size
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    print(f'  [{name}] fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}  (centro ideal ~ {w / 2:.0f},{h / 2:.0f})')
    if abs(cx - w / 2) > 0.10 * w or abs(cy - h / 2) > 0.10 * h:
        print(f'  [AVISO {name}] Ponto principal longe do centro (>10%). Provavel calibracao ruim '
              f'(reflexo, pouca variedade de pose) ou resolucao diferente da de runtime.')
    if max(fx, fy) / min(fx, fy) > 1.05:
        print(f'  [AVISO {name}] fx e fy diferem >5%. Pixel deveria ser quadrado: '
              f'sinal de poucas poses/tabuleiro muito frontal.')


# ===========================================================================
# CALIBRACAO INTRINSECA
# ===========================================================================

def calibrate_camera_for_intrinsic_parameters(camera_name):
    cam_frames = frames_dict(FRAMES_DIR, camera_name)
    if not cam_frames:
        print(f'Nenhuma imagem encontrada para {camera_name} em {FRAMES_DIR}/')
        quit()

    rows = cfg('checkerboard_rows')
    columns = cfg('checkerboard_columns')
    manual = bool(cfg('manual_review', False))
    objp = board_objp()

    objpoints, imgpoints, labels = [], [], []
    size = None

    for idx in sorted(cam_frames):
        path = cam_frames[idx]
        img = cv.imread(path, 1)
        if img is None:
            continue
        if size is None:
            size = (img.shape[1], img.shape[0])
        elif (img.shape[1], img.shape[0]) != size:
            print(f'  {path}: tamanho diferente dos demais, ignorado.')
            continue

        corners, reason = check_frame(img, columns, rows)
        if corners is None:
            print(f'  {path}: {reason}. Ignorado.')
            continue

        if manual:
            disp = img.copy()
            cv.drawChessboardCorners(disp, (columns, rows), corners, True)
            cv.putText(disp, f'{path}  S=pular  outra tecla=aceitar',
                       (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
            cv.imshow('Verificacao intrinseca', cv.resize(disp, None, fx=0.6, fy=0.6))
            k = cv.waitKey(WAITKEY_TIMEOUT_MS) & 0xFF
            if k == ord('s'):
                print(f'  {path} pulado.')
                continue

        objpoints.append(objp)
        imgpoints.append(corners)
        labels.append(os.path.basename(path))

    cv.destroyAllWindows()

    if len(objpoints) < 3:
        print('Poucos frames validos. Capture mais imagens.')
        quit()

    width, height = size
    coverage_report(imgpoints, size, camera_name)

    # Flags: trava K3 e tangencial (evita que a matematica exploda)
    flags = cv.CALIB_FIX_K3 | cv.CALIB_ZERO_TANGENT_DIST
    fix_pp = bool(cfg('fix_principal_point', False))
    fix_ar = bool(cfg('fix_aspect_ratio', False))
    K_init = None
    if fix_pp or fix_ar:
        f_guess = float(cfg('focal_guess_px', width))
        K_init = np.array([[f_guess, 0, width / 2.0],
                           [0, f_guess, height / 2.0],
                           [0, 0, 1.0]], dtype=np.float64)
        flags |= cv.CALIB_USE_INTRINSIC_GUESS
        if fix_pp:
            flags |= cv.CALIB_FIX_PRINCIPAL_POINT
        if fix_ar:
            flags |= cv.CALIB_FIX_ASPECT_RATIO

    it = 0
    while True:
        K0 = None if K_init is None else K_init.copy()
        ret, cmtx, dist, rvecs, tvecs = cv.calibrateCamera(
            objpoints, imgpoints, (width, height), K0, None, flags=flags)
        errs = per_view_errors(objpoints, imgpoints, rvecs, tvecs, cmtx, dist)
        keep = select_inliers(errs, labels)
        if keep.all() or it >= MAX_REJECT_PASSES - 1:
            break
        objpoints = [o for o, k in zip(objpoints, keep) if k]
        imgpoints = [p for p, k in zip(imgpoints, keep) if k]
        labels = [l for l, k in zip(labels, keep) if k]
        it += 1

    print(f'  [{camera_name}] {len(objpoints)} frames usados. RMSE global: {ret:.4f}  '
          f'| erro/frame: mediana={np.median(errs):.3f}px max={errs.max():.3f}px  '
          f'(bom < 1.0, otimo < 0.5)')
    print(f'  Distorcao: {dist}')
    check_intrinsics(cmtx, (width, height), camera_name)
    return cmtx, dist, (width, height)


# ===========================================================================
# SALVAR INTRINSECOS
# ===========================================================================

def save_camera_intrinsics(camera_matrix, distortion_coefs, camera_name):
    os.makedirs(PARAMS_DIR, exist_ok=True)
    out = os.path.join(PARAMS_DIR, camera_name + '_intrinsics.dat')
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

def stereo_calibrate(mtx0, dist0, mtx1, dist1, name0, name1, mono_size):
    d0 = frames_dict(PAIRS_DIR, name0)
    d1 = frames_dict(PAIRS_DIR, name1)
    common = sorted(set(d0) & set(d1))

    if not common:
        print('Frames stereo nao encontrados.')
        quit()
    if len(d0) != len(d1):
        print(f'  AVISO: pares desiguais ({len(d0)} vs {len(d1)}). Usando {len(common)} em comum.')

    rows = cfg('checkerboard_rows')
    columns = cfg('checkerboard_columns')
    manual = bool(cfg('manual_review', False))
    objp = board_objp()

    objpoints, imgpoints_left, imgpoints_right, labels = [], [], [], []
    size = None

    for idx in common:
        frame0 = cv.imread(d0[idx], 1)
        frame1 = cv.imread(d1[idx], 1)
        if frame0 is None or frame1 is None:
            continue
        s0 = (frame0.shape[1], frame0.shape[0])
        s1 = (frame1.shape[1], frame1.shape[0])
        if s0 != s1:
            print(f'  Par {idx}: tamanhos diferentes entre cameras {s0} vs {s1}. Pulando.')
            continue
        if size is None:
            size = s0
        elif s0 != size:
            print(f'  Par {idx}: tamanho diferente dos demais. Pulando.')
            continue

        c0, r0 = check_frame(frame0, columns, rows)
        c1, r1 = check_frame(frame1, columns, rows)
        if c0 is None or c1 is None:
            print(f'  Par {idx}: cam0={r0 or "ok"} | cam1={r1 or "ok"}. Pulando.')
            continue
        if not order_consistent(c0, c1):
            print(f'  Par {idx}: ordem dos cantos invertida entre cameras. Pulando.')
            continue

        if manual:
            disp0, disp1 = frame0.copy(), frame1.copy()
            cv.drawChessboardCorners(disp0, (columns, rows), c0, True)
            cv.drawChessboardCorners(disp1, (columns, rows), c1, True)
            cv.putText(disp0, f'Par {idx}  S=pular  outra=aceitar',
                       (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
            both = np.hstack([cv.resize(disp0, None, fx=0.6, fy=0.6),
                              cv.resize(disp1, None, fx=0.6, fy=0.6)])
            cv.imshow('Stereo Validation', both)
            k = cv.waitKey(WAITKEY_TIMEOUT_MS) & 0xFF
            if k == ord('s'):
                print(f'  Par {idx} pulado.')
                continue

        objpoints.append(objp)
        imgpoints_left.append(c0)
        imgpoints_right.append(c1)
        labels.append(f'par {idx}')

    cv.destroyAllWindows()

    if len(objpoints) < 3:
        print('Poucos pares validos para calibracao stereo.')
        quit()

    if size != mono_size:
        print(f'  [ERRO] Resolucao dos pares stereo {size} != resolucao das intrinsecas {mono_size}. '
              f'As intrinsecas nao valem para essas imagens.')
        quit()

    coverage_report(imgpoints_left, size, 'stereo cam0')
    coverage_report(imgpoints_right, size, 'stereo cam1')
    print(f'  Calibrando com {len(objpoints)} pares validos...')

    it = 0
    while True:
        res = cv.stereoCalibrateExtended(
            objpoints, imgpoints_left, imgpoints_right,
            mtx0.copy(), dist0.copy(), mtx1.copy(), dist1.copy(),
            size, None, None,
            flags=cv.CALIB_FIX_INTRINSIC,
            criteria=CRITERIA)
        ret, _, _, _, _, R, T, E, F, rvecs, tvecs, pve = res
        errs = np.asarray(pve).reshape(-1, 2).max(axis=1)
        keep = select_inliers(errs, labels)
        if keep.all() or it >= MAX_REJECT_PASSES - 1:
            break
        objpoints = [o for o, k in zip(objpoints, keep) if k]
        imgpoints_left = [p for p, k in zip(imgpoints_left, keep) if k]
        imgpoints_right = [p for p, k in zip(imgpoints_right, keep) if k]
        labels = [l for l, k in zip(labels, keep) if k]
        it += 1

    print(f'  {len(objpoints)} pares usados. RMSE stereo: {ret:.4f}  '
          f'| erro/par: mediana={np.median(errs):.3f}px max={errs.max():.3f}px  '
          f'(bom < 1.0, otimo < 0.5)')
    print(f'  Baseline: {np.linalg.norm(T):.4f} (mesma unidade de checkerboard_box_size_scale)')
    print(f'  R:\n{R}')
    print(f'  T: {T.flatten()}')
    print('  Convencao OpenCV: x_cam1 = R * x_cam0 + T')
    return R, T


# ===========================================================================
# UTILITARIOS
# ===========================================================================

def save_extrinsic_calibration_parameters(R0, T0, R1, T1, prefix=''):
    os.makedirs(PARAMS_DIR, exist_ok=True)

    def write_rt(filename, R, T):
        with open(filename, 'w') as f:
            f.write('R:\n')
            for row in R:
                f.write(' '.join(str(v) for v in row) + '\n')
            f.write('T:\n')
            for v in T.flatten():
                f.write(str(v) + '\n')
        print(f'  Extrinsecos salvos: {filename}')

    write_rt(os.path.join(PARAMS_DIR, prefix + 'camera0_rot_trans.dat'), R0, T0)
    write_rt(os.path.join(PARAMS_DIR, prefix + 'camera1_rot_trans.dat'), R1, T1)
    return R0, T0, R1, T1


# ===========================================================================
# EXPORTA SNIPPET PRONTO PRA COLAR NO load_params()
# ===========================================================================

def _fmt_np(name, M):
    M = np.atleast_2d(np.asarray(M, dtype=np.float64))
    rows = ",\n".join(
        "            [" + ", ".join(f"{v:.8f}" for v in r) + "]" for r in M
    )
    return (f"    {name} = np.array(\n"
            f"        [\n{rows}\n        ],\n"
            f"        dtype=np.float64,\n"
            f"    )")


def export_load_params_snippet(cmtx0, dist0, cmtx1, dist1, R, T,
                               filename=os.path.join(PARAMS_DIR, 'load_params_snippet.dat')):
    os.makedirs(PARAMS_DIR, exist_ok=True)

    dist0 = np.asarray(dist0, dtype=np.float64).reshape(1, -1)
    dist1 = np.asarray(dist1, dtype=np.float64).reshape(1, -1)
    T = np.asarray(T, dtype=np.float64).reshape(3, 1)

    txt = "def load_params():\n"
    txt += _fmt_np("cmtx0", cmtx0) + "\n"
    txt += _fmt_np("dist0", dist0) + "\n"
    txt += _fmt_np("cmtx1", cmtx1) + "\n"
    txt += _fmt_np("dist1", dist1) + "\n"
    txt += _fmt_np("R_rel", R) + "\n"
    txt += _fmt_np("T_rel", T) + "\n"
    txt += ("    baseline_m = abs(float(T_rel[0][0]))\n"
            "    if baseline_m > 2.0:\n"
            "        T_rel = T_rel / 100.0\n"
            "        baseline_m = abs(float(T_rel[0][0]))\n"
            "    return cmtx0, dist0, cmtx1, dist1, R_rel, T_rel\n")

    with open(filename, 'w') as f:
        f.write(txt)

    print(f'\n  Snippet salvo em: {filename}')
    print('  ----- COPIE DAQUI -----')
    print(txt)
    print('  ----- ATE AQUI -----')


# ===========================================================================
# MAIN
# ===========================================================================

if __name__ == '__main__':

    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    reuse = '--reuse-frames' in sys.argv[1:]

    if len(args) != 1:
        print('Uso: python3 calib.py calibration_settings.yaml [--reuse-frames]')
        quit()

    parse_calibration_settings_file(args[0])

    scale = calibration_settings['checkerboard_box_size_scale']
    if scale >= 1.0:
        print(f'\n[AVISO] checkerboard_box_size_scale = {scale}')
        print('        Para profundidade em metros, use o tamanho real do quadrado em metros.')
        print('        Ex: quadrado de 25mm -> 0.025\n')

    if reuse:
        print('\n[--reuse-frames] Pulando capturas; usando frames em disco.')
    else:
        print('\n=== PASSO 1: Frames individuais ===')
        save_frames_single_camera('camera0')
        save_frames_single_camera('camera1')

    print('\n=== PASSO 2: Calibracao intrinseca ===')
    cmtx0, dist0, size0 = calibrate_camera_for_intrinsic_parameters('camera0')
    save_camera_intrinsics(cmtx0, dist0, 'camera0')

    cmtx1, dist1, size1 = calibrate_camera_for_intrinsic_parameters('camera1')
    save_camera_intrinsics(cmtx1, dist1, 'camera1')

    if size0 != size1:
        print(f'[ERRO] Resolucoes diferentes entre cameras: {size0} vs {size1}')
        quit()

    if not reuse:
        print('\n=== PASSO 3: Pares sincronizados ===')
        save_frames_two_cams('camera0', 'camera1')

    print('\n=== PASSO 4: Calibracao stereo ===')
    R, T = stereo_calibrate(cmtx0, dist0, cmtx1, dist1, 'camera0', 'camera1', size0)

    print('\n=== PASSO 5: Salvar extrinsecos ===')
    R0 = np.eye(3, dtype=np.float32)
    T0 = np.zeros((3, 1), dtype=np.float32)
    save_extrinsic_calibration_parameters(R0, T0, R, T)

    print('\n=== PASSO 6: Snippet para load_params() ===')
    export_load_params_snippet(cmtx0, dist0, cmtx1, dist1, R, T)

    print('\n=== Calibracao concluida! ===')
