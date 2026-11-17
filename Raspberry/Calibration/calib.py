# calib.py - Calibracao estereo robusta e ASSINCRONA para Raspberry Pi + RTSP/V4L2
# Compativel com calibration_settings.yaml
#
# Uso:
#     python3 calib.py calibration_settings.yaml
#     python3 calib.py calibration_settings.yaml --reuse-frames
#     python3 calib.py calibration_settings.yaml --migrate-old-frames  (inverte cam0<->cam1 e gira 180 nos PNGs antigos e recalibra)

import cv2 as cv
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

WAITKEY_TIMEOUT_MS = 30000
SATURATION_MAX_FRAC = 0.08
MIN_VIEWS = 8
MAX_REJECT_PASSES = 12

FRAMES_DIR = 'frames'
PAIRS_DIR = 'frames_pair'
PARAMS_DIR = 'camera_parameters'

CRITERIA = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 1e-5)

# Grade fisica de distancias focais (px) para 1280x720 (HFOV ~ 45 a 95 graus)
FOCAL_GRID_PX = [600.0, 700.0, 800.0, 900.0, 1000.0, 1100.0, 1250.0, 1400.0]


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
# MIGRACAO DE FRAMES ANTIGOS (SWAP CAM0<->CAM1 + ROTATE 180)
# ===========================================================================

def migrate_old_frames_on_disk():
    """Converte in-place os PNGs capturados com o script antigo (onde cam0 e cam1
    estavam invertidos no yaml e sem ROTATE_180)."""
    print('\n[MIGRACAO] Convertendo frames antigos (swap camera0 <-> camera1 + ROTATE_180)...')
    for folder in (FRAMES_DIR, PAIRS_DIR):
        if not os.path.isdir(folder):
            continue
        d0 = frames_dict(folder, 'camera0')
        d1 = frames_dict(folder, 'camera1')
        # Move para arquivos temporarios primeiro para nao sobrescrever
        tmp0, tmp1 = {}, {}
        for idx, p in d0.items():
            img = cv.imread(p, 1)
            if img is not None:
                img = cv.rotate(img, cv.ROTATE_180)
                tp = os.path.join(folder, f'_tmp_cam1_{idx}.png')
                cv.imwrite(tp, img)
                tmp1[idx] = tp
            os.remove(p)
        for idx, p in d1.items():
            img = cv.imread(p, 1)
            if img is not None:
                img = cv.rotate(img, cv.ROTATE_180)
                tp = os.path.join(folder, f'_tmp_cam0_{idx}.png')
                cv.imwrite(tp, img)
                tmp0[idx] = tp
            os.remove(p)
        for idx, tp in tmp0.items():
            os.rename(tp, os.path.join(folder, f'camera0_{idx}.png'))
        for idx, tp in tmp1.items():
            os.rename(tp, os.path.join(folder, f'camera1_{idx}.png'))
        print(f'  [{folder}] {len(tmp0)} frames camera0 e {len(tmp1)} frames camera1 convertidos.')


# ===========================================================================
# CLASSE DE CAPTURA ASSINCRONA (ALINHADA COM video_streamer.py)
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
        print(f'  [{self.name}] Conectando ({self.src})...')
        is_local = isinstance(self.src, int) or str(self.src).startswith("/dev/")
        backend = cv.CAP_V4L2 if is_local else cv.CAP_ANY
        self._cap = cv.VideoCapture(self.src, backend)
        if not self._cap.isOpened():
            print(f'  [ERRO] Nao foi possivel abrir a camera {self.name} ({self.src})')
            return False

        if is_local:
            self._cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*"MJPG"))
        self._cap.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv.CAP_PROP_BUFFERSIZE, 1)

        last = None
        for _ in range(5):
            ret, f = self._cap.read()
            if ret and f is not None and f.size > 0:
                last = cv.rotate(f, cv.ROTATE_180)

        if last is None:
            self._cap.release()
            print(f'  [ERRO] {self.name} abriu mas nao entregou frames.')
            return False

        h, w = last.shape[:2]
        if (w, h) != (self.width, self.height):
            print(f'  [AVISO] {self.name}: resolucao real {w}x{h} != configurada {self.width}x{self.height}.')
        self.frame = last

        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()
        print(f'  [{self.name}] Conectada e rodando em background (ROTATE_180 ativo).')
        return True

    def _loop(self):
        while self.running:
            if self._cap is not None and self._cap.isOpened():
                ret, frame = self._cap.read()
                if ret and frame is not None and frame.size > 0:
                    frame = cv.rotate(frame, cv.ROTATE_180)
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
# DETECCAO ROBUSTA E ALINHAMENTO DE ORIENTACAO DO TABULEIRO 4x6
# ===========================================================================

def _canonicalize_corners(corners, columns, rows):
    """Em tabuleiros par x par (como 4x6), garante que o canto [0] seja sempre
    o canto superior-esquerdo (orientacao consistente entre frames)."""
    c = corners.astype(np.float32).reshape(-1, 1, 2)
    v_diag = c[-1, 0] - c[0, 0]
    # Se a diagonal principal aponta para cima/esquerda, inverte a ordem (180 graus)
    if (v_diag[0] + v_diag[1]) < 0:
        c = np.ascontiguousarray(c[::-1])
    return c


def find_corners(gray, columns, rows):
    flags = cv.CALIB_CB_ADAPTIVE_THRESH | cv.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv.findChessboardCorners(gray, (columns, rows), flags)
    if ok:
        corners = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), CRITERIA)
        return True, _canonicalize_corners(corners, columns, rows)

    flags_sb = cv.CALIB_CB_EXHAUSTIVE | cv.CALIB_CB_ACCURACY
    try:
        ok, corners = cv.findChessboardCornersSB(gray, (columns, rows), flags_sb)
    except cv.error:
        ok, corners = False, None
    if ok:
        return True, _canonicalize_corners(corners, columns, rows)

    return False, None


def align_stereo_corners(c0, c1, columns, rows):
    """Alinha a ordem dos cantos de c1 com c0 (testando inversao de 180 graus)
    e valida se os eixos das linhas e colunas do tabuleiro sao paralelos nas duas cameras."""
    c0 = c0.reshape(-1, 1, 2)
    c1_norm = c1.reshape(-1, 1, 2)
    c1_rev = np.ascontiguousarray(c1_norm[::-1])

    def axis_cosines(ca, cb):
        r_a = ca[columns - 1, 0] - ca[0, 0]
        r_b = cb[columns - 1, 0] - cb[0, 0]
        d_a = ca[(rows - 1) * columns, 0] - ca[0, 0]
        d_b = cb[(rows - 1) * columns, 0] - cb[0, 0]

        nr = np.linalg.norm(r_a) * np.linalg.norm(r_b)
        nd = np.linalg.norm(d_a) * np.linalg.norm(d_b)
        if nr < 1e-6 or nd < 1e-6:
            return -1.0, -1.0
        cos_r = float(np.dot(r_a, r_b) / nr)
        cos_d = float(np.dot(d_a, d_b) / nd)
        return cos_r, cos_d

    cos_r1, cos_d1 = axis_cosines(c0, c1_norm)
    cos_r2, cos_d2 = axis_cosines(c0, c1_rev)

    if (cos_r2 + cos_d2) > (cos_r1 + cos_d1):
        c1_best = c1_rev
        cos_r, cos_d = cos_r2, cos_d2
    else:
        c1_best = c1_norm
        cos_r, cos_d = cos_r1, cos_d1

    # Exige que ambos os eixos (linhas e colunas) estejam alinhados em ate ~35 graus (cos > 0.80)
    if cos_r < 0.80 or cos_d < 0.80:
        return None
    return c1_best


def saturation_fraction(gray, corners):
    pts = corners.reshape(-1, 2).astype(np.float32)
    x, y, w, h = cv.boundingRect(pts)
    x0, y0 = max(x, 0), max(y, 0)
    x1, y1 = min(x + w, gray.shape[1]), min(y + h, gray.shape[0])
    region = gray[y0:y1, x0:x1]
    if region.size == 0:
        return 0.0
    return float(np.mean(region >= 252))


def check_frame(frame, columns, rows):
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


# ===========================================================================
# ARQUIVOS DE FRAMES
# ===========================================================================

def frames_dict(folder, name):
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
        print(f'  {len(old)} frames antigos de {name} removidos de {folder}/.')


# ===========================================================================
# CAPTURA MONO E STEREO
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
    print(f'\n[{camera_name}] SPACE = capturar | ESC = sair')

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

        if saved_count == number_to_save:
            break

    cam.stop()
    cv.destroyAllWindows()


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
    print('\n[Stereo] Segure o tabuleiro TOTALMENTE PARADO ao apertar SPACE.')

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
            # Aguarda 80ms e rele para garantir que o tabuleiro esteja parado nas duas USB
            c0_a, r0 = check_frame(frame0, columns, rows)
            c1_a, r1 = check_frame(frame1, columns, rows)
            if c0_a is None or c1_a is None:
                status = f'REJEITADO: cam0={r0 or "ok"} | cam1={r1 or "ok"}'
                print(f'  {status}')
                continue

            time.sleep(0.08)
            frame0_b = cam0.read()
            frame1_b = cam1.read()
            c0_b, r0_b = check_frame(frame0_b, columns, rows)
            c1_b, r1_b = check_frame(frame1_b, columns, rows)
            if c0_b is None or c1_b is None:
                status = 'REJEITADO: perdeu deteccao durante captura'
                print(f'  {status}')
                continue

            # Verifica se o tabuleiro se moveu entre as duas leituras (evita lag assincrono USB)
            motion0 = float(np.max(np.linalg.norm(c0_a - c0_b, axis=2)))
            motion1 = float(np.max(np.linalg.norm(c1_a - c1_b, axis=2)))
            if motion0 > 2.0 or motion1 > 2.0:
                status = f'REJEITADO: tabuleiro em movimento ({max(motion0, motion1):.1f}px). Segure parado!'
                print(f'  {status}')
                continue

            c1_aligned = align_stereo_corners(c0_b, c1_b, columns, rows)
            if c1_aligned is None:
                status = 'REJEITADO: orientacao inconsistente entre cameras'
                print(f'  {status}')
                continue

            s0 = os.path.join(PAIRS_DIR, f'{camera0_name}_{saved_count}.png')
            s1 = os.path.join(PAIRS_DIR, f'{camera1_name}_{saved_count}.png')
            if cv.imwrite(s0, frame0_b) and cv.imwrite(s1, frame1_b):
                print(f'  Par {saved_count} salvo (movimento={max(motion0, motion1):.2f}px).')
                saved_count += 1
                status = ''

        if saved_count == number_to_save:
            break

    cam0.stop()
    cam1.stop()
    cv.destroyAllWindows()


# ===========================================================================
# DIAGNOSTICOS E PODA DE OUTLIERS
# ===========================================================================

def per_view_errors(objpoints, imgpoints, rvecs, tvecs, K, dist):
    errs = []
    for op, ip, rv, tv in zip(objpoints, imgpoints, rvecs, tvecs):
        proj, _ = cv.projectPoints(op, rv, tv, K, dist)
        d = ip.reshape(-1, 2) - proj.reshape(-1, 2)
        errs.append(float(np.sqrt(np.mean(np.sum(d * d, axis=1)))))
    return np.array(errs)


def coverage_report(imgpoints, size, name):
    w, h = size
    hit = set()
    for ip in imgpoints:
        cx, cy = ip.reshape(-1, 2).mean(axis=0)
        gx = min(int(3 * cx / w), 2)
        gy = min(int(3 * cy / h), 2)
        hit.add((gx, gy))
    print(f'  [{name}] Cobertura do frame: {len(hit)}/9 regioes.')


def check_intrinsics(K, size, name):
    w, h = size
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    print(f'  [{name}] fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}  (centro ideal ~ {w / 2:.0f},{h / 2:.0f})')


def compute_epipolar_y_error(imgpoints_l, imgpoints_r, K0, D0, K1, D1, R1, P1, R2, P2):
    """Mede o desalinhamento vertical real (em pixels) nas imagens retificadas."""
    y_diffs = []
    for pts_l, pts_r in zip(imgpoints_l, imgpoints_r):
        u_l = cv.undistortPoints(pts_l, K0, D0, R=R1, P=P1).reshape(-1, 2)
        u_r = cv.undistortPoints(pts_r, K1, D1, R=R2, P=P2).reshape(-1, 2)
        y_diffs.append(np.abs(u_l[:, 1] - u_r[:, 1]))
    all_diffs = np.concatenate(y_diffs)
    return float(np.mean(all_diffs)), float(np.percentile(all_diffs, 95))


# ===========================================================================
# CALIBRACAO INTRINSECA COM PROTECAO DE FOCO FISICO
# ===========================================================================

def _calibrate_mono_constrained(objpoints, imgpoints, size, f_guess):
    w, h = size
    K_init = np.array([[f_guess, 0.0, w / 2.0],
                       [0.0, f_guess, h / 2.0],
                       [0.0, 0.0, 1.0]], dtype=np.float64)

    # 1. Tenta otimizar fx=fy livre a partir do chute inicial
    flags_free = (cv.CALIB_USE_INTRINSIC_GUESS |
                  cv.CALIB_FIX_ASPECT_RATIO |
                  cv.CALIB_ZERO_TANGENT_DIST |
                  cv.CALIB_FIX_K2 |
                  cv.CALIB_FIX_K3)
    ret, K, D, rvecs, tvecs = cv.calibrateCamera(
        objpoints, imgpoints, size, K_init.copy(), None, flags=flags_free, criteria=CRITERIA
    )

    fx, cx, cy = K[0, 0], K[0, 2], K[1, 2]
    # Verifica se o solver explodiu a distancia focal (ex: > 2000px) ou jogou cx,cy para fora
    if not (450.0 <= fx <= 1800.0) or abs(cx - w / 2.0) > 0.20 * w or abs(cy - h / 2.0) > 0.20 * h:
        # 2. Fallback fisico: busca na grade de focos fisicos reais travando CALIB_FIX_FOCAL_LENGTH
        best = None
        for f_cand in FOCAL_GRID_PX:
            K_c = np.array([[f_cand, 0.0, w / 2.0],
                            [0.0, f_cand, h / 2.0],
                            [0.0, 0.0, 1.0]], dtype=np.float64)
            flags_lock = (cv.CALIB_USE_INTRINSIC_GUESS |
                          cv.CALIB_FIX_FOCAL_LENGTH |
                          cv.CALIB_FIX_PRINCIPAL_POINT |
                          cv.CALIB_ZERO_TANGENT_DIST |
                          cv.CALIB_FIX_K2 |
                          cv.CALIB_FIX_K3)
            r_c, K_out, D_out, rv_c, tv_c = cv.calibrateCamera(
                objpoints, imgpoints, size, K_c, None, flags=flags_lock, criteria=CRITERIA
            )
            if best is None or r_c < best[0]:
                best = (r_c, K_out, D_out, rv_c, tv_c)
        ret, K, D, rvecs, tvecs = best

    return ret, K, D, rvecs, tvecs


def calibrate_camera_for_intrinsic_parameters(camera_name):
    # Combina frames de frames/ e frames_pair/ para maxima cobertura geometrica
    cam_frames = list(frames_dict(FRAMES_DIR, camera_name).items())
    pair_frames = list(frames_dict(PAIRS_DIR, camera_name).items())

    all_items = [(f'mono_{i}', p) for i, p in sorted(cam_frames)] + \
                [(f'pair_{i}', p) for i, p in sorted(pair_frames)]

    if not all_items:
        print(f'Nenhuma imagem encontrada para {camera_name}')
        quit()

    rows = cfg('checkerboard_rows')
    columns = cfg('checkerboard_columns')
    objp = board_objp()

    objpoints, imgpoints, labels = [], [], []
    size = None

    for label, path in all_items:
        img = cv.imread(path, 1)
        if img is None:
            continue
        if size is None:
            size = (img.shape[1], img.shape[0])
        elif (img.shape[1], img.shape[0]) != size:
            continue

        corners, _ = check_frame(img, columns, rows)
        if corners is None:
            continue

        objpoints.append(objp)
        imgpoints.append(corners)
        labels.append(label)

    if len(objpoints) < 4:
        print(f'Poucos frames validos para {camera_name}.')
        quit()

    f_guess = float(cfg('focal_guess_px', 950.0))

    # Poda iterativa Leave-Worst-Out (remove o pior frame um por um se erro > 1.0px)
    while len(objpoints) > MIN_VIEWS:
        ret, cmtx, dist, rvecs, tvecs = _calibrate_mono_constrained(objpoints, imgpoints, size, f_guess)
        errs = per_view_errors(objpoints, imgpoints, rvecs, tvecs, cmtx, dist)
        worst_idx = int(np.argmax(errs))
        if errs[worst_idx] <= max(1.0, 2.0 * float(np.median(errs))):
            break
        print(f'  [{camera_name}] Outlier mono removido ({labels[worst_idx]}): erro={errs[worst_idx]:.2f}px')
        objpoints.pop(worst_idx)
        imgpoints.pop(worst_idx)
        labels.pop(worst_idx)

    ret, cmtx, dist, rvecs, tvecs = _calibrate_mono_constrained(objpoints, imgpoints, size, f_guess)
    errs = per_view_errors(objpoints, imgpoints, rvecs, tvecs, cmtx, dist)

    coverage_report(imgpoints, size, camera_name)
    print(f'  [{camera_name}] {len(objpoints)} frames usados. RMSE mono: {ret:.4f} '
          f'(mediana={np.median(errs):.3f}px, max={errs.max():.3f}px)')
    check_intrinsics(cmtx, size, camera_name)
    return cmtx, dist, size


def save_camera_intrinsics(camera_matrix, distortion_coefs, camera_name):
    os.makedirs(PARAMS_DIR, exist_ok=True)
    out = os.path.join(PARAMS_DIR, camera_name + '_intrinsics.dat')
    with open(out, 'w') as f:
        f.write('intrinsic:\n')
        for row in camera_matrix:
            f.write(' '.join(str(v) for v in row) + '\n')
        f.write('distortion:\n')
        f.write(' '.join(str(v) for v in distortion_coefs.flatten()) + '\n')


# ===========================================================================
# CALIBRACAO STEREO CONJUNTA (BUNDLE ADJUSTMENT + LEAVE-WORST-OUT)
# ===========================================================================

def _run_stereo_candidate(objpoints, imgpoints_l, imgpoints_r, size, K0_init, D0_init, K1_init, D1_init, flags):
    res = cv.stereoCalibrateExtended(
        objpoints, imgpoints_l, imgpoints_r,
        K0_init.copy(), D0_init.copy(), K1_init.copy(), D1_init.copy(),
        size, None, None,
        flags=flags,
        criteria=CRITERIA
    )
    ret, K0, D0, K1, D1, R, T, E, F, rvecs, tvecs, pve = res
    pair_errs = np.asarray(pve, dtype=np.float64).reshape(-1, 2).max(axis=1)
    return ret, K0, D0, K1, D1, R, T, pair_errs


def stereo_calibrate(mtx0, dist0, mtx1, dist1, name0, name1, mono_size):
    d0 = frames_dict(PAIRS_DIR, name0)
    d1 = frames_dict(PAIRS_DIR, name1)
    common = sorted(set(d0) & set(d1))

    if not common:
        print('Frames stereo nao encontrados.')
        quit()

    rows = cfg('checkerboard_rows')
    columns = cfg('checkerboard_columns')
    objp = board_objp()

    objpoints, imgpoints_left, imgpoints_right, labels = [], [], [], []
    sample_frames = None
    size = None

    for idx in common:
        frame0 = cv.imread(d0[idx], 1)
        frame1 = cv.imread(d1[idx], 1)
        if frame0 is None or frame1 is None:
            continue
        s0 = (frame0.shape[1], frame0.shape[0])
        s1 = (frame1.shape[1], frame1.shape[0])
        if s0 != s1:
            continue
        if size is None:
            size = s0

        c0, r0 = check_frame(frame0, columns, rows)
        c1, r1 = check_frame(frame1, columns, rows)
        if c0 is None or c1 is None:
            print(f'  Par {idx} ignorado: cam0={r0 or "ok"} | cam1={r1 or "ok"}')
            continue

        c1_aligned = align_stereo_corners(c0, c1, columns, rows)
        if c1_aligned is None:
            print(f'  Par {idx} ignorado: eixos do tabuleiro desalinhados entre cam0 e cam1.')
            continue

        objpoints.append(objp)
        imgpoints_left.append(c0)
        imgpoints_right.append(c1_aligned)
        labels.append(f'par_{idx}')
        if sample_frames is None:
            sample_frames = (frame0, frame1)

    if len(objpoints) < 5:
        print('Poucos pares validos para calibracao stereo.')
        quit()

    w, h = size
    # Forca ambas as cameras a partirem da mesma escala focal no stereo
    f_avg = float(np.clip(0.5 * (mtx0[0, 0] + mtx1[0, 0]), 550.0, 1500.0))
    K_shared = np.array([[f_avg, 0.0, w / 2.0],
                         [0.0, f_avg, h / 2.0],
                         [0.0, 0.0, 1.0]], dtype=np.float64)
    D_zero = np.zeros((1, 5), dtype=np.float64)

    flags_joint = (cv.CALIB_USE_INTRINSIC_GUESS |
                   cv.CALIB_SAME_FOCAL_LENGTH |
                   cv.CALIB_FIX_ASPECT_RATIO |
                   cv.CALIB_ZERO_TANGENT_DIST |
                   cv.CALIB_FIX_K2 |
                   cv.CALIB_FIX_K3)

    # 1. Poda iterativa Leave-Worst-Out: remove um par por vez se estiver puxando o RMSE para cima
    print(f'  Iniciando otimizacao stereo conjunta com {len(objpoints)} pares...')
    while len(objpoints) > MIN_VIEWS:
        ret, K0, D0, K1, D1, R, T, pair_errs = _run_stereo_candidate(
            objpoints, imgpoints_left, imgpoints_right, size,
            K_shared, D_zero, K_shared, D_zero, flags_joint
        )
        worst_idx = int(np.argmax(pair_errs))
        worst_err = float(pair_errs[worst_idx])
        med_err = float(np.median(pair_errs))

        # Para de remover quando o RMSE global for < 0.75px e nenhum par passar de max(1.2px, 2.2*mediana)
        if ret <= 0.75 and worst_err <= max(1.2, 2.2 * med_err):
            break
        if worst_err <= 0.85:
            break

        print(f'  [PODANDO OUTLIER STEREO] Removendo {labels[worst_idx]} '
              f'(erro={worst_err:.2f}px | mediana={med_err:.2f}px | RMSE={ret:.2f}px)')
        objpoints.pop(worst_idx)
        imgpoints_left.pop(worst_idx)
        imgpoints_right.pop(worst_idx)
        labels.pop(worst_idx)

    # 2. Testa os candidatos de distancia focal na grade fisica + otimizacao conjunta para garantir |Tz| << |Tx|
    best_solution = None
    best_score = 1e9

    candidates_to_test = [('joint_free', flags_joint, f_avg)]
    for f_cand in FOCAL_GRID_PX:
        flags_grid = (cv.CALIB_USE_INTRINSIC_GUESS |
                      cv.CALIB_FIX_FOCAL_LENGTH |
                      cv.CALIB_FIX_PRINCIPAL_POINT |
                      cv.CALIB_ZERO_TANGENT_DIST |
                      cv.CALIB_FIX_K2 |
                      cv.CALIB_FIX_K3)
        candidates_to_test.append((f'grid_f={f_cand:.0f}', flags_grid, f_cand))

    for tag, flg, f_val in candidates_to_test:
        K_init = np.array([[f_val, 0.0, w / 2.0],
                           [0.0, f_val, h / 2.0],
                           [0.0, 0.0, 1.0]], dtype=np.float64)
        try:
            ret, K0, D0, K1, D1, R, T, pair_errs = _run_stereo_candidate(
                objpoints, imgpoints_left, imgpoints_right, size,
                K_init, D_zero, K_init, D_zero, flg
            )
        except cv.error:
            continue

        fx0, fx1 = float(K0[0, 0]), float(K1[0, 0])
        tx, ty, tz = float(T[0, 0]), float(T[1, 0]), float(T[2, 0])
        if not (450.0 <= fx0 <= 1850.0 and 450.0 <= fx1 <= 1850.0):
            continue
        if abs(tx) < 1e-4:
            continue

        R1r, R2r, P1, P2, Q, _, _ = cv.stereoRectify(
            K0, D0, K1, D1, size, R, T, flags=cv.CALIB_ZERO_DISPARITY, alpha=0
        )
        epi_mean, epi_p95 = compute_epipolar_y_error(
            imgpoints_left, imgpoints_right, K0, D0, K1, D1, R1r, P1, R2r, P2
        )

        tz_ratio = abs(tz) / max(abs(tx), 1e-6)
        # Penaliza fortemente qualquer solucao que tente jogar Tz > 25% do baseline Tx
        tz_penalty = 15.0 * max(0.0, tz_ratio - 0.25)
        score = ret + 1.5 * epi_mean + tz_penalty

        if score < best_score:
            best_score = score
            best_solution = (tag, ret, K0, D0, K1, D1, R, T, pair_errs, epi_mean, epi_p95, R1r, R2r, P1, P2)

    if best_solution is None:
        print('[ERRO] Nao foi possivel encontrar solucao stereo fisicamente valida.')
        quit()

    tag, ret, K0, D0, K1, D1, R, T, pair_errs, epi_mean, epi_p95, R1r, R2r, P1, P2 = best_solution

    print(f'\n  >>> Solucao selecionada: [{tag}] com {len(objpoints)} pares inliers')
    print(f'  >>> RMSE stereo: {ret:.4f} px (mediana={np.median(pair_errs):.3f}px, max={pair_errs.max():.3f}px)')
    print(f'  >>> Erro epipolar vertical retificado |y_L - y_R|: media={epi_mean:.3f}px (95%={epi_p95:.3f}px)')
    print(f'  >>> Intrinsecos finais: f0={K0[0,0]:.1f}px, f1={K1[0,0]:.1f}px, cx={K0[0,2]:.1f}, cy={K0[1,2]:.1f}')
    print(f'  >>> Baseline ||T||: {np.linalg.norm(T)*100:.2f} cm  |  T = [{T[0,0]:.4f}, {T[1,0]:.4f}, {T[2,0]:.4f}] m')

    if T[0, 0] > 0:
        print('  [NOTA] Tx > 0: nos frames salvos, camera0 esta a DIREITA da camera1 (o pipeline trata isso automaticamente).')

    # Salva imagem de verificacao da retificacao com linhas epipolares horizontais
    if sample_frames is not None:
        os.makedirs(PARAMS_DIR, exist_ok=True)
        m1x, m1y = cv.initUndistortRectifyMap(K0, D0, R1r, P1, size, cv.CV_32FC1)
        m2x, m2y = cv.initUndistortRectifyMap(K1, D1, R2r, P2, size, cv.CV_32FC1)
        rl = cv.remap(sample_frames[0], m1x, m1y, cv.INTER_LINEAR)
        rr = cv.remap(sample_frames[1], m2x, m2y, cv.INTER_LINEAR)
        canvas = np.hstack([rl, rr])
        for y_line in range(40, size[1], 40):
            cv.line(canvas, (0, y_line), (size[0] * 2, y_line), (0, 255, 0), 1)
        check_path = os.path.join(PARAMS_DIR, 'rectification_check.png')
        cv.imwrite(check_path, canvas)
        print(f'  >>> Imagem de verificacao salva em: {check_path}')

    return K0, D0, K1, D1, R, T


# ===========================================================================
# EXPORTACAO DE ARQUIVOS E SNIPPET
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

    write_rt(os.path.join(PARAMS_DIR, prefix + 'camera0_rot_trans.dat'), R0, T0)
    write_rt(os.path.join(PARAMS_DIR, prefix + 'camera1_rot_trans.dat'), R1, T1)


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
    migrate = '--migrate-old-frames' in sys.argv[1:]
    reuse = ('--reuse-frames' in sys.argv[1:]) or migrate

    if len(args) != 1:
        print('Uso: python3 calib.py calibration_settings.yaml [--reuse-frames | --migrate-old-frames]')
        quit()

    parse_calibration_settings_file(args[0])

    if migrate:
        migrate_old_frames_on_disk()

    if reuse:
        print('\n[--reuse-frames] Pulando capturas; usando frames em disco.')
    else:
        print('\n=== PASSO 1: Frames individuais ===')
        save_frames_single_camera('camera0')
        save_frames_single_camera('camera1')

    print('\n=== PASSO 2: Calibracao intrinseca inicial ===')
    cmtx0, dist0, size0 = calibrate_camera_for_intrinsic_parameters('camera0')
    cmtx1, dist1, size1 = calibrate_camera_for_intrinsic_parameters('camera1')

    if size0 != size1:
        print(f'[ERRO] Resolucoes diferentes entre cameras: {size0} vs {size1}')
        quit()

    if not reuse:
        print('\n=== PASSO 3: Pares sincronizados ===')
        save_frames_two_cams('camera0', 'camera1')

    print('\n=== PASSO 4: Calibracao stereo conjunta ===')
    cmtx0, dist0, cmtx1, dist1, R, T = stereo_calibrate(
        cmtx0, dist0, cmtx1, dist1, 'camera0', 'camera1', size0
    )

    print('\n=== PASSO 5: Salvar intrinsecos e extrinsecos ===')
    save_camera_intrinsics(cmtx0, dist0, 'camera0')
    save_camera_intrinsics(cmtx1, dist1, 'camera1')
    R0 = np.eye(3, dtype=np.float32)
    T0 = np.zeros((3, 1), dtype=np.float32)
    save_extrinsic_calibration_parameters(R0, T0, R, T)

    print('\n=== PASSO 6: Snippet para load_params() ===')
    export_load_params_snippet(cmtx0, dist0, cmtx1, dist1, R, T)

    print('\n=== Calibracao concluida! ===')
