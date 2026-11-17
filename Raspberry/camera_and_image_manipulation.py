from collections import deque
import cv2
import numpy as np

from serial_communication import telemetria_dados  # noqa: F401


# ===========================================================================
# PRE-PROCESSAMENTO E CONFIGURACOES
# ===========================================================================

CLAHE_CLIP_LIMIT = 2.0
CLAHE_TILE_GRID_SIZE = (8, 8)

clahe = cv2.createCLAHE(
    clipLimit=CLAHE_CLIP_LIMIT,
    tileGridSize=CLAHE_TILE_GRID_SIZE
)

USE_ADAPTIVE_THRESHOLD = True
ADAPTIVE_BLOCK_SIZE = 51
ADAPTIVE_C = 8
THRESHOLD_VALUE = 60

MAX_CANDIDATE_LINES = 20
MIN_LINE_LENGTH_FACTOR = 0.3

# Agrupamento estrito das duas bordas da mesma corda
CLUSTER_ANGLE_TOL_DEG = 8.0
CLUSTER_RHO_TOL_PX = 12.0

# Limites geométricos amplos para NUNCA descartar uma linha válida como 2D
MAX_DISTANCE = 6.0               # Aceita até 6.0m sem derrubar para "SEM PROFUNDIDADE"
MIN_DISTANCE = 0.15
MIN_PLANE_SINE = 0.015           # Permite interseção de planos mesmo em ângulos rasos
MIN_COMMON_SEGMENT_LENGTH = 0.02
MIN_RAY_DEPTH = 0.05

DISTANCE_SAMPLES = 30

# Configurações do SGBM + WLS restrito à ROI e Vizinhança da Linha
USE_ROI_SGBM = True
USE_WLS_FILTER = True
SGBM_NUM_DISP_BASE = 64
SGBM_BLOCK_SIZE = 3
WLS_LAMBDA = 8000.0
WLS_SIGMA_COLOR = 1.5

# Espessura da vizinhança ao redor da linha (px) e faixa dos maiores valores de disparidade
SGBM_NEIGHBORHOOD_THICKNESS = 15
TOP_DISP_PERCENTILE_MIN = 65.0
TOP_DISP_PERCENTILE_MAX = 96.0

# Filtro temporal
TEMPORAL_WINDOW = 7
_hist_dist = deque(maxlen=TEMPORAL_WINDOW)
_hist_sin2 = deque(maxlen=TEMPORAL_WINDOW)
_hist_cos2 = deque(maxlen=TEMPORAL_WINDOW)
_prev_bearing_l = None
_prev_bearing_r = None
_last_valid_distance = -1.0


def preprocessar_imagem(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    enhanced = clahe.apply(gray)

    blurred = cv2.GaussianBlur(
        enhanced,
        (5, 5),
        0
    )

    if USE_ADAPTIVE_THRESHOLD:
        block_size = ADAPTIVE_BLOCK_SIZE
        if block_size % 2 == 0:
            block_size += 1

        block_size = min(
            block_size,
            min(frame.shape[:2]) // 2 * 2 - 1
        )
        block_size = max(block_size, 3)

        bin_img = cv2.adaptiveThreshold(
            blurred,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            block_size,
            ADAPTIVE_C
        )
    else:
        _, bin_img = cv2.threshold(
            blurred,
            THRESHOLD_VALUE,
            255,
            cv2.THRESH_BINARY_INV
        )

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_CLOSE, kernel)

    return bin_img


# ===========================================================================
# RETIFICAÇÃO E CACHE SGBM + WLS
# ===========================================================================

def scale_intrinsics(cmtx, scale):
    m = cmtx.copy().astype(np.float64)
    m[0, 0] *= scale
    m[1, 1] *= scale
    m[0, 2] *= scale
    m[1, 2] *= scale
    return m


def _create_sgbm_wls(num_disp, block_size=SGBM_BLOCK_SIZE):
    num_disp = max(16, int(np.ceil(num_disp / 16.0) * 16))
    left_matcher = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=num_disp,
        blockSize=block_size,
        P1=8 * 1 * (block_size ** 2),
        P2=32 * 1 * (block_size ** 2),
        disp12MaxDiff=1,
        preFilterCap=63,
        uniquenessRatio=10,
        speckleWindowSize=50,
        speckleRange=2,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )

    right_matcher = None
    wls_filter = None

    if USE_WLS_FILTER and hasattr(cv2, "ximgproc"):
        try:
            right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
            wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
            wls_filter.setLambda(WLS_LAMBDA)
            wls_filter.setSigmaColor(WLS_SIGMA_COLOR)
        except Exception as e:
            print(f"[AVISO] Falha ao iniciar WLS (cv2.ximgproc): {e}. Usando SGBM puro.")
            right_matcher = None
            wls_filter = None

    return left_matcher, right_matcher, wls_filter, num_disp


def build_rectification(
    cmtx0,
    dist0,
    cmtx1,
    dist1,
    R_rel,
    T_rel,
    img_size,
    scale=1.0
):
    R1r, R2r, P1, P2, Q, _, _ = cv2.stereoRectify(
        cmtx0,
        dist0,
        cmtx1,
        dist1,
        img_size,
        R_rel,
        T_rel,
        flags=cv2.CALIB_ZERO_DISPARITY,
        alpha=0
    )

    map1x, map1y = cv2.initUndistortRectifyMap(
        cmtx0,
        dist0,
        R1r,
        P1,
        img_size,
        cv2.CV_32FC1
    )

    map2x, map2y = cv2.initUndistortRectifyMap(
        cmtx1,
        dist1,
        R2r,
        P2,
        img_size,
        cv2.CV_32FC1
    )

    K1_rect = P1[:, :3].astype(np.float64)
    K2_rect = P2[:, :3].astype(np.float64)

    t_rect = np.linalg.solve(K2_rect, P2[:, 3])
    C2_rect = -t_rect

    f_rect = float(P1[0, 0])
    baseline = float(np.linalg.norm(C2_rect))
    swap_left_right = bool(P2[0, 3] > 0)
    disp_sign = -1.0 if P2[0, 3] > 0 else 1.0

    left_matcher, right_matcher, wls_filter, num_disp = _create_sgbm_wls(
        int(SGBM_NUM_DISP_BASE * scale)
    )

    return {
        "map1x": map1x,
        "map1y": map1y,
        "map2x": map2x,
        "map2y": map2y,
        "Q": Q,
        "P1": P1,
        "P2": P2,
        "K1": K1_rect,
        "K2": K2_rect,
        "C1": np.zeros(3, dtype=np.float64),
        "C2": C2_rect,
        "R1r": R1r,
        "R2r": R2r,
        "f_rect": f_rect,
        "baseline": baseline,
        "swap_lr": swap_left_right,
        "disp_sign": disp_sign,
        "sgbm": left_matcher,
        "right_matcher": right_matcher,
        "wls_filter": wls_filter,
        "num_disp": num_disp,
    }


# ===========================================================================
# DETECÇÃO DE SEGMENTOS E FUSÃO ESTRITA DAS BORDAS DA CORDA
# ===========================================================================

def bearing_com_sinal(x1, y1, x2, y2):
    angle = np.degrees(np.arctan2(x2 - x1, y2 - y1))

    if angle > 90:
        angle -= 180
    elif angle <= -90:
        angle += 180

    return float(angle)


def _diff_angular_180(a, b):
    d = abs(a - b) % 180.0
    return min(d, 180.0 - d)


def _media_angular_180(ang1, ang2, w1=1.0, w2=1.0):
    a1 = np.radians(ang1 * 2.0)
    a2 = np.radians(ang2 * 2.0)
    s = w1 * np.sin(a1) + w2 * np.sin(a2)
    c = w1 * np.cos(a1) + w2 * np.cos(a2)
    return float(np.degrees(np.arctan2(s, c)) / 2.0)


def _construir_candidata_de_segmento(x1, y1, x2, y2, score_weight=None):
    if y1 > y2 or (y1 == y2 and x1 > x2):
        x1, y1, x2, y2 = x2, y2, x1, y1

    length = float(np.hypot(x2 - x1, y2 - y1))
    if length < 1e-6:
        return None

    p1 = np.array([x1, y1, 1.0], dtype=np.float64)
    p2 = np.array([x2, y2, 1.0], dtype=np.float64)
    line = np.cross(p1, p2)
    norm = np.hypot(line[0], line[1])
    if norm < 1e-9:
        return None
    line /= norm

    return {
        "segment": (int(round(x1)), int(round(y1)), int(round(x2)), int(round(y2))),
        "segment_subpx": (float(x1), float(y1), float(x2), float(y2)),
        "line": line,
        "length": length,
        "weight": float(score_weight if score_weight is not None else length),
        "midpoint": ((x1 + x2) / 2.0, (y1 + y2) / 2.0),
        "bearing": bearing_com_sinal(x1, y1, x2, y2)
    }


def _fundir_bordas_paralelas(raw_candidates, min_len):
    if not raw_candidates:
        return []

    clusters = []
    for cand in raw_candidates:
        placed = False
        x1, y1, x2, y2 = cand["segment_subpx"]
        mx, my = cand["midpoint"]

        for cl in clusters:
            if _diff_angular_180(cand["bearing"], cl["bearing"]) <= CLUSTER_ANGLE_TOL_DEG:
                a, b, c = cl["ref_line"]
                d_mid = abs(a * mx + b * my + c)
                d_p1 = abs(a * x1 + b * y1 + c)
                d_p2 = abs(a * x2 + b * y2 + c)
                if d_mid <= CLUSTER_RHO_TOL_PX and max(d_p1, d_p2) <= (CLUSTER_RHO_TOL_PX * 1.4):
                    cl["items"].append(cand)
                    w_old = cl["total_len"]
                    w_new = cand["length"]
                    cl["bearing"] = _media_angular_180(cl["bearing"], cand["bearing"], w_old, w_new)
                    cl["total_len"] += w_new
                    placed = True
                    break
        if not placed:
            clusters.append({
                "bearing": cand["bearing"],
                "ref_line": cand["line"],
                "total_len": cand["length"],
                "items": [cand]
            })

    merged_candidates = []

    for cl in clusters:
        items = cl["items"]
        if len(items) == 1:
            merged_candidates.append(items[0])
            continue

        total_w = cl["total_len"]
        cx_m = sum(it["midpoint"][0] * it["length"] for it in items) / total_w
        cy_m = sum(it["midpoint"][1] * it["length"] for it in items) / total_w

        ang_rad = np.radians(cl["bearing"])
        vx = float(np.sin(ang_rad))
        vy = float(np.cos(ang_rad))

        t_vals = []
        for it in items:
            ix1, iy1, ix2, iy2 = it["segment_subpx"]
            t_vals.append((ix1 - cx_m) * vx + (iy1 - cy_m) * vy)
            t_vals.append((ix2 - cx_m) * vx + (iy2 - cy_m) * vy)

        t_min, t_max = float(min(t_vals)), float(max(t_vals))
        fx1 = cx_m + t_min * vx
        fy1 = cy_m + t_min * vy
        fx2 = cx_m + t_max * vx
        fy2 = cy_m + t_max * vy

        merged = _construir_candidata_de_segmento(fx1, fy1, fx2, fy2, score_weight=total_w)
        if merged is not None and merged["length"] >= min_len:
            merged_candidates.append(merged)

    merged_candidates.sort(key=lambda item: item["weight"], reverse=True)
    return merged_candidates[:MAX_CANDIDATE_LINES]


def detectar_candidatas(rect, cx, cy, roi_radius, max_gap=50, prev_bearing=None):
    bin_img = preprocessar_imagem(rect)

    mask = np.zeros_like(bin_img)
    cv2.circle(mask, (cx, cy), roi_radius, 255, -1)

    roi_bin = cv2.bitwise_and(bin_img, bin_img, mask=mask)
    edges = cv2.Canny(roi_bin, 50, 150)

    min_len = max(10, int(roi_radius * MIN_LINE_LENGTH_FACTOR))

    lines = cv2.HoughLinesP(
        edges,
        1,
        np.pi / 180,
        threshold=25,
        minLineLength=min_len,
        maxLineGap=max_gap
    )

    if lines is None:
        return [], roi_bin, edges

    raw_candidates = []
    for item in lines:
        x1, y1, x2, y2 = map(int, item[0])
        cand = _construir_candidata_de_segmento(x1, y1, x2, y2)
        if cand is not None and cand["length"] >= min_len:
            raw_candidates.append(cand)

    raw_candidates.sort(key=lambda item: item["length"], reverse=True)

    candidates = _fundir_bordas_paralelas(raw_candidates, min_len)

    if prev_bearing is not None and candidates:
        for c in candidates:
            ang_diff = _diff_angular_180(c["bearing"], prev_bearing)
            continuity_factor = 1.30 if ang_diff < 10.0 else (1.0 if ang_diff < 22.0 else 0.80)
            c["weight"] *= continuity_factor
        candidates.sort(key=lambda item: item["weight"], reverse=True)

    return candidates, roi_bin, edges


def detectar_linha_mono(rect_l, cx, cy, roi_radius):
    global _prev_bearing_l
    candidates, roi_bin, _ = detectar_candidatas(
        rect_l,
        cx,
        cy,
        roi_radius,
        max_gap=20,
        prev_bearing=_prev_bearing_l
    )

    if not candidates:
        return None, None, None, roi_bin

    best = candidates[0]
    _prev_bearing_l = best["bearing"]

    return (
        best["bearing"],
        best["segment"],
        None,
        roi_bin
    )


# ===========================================================================
# DETECÇÃO AUTOMÁTICA DO SENTIDO DE DISPARIDADE NA MESMA SCANLINE Y
# ===========================================================================

def _detectar_swap_na_mesma_linha_y(left_cand, right_cand, default_swap):
    """
    Avalia x_L(y_ref) e x_R(y_ref) exatamente na mesma altura vertical y_ref.
    Não sofre com recortes diferentes do círculo da ROI.
    """
    lx1, ly1, lx2, ly2 = left_cand["segment_subpx"]
    rx1, ry1, rx2, ry2 = right_cand["segment_subpx"]

    if abs(ly2 - ly1) < 12.0 or abs(ry2 - ry1) < 12.0:
        return default_swap

    y_min = max(min(ly1, ly2), min(ry1, ry2))
    y_max = min(max(ly1, ly2), max(ry1, ry2))
    if y_max <= y_min:
        return default_swap

    y_ref = 0.5 * (y_min + y_max)
    u_l = lx1 + (y_ref - ly1) * (lx2 - lx1) / (ly2 - ly1)
    u_r = rx1 + (y_ref - ry1) * (rx2 - rx1) / (ry2 - ry1)

    if abs(u_l - u_r) > 2.5:
        return bool(u_l < u_r)

    return default_swap


# ===========================================================================
# MÉTODO 1: SGBM + WLS SEMPRE ALINHADO NO REFERENCIAL DA CAM1 (rect_l)
# ===========================================================================

def computar_mapa_sgbm_roi(rect_l, rect_r, cx, cy, roi_radius, geometry, force_swap=None):
    left_matcher = geometry["sgbm"]
    right_matcher = geometry.get("right_matcher")
    wls_filter = geometry.get("wls_filter")
    num_disp = geometry["num_disp"]
    use_swap = geometry["swap_lr"] if force_swap is None else bool(force_swap)

    gray_l = cv2.cvtColor(rect_l, cv2.COLOR_BGR2GRAY) if rect_l.ndim == 3 else rect_l
    gray_r = cv2.cvtColor(rect_r, cv2.COLOR_BGR2GRAY) if rect_r.ndim == 3 else rect_r

    h, w = gray_l.shape[:2]

    x_roi_0 = max(0, cx - roi_radius)
    x_roi_1 = min(w, cx + roi_radius)
    y_start = max(0, cy - roi_radius)
    y_end = min(h, cy + roi_radius)

    if not use_swap:
        x_start = max(0, x_roi_0 - num_disp)
        x_end = x_roi_1
    else:
        x_start = x_roi_0
        x_end = min(w, x_roi_1 + num_disp)

    if (x_end - x_start) <= num_disp or (y_end - y_start) < 6:
        return None

    roi_l = clahe.apply(gray_l[y_start:y_end, x_start:x_end])
    roi_r = clahe.apply(gray_r[y_start:y_end, x_start:x_end])
    guide_l = rect_l[y_start:y_end, x_start:x_end]

    if not use_swap:
        img_left = roi_l
        img_right = roi_r
        guide_img = guide_l
    else:
        img_left = cv2.flip(roi_l, 1)
        img_right = cv2.flip(roi_r, 1)
        guide_img = cv2.flip(guide_l, 1)

    disp_left_i16 = left_matcher.compute(img_left, img_right)

    used_wls = False
    if wls_filter is not None and right_matcher is not None:
        disp_right_i16 = right_matcher.compute(img_right, img_left)
        disp_wls_i16 = wls_filter.filter(
            disp_left_i16,
            guide_img,
            None,
            disp_right_i16
        )
        disp_full = disp_wls_i16.astype(np.float32) / 16.0
        used_wls = True
    else:
        disp_full = disp_left_i16.astype(np.float32) / 16.0

    if use_swap:
        disp_full = cv2.flip(disp_full, 1)

    local_x0 = x_roi_0 - x_start
    local_x1 = local_x0 + (x_roi_1 - x_roi_0)
    disp_crop = disp_full[:, local_x0:local_x1]

    circle_mask = np.zeros(disp_crop.shape, dtype=np.uint8)
    cv2.circle(
        circle_mask,
        (cx - x_roi_0, cy - y_start),
        roi_radius,
        255,
        -1
    )

    return {
        "disp_crop": disp_crop,
        "used_wls": used_wls,
        "circle_mask": circle_mask,
        "x_roi_0": x_roi_0,
        "x_roi_1": x_roi_1,
        "y_start": y_start,
        "y_end": y_end,
        "num_disp": num_disp,
        "gray_l": gray_l,
        "gray_r": gray_r,
    }


def extrair_profundidade_linha_sgbm(sgbm_data, left_cand, geometry):
    if sgbm_data is None or left_cand is None:
        return None, None, None

    f_rect = geometry["f_rect"]
    baseline = geometry["baseline"]
    K1 = geometry["K1"]

    if baseline <= 1e-5 or f_rect <= 1.0:
        return None, None, None

    disp_crop = sgbm_data["disp_crop"]
    circle_mask = sgbm_data["circle_mask"]
    x_roi_0 = sgbm_data["x_roi_0"]
    y_start = sgbm_data["y_start"]

    x1, y1, x2, y2 = left_cand["segment"]

    neighborhood_mask = np.zeros_like(circle_mask, dtype=np.uint8)
    cv2.line(
        neighborhood_mask,
        (x1 - x_roi_0, y1 - y_start),
        (x2 - x_roi_0, y2 - y_start),
        255,
        thickness=SGBM_NEIGHBORHOOD_THICKNESS
    )

    d_min_fg = max(1.0, (f_rect * baseline) / MAX_DISTANCE)
    d_max_fg = (f_rect * baseline) / MIN_DISTANCE

    valid = (
        (neighborhood_mask > 0)
        & (circle_mask > 0)
        & (disp_crop >= d_min_fg)
        & (disp_crop <= d_max_fg)
    )

    if np.count_nonzero(valid) < 6:
        return None, None, neighborhood_mask

    vals = disp_crop[valid]
    p_low = float(np.percentile(vals, TOP_DISP_PERCENTILE_MIN))
    p_high = float(np.percentile(vals, TOP_DISP_PERCENTILE_MAX))

    top_pixels_mask = valid & (disp_crop >= p_low) & (disp_crop <= p_high)
    if np.count_nonzero(top_pixels_mask) == 0:
        top_pixels_mask = valid & (disp_crop >= p_low)

    top_vals = disp_crop[top_pixels_mask]
    mean_top_disp = float(np.mean(top_vals))
    z_depth = (f_rect * baseline) / mean_top_disp

    mx, my = left_cand["midpoint"]
    cx_cam, cy_cam = K1[0, 2], K1[1, 2]
    X = (mx - cx_cam) * z_depth / f_rect
    Y = (my - cy_cam) * z_depth / f_rect
    dist_3d = float(np.sqrt(X * X + Y * Y + z_depth * z_depth))

    if not (MIN_DISTANCE <= dist_3d <= MAX_DISTANCE):
        return None, mean_top_disp, top_pixels_mask

    return dist_3d, mean_top_disp, top_pixels_mask


# ===========================================================================
# MÉTODO 2: CORRELAÇÃO EPIPOLAR DIRETA NA FAIXA DA LINHA (FUNCIONA EM QUALQUER ÂNGULO)
# ===========================================================================

def calcular_profundidade_correlacao_epipolar(sgbm_data, left_cand, geometry, roi_bin_l=None, roi_bin_r=None):
    """
    Faz template matching 2D ao longo da linha epipolar ao redor de pontos da reta.
    Garante estimativa de profundidade mesmo quando a linha está horizontal ou
    quando o WLS tem poucos pixels válidos.
    """
    if sgbm_data is None or left_cand is None:
        return None

    f_rect = geometry["f_rect"]
    baseline = geometry["baseline"]
    K1 = geometry["K1"]
    num_disp = sgbm_data["num_disp"]

    if baseline <= 1e-5 or f_rect <= 1.0:
        return None

    gray_l = sgbm_data["gray_l"]
    gray_r = sgbm_data["gray_r"]
    h, w = gray_l.shape[:2]

    lx1, ly1, lx2, ly2 = left_cand["segment_subpx"]
    patch_half_y = 9
    patch_half_x = 12

    d_min_fg = max(1.5, (f_rect * baseline) / MAX_DISTANCE)
    d_max_fg = min(float(num_disp * 1.5), (f_rect * baseline) / MIN_DISTANCE)

    disparities = []
    # Amostra 5 pontos ao longo do segmento detectado na Cam1
    for alpha in (0.2, 0.35, 0.5, 0.65, 0.8):
        px = int(round(lx1 + alpha * (lx2 - lx1)))
        py = int(round(ly1 + alpha * (ly2 - ly1)))

        y0, y1 = py - patch_half_y, py + patch_half_y + 1
        x0, x1 = px - patch_half_x, px + patch_half_x + 1
        if y0 < 2 or y1 >= h - 2 or x0 < 2 or x1 >= w - 2:
            continue

        # Combina intensidade CLAHE + máscara binária para ter borda forte em qualquer direção
        if roi_bin_l is not None and roi_bin_r is not None:
            tpl = cv2.addWeighted(gray_l[y0:y1, x0:x1], 0.6, roi_bin_l[y0:y1, x0:x1], 0.4, 0)
            sx0 = max(0, x0 - int(d_max_fg) - 4)
            sx1 = min(w, x1 + int(d_max_fg) + 4)
            strip = cv2.addWeighted(gray_r[y0 - 2:y1 + 2, sx0:sx1], 0.6, roi_bin_r[y0 - 2:y1 + 2, sx0:sx1], 0.4, 0)
        else:
            tpl = gray_l[y0:y1, x0:x1]
            sx0 = max(0, x0 - int(d_max_fg) - 4)
            sx1 = min(w, x1 + int(d_max_fg) + 4)
            strip = gray_r[y0 - 2:y1 + 2, sx0:sx1]

        if strip.shape[1] <= tpl.shape[1] or strip.shape[0] < tpl.shape[0]:
            continue

        res = cv2.matchTemplate(strip, tpl, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, max_loc = cv2.minMaxLoc(res)
        if max_val < 0.45:
            continue

        best_x_in_r = sx0 + max_loc[0] + patch_half_x
        disp_val = abs(float(px - best_x_in_r))
        if d_min_fg <= disp_val <= d_max_fg:
            disparities.append(disp_val)

    if not disparities:
        return None

    med_disp = float(np.median(disparities))
    z_depth = (f_rect * baseline) / med_disp
    mx, my = left_cand["midpoint"]
    X = (mx - K1[0, 2]) * z_depth / f_rect
    Y = (my - K1[1, 2]) * z_depth / f_rect
    dist_3d = float(np.sqrt(X * X + Y * Y + z_depth * z_depth))

    if not (MIN_DISTANCE <= dist_3d <= MAX_DISTANCE):
        return None
    return dist_3d


# ===========================================================================
# MÉTODO 3: DISPARIDADE SUBPIXEL POR SCANLINE (COM SUPORTE A QUALQUER INCLINAÇÃO)
# ===========================================================================

def _refinar_centroide_scanline(roi_bin, y_idx, u_est, half_win=10):
    if roi_bin is None:
        return u_est
    h, w = roi_bin.shape[:2]
    yi = int(round(y_idx))
    if yi < 0 or yi >= h:
        return u_est

    x0 = max(0, int(round(u_est)) - half_win)
    x1 = min(w, int(round(u_est)) + half_win + 1)
    if x1 <= x0:
        return u_est

    row_slice = roi_bin[yi, x0:x1]
    xs_local = np.nonzero(row_slice > 0)[0]
    if len(xs_local) == 0:
        return u_est

    return float(x0 + np.mean(xs_local))


def reconstruir_por_scanline(left_cand, right_cand, geometry, roi_bin_l=None, roi_bin_r=None):
    f_rect = geometry["f_rect"]
    baseline = geometry["baseline"]
    K1 = geometry["K1"]

    if baseline <= 1e-5 or f_rect <= 1.0:
        return None

    lx1, ly1, lx2, ly2 = left_cand.get("segment_subpx", left_cand["segment"])
    rx1, ry1, rx2, ry2 = right_cand.get("segment_subpx", right_cand["segment"])

    y_min = max(min(ly1, ly2), min(ry1, ry2))
    y_max = min(max(ly1, ly2), max(ry1, ry2))

    # Permite scanline mesmo em linhas bastante inclinadas (até 3px de variação vertical)
    if (y_max - y_min) < 3 or abs(ly2 - ly1) < 3 or abs(ry2 - ry1) < 3:
        return None

    ys = np.linspace(y_min, y_max, DISTANCE_SAMPLES)
    u_l_raw = lx1 + (ys - ly1) * (lx2 - lx1) / float(ly2 - ly1)
    u_r_raw = rx1 + (ys - ry1) * (rx2 - rx1) / float(ry2 - ry1)

    if abs(left_cand["bearing"]) < 70.0:
        u_l = np.array([
            _refinar_centroide_scanline(roi_bin_l, y, u) for y, u in zip(ys, u_l_raw)
        ], dtype=np.float64)
        u_r = np.array([
            _refinar_centroide_scanline(roi_bin_r, y, u) for y, u in zip(ys, u_r_raw)
        ], dtype=np.float64)
    else:
        u_l = u_l_raw
        u_r = u_r_raw

    disp = np.abs(u_l - u_r)
    d_min_fg = max(1.0, (f_rect * baseline) / MAX_DISTANCE)
    valid = disp >= d_min_fg
    if np.count_nonzero(valid) < 4:
        return None

    ys = ys[valid]
    u_l = u_l[valid]
    disp = disp[valid]

    p_cut = float(np.percentile(disp, 50.0))
    top_mask = disp >= p_cut
    ys = ys[top_mask]
    u_l = u_l[top_mask]
    disp = disp[top_mask]

    zs = (f_rect * baseline) / disp
    xs = (u_l - K1[0, 2]) * zs / f_rect
    ys_3d = (ys - K1[1, 2]) * zs / f_rect

    pts = np.column_stack((xs, ys_3d, zs))
    dists = np.linalg.norm(pts, axis=1)
    mean_dist = float(np.mean(dists))

    if not (MIN_DISTANCE <= mean_dist <= MAX_DISTANCE):
        return None

    return mean_dist, pts


# ===========================================================================
# MÉTODO 4: INTERSEÇÃO DE PLANOS 3D
# ===========================================================================

def plano_retroprojecao(line_2d, K, camera_center):
    normal = K.T @ line_2d
    norm = np.linalg.norm(normal)

    if norm < 1e-12:
        return None

    normal /= norm
    offset = float(np.dot(normal, camera_center))

    return normal, offset


def intersecao_planos(line_left, line_right, K1, K2, C1, C2):
    plane1 = plano_retroprojecao(line_left, K1, C1)
    plane2 = plano_retroprojecao(line_right, K2, C2)

    if plane1 is None or plane2 is None:
        return None

    n1, b1 = plane1
    n2, b2 = plane2

    direction = np.cross(n1, n2)
    plane_sine = float(np.linalg.norm(direction))

    if plane_sine < MIN_PLANE_SINE:
        return None

    direction /= plane_sine

    A = np.vstack((n1, n2))
    b = np.array([b1, b2], dtype=np.float64)

    X0, _, rank, _ = np.linalg.lstsq(A, b, rcond=None)

    if rank < 2 or not np.all(np.isfinite(X0)):
        return None

    return X0, direction, plane_sine


def parametro_reta_por_raio(pixel, K, camera_center, X0, direction):
    pixel_h = np.array([pixel[0], pixel[1], 1.0], dtype=np.float64)
    ray = np.linalg.solve(K, pixel_h)
    ray_norm = np.linalg.norm(ray)

    if ray_norm < 1e-12:
        return None, "raio_invalido", None

    ray /= ray_norm

    A = np.column_stack((ray, -direction))
    rhs = X0 - camera_center

    solution, _, rank, _ = np.linalg.lstsq(A, rhs, rcond=None)
    if rank < 2:
        return None, "rank_invalido", None

    ray_distance, t = solution
    point_ray = camera_center + ray_distance * ray
    point_line = X0 + t * direction
    residual = float(np.linalg.norm(point_ray - point_line))

    if ray_distance <= MIN_RAY_DEPTH:
        return None, "raio_tras_camera", {
            "depth": float(ray_distance),
            "t": float(t),
            "residual": residual
        }

    return float(t), "valido", {
        "depth": float(ray_distance),
        "t": float(t),
        "residual": residual
    }


def intervalo_segmento(candidate, K, camera_center, X0, direction):
    x1, y1, x2, y2 = candidate.get("segment_subpx", candidate["segment"])
    t1, reason1, info1 = parametro_reta_por_raio((x1, y1), K, camera_center, X0, direction)
    t2, reason2, info2 = parametro_reta_por_raio((x2, y2), K, camera_center, X0, direction)

    if t1 is None:
        return None, {"endpoint": "inicio", "reason": reason1, "info": info1}
    if t2 is None:
        return None, {"endpoint": "fim", "reason": reason2, "info": info2}

    return (min(t1, t2), max(t1, t2)), None


def reconstruir_par(left, right, geometry):
    K1 = geometry["K1"]
    K2 = geometry["K2"]
    C1 = geometry["C1"]
    C2 = geometry["C2"]

    plane1 = plano_retroprojecao(left["line"], K1, C1)
    plane2 = plano_retroprojecao(right["line"], K2, C2)

    if plane1 is None or plane2 is None:
        return None, "planos_invalidos"

    n1, b1 = plane1
    n2, b2 = plane2

    direction = np.cross(n1, n2)
    plane_sine = float(np.linalg.norm(direction))

    if plane_sine < MIN_PLANE_SINE:
        return None, "planos_paralelos"

    direction /= plane_sine

    A = np.vstack((n1, n2))
    b = np.array([b1, b2], dtype=np.float64)

    X0, _, rank, _ = np.linalg.lstsq(A, b, rcond=None)
    if rank < 2 or not np.all(np.isfinite(X0)):
        return None, "intersecao_invalida"

    interval1, _ = intervalo_segmento(left, K1, C1, X0, direction)
    if interval1 is None:
        return None, "raios_esquerda"

    interval2, _ = intervalo_segmento(right, K2, C2, X0, direction)
    if interval2 is None:
        return None, "raios_direita"

    t_min = max(interval1[0], interval2[0])
    t_max = min(interval1[1], interval2[1])
    common_length = t_max - t_min

    if common_length <= 0:
        return None, "sem_sobreposicao_3d"
    if common_length < MIN_COMMON_SEGMENT_LENGTH:
        return None, "trecho_3d_curto"

    ts = np.linspace(t_min, t_max, DISTANCE_SAMPLES)
    points = X0[None, :] + ts[:, None] * direction[None, :]
    distances = np.linalg.norm(points - C1[None, :], axis=1)

    if not np.all(np.isfinite(distances)):
        return None, "distancia_invalida"

    mean_distance = float(np.mean(distances))
    if not (MIN_DISTANCE <= mean_distance <= MAX_DISTANCE):
        return None, "fora_do_alcance"

    return {
        "X0": X0,
        "direction": direction,
        "points": points,
        "distance": mean_distance,
        "common_length": common_length,
        "plane_sine": plane_sine,
        "interval": (t_min, t_max),
        "left": left,
        "right": right
    }, "valido"


# ===========================================================================
# PAINEL VISUAL SGBM + WLS NA ROI
# ===========================================================================

def gerar_painel_sgbm_roi(
    rect_l,
    sgbm_data,
    cx,
    cy,
    roi_radius,
    left_cand=None,
    sgbm_dist=None,
    median_disp=None,
    final_dist=None,
    sampled_mask=None
):
    h, w = rect_l.shape[:2]
    panel_w = w * 2
    panel = np.full((h, panel_w, 3), 18, dtype=np.uint8)

    if sgbm_data is None:
        cv2.putText(
            panel,
            "SGBM + WLS na ROI indisponivel",
            (20, h // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )
        return panel

    disp_crop = sgbm_data["disp_crop"]
    circle_mask = sgbm_data["circle_mask"]
    used_wls = sgbm_data.get("used_wls", False)
    x_roi_0 = sgbm_data["x_roi_0"]
    x_roi_1 = sgbm_data["x_roi_1"]
    y_start = sgbm_data["y_start"]
    y_end = sgbm_data["y_end"]
    num_disp = max(16, sgbm_data["num_disp"])

    valid_circle = (circle_mask > 0) & (disp_crop > 0.5)
    if np.count_nonzero(valid_circle) > 20:
        d_min = float(np.percentile(disp_crop[valid_circle], 5))
        d_max = float(np.percentile(disp_crop[valid_circle], 98))
        if (d_max - d_min) < 2.0:
            d_min, d_max = 0.0, float(num_disp)
    else:
        d_min, d_max = 0.0, float(num_disp)

    disp_norm = np.clip((disp_crop - d_min) / max(d_max - d_min, 1e-3), 0.0, 1.0)
    disp_u8 = np.uint8(disp_norm * 255.0)
    colormap = getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET)
    color_crop = cv2.applyColorMap(disp_u8, colormap)

    invalid_mask = (disp_crop <= 0.5) | (circle_mask == 0)
    color_crop[invalid_mask] = (25, 25, 25)

    context_view = (rect_l * 0.35).astype(np.uint8)
    roi_slice = context_view[y_start:y_end, x_roi_0:x_roi_1]
    mask_bool = circle_mask > 0
    roi_slice[mask_bool] = color_crop[mask_bool]

    cv2.circle(context_view, (cx, cy), roi_radius, (0, 255, 255), 2)
    if left_cand is not None:
        lx1, ly1, lx2, ly2 = left_cand["segment"]
        cv2.line(context_view, (lx1, ly1), (lx2, ly2), (0, 255, 0), 2)

    x_ctx = max(10, (panel_w // 2) - w - 10)
    panel[0:h, x_ctx:x_ctx + w] = context_view

    zoom_size = min(h - 24, (x_roi_1 - x_roi_0) * 2, w - 40)
    zoom_size = max(40, zoom_size)
    zoom_crop = color_crop.copy()

    if sampled_mask is not None and sampled_mask.shape == zoom_crop.shape[:2]:
        sampled_u8 = (sampled_mask.astype(np.uint8)) * 255
        contours, _ = cv2.findContours(sampled_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(zoom_crop, contours, -1, (255, 255, 0), 1)

    if left_cand is not None:
        lx1, ly1, lx2, ly2 = left_cand["segment"]
        corridor = np.zeros(zoom_crop.shape[:2], dtype=np.uint8)
        cv2.line(
            corridor,
            (lx1 - x_roi_0, ly1 - y_start),
            (lx2 - x_roi_0, ly2 - y_start),
            255,
            SGBM_NEIGHBORHOOD_THICKNESS
        )
        c_contours, _ = cv2.findContours(corridor, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(zoom_crop, c_contours, -1, (220, 220, 220), 1)

        cv2.line(
            zoom_crop,
            (lx1 - x_roi_0, ly1 - y_start),
            (lx2 - x_roi_0, ly2 - y_start),
            (0, 255, 0),
            2
        )

    cv2.circle(
        zoom_crop,
        (cx - x_roi_0, cy - y_start),
        roi_radius,
        (0, 255, 255),
        2
    )

    zoom_vis = cv2.resize(zoom_crop, (zoom_size, zoom_size), interpolation=cv2.INTER_LINEAR)
    y_z0 = (h - zoom_size) // 2
    x_z0 = (panel_w // 2) + 20
    if x_z0 + zoom_size < panel_w:
        panel[y_z0:y_z0 + zoom_size, x_z0:x_z0 + zoom_size] = zoom_vis

    tag_wls = "SGBM + WLS (ALINHADO CAM1)" if used_wls else "SGBM (ALINHADO CAM1)"
    x_txt = x_z0 + zoom_size + 20
    if x_txt < panel_w - 80:
        f_scale = max(0.45, h / 650.0)
        disp_str = f"Media Top Disp: {median_disp:.1f} px" if median_disp is not None else "Media Top Disp: --"
        z_sgbm_str = f"Z (Vizinhanca): {sgbm_dist:.2f} m" if sgbm_dist is not None else "Z (Vizinhanca): sem leitura"
        z_fin_str = f"D Filtrada: {final_dist:.2f} m" if (final_dist is not None and final_dist > 0) else "D Filtrada: --"

        cv2.putText(panel, tag_wls, (x_txt, y_z0 + 25), cv2.FONT_HERSHEY_SIMPLEX, f_scale, (0, 255, 255), 2)
        cv2.putText(panel, disp_str, (x_txt, y_z0 + 60), cv2.FONT_HERSHEY_SIMPLEX, f_scale, (255, 255, 255), 1)
        cv2.putText(panel, z_sgbm_str, (x_txt, y_z0 + 95), cv2.FONT_HERSHEY_SIMPLEX, f_scale, (0, 255, 0), 2)
        cv2.putText(panel, z_fin_str, (x_txt, y_z0 + 130), cv2.FONT_HERSHEY_SIMPLEX, f_scale, (255, 200, 50), 2)

    cv2.putText(
        panel,
        tag_wls,
        (15, h - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2
    )

    return panel


# ===========================================================================
# PIPELINE INTEGRADO (GARANTIA DE PROFUNDIDADE 3D EM QUALQUER ÂNGULO)
# ===========================================================================

def detectar_e_reconstruir(
    rect_l,
    rect_r,
    geometry,
    cx,
    cy,
    roi_radius,
    scale=1.0
):
    global _prev_bearing_l, _prev_bearing_r, _last_valid_distance

    left_candidates, roi_left, edges_left = detectar_candidatas(
        rect_l, cx, cy, roi_radius, prev_bearing=_prev_bearing_l
    )

    right_candidates, roi_right, edges_right = detectar_candidatas(
        rect_r, cx, cy, roi_radius, prev_bearing=_prev_bearing_r
    )

    force_swap = geometry["swap_lr"]
    if left_candidates and right_candidates:
        force_swap = _detectar_swap_na_mesma_linha_y(
            left_candidates[0], right_candidates[0], geometry["swap_lr"]
        )

    sgbm_data = (
        computar_mapa_sgbm_roi(rect_l, rect_r, cx, cy, roi_radius, geometry, force_swap=force_swap)
        if USE_ROI_SGBM else None
    )

    best_result = None
    fallback_result = None
    best_sgbm_dist = None
    best_median_disp = None
    best_sampled_mask = None
    pares_testados = 0
    pares_validos = 0

    if left_candidates and right_candidates:
        for left in left_candidates[:3]:
            for right in right_candidates[:3]:
                diff_ang = _diff_angular_180(left["bearing"], right["bearing"])
                if diff_ang > 20.0:
                    continue

                pares_testados += 1
                mean_angle = _media_angular_180(left["bearing"], right["bearing"])

                plane_res, _ = reconstruir_par(left, right, geometry)
                scan_res = reconstruir_por_scanline(
                    left, right, geometry, roi_bin_l=roi_left, roi_bin_r=roi_right
                )
                sgbm_dist, median_disp, sampled_mask = extrair_profundidade_linha_sgbm(
                    sgbm_data, left, geometry
                )

                # Se a direção inicial do SGBM não achou pixels na linha, testa o sentido inverso
                if sgbm_dist is None and USE_ROI_SGBM:
                    sgbm_alt = computar_mapa_sgbm_roi(
                        rect_l, rect_r, cx, cy, roi_radius, geometry, force_swap=(not force_swap)
                    )
                    alt_dist, alt_disp, alt_mask = extrair_profundidade_linha_sgbm(
                        sgbm_alt, left, geometry
                    )
                    if alt_dist is not None:
                        sgbm_data = sgbm_alt
                        sgbm_dist, median_disp, sampled_mask = alt_dist, alt_disp, alt_mask

                # 4º Estimador de segurança: correlação de template 2D na linha epipolar
                corr_dist = calcular_profundidade_correlacao_epipolar(
                    sgbm_data, left, geometry, roi_bin_l=roi_left, roi_bin_r=roi_right
                )

                plane_sine = plane_res["plane_sine"] if plane_res is not None else 0.0
                if plane_sine == 0.0:
                    p1 = plano_retroprojecao(left["line"], geometry["K1"], geometry["C1"])
                    p2 = plano_retroprojecao(right["line"], geometry["K2"], geometry["C2"])
                    if p1 is not None and p2 is not None:
                        plane_sine = float(np.linalg.norm(np.cross(p1[0], p2[0])))

                score = left["weight"] + right["weight"]

                if fallback_result is None or score > fallback_result["score"]:
                    fallback_result = {
                        "left": left,
                        "right": right,
                        "bearing": mean_angle,
                        "angle": mean_angle,
                        "distance": _last_valid_distance if _last_valid_distance > 0 else -1.0,
                        "plane_sine": plane_sine,
                        "score": score,
                        "sgbm_dist": sgbm_dist,
                        "median_disp": median_disp,
                        "sampled_mask": sampled_mask
                    }

                # Fusão hierárquica dos 4 estimadores
                if sgbm_dist is not None:
                    near_estimates = [sgbm_dist]
                    if corr_dist is not None and abs(corr_dist - sgbm_dist) <= 0.35:
                        near_estimates.append(corr_dist)
                    if scan_res is not None and abs(scan_res[0] - sgbm_dist) <= 0.35:
                        near_estimates.append(scan_res[0])
                    if plane_res is not None and plane_sine > 0.06 and abs(plane_res["distance"] - sgbm_dist) <= 0.35:
                        near_estimates.append(plane_res["distance"])
                    raw_distance = float(np.mean(near_estimates))
                else:
                    backup_estimates = []
                    if corr_dist is not None:
                        backup_estimates.append(corr_dist)
                    if scan_res is not None:
                        backup_estimates.append(scan_res[0])
                    if plane_res is not None:
                        backup_estimates.append(plane_res["distance"])
                    if not backup_estimates:
                        continue
                    raw_distance = float(np.median(backup_estimates))

                pares_validos += 1

                candidate_result = {
                    "left": left,
                    "right": right,
                    "bearing": mean_angle,
                    "angle": mean_angle,
                    "distance": raw_distance,
                    "plane_sine": plane_sine,
                    "score": score,
                    "sgbm_dist": sgbm_dist if sgbm_dist is not None else corr_dist,
                    "median_disp": median_disp,
                    "sampled_mask": sampled_mask
                }

                if best_result is None or candidate_result["score"] > best_result["score"]:
                    best_result = candidate_result

    final_output = best_result if best_result is not None else fallback_result

    if final_output is not None:
        _prev_bearing_l = final_output["left"]["bearing"]
        _prev_bearing_r = final_output["right"]["bearing"]

        ang_rad2 = np.radians(final_output["bearing"] * 2.0)
        _hist_sin2.append(np.sin(ang_rad2))
        _hist_cos2.append(np.cos(ang_rad2))
        smooth_bearing = float(
            np.degrees(np.arctan2(np.median(_hist_sin2), np.median(_hist_cos2))) / 2.0
        )
        final_output["bearing"] = smooth_bearing
        final_output["angle"] = smooth_bearing

        if final_output["distance"] > 0:
            _hist_dist.append(final_output["distance"])
            filt_d = float(np.percentile(_hist_dist, 40))
            final_output["distance"] = filt_d
            _last_valid_distance = filt_d
        elif _last_valid_distance > 0:
            final_output["distance"] = _last_valid_distance

    chosen_left = final_output["left"] if final_output is not None else (
        left_candidates[0] if left_candidates else None
    )
    if final_output is not None:
        best_sgbm_dist = final_output.get("sgbm_dist")
        best_median_disp = final_output.get("median_disp")
        best_sampled_mask = final_output.get("sampled_mask")
        final_dist_val = final_output.get("distance")
    else:
        best_sgbm_dist, best_median_disp, best_sampled_mask = extrair_profundidade_linha_sgbm(
            sgbm_data, chosen_left, geometry
        )
        final_dist_val = best_sgbm_dist

    sgbm_panel = gerar_painel_sgbm_roi(
        rect_l,
        sgbm_data,
        cx,
        cy,
        roi_radius,
        left_cand=chosen_left,
        sgbm_dist=best_sgbm_dist,
        median_disp=best_median_disp,
        final_dist=final_dist_val,
        sampled_mask=best_sampled_mask
    )

    if final_output is not None:
        final_output["sgbm_panel"] = sgbm_panel
    detectar_e_reconstruir.last_sgbm_panel = sgbm_panel

    detectar_e_reconstruir.debug = {
        "candidatas_esquerda": len(left_candidates),
        "candidatas_direita": len(right_candidates),
        "pares_testados": pares_testados,
        "pares_validos": pares_validos
    }

    return (
        final_output,
        roi_left,
        roi_right,
        edges_left,
        edges_right
    )


# ===========================================================================
# VISUALIZAÇÃO DA RECONSTRUÇÃO
# ===========================================================================

def desenhar_segmento(frame, candidate, color=(0, 255, 0), thickness=3):
    if candidate is None:
        return

    x1, y1, x2, y2 = candidate["segment"]
    cv2.line(frame, (x1, y1), (x2, y2), color, thickness)


# ===========================================================================
# SCALE MANAGER
# ===========================================================================

SCALES = {
    ord("1"): 1.00,
    ord("2"): 0.75,
    ord("3"): 0.50,
    ord("4"): 0.25
}


class ScaleManager:
    def __init__(
        self,
        cmtx0,
        dist0,
        cmtx1,
        dist1,
        R_rel,
        T_rel,
        base_w,
        base_h
    ):
        self.cmtx0 = cmtx0
        self.dist0 = dist0
        self.cmtx1 = cmtx1
        self.dist1 = dist1
        self.R_rel = R_rel
        self.T_rel = T_rel
        self.base_w = base_w
        self.base_h = base_h

        self._cache = {}
        self.current = 1.0

        for scale in SCALES.values():
            self._build(scale)

    def _build(self, scale):
        if scale in self._cache:
            return

        w = max(2, int(self.base_w * scale))
        h = max(2, int(self.base_h * scale))

        w -= w % 2
        h -= h % 2

        size = (w, h)

        cm0 = scale_intrinsics(self.cmtx0, scale)
        cm1 = scale_intrinsics(self.cmtx1, scale)

        geometry = build_rectification(
            cm0,
            self.dist0,
            cm1,
            self.dist1,
            self.R_rel,
            self.T_rel,
            size,
            scale=scale
        )

        self._cache[scale] = {
            "geometry": geometry,
            "size": size
        }

    def set(self, scale):
        self.current = scale

    def get(self):
        return self._cache[self.current]


# ===========================================================================
# HUD
# ===========================================================================

def draw_mini_hud(frame, fps, stats_str, stats_color, visao_ativa=True):
    h, w = frame.shape[:2]
    cell_h = (h / 3.0) if h > w else float(h)
    s = max(0.55, cell_h / 720.0)

    margem = max(8, int(10 * s))
    linha1 = margem + int(16 * s)
    linha2 = linha1 + int(24 * s)
    linha3 = linha2 + int(24 * s)

    altura_hud = linha3 + margem

    cv2.rectangle(
        frame,
        (0, 0),
        (min(w, 1250), altura_hud),
        (0, 0, 0),
        -1
    )

    fonte = max(0.38, 0.48 * s)
    espessura = max(1, int(s))

    vis_tag = (
        "[ENVIO ANGULO ON ('v')]"
        if visao_ativa
        else "[ENVIO ANGULO PAUSADO ('v')]"
    )

    vis_cor = (
        (0, 255, 200)
        if visao_ativa
        else (120, 120, 255)
    )

    cv2.putText(
        frame,
        f"FPS: {fps:.1f} | {vis_tag} | {stats_str}",
        (10, linha1),
        cv2.FONT_HERSHEY_SIMPLEX,
        fonte,
        vis_cor,
        espessura
    )

    yaw_on = bool(telemetria_dados.get("yaw_ativo", 1))
    yaw_tag = "ON" if yaw_on else "OFF ('y')"

    str_motores = (
        f"STM32 -> Yaw: {telemetria_dados['yaw']:+.1f}° [{yaw_tag}] | "
        f"GiroZ: {telemetria_dados['rate_z']:+.1f}°/s | "
        f"BF: {telemetria_dados['pwm_bf']} | "
        f"AS: {telemetria_dados['pwm_as']} | "
        f"TR: {telemetria_dados.get('pwm_tr', 1500)} | "
        f"Erro: {telemetria_dados['erro']:+.1f}°"
    )

    cv2.putText(
        frame,
        str_motores,
        (10, linha2),
        cv2.FONT_HERSHEY_SIMPLEX,
        fonte * 0.95,
        (255, 160, 50),
        espessura
    )

    swing_on = bool(telemetria_dados.get("swing_ativo", 0))
    status_sw = "ATIVO" if swing_on else "DESLIGADO ('b')"
    cor_sw = (0, 255, 0) if swing_on else (100, 100, 255)

    d_val = telemetria_dados.get("D", -1.0)
    d_txt = (
        f"{d_val:.2f}m"
        if d_val is not None and d_val >= 0
        else "sem visao"
    )

    str_pendulo = (
        f"Anti-Balanco: [{status_sw}] | "
        f"Theta: {telemetria_dados.get('theta', 0.0):.2f}° | "
        f"L: {telemetria_dados.get('L', 1.0):.2f}m | "
        f"D: {d_txt}"
    )

    cv2.putText(
        frame,
        str_pendulo,
        (10, linha3),
        cv2.FONT_HERSHEY_SIMPLEX,
        fonte * 0.95,
        cor_sw,
        espessura
    )

    return altura_hud
