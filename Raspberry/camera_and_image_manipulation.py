from collections import deque
import cv2
import numpy as np

from serial_communication import telemetria_dados  # noqa: F401


# ===========================================================================
# PRE-PROCESSAMENTO E CONFIGURACOES ANTI-SOMBRA
# ===========================================================================

CLAHE_CLIP_LIMIT = 1.4           # Reduzido de 2.0 para não amplificar sombras suaves no fundo
CLAHE_TILE_GRID_SIZE = (8, 8)

clahe = cv2.createCLAHE(
    clipLimit=CLAHE_CLIP_LIMIT,
    tileGridSize=CLAHE_TILE_GRID_SIZE
)

USE_ADAPTIVE_THRESHOLD = True
ADAPTIVE_BLOCK_SIZE = 21         # Janela mais justa (era 51) para ignorar sombras largas
ADAPTIVE_C = 10                  # Mais rigoroso (era 8) contra penumbras

# --- FILTROS ANTI-SOMBRA (BLACK-HAT + ESPESSURA + VALE BILATERAL) ---
BLACKHAT_KERNEL_BASE = 23        # Largura máxima do filtro Black-Hat na escala 1.0 (apaga sombras largas)
BLACKHAT_MIN_PROMINENCE = 12     # Contraste mínimo (0..255) do cabo no Black-Hat
MIN_ROPE_RADIUS_BASE = 1.1       # Meia-espessura mínima da corda (px)
MAX_ROPE_RADIUS_BASE = 9.5       # Meia-espessura máxima da corda na escala 1.0 (px) — bloqueia manchas/sombras
VALLEY_MIN_BILATERAL_DIFF = 7.5  # Diferença mínima de brilho exigida nos DOIS lados da reta (elimina borda de sombra)

MAX_CANDIDATE_LINES = 10
MIN_LINE_LENGTH_FACTOR = 0.3

# Agrupamento de segmentos colineares da mesma crista central
CLUSTER_ANGLE_TOL_DEG = 10.0
CLUSTER_RHO_TOL_PX = 12.0
RIDGE_CORRIDOR_HALF_PX = 8

# Limites geométricos amplos (nunca descarta uma linha válida como 2D)
MAX_DISTANCE = 6.0
MIN_DISTANCE = 0.15
MIN_PLANE_SINE = 0.015
MIN_COMMON_SEGMENT_LENGTH = 0.02
MIN_RAY_DEPTH = 0.05

# Distância de convergência digital ("envesgamento digital" em metros).
DIGITAL_CONVERGENCE_DIST_M = 2.2

DISTANCE_SAMPLES = 30
SCANLINE_HALF_WIN_PX = 10

# Filtro temporal
TEMPORAL_WINDOW = 7
_hist_dist = deque(maxlen=TEMPORAL_WINDOW)
_hist_sin2 = deque(maxlen=TEMPORAL_WINDOW)
_hist_cos2 = deque(maxlen=TEMPORAL_WINDOW)
_prev_bearing_l = None
_prev_bearing_r = None
_last_valid_distance = -1.0


def _preprocessar_recorte(crop_bgr, scale=0.5):
    """
    Combina filtro Morfológico Black-Hat (que suprime sombras largas e fundos escuros)
    com Adaptive Threshold de janela curta APENAS no recorte da ROI.
    """
    gray = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2GRAY)
    enhanced = clahe.apply(gray)
    blurred = cv2.GaussianBlur(enhanced, (5, 5), 0)

    # 1. Filtro Black-Hat: destaca apenas estruturas escuras MAIS FINAS que k_bh.
    # Sombras largas e paredes escuras desaparecem (ficam com valor ~0).
    k_bh = max(9, int(round(BLACKHAT_KERNEL_BASE * max(scale, 0.35))) | 1)
    kernel_bh = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_bh, k_bh))
    blackhat = cv2.morphologyEx(blurred, cv2.MORPH_BLACKHAT, kernel_bh)

    _, mask_bh = cv2.threshold(
        blackhat,
        BLACKHAT_MIN_PROMINENCE,
        255,
        cv2.THRESH_BINARY
    )

    # 2. Threshold adaptativo de janela curta (rejeita gradientes lentos de penumbra)
    if USE_ADAPTIVE_THRESHOLD:
        block_size = max(11, int(round(ADAPTIVE_BLOCK_SIZE * max(scale, 0.5))) | 1)
        block_size = min(block_size, min(crop_bgr.shape[:2]) // 2 * 2 - 1)
        block_size = max(block_size, 3)

        bin_adapt = cv2.adaptiveThreshold(
            blurred,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            block_size,
            ADAPTIVE_C
        )
        # Só aceita como cabo se passar TANTO no Adaptive Threshold QUANTO no Black-Hat
        bin_crop = cv2.bitwise_and(bin_adapt, mask_bh)
    else:
        bin_crop = mask_bh

    # Remove ruídos puntiformes isolados e fecha micro-falhas na corda
    kernel_clean = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    bin_crop = cv2.morphologyEx(bin_crop, cv2.MORPH_OPEN, kernel_clean)
    bin_crop = cv2.morphologyEx(bin_crop, cv2.MORPH_CLOSE, kernel_clean)

    return bin_crop, blurred


def preprocessar_imagem(frame):
    bin_img, _ = _preprocessar_recorte(frame, scale=0.5)
    return bin_img


# ===========================================================================
# RETIFICAÇÃO COM CONVERGÊNCIA DIGITAL ("ENVESGAMENTO DIGITAL")
# ===========================================================================

def scale_intrinsics(cmtx, scale):
    m = cmtx.copy().astype(np.float64)
    m[0, 0] *= scale
    m[1, 1] *= scale
    m[0, 2] *= scale
    m[1, 2] *= scale
    return m


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

    K1_base = P1[:, :3].astype(np.float64)
    K2_base = P2[:, :3].astype(np.float64)

    t_rect = np.linalg.solve(K2_base, P2[:, 3])
    C2_rect = -t_rect

    f_rect = float(P1[0, 0])
    baseline = float(np.linalg.norm(C2_rect))
    swap_left_right = bool(P2[0, 3] > 0)
    disp_sign = -1.0 if P2[0, 3] > 0 else 1.0

    delta_u = 0.0
    if DIGITAL_CONVERGENCE_DIST_M is not None and DIGITAL_CONVERGENCE_DIST_M > 0.15:
        raw_delta_u = (f_rect * float(C2_rect[0])) / (2.0 * float(DIGITAL_CONVERGENCE_DIST_M))
        max_shift = 0.28 * float(img_size[0])
        delta_u = float(np.clip(raw_delta_u, -max_shift, max_shift))
        P1[0, 2] -= delta_u
        P2[0, 2] += delta_u

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
    cx_offset = float(K1_rect[0, 2] - K2_rect[0, 2])

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
        "cx_offset": cx_offset,
        "delta_u": delta_u,
        "scale": scale,
    }


# ===========================================================================
# DETECÇÃO POR CRISTA CENTRAL + REJEIÇÃO DE DEGRAU DE SOMBRA
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


def _validar_vale_bilateral(cand, gray_crop, dist_crop, x0_off, y0_off, max_rope_r):
    """
    Verifica se a reta candidata é realmente um VALE (escuro no meio e mais claro
    nos DOIS lados perpendiculares) e não a borda de uma sombra (claro de um lado
    e escuro do outro).
    Retorna (valido_bool, fator_contraste).
    """
    x1, y1, x2, y2 = cand["segment_subpx"]
    dx = x2 - x1
    dy = y2 - y1
    length = float(np.hypot(dx, dy))
    if length < 6.0:
        return False, 0.0

    # Vetor normal unitário perpendicular à reta
    nx = -dy / length
    ny = dx / length

    ch, cw = gray_crop.shape[:2]
    alphas = np.linspace(0.12, 0.88, 12)

    # Mede o raio médio da crista ao longo da reta
    radii = []
    for a in alphas:
        px = int(round((x1 + a * dx) - x0_off))
        py = int(round((y1 + a * dy) - y0_off))
        if 0 <= py < ch and 0 <= px < cw:
            d_val = float(dist_crop[py, px])
            if d_val > 0.5:
                radii.append(d_val)

    est_r = float(np.median(radii)) if radii else 3.0
    if est_r > max_rope_r:
        return False, 0.0

    # Distância perpendicular para amostrar o fundo de cada lado da corda
    step_n = float(np.clip(est_r + 4.5, 4.0, max_rope_r + 6.0))

    diffs_left = []
    diffs_right = []

    for a in alphas:
        cx_p = (x1 + a * dx) - x0_off
        cy_p = (y1 + a * dy) - y0_off

        ix, iy = int(round(cx_p)), int(round(cy_p))
        lx, ly = int(round(cx_p + step_n * nx)), int(round(cy_p + step_n * ny))
        rx, ry = int(round(cx_p - step_n * nx)), int(round(cy_p - step_n * ny))

        if (0 <= iy < ch and 0 <= ix < cw and
                0 <= ly < ch and 0 <= lx < cw and
                0 <= ry < ch and 0 <= rx < cw):
            i_center = float(gray_crop[iy, ix])
            i_left = float(gray_crop[ly, lx])
            i_right = float(gray_crop[ry, rx])

            diffs_left.append(i_left - i_center)
            diffs_right.append(i_right - i_center)

    if len(diffs_left) < 5:
        return True, 1.0

    med_left = float(np.median(diffs_left))
    med_right = float(np.median(diffs_right))

    # Borda de sombra tem med_left alto mas med_right <= 0 (ou vice-versa).
    # Corda real tem AMBOS med_left e med_right positivos!
    min_side_diff = min(med_left, med_right)
    if min_side_diff < VALLEY_MIN_BILATERAL_DIFF:
        return False, 0.0

    # Fator de qualidade pelo contraste bilateral
    contrast_boost = float(np.clip(1.0 + 0.02 * min_side_diff, 1.0, 1.6))
    return True, contrast_boost


def _refinar_eixo_ponderado_por_crista(clusters, dist_crop, gray_crop, x0_off, y0_off, min_len, max_rope_r):
    refined = []
    ch, cw = dist_crop.shape[:2]

    for cl in clusters:
        items = cl["items"]
        total_w = cl["total_len"]

        corridor = np.zeros((ch, cw), dtype=np.uint8)
        pts_ext = []
        for it in items:
            x1, y1, x2, y2 = it["segment_subpx"]
            lx1, ly1 = int(round(x1 - x0_off)), int(round(y1 - y0_off))
            lx2, ly2 = int(round(x2 - x0_off)), int(round(y2 - y0_off))
            cv2.line(corridor, (lx1, ly1), (lx2, ly2), 255, thickness=RIDGE_CORRIDOR_HALF_PX * 2)
            pts_ext.append((x1, y1))
            pts_ext.append((x2, y2))

        valid_ridge_px = (corridor > 0) & (dist_crop >= MIN_ROPE_RADIUS_BASE) & (dist_crop <= max_rope_r)
        ys_l, xs_l = np.nonzero(valid_ridge_px)

        if len(xs_l) >= 15:
            # Limita o valor máximo de d no peso para que manchas largas nunca ganhem vantagem
            d_clamped = np.clip(dist_crop[ys_l, xs_l].astype(np.float64), 1.0, max_rope_r)
            weights = d_clamped ** 2
            w_sum = float(np.sum(weights))
            xs_g = xs_l.astype(np.float64) + x0_off
            ys_g = ys_l.astype(np.float64) + y0_off

            mx = float(np.sum(xs_g * weights) / w_sum)
            my = float(np.sum(ys_g * weights) / w_sum)

            dx = xs_g - mx
            dy = ys_g - my
            cov_xx = float(np.sum(weights * dx * dx) / w_sum)
            cov_yy = float(np.sum(weights * dy * dy) / w_sum)
            cov_xy = float(np.sum(weights * dx * dy) / w_sum)

            theta = 0.5 * np.arctan2(2.0 * cov_xy, cov_xx - cov_yy)
            vx = float(np.cos(theta))
            vy = float(np.sin(theta))

            proj_t = dx * vx + dy * vy
            t_min = float(np.percentile(proj_t, 1.0))
            t_max = float(np.percentile(proj_t, 99.0))
        else:
            pts_arr = np.array(pts_ext, dtype=np.float64)
            mx, my = np.mean(pts_arr[:, 0]), np.mean(pts_arr[:, 1])
            ang_rad = np.radians(cl["bearing"])
            vx, vy = float(np.sin(ang_rad)), float(np.cos(ang_rad))
            proj_t = (pts_arr[:, 0] - mx) * vx + (pts_arr[:, 1] - my) * vy
            t_min, t_max = float(np.min(proj_t)), float(np.max(proj_t))

        fx1 = mx + t_min * vx
        fy1 = my + t_min * vy
        fx2 = mx + t_max * vx
        fy2 = my + t_max * vy

        cand = _construir_candidata_de_segmento(fx1, fy1, fx2, fy2, score_weight=total_w)
        if cand is None or cand["length"] < min_len:
            continue

        # Validação anti-sombra: exige que seja um vale escuro cercado por fundo mais claro nos 2 lados
        is_valley, contrast_boost = _validar_vale_bilateral(
            cand, gray_crop, dist_crop, x0_off, y0_off, max_rope_r
        )
        if not is_valley:
            continue

        cand["weight"] *= contrast_boost
        refined.append(cand)

    refined.sort(key=lambda item: item["weight"], reverse=True)
    return refined[:MAX_CANDIDATE_LINES]


def detectar_candidatas(rect, cx, cy, roi_radius, max_gap=45, prev_bearing=None):
    h, w = rect.shape[:2]
    scale_est = float(w) / 1280.0

    x0 = max(0, cx - roi_radius)
    x1 = min(w, cx + roi_radius)
    y0 = max(0, cy - roi_radius)
    y1 = min(h, cy + roi_radius)

    crop_bgr = rect[y0:y1, x0:x1]
    bin_crop, clahe_crop = _preprocessar_recorte(crop_bgr, scale=scale_est)

    circle_mask = np.zeros_like(bin_crop, dtype=np.uint8)
    cv2.circle(circle_mask, (cx - x0, cy - y0), max(5, roi_radius - 2), 255, -1)
    bin_crop = cv2.bitwise_and(bin_crop, bin_crop, mask=circle_mask)

    dist_crop = cv2.distanceTransform(bin_crop, cv2.DIST_L2, 3)

    # Teto máximo de espessura da corda ajustado pela escala:
    # qualquer mancha cuja distância até a borda supere max_rope_r é sombra/objeto largo e é zerada!
    max_rope_r = max(4.5, MAX_ROPE_RADIUS_BASE * max(scale_est, 0.45))
    bin_crop[dist_crop > max_rope_r] = 0
    dist_crop[dist_crop > max_rope_r] = 0.0

    dilated = cv2.dilate(dist_crop, np.ones((3, 3), dtype=np.uint8))
    ridge_crop = np.where(
        (dist_crop >= dilated - 0.15)
        & (dist_crop >= MIN_ROPE_RADIUS_BASE)
        & (dist_crop <= max_rope_r)
        & (circle_mask > 0),
        255,
        0
    ).astype(np.uint8)

    roi_bin_full = np.zeros((h, w), dtype=np.uint8)
    roi_bin_full[y0:y1, x0:x1] = bin_crop

    ridge_full = np.zeros((h, w), dtype=np.uint8)
    ridge_full[y0:y1, x0:x1] = ridge_crop

    roi_meta = {
        "x0": x0,
        "x1": x1,
        "y0": y0,
        "y1": y1,
        "dist_crop": dist_crop,
        "ridge_crop": ridge_crop,
        "clahe_crop": clahe_crop,
        "circle_mask": circle_mask,
        "max_rope_r": max_rope_r,
    }

    min_len = max(10, int(roi_radius * MIN_LINE_LENGTH_FACTOR))

    lines = cv2.HoughLinesP(
        ridge_crop,
        1,
        np.pi / 180,
        threshold=15,
        minLineLength=min_len,
        maxLineGap=max_gap
    )

    if lines is None:
        detectar_candidatas.last_roi_meta = roi_meta
        return [], roi_bin_full, ridge_full

    raw_candidates = []
    for item in lines:
        lx1, ly1, lx2, ly2 = map(int, item[0])
        cand = _construir_candidata_de_segmento(
            lx1 + x0, ly1 + y0, lx2 + x0, ly2 + y0
        )
        if cand is not None and cand["length"] >= min_len:
            raw_candidates.append(cand)

    raw_candidates.sort(key=lambda item: item["length"], reverse=True)

    clusters = []
    for cand in raw_candidates:
        placed = False
        sx1, sy1, sx2, sy2 = cand["segment_subpx"]
        mx, my = cand["midpoint"]
        for cl in clusters:
            if _diff_angular_180(cand["bearing"], cl["bearing"]) <= CLUSTER_ANGLE_TOL_DEG:
                a, b, c = cl["ref_line"]
                d_mid = abs(a * mx + b * my + c)
                d_p1 = abs(a * sx1 + b * sy1 + c)
                d_p2 = abs(a * sx2 + b * sy2 + c)
                if d_mid <= CLUSTER_RHO_TOL_PX and max(d_p1, d_p2) <= (CLUSTER_RHO_TOL_PX * 1.5):
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

    candidates = _refinar_eixo_ponderado_por_crista(
        clusters, dist_crop, clahe_crop, x0, y0, min_len, max_rope_r
    )

    if prev_bearing is not None and candidates:
        for c in candidates:
            ang_diff = _diff_angular_180(c["bearing"], prev_bearing)
            continuity_factor = 1.35 if ang_diff < 10.0 else (1.0 if ang_diff < 22.0 else 0.75)
            c["weight"] *= continuity_factor
        candidates.sort(key=lambda item: item["weight"], reverse=True)

    detectar_candidatas.last_roi_meta = roi_meta
    return candidates, roi_bin_full, ridge_full


def detectar_linha_mono(rect_l, cx, cy, roi_radius):
    global _prev_bearing_l
    candidates, roi_bin, _ = detectar_candidatas(
        rect_l,
        cx,
        cy,
        roi_radius,
        max_gap=35,
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
# ESTIMAÇÃO DE PROFUNDIDADE 1D SUBPIXEL (COMPENSADA COM CONVERGÊNCIA DIGITAL)
# ===========================================================================

def _centroide_subpixel_crista_1d(meta, y_global, x_est_global, half_win=SCANLINE_HALF_WIN_PX):
    dist_crop = meta["dist_crop"]
    x0, y0 = meta["x0"], meta["y0"]
    ch, cw = dist_crop.shape[:2]

    yl = int(round(y_global - y0))
    if yl < 0 or yl >= ch:
        return None

    xl_c = int(round(x_est_global - x0))
    xa = max(0, xl_c - half_win)
    xb = min(cw, xl_c + half_win + 1)
    if xb <= xa + 1:
        return None

    profile = dist_crop[yl, xa:xb].astype(np.float64)
    max_val = float(np.max(profile))
    if max_val < 0.8:
        return None

    weights = np.where(profile >= 0.4 * max_val, profile * profile, 0.0)
    w_sum = float(np.sum(weights))
    if w_sum <= 1e-6:
        return None

    xs_local = np.arange(xa, xb, dtype=np.float64)
    u_subpx_local = float(np.sum(xs_local * weights) / w_sum)
    return float(x0 + u_subpx_local)


def estimar_profundidade_scanline_1d(left_cand, right_cand, meta_l, meta_r, geometry):
    f_rect = geometry["f_rect"]
    baseline = geometry["baseline"]
    K1 = geometry["K1"]
    cx_offset = geometry.get("cx_offset", 0.0)

    if baseline <= 1e-5 or f_rect <= 1.0 or meta_l is None or meta_r is None:
        return None, None, None, []

    lx1, ly1, lx2, ly2 = left_cand["segment_subpx"]
    rx1, ry1, rx2, ry2 = right_cand["segment_subpx"]

    y_min = max(min(ly1, ly2), min(ry1, ry2))
    y_max = min(max(ly1, ly2), max(ry1, ry2))

    if (y_max - y_min) < 4.0 or abs(ly2 - ly1) < 4.0 or abs(ry2 - ry1) < 4.0:
        return None, None, None, []

    d_min_fg = max(1.0, (f_rect * baseline) / MAX_DISTANCE)
    d_max_fg = (f_rect * baseline) / MIN_DISTANCE

    ys = np.linspace(y_min, y_max, DISTANCE_SAMPLES)
    scan_samples = []

    use_1d_centroid = abs(left_cand["bearing"]) <= 72.0

    for y_val in ys:
        ul_line = lx1 + (y_val - ly1) * (lx2 - lx1) / (ly2 - ly1)
        ur_line = rx1 + (y_val - ry1) * (rx2 - rx1) / (ry2 - ry1)

        if use_1d_centroid:
            ul_sub = _centroide_subpixel_crista_1d(meta_l, y_val, ul_line)
            ur_sub = _centroide_subpixel_crista_1d(meta_r, y_val, ur_line)
            if ul_sub is None:
                ul_sub = ul_line
            if ur_sub is None:
                ur_sub = ur_line
        else:
            ul_sub = ul_line
            ur_sub = ur_line

        disp_val = abs((ul_sub - ur_sub) - cx_offset)
        if d_min_fg <= disp_val <= d_max_fg:
            scan_samples.append((float(y_val), float(ul_sub), float(ur_sub), float(disp_val)))

    if len(scan_samples) < 4:
        return None, None, None, scan_samples

    disps = np.array([s[3] for s in scan_samples], dtype=np.float64)

    q35, q85 = np.percentile(disps, [35.0, 85.0])
    inliers = disps[(disps >= q35 - 0.5) & (disps <= q85 + 0.5)]
    if len(inliers) == 0:
        inliers = disps

    best_disp = float(np.mean(inliers))
    std_disp = float(np.std(inliers))
    z_depth = (f_rect * baseline) / best_disp

    mx, my = left_cand["midpoint"]
    X = (mx - K1[0, 2]) * z_depth / f_rect
    Y = (my - K1[1, 2]) * z_depth / f_rect
    dist_3d = float(np.sqrt(X * X + Y * Y + z_depth * z_depth))

    if not (MIN_DISTANCE <= dist_3d <= MAX_DISTANCE):
        return None, best_disp, std_disp, scan_samples

    return dist_3d, best_disp, std_disp, scan_samples


def estimar_profundidade_correlacao_subpixel(left_cand, meta_l, meta_r, geometry):
    if meta_l is None or meta_r is None or left_cand is None:
        return None, None

    f_rect = geometry["f_rect"]
    baseline = geometry["baseline"]
    K1 = geometry["K1"]
    cx_offset = geometry.get("cx_offset", 0.0)

    if baseline <= 1e-5 or f_rect <= 1.0:
        return None, None

    dist_l = np.clip(meta_l["dist_crop"] * 25.0, 0, 255).astype(np.uint8)
    dist_r = np.clip(meta_r["dist_crop"] * 25.0, 0, 255).astype(np.uint8)
    feat_l = cv2.addWeighted(meta_l["clahe_crop"], 0.45, dist_l, 0.55, 0)
    feat_r = cv2.addWeighted(meta_r["clahe_crop"], 0.45, dist_r, 0.55, 0)

    ch, cw = feat_l.shape[:2]
    x0_l, y0_l = meta_l["x0"], meta_l["y0"]
    x0_r, y0_r = meta_r["x0"], meta_r["y0"]

    lx1, ly1, lx2, ly2 = left_cand["segment_subpx"]
    ph_y, ph_x = 8, 10

    d_min_fg = max(1.0, (f_rect * baseline) / MAX_DISTANCE)
    d_max_fg = (f_rect * baseline) / MIN_DISTANCE

    subpx_disps = []

    for alpha in (0.15, 0.30, 0.45, 0.55, 0.70, 0.85):
        gx = lx1 + alpha * (lx2 - lx1)
        gy = ly1 + alpha * (ly2 - ly1)

        lx = int(round(gx - x0_l))
        ly = int(round(gy - y0_l))
        ry = int(round(gy - y0_r))

        if (ly - ph_y < 1) or (ly + ph_y + 1 >= ch - 1) or (lx - ph_x < 1) or (lx + ph_x + 1 >= cw - 1):
            continue
        if (ry - ph_y - 1 < 0) or (ry + ph_y + 2 >= feat_r.shape[0]):
            continue

        tpl = feat_l[ly - ph_y:ly + ph_y + 1, lx - ph_x:lx + ph_x + 1]
        strip = feat_r[ry - ph_y - 1:ry + ph_y + 2, :]

        if strip.shape[1] <= tpl.shape[1] + 2:
            continue

        res = cv2.matchTemplate(strip, tpl, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, max_loc = cv2.minMaxLoc(res)
        if max_val < 0.42:
            continue

        pk_x = max_loc[0]
        pk_y = max_loc[1]

        delta_sub = 0.0
        if 0 < pk_x < (res.shape[1] - 1):
            c_left = float(res[pk_y, pk_x - 1])
            c_mid = float(res[pk_y, pk_x])
            c_right = float(res[pk_y, pk_x + 1])
            denom = 2.0 * (c_left - 2.0 * c_mid + c_right)
            if abs(denom) > 1e-6:
                delta_sub = float(np.clip((c_left - c_right) / denom, -0.5, 0.5))

        matched_x_global = x0_r + pk_x + delta_sub + ph_x
        disp_val = abs(float((gx - matched_x_global) - cx_offset))

        if d_min_fg <= disp_val <= d_max_fg:
            subpx_disps.append(disp_val)

    if not subpx_disps:
        return None, None

    med_disp = float(np.median(subpx_disps))
    z_depth = (f_rect * baseline) / med_disp

    mx, my = left_cand["midpoint"]
    X = (mx - K1[0, 2]) * z_depth / f_rect
    Y = (my - K1[1, 2]) * z_depth / f_rect
    dist_3d = float(np.sqrt(X * X + Y * Y + z_depth * z_depth))

    if not (MIN_DISTANCE <= dist_3d <= MAX_DISTANCE):
        return None, med_disp

    return dist_3d, med_disp


# ===========================================================================
# INTERSEÇÃO DE PLANOS 3D (AUXILIAR / TELEMETRIA DO PLANO)
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
# PAINEL DA 3ª LINHA DO GRID: MAPA DA CRISTA + GRÁFICO SUBPIXEL 1D
# ===========================================================================

def gerar_painel_subpixel_roi(
    rect_l,
    meta_l,
    cx,
    cy,
    roi_radius,
    left_cand=None,
    scan_dist=None,
    corr_dist=None,
    best_disp=None,
    std_disp=None,
    final_dist=None,
    scan_samples=None,
    delta_u=0.0
):
    h, w = rect_l.shape[:2]
    panel_w = w * 2
    panel = np.full((h, panel_w, 3), 18, dtype=np.uint8)

    if meta_l is None:
        return panel

    dist_crop = meta_l["dist_crop"]
    circle_mask = meta_l["circle_mask"]
    x0, x1 = meta_l["x0"], meta_l["x1"]
    y0, y1 = meta_l["y0"], meta_l["y1"]

    max_d = max(float(np.max(dist_crop)), 2.0)
    dist_u8 = np.uint8(np.clip((dist_crop / max_d) * 255.0, 0, 255))
    colormap = getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET)
    color_crop = cv2.applyColorMap(dist_u8, colormap)
    color_crop[(dist_crop < 0.5) | (circle_mask == 0)] = (22, 22, 22)

    context_view = (rect_l * 0.35).astype(np.uint8)
    roi_slice = context_view[y0:y1, x0:x1]
    mask_bool = circle_mask > 0
    roi_slice[mask_bool] = color_crop[mask_bool]

    cv2.circle(context_view, (cx, cy), roi_radius, (0, 255, 255), 2)
    if left_cand is not None:
        lx1, ly1, lx2, ly2 = left_cand["segment"]
        cv2.line(context_view, (lx1, ly1), (lx2, ly2), (0, 255, 0), 2)

    if scan_samples:
        for y_s, ul_s, _, _ in scan_samples:
            cv2.circle(context_view, (int(round(ul_s)), int(round(y_s))), 2, (255, 255, 255), -1)

    panel[0:h, 0:w] = context_view

    plot_w = min(260, max(140, w // 2))
    plot_h = max(80, h - 40)
    px0 = w + 15
    py0 = 20
    cv2.rectangle(panel, (px0, py0), (px0 + plot_w, py0 + plot_h), (32, 32, 32), -1)
    cv2.rectangle(panel, (px0, py0), (px0 + plot_w, py0 + plot_h), (90, 90, 90), 1)

    if scan_samples and best_disp is not None:
        d_min_plot = max(0.0, best_disp - 12.0)
        d_max_plot = best_disp + 12.0
        x_ref = int(px0 + 0.5 * plot_w)
        cv2.line(panel, (x_ref, py0), (x_ref, py0 + plot_h), (0, 255, 0), 1)

        prev_pt = None
        for y_s, _, _, d_s in scan_samples:
            rel_y = (y_s - y0) / max(float(y1 - y0), 1.0)
            rel_x = (d_s - d_min_plot) / max(d_max_plot - d_min_plot, 1e-3)
            gx = int(np.clip(px0 + rel_x * plot_w, px0 + 2, px0 + plot_w - 2))
            gy = int(np.clip(py0 + rel_y * plot_h, py0 + 2, py0 + plot_h - 2))
            cv2.circle(panel, (gx, gy), 3, (0, 220, 255), -1)
            if prev_pt is not None:
                cv2.line(panel, prev_pt, (gx, gy), (0, 160, 255), 1)
            prev_pt = (gx, gy)

    cv2.putText(
        panel,
        "Perfil Disp(y)",
        (px0 + 8, py0 + 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (0, 255, 255),
        1
    )

    x_txt = px0 + plot_w + 20
    if x_txt < panel_w - 60:
        f_scale = max(0.45, h / 640.0)
        std_txt = f" (std {std_disp:.2f}px)" if std_disp is not None else ""
        disp_str = f"Disp. Real: {best_disp:.2f} px{std_txt}" if best_disp is not None else "Disp. Real: --"
        conv_str = f"Toe-In Digital: {DIGITAL_CONVERGENCE_DIST_M:.2f}m ({delta_u:+.1f}px/cam)"
        z_scan_str = f"Z (1D Crista): {scan_dist:.2f} m" if scan_dist is not None else "Z (1D Crista): --"
        z_corr_str = f"Z (Correlacao): {corr_dist:.2f} m" if corr_dist is not None else "Z (Correlacao): --"
        z_fin_str = f"D Final: {final_dist:.2f} m" if (final_dist is not None and final_dist > 0) else "D Final: --"

        cv2.putText(panel, "1D SUBPIXEL + ANTI-SOMBRA", (x_txt, py0 + 22), cv2.FONT_HERSHEY_SIMPLEX, f_scale, (0, 255, 255), 2)
        cv2.putText(panel, conv_str, (x_txt, py0 + 50), cv2.FONT_HERSHEY_SIMPLEX, f_scale * 0.9, (200, 200, 255), 1)
        cv2.putText(panel, disp_str, (x_txt, py0 + 80), cv2.FONT_HERSHEY_SIMPLEX, f_scale, (255, 255, 255), 1)
        cv2.putText(panel, z_scan_str, (x_txt, py0 + 110), cv2.FONT_HERSHEY_SIMPLEX, f_scale, (0, 255, 0), 2)
        cv2.putText(panel, z_corr_str, (x_txt, py0 + 140), cv2.FONT_HERSHEY_SIMPLEX, f_scale, (180, 255, 180), 1)
        cv2.putText(panel, z_fin_str, (x_txt, py0 + 172), cv2.FONT_HERSHEY_SIMPLEX, f_scale * 1.05, (255, 200, 50), 2)

    cv2.putText(
        panel,
        "Crista 1D Subpixel na ROI (Black-Hat Anti-Sombra)",
        (15, h - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.60,
        (0, 255, 255),
        2
    )

    return panel


# ===========================================================================
# PIPELINE INTEGRADO 1D SUBPIXEL
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
    meta_l = getattr(detectar_candidatas, "last_roi_meta", None)

    right_candidates, roi_right, edges_right = detectar_candidatas(
        rect_r, cx, cy, roi_radius, prev_bearing=_prev_bearing_r
    )
    meta_r = getattr(detectar_candidatas, "last_roi_meta", None)

    best_result = None
    fallback_result = None
    pares_testados = 0
    pares_validos = 0

    if left_candidates and right_candidates:
        for left in left_candidates[:3]:
            for right in right_candidates[:3]:
                diff_ang = _diff_angular_180(left["bearing"], right["bearing"])
                if diff_ang > 18.0:
                    continue

                pares_testados += 1
                mean_angle = _media_angular_180(
                    left["bearing"], right["bearing"], left["weight"], right["weight"]
                )

                scan_dist, scan_disp, scan_std, scan_samples = estimar_profundidade_scanline_1d(
                    left, right, meta_l, meta_r, geometry
                )

                corr_dist, corr_disp = estimar_profundidade_correlacao_subpixel(
                    left, meta_l, meta_r, geometry
                )

                plane_res, _ = reconstruir_par(left, right, geometry)
                plane_sine = plane_res["plane_sine"] if plane_res is not None else 0.0
                if plane_sine == 0.0:
                    p1 = plano_retroprojecao(left["line"], geometry["K1"], geometry["C1"])
                    p2 = plano_retroprojecao(right["line"], geometry["K2"], geometry["C2"])
                    if p1 is not None and p2 is not None:
                        plane_sine = float(np.linalg.norm(np.cross(p1[0], p2[0])))

                score = left["weight"] + right["weight"]
                disp_ref = scan_disp if scan_disp is not None else corr_disp

                if fallback_result is None or score > fallback_result["score"]:
                    fallback_result = {
                        "left": left,
                        "right": right,
                        "bearing": mean_angle,
                        "angle": mean_angle,
                        "distance": _last_valid_distance if _last_valid_distance > 0 else -1.0,
                        "plane_sine": plane_sine,
                        "score": score,
                        "scan_dist": scan_dist,
                        "corr_dist": corr_dist,
                        "best_disp": disp_ref,
                        "std_disp": scan_std,
                        "scan_samples": scan_samples,
                    }

                estimates = []
                if scan_dist is not None:
                    estimates.append(scan_dist)
                    if corr_dist is not None and abs(corr_dist - scan_dist) <= 0.30:
                        estimates.append(corr_dist)
                    if plane_res is not None and plane_sine > 0.08 and abs(plane_res["distance"] - scan_dist) <= 0.25:
                        estimates.append(plane_res["distance"])
                else:
                    if corr_dist is not None:
                        estimates.append(corr_dist)
                    if plane_res is not None:
                        estimates.append(plane_res["distance"])

                if not estimates:
                    continue

                pares_validos += 1
                raw_distance = float(np.mean(estimates))

                candidate_result = {
                    "left": left,
                    "right": right,
                    "bearing": mean_angle,
                    "angle": mean_angle,
                    "distance": raw_distance,
                    "plane_sine": plane_sine,
                    "score": score,
                    "scan_dist": scan_dist,
                    "corr_dist": corr_dist,
                    "best_disp": disp_ref,
                    "std_disp": scan_std,
                    "scan_samples": scan_samples,
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
            filt_d = float(np.median(_hist_dist))
            final_output["distance"] = filt_d
            _last_valid_distance = filt_d
        elif _last_valid_distance > 0:
            final_output["distance"] = _last_valid_distance

    chosen_left = final_output["left"] if final_output is not None else (
        left_candidates[0] if left_candidates else None
    )

    panel = gerar_painel_subpixel_roi(
        rect_l,
        meta_l,
        cx,
        cy,
        roi_radius,
        left_cand=chosen_left,
        scan_dist=final_output.get("scan_dist") if final_output else None,
        corr_dist=final_output.get("corr_dist") if final_output else None,
        best_disp=final_output.get("best_disp") if final_output else None,
        std_disp=final_output.get("std_disp") if final_output else None,
        final_dist=final_output.get("distance") if final_output else None,
        scan_samples=final_output.get("scan_samples") if final_output else None,
        delta_u=geometry.get("delta_u", 0.0),
    )

    if final_output is not None:
        final_output["sgbm_panel"] = panel
    detectar_e_reconstruir.last_sgbm_panel = panel

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
