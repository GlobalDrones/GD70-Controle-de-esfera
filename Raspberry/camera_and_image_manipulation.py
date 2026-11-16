
import cv2
import numpy as np

from serial_communication import telemetria_dados  # noqa: F401


# ===========================================================================
# PRE-PROCESSAMENTO
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

# Número máximo de segmentos Hough considerados por câmera.
MAX_CANDIDATE_LINES = 20

# Comprimento mínimo de segmento em relação ao raio da ROI.
MIN_LINE_LENGTH_FACTOR = 0.3 #pelo menbs 80% da ROI

# Limites geométricos.
MAX_DISTANCE = 3.0
MIN_PLANE_SINE = 0.05
MIN_COMMON_SEGMENT_LENGTH = 0.03
MIN_RAY_DEPTH = 0.0

# Quantidade de pontos para calcular a distância média.
DISTANCE_SAMPLES = 30


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

    return bin_img


# ===========================================================================
# RETIFICAÇÃO
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
    img_size
):
    R1r, R2r, P1, P2, Q, _, _ = cv2.stereoRectify(
        cmtx0,
        dist0,
        cmtx1,
        dist1,
        img_size,
        R_rel,
        T_rel,
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

    # Matrizes intrínsecas das câmeras retificadas.
    K1_rect = P1[:, :3].astype(np.float64)
    K2_rect = P2[:, :3].astype(np.float64)

    # P2 = K2 [I | t_rect].
    # O centro óptico direito no referencial retificado esquerdo é -t_rect.
    t_rect = np.linalg.solve(K2_rect, P2[:, 3])
    C2_rect = -t_rect

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
    }


# ===========================================================================
# DETECÇÃO DE SEGMENTOS
# ===========================================================================

def detectar_candidatas(rect, cx, cy, roi_radius, max_gap=50):
    bin_img = preprocessar_imagem(rect)

    mask = np.zeros_like(bin_img)
    cv2.circle(mask, (cx, cy), roi_radius, 255, -1)

    roi_bin = cv2.bitwise_and(bin_img, bin_img, mask=mask)
    edges = cv2.Canny(roi_bin, 50, 150)

    lines = cv2.HoughLinesP(
        edges,
        1,
        np.pi / 180,
        threshold=25,
        minLineLength=max(10, int(roi_radius * MIN_LINE_LENGTH_FACTOR)),
        maxLineGap=max_gap
    )

    if lines is None:
        return [], roi_bin, edges

    candidates = []

    for item in lines:
        x1, y1, x2, y2 = map(int, item[0])
        length = float(np.hypot(x2 - x1, y2 - y1))

        if length < roi_radius * MIN_LINE_LENGTH_FACTOR:
            continue

        # Equação homogênea da reta: ax + by + c = 0.
        p1 = np.array([x1, y1, 1.0], dtype=np.float64)
        p2 = np.array([x2, y2, 1.0], dtype=np.float64)

        line = np.cross(p1, p2)
        norm = np.hypot(line[0], line[1])

        if norm < 1e-9:
            continue

        line /= norm

        candidates.append({
            "segment": (x1, y1, x2, y2),
            "line": line,
            "length": length,
            "midpoint": ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
        })

    candidates.sort(
        key=lambda item: item["length"],
        reverse=True
    )

    return candidates[:MAX_CANDIDATE_LINES], roi_bin, edges


def bearing_com_sinal(x1, y1, x2, y2):
    """
    Bearing da linha em relação à vertical da imagem, com sinal,
    no intervalo (-90, 90].
    """
    angle = np.degrees(np.arctan2(x2 - x1, y2 - y1))

    if angle > 90:
        angle -= 180
    elif angle <= -90:
        angle += 180

    return angle


def detectar_linha_mono(rect_l, cx, cy, roi_radius):
    candidates, roi_bin, _ = detectar_candidatas(
        rect_l,
        cx,
        cy,
        roi_radius,
        max_gap=20
    )

    if not candidates:
        return None, None, None, roi_bin

    def distancia_centro(candidate):
        mx, my = candidate["midpoint"]
        return np.hypot(mx - cx, my - cy)

    best = min(candidates, key=distancia_centro)
    x1, y1, x2, y2 = best["segment"]

    return (
        bearing_com_sinal(x1, y1, x2, y2),
        best["segment"],
        None,
        roi_bin
    )


# ===========================================================================
# GEOMETRIA 3D
# ===========================================================================

def plano_retroprojecao(line_2d, K, camera_center):
    """
    Converte uma reta da imagem retificada em um plano 3D.

    Os planos são expressos no referencial retificado da câmera esquerda.
    """
    normal = K.T @ line_2d
    norm = np.linalg.norm(normal)

    if norm < 1e-12:
        return None

    normal /= norm
    offset = float(np.dot(normal, camera_center))

    return normal, offset


def intersecao_planos(line_left, line_right, K1, K2, C1, C2):
    """
    Retorna:
        X0: ponto da reta 3D
        direction: direção unitária da reta
        plane_sine: seno do ângulo entre as normais
    """
    plane1 = plano_retroprojecao(line_left, K1, C1)
    plane2 = plano_retroprojecao(line_right, K2, C2)

    if plane1 is None or plane2 is None:
        return None

    n1, b1 = plane1
    n2, b2 = plane2

    direction = np.cross(n1, n2)
    plane_sine = float(np.linalg.norm(direction))

    # Normais quase paralelas: reconstrução mal condicionada.
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
    pixel_h = np.array(
        [pixel[0], pixel[1], 1.0],
        dtype=np.float64
    )

    ray = np.linalg.solve(K, pixel_h)
    ray_norm = np.linalg.norm(ray)

    if ray_norm < 1e-12:
        return None, "raio_invalido", None

    ray /= ray_norm

    # camera_center + s * ray = X0 + t * direction
    A = np.column_stack((ray, -direction))
    rhs = X0 - camera_center

    solution, _, rank, _ = np.linalg.lstsq(A, rhs, rcond=None)

    if rank < 2:
        return None, "rank_invalido", None

    ray_distance, t = solution

    # Verifica a distância entre os pontos calculados.
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

    x1, y1, x2, y2 = candidate["segment"]

    t1, reason1, info1 = parametro_reta_por_raio(
        (x1, y1), K, camera_center, X0, direction
    )

    t2, reason2, info2 = parametro_reta_por_raio(
        (x2, y2), K, camera_center, X0, direction
    )

    if t1 is None:
        return None, {
            "endpoint": "inicio",
            "reason": reason1,
            "info": info1
        }

    if t2 is None:
        return None, {
            "endpoint": "fim",
            "reason": reason2,
            "info": info2
        }

    return (min(t1, t2), max(t1, t2)), None




def reconstruir_par(left, right, geometry):
    K1 = geometry["K1"]
    K2 = geometry["K2"]
    C1 = geometry["C1"]
    C2 = geometry["C2"]

    # 1. Construção dos planos.
    plane1 = plano_retroprojecao(left["line"], K1, C1)
    plane2 = plano_retroprojecao(right["line"], K2, C2)

    if plane1 is None or plane2 is None:
        return None, "planos_invalidos"

    n1, b1 = plane1
    n2, b2 = plane2

    # 2. Interseção dos planos.
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

    # 3. Projeção dos segmentos das duas câmeras sobre a reta 3D.
    interval1, ray_error_left = intervalo_segmento(
        left, K1, C1, X0, direction
    )

    if interval1 is None:
        if getattr(reconstruir_par, "debug_count", 0) < 10:
            reconstruir_par.debug_count = (
                getattr(reconstruir_par, "debug_count", 0) + 1
            )

            print("\n[RAIO ESQUERDA]")
            print("Segmento:", left["segment"])
            print("Extremidade:", ray_error_left["endpoint"])
            print("Motivo:", ray_error_left["reason"])
            print("Informações:", ray_error_left["info"])
            print("X0:", X0)
            print("Direção:", direction)

        return None, "raios_esquerda"





    interval2, ray_error_right = intervalo_segmento(
        right, K2, C2, X0, direction
    )

    if interval2 is None:
        if getattr(reconstruir_par, "debug_count_right", 0) < 10:
            reconstruir_par.debug_count_right = (
                getattr(reconstruir_par, "debug_count_right", 0) + 1
            )

            print("\n[RAIO DIREITA]")
            print("Segmento:", right["segment"])
            print("Extremidade:", ray_error_right["endpoint"])
            print("Motivo:", ray_error_right["reason"])
            print("Informações:", ray_error_right["info"])
            print("X0:", X0)
            print("Direção:", direction)

        return None, "raios_direita"

    # 4. Trecho compartilhado.
    t_min = max(interval1[0], interval2[0])
    t_max = min(interval1[1], interval2[1])

    common_length = t_max - t_min

    if common_length <= 0:
        return None, "sem_sobreposicao_3d"

    if common_length < MIN_COMMON_SEGMENT_LENGTH:
        return None, "trecho_3d_curto"

    # 5. Reconstrução dos pontos do trecho compartilhado.
    ts = np.linspace(t_min, t_max, DISTANCE_SAMPLES)

    points = (
        X0[None, :]
        + ts[:, None] * direction[None, :]
    )

    distances = np.linalg.norm(
        points - C1[None, :],
        axis=1
    )

    if not np.all(np.isfinite(distances)):
        return None, "distancia_invalida"

    mean_distance = float(np.mean(distances))

    if mean_distance <= 0:
        return None, "distancia_invalida"

    if mean_distance > MAX_DISTANCE:
        return None, "fora_do_alcance"

    result = {
        "X0": X0,
        "direction": direction,
        "points": points,
        "distance": mean_distance,
        "common_length": common_length,
        "plane_sine": plane_sine,
        "interval": (t_min, t_max),
        "left": left,
        "right": right
    }

    return result, "valido"




def detectar_e_reconstruir(
    rect_l,
    rect_r,
    geometry,
    cx,
    cy,
    roi_radius,
    scale=1.0
):
    left_candidates, roi_left, edges_left = detectar_candidatas(
        rect_l, cx, cy, roi_radius
    )

    right_candidates, roi_right, edges_right = detectar_candidatas(
        rect_r, cx, cy, roi_radius
    )

    best_result = None

    if left_candidates and right_candidates:

        # Maior linha de cada câmera.
        left = max(left_candidates, key=lambda c: c["length"])
        right = max(right_candidates, key=lambda c: c["length"])

        # Inclinação individual.
        x1, y1, x2, y2 = left["segment"]
        angle_left = bearing_com_sinal(x1, y1, x2, y2)

        x1, y1, x2, y2 = right["segment"]
        angle_right = bearing_com_sinal(x1, y1, x2, y2)

        # Média angular considerando a orientação de uma reta
        # (ângulos equivalentes a cada 180 graus).
        a1 = np.radians(angle_left * 2.0)
        a2 = np.radians(angle_right * 2.0)

        mean_angle = np.degrees(
            np.arctan2(
                np.sin(a1) + np.sin(a2),
                np.cos(a1) + np.cos(a2)
            )
        ) / 2.0

        best_result = {
            "left": left,
            "right": right,
            "angle_left": angle_left,
            "angle_right": angle_right,
            "angle": mean_angle
        }

    detectar_e_reconstruir.debug = {
        "candidatas_esquerda": len(left_candidates),
        "candidatas_direita": len(right_candidates),
        "pares_testados": 1 if best_result is not None else 0,
        "pares_validos": 1 if best_result is not None else 0
    }

    return (
        best_result,
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

    cv2.line(
        frame,
        (x1, y1),
        (x2, y2),
        color,
        thickness
    )


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
            size
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
    s = h / 720.0

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

    fonte = max(0.35, 0.48 * s)
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
