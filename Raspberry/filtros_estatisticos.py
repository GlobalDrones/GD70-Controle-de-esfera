"""
filtros_estatisticos.py — Filtragem Estatística Robusta para Visão Estéreo 1D.

POR QUE FILTRAR NO ESPAÇO DA DISPARIDADE (PIXELS) E NÃO EM METROS?
------------------------------------------------------------------
Na geometria estéreo retificada, a profundidade Z relaciona-se com a disparidade d por:
    Z = (f_rect * B) / d

Essa função é fortemente não-linear (hiperbólica em 1/d). Um erro simétrico de ±Δd pixels
na detecção do centro da corda produz um erro ASSIMÉTRICO em metros:
  - Quando a disparidade sobe (+Δd), Z diminui pouco (ex: 170 cm -> 160 cm).
  - Quando a disparidade cai (-Δd), o denominador aproxima-se de zero e Z explode
    formando uma distribuição de cauda longa (ex: 170 cm -> 230 cm).

Tirar a média aritmética diretamente em metros contamina a estimativa com esses picos
superiores. Por isso, este módulo executa toda a rejeição de outliers e a fusão temporal
(MAD + Trimmed Mean + Filtro de Hampel + Filtro de Kalman 1D) estritamente sobre a
disparidade d (em pixels), convertendo para metros apenas no último passo.
"""

from collections import deque
import numpy as np


# Fator de escala para converter o MAD (Median Absolute Deviation) no desvio padrão
# equivalente de uma distribuição Normal: sigma ≈ 1.4826 * MAD.
MAD_NORMAL_FACTOR = 1.4826


# ===========================================================================
# 1. FILTRAGEM ESPACIAL INTRA-FRAME (POR SCANLINE DA CORDA)
# ===========================================================================

def filtrar_disparidades_espaciais(
    raw_samples,
    mad_k=2.0,
    linear_res_k=2.2,
    trim_low_pct=20.0,
    trim_high_pct=80.0
):
    """
    Filtra as amostras de disparidade colhidas ao longo das scanlines horizontais (y)
    de um único frame em 3 etapas:

      Etapa A — Corte por MAD Global:
          Calcula a mediana (d_med) e o desvio absoluto mediano (MAD). Remove scanlines
          com |d_i - d_med| > mad_k * sigma_mad.

      Etapa B — Consistência Geométrica Linear d(y) = a*y + b:
          Como o cabo/corda é fisicamente reto dentro da ROI circular, a disparidade
          verdadeira ao longo da coordenada vertical y deve seguir uma reta. Ajusta-se
          uma reta aos inliers da Etapa A e descartam-se pontos que desviem da tendência
          linear (comuns nas bordas superior/inferior do círculo da ROI).

      Etapa C — Média Aparada (Interpercentile Trimmed Mean):
          Descarta as caudas inferiores (< 20%) e superiores (> 80%) dos inliers finais
          e calcula a média aritmética do miolo central, obtendo precisão subpixel
          imune a outliers.

    Parâmetros:
        raw_samples: lista de tuplas (y_val, ul_sub, ur_sub, disp_val)
        mad_k: multiplicador do desvio MAD para o corte inicial (default = 2.0)
        linear_res_k: multiplicador do resíduo linear para o corte da Etapa B (default = 2.2)
        trim_low_pct: percentil inferior da média aparada (default = 20.0)
        trim_high_pct: percentil superior da média aparada (default = 80.0)

    Retorna:
        (best_disp_px, std_inliers_px, clean_samples)
        Se não houver amostras suficientes, retorna (None, None, raw_samples).
    """
    if not raw_samples or len(raw_samples) < 4:
        return None, None, raw_samples

    arr_y = np.array([s[0] for s in raw_samples], dtype=np.float64)
    arr_d = np.array([s[3] for s in raw_samples], dtype=np.float64)

    # --- Etapa A: Rejeição de Outliers Brutos via Mediana + MAD ---
    med_d = float(np.median(arr_d))
    mad_d = float(np.median(np.abs(arr_d - med_d)))
    # Piso mínimo de 0.30 px evita rejeitar pontos bons quando a corda está perfeitamente vertical
    sigma_mad = max(0.30, MAD_NORMAL_FACTOR * mad_d)

    mask_inliers = np.abs(arr_d - med_d) <= (mad_k * sigma_mad)

    # Garante ao menos 4 pontos caso todas as amostras estejam muito espalhadas
    if np.count_nonzero(mask_inliers) < 4:
        idx_sorted = np.argsort(np.abs(arr_d - med_d))
        mask_inliers = np.zeros_like(arr_d, dtype=bool)
        mask_inliers[idx_sorted[:max(4, len(arr_d) // 2)]] = True

    # --- Etapa B: Consistência Linear d(y) = slope * y + intercept ---
    if np.count_nonzero(mask_inliers) >= 5:
        y_in = arr_y[mask_inliers]
        d_in = arr_d[mask_inliers]
        # Só ajusta reta se houver espalhamento vertical suficiente (> 10 pixels)
        if np.ptp(y_in) > 10.0:
            try:
                slope, intercept = np.polyfit(y_in, d_in, 1)
                d_pred = slope * arr_y + intercept
                residuals = np.abs(arr_d - d_pred)

                res_med = float(np.median(residuals[mask_inliers]))
                res_sigma = max(0.35, MAD_NORMAL_FACTOR * res_med)
                mask_linear = mask_inliers & (residuals <= (linear_res_k * res_sigma))

                if np.count_nonzero(mask_linear) >= 4:
                    mask_inliers = mask_linear
            except np.linalg.LinAlgError:
                pass

    inliers = arr_d[mask_inliers]
    if len(inliers) == 0:
        inliers = arr_d

    # --- Etapa C: Média Aparada (Trimmed Mean 20% a 80%) ---
    if len(inliers) >= 5:
        p_low, p_high = np.percentile(inliers, [trim_low_pct, trim_high_pct])
        trimmed = inliers[(inliers >= p_low) & (inliers <= p_high)]
        best_disp_px = float(np.mean(trimmed)) if len(trimmed) > 0 else float(np.median(inliers))
    else:
        best_disp_px = float(np.median(inliers))

    std_inliers_px = float(np.std(inliers))
    clean_samples = [s for s, keep in zip(raw_samples, mask_inliers) if keep]

    return best_disp_px, std_inliers_px, clean_samples


# ===========================================================================
# 2. FILTRAGEM TEMPORAL INTER-FRAME (HAMPEL SLIDING WINDOW + KALMAN 1D)
# ===========================================================================

class RobustDisparityKalmanFilter:
    """
    Filtro Temporal em dois estágios operando sobre a disparidade (em pixels):

      Estágio 1 — Filtro de Hampel Deslizante (Anti-Spike por MAD):
          Armazena as últimas `window_size` medições brutas de disparidade.
          Calcula a mediana temporal e o sigma_MAD da janela.
          Se a medição do frame atual desviar mais que `hampel_k * sigma_MAD`
          (ex: um frame em que a linha piscou ou perdeu pixels e jogaria a distância
          de 170 cm para 230 cm), o valor bruto é barrado e substituído pela mediana
          da janela, além de penalizar a variância de observação R do Kalman.

      Estágio 2 — Filtro de Kalman Escalar 1D com Ruído Adaptativo e Innovation Gate:
          Mantém o estado estimado `d_est` (px) e a covariância do erro `p_cov` (px^2).
          A variância de medição R adapta-se automaticamente ao desvio padrão espacial
          (`std_spatial`) das scanlines do frame atual: frames com alta dispersão
          intra-frame recebem ganho de Kalman menor; frames limpos recebem ganho maior.
    """

    def __init__(
        self,
        window_size=9,
        hampel_k=2.0,
        q_process=0.06,
        r_base=0.50,
        gate_k=3.0,
        max_missed_frames=10
    ):
        """
        Parâmetros:
            window_size: tamanho da janela deslizante do filtro de Hampel (frames)
            hampel_k: limiar de corte de outliers em múltiplos de sigma_MAD (default = 2.0)
            q_process: variância do ruído de processo Q do Kalman (quão rápido a distância real muda)
            r_base: variância base do ruído de medição R do Kalman (px^2)
            gate_k: limiar do Innovation Gate em múltiplos de sigma_MAD
            max_missed_frames: número de frames sem detecção antes de resetar o estado
        """
        self.window = deque(maxlen=window_size)
        self.hampel_k = float(hampel_k)
        self.q_process = float(q_process)
        self.r_base = float(r_base)
        self.gate_k = float(gate_k)
        self.max_missed_frames = int(max_missed_frames)

        self.d_est = None
        self.p_cov = 1.0
        self.missed_count = 0

    def reset(self):
        """Reinicia o histórico do filtro de Hampel e o estado do Filtro de Kalman."""
        self.window.clear()
        self.d_est = None
        self.p_cov = 1.0
        self.missed_count = 0

    def update(self, d_raw, std_spatial=0.5):
        """
        Processa uma nova leitura de disparidade `d_raw` (px) vinda do frame atual
        e retorna a disparidade filtrada (px).

        Se `d_raw` for None (falha momentânea de detecção), mantém a predição anterior
        por até `max_missed_frames` frames.
        """
        if d_raw is None or not np.isfinite(d_raw) or d_raw <= 0.5:
            self.missed_count += 1
            if self.missed_count > self.max_missed_frames:
                self.reset()
            return self.d_est

        self.missed_count = 0
        d_raw = float(d_raw)
        std_spatial = float(np.clip(std_spatial if std_spatial is not None else 0.5, 0.1, 4.0))

        # --- Estágio 1: Filtro de Hampel na Janela Deslizante ---
        self.window.append(d_raw)
        arr = np.asarray(self.window, dtype=np.float64)

        med = float(np.median(arr))
        mad = float(np.median(np.abs(arr - med)))
        sigma_mad = max(0.35, MAD_NORMAL_FACTOR * mad)

        is_spike = (len(arr) >= 4) and (abs(d_raw - med) > self.hampel_k * sigma_mad)

        if is_spike:
            # Substitui o spike pela mediana da janela e eleva a incerteza da medição
            d_hampel = med
            r_effective = (self.r_base + std_spatial ** 2) * 4.0
        else:
            # Média aparada dos inliers temporais dentro da janela deslizante
            inliers_temp = arr[np.abs(arr - med) <= self.hampel_k * sigma_mad]
            d_hampel = float(np.mean(inliers_temp)) if len(inliers_temp) > 0 else d_raw
            r_effective = self.r_base + (std_spatial ** 2)

        # --- Estágio 2: Filtro de Kalman 1D ---
        if self.d_est is None:
            self.d_est = d_hampel
            self.p_cov = 1.0
            return float(self.d_est)

        # 2.1. Etapa de Predição (modelo de velocidade quase nula entre frames de 30-50Hz)
        d_pred = self.d_est
        p_pred = self.p_cov + self.q_process

        # 2.2. Inovação (Resíduo de Medição) e Innovation Gate
        innovation = d_hampel - d_pred
        if len(arr) >= 5 and abs(innovation) > (self.gate_k * sigma_mad):
            # Amortecimento suave contra degraus bruscos
            r_effective *= 3.0

        # 2.3. Ganho de Kalman e Atualização de Estado/Covariância
        kalman_gain = p_pred / (p_pred + r_effective)
        self.d_est = d_pred + kalman_gain * innovation
        self.p_cov = (1.0 - kalman_gain) * p_pred

        return float(self.d_est)


# ===========================================================================
# 3. FILTRO ESTATÍSTICO CIRCULAR PARA O ÂNGULO (BEARING EM GRAUS)
# ===========================================================================

class CircularAngleFilter:
    """
    Filtro estatístico para o ângulo de inclinação da reta (bearing no intervalo (-90, +90] graus).
    Como uma reta não orientada tem período de 180° (-89° e +89° diferem em apenas 2°),
    mapeia o ângulo para o dobro (2*theta) no círculo unitário (sin(2*theta), cos(2*theta)),
    aplica mediana deslizante para rejeitar saltos angulares e suaviza exponencialmente.
    """

    def __init__(self, window_size=7, alpha_ema=0.45):
        self.sin_win = deque(maxlen=window_size)
        self.cos_win = deque(maxlen=window_size)
        self.alpha_ema = float(alpha_ema)
        self.sin_est = None
        self.cos_est = None

    def reset(self):
        self.sin_win.clear()
        self.cos_win.clear()
        self.sin_est = None
        self.cos_est = None

    def update(self, angle_deg):
        if angle_deg is None or not np.isfinite(angle_deg):
            if self.sin_est is not None and self.cos_est is not None:
                return float(np.degrees(np.arctan2(self.sin_est, self.cos_est)) / 2.0)
            return None

        rad2 = np.radians(float(angle_deg) * 2.0)
        s_val = float(np.sin(rad2))
        c_val = float(np.cos(rad2))

        self.sin_win.append(s_val)
        self.cos_win.append(c_val)

        # Mediana nas componentes do vetor duplicado (imune a outliers angulares)
        s_med = float(np.median(self.sin_win))
        c_med = float(np.median(self.cos_win))

        if self.sin_est is None or self.cos_est is None:
            self.sin_est = s_med
            self.cos_est = c_med
        else:
            self.sin_est = self.alpha_ema * s_med + (1.0 - self.alpha_ema) * self.sin_est
            self.cos_est = self.alpha_ema * c_med + (1.0 - self.alpha_ema) * self.cos_est

        norm = float(np.hypot(self.sin_est, self.cos_est))
        if norm > 1e-9:
            self.sin_est /= norm
            self.cos_est /= norm

        return float(np.degrees(np.arctan2(self.sin_est, self.cos_est)) / 2.0)
