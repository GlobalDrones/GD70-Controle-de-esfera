from video_streamer import *
from serial_communication import *
from camera_and_image_manipulation import *
from configs import *
from pwm_input import iniciar_pwm_control

# Inicializa a Thread se houver conexão com o microcontrolador
if arduino is not None:
    threading.Thread(target=thread_leitura_telemetria, daemon=True).start()

# ===========================================================================
# MAIN
# ===========================================================================
def main():
    cmtx0, dist0, cmtx1, dist1, R_rel, T_rel = load_params()
    sgbm = SGBMParams()
    left_m, right_m, wls = sgbm.build()
    cam0 = AsyncCamera(CAM0_ID, "cam0-esq", FRAME_W, FRAME_H)
    cam1 = AsyncCamera(CAM1_ID, "cam1-dir", FRAME_W, FRAME_H)
    ok0 = cam0.start()
    ok1 = cam1.start()

    if not ok0 and not ok1:
        print("[ERRO] Nenhuma camera abriu.")
        if arduino is not None: arduino.close()
        sys.exit(1)

    # Modo mono: só uma das duas abriu -> não da pra fazer estereoscopia
    modo_mono = not (ok0 and ok1)
    cam_ativa = cam0 if ok0 else cam1
    scale_mgr = None
    if not modo_mono:
        scale_mgr = ScaleManager(cmtx0, dist0, cmtx1, dist1, R_rel, T_rel, FRAME_W, FRAME_H)
    else:
        print(f"[AVISO] So {cam_ativa.name} abriu. Rodando em modo MONO (sem profundidade/estereoscopia).")

    print("[INFO] Aguardando o primeiro frame...")
    t_wait = time.time() + 15
    while time.time() < t_wait:
        if modo_mono:
            if cam_ativa.read() is not None:
                break
        elif cam0.read() is not None and cam1.read() is not None:
            break
        time.sleep(0.1)
    else:
        print("[ERRO] Timeout aguardando as cameras. Verifique o RTSP.")
        cam0.stop()
        cam1.stop()
        if arduino is not None: arduino.close()
        sys.exit(1)
        
    print("[OK] Loop de tempo real iniciado!")
    gray_disp = False
    use_wls = True
    scale_key = ord("3")
    if not modo_mono:
        scale_mgr.set(SCALES[scale_key])
    fps = 0.0
    t_last = time.time()
    historico_angulos = deque(maxlen=1)
    media_angulo = 0
    cv2.namedWindow("Stereo Profundidade", cv2.WINDOW_NORMAL)
    
    # Se quiser testar o streamer de vídeo via FFmpeg, descomente as duas linhas abaixo:
    # streamer = iniciar_streamer(FRAME_W, FRAME_H, SCALES[scale_key])
    
    iniciar_pwm_control()

    while True:
        stats_str = "NUVEM CENTRAL -> Sem dados validos na area"
        stats_color = (255, 255, 0)

        if modo_mono:
            # -------------------------------------------------------------
            # MODO MONO: só uma camera ativa, sem rectify/SGBM/profundidade
            # -------------------------------------------------------------
            f0 = cam_ativa.read()
            if f0 is None:
                time.sleep(0.005)
                continue
            escala = SCALES[scale_key]
            out_size = (max(2, int(FRAME_W * escala)), max(2, int(FRAME_H * escala)))
            if (f0.shape[1], f0.shape[0]) != out_size:
                f0 = cv2.resize(f0, out_size)
            rect_l = f0

            hough_vis = rect_l.copy()
            cx, cy = out_size[0] // 2, out_size[1] // 2
            roi_radius = int(160 * escala)

            angulo, linha, dist_alvo, roi_bin = detectar_linha_mono(
                rect_l, cx, cy, roi_radius
            )
            bin_vis = cv2.cvtColor(roi_bin, cv2.COLOR_GRAY2BGR)
            if angulo is not None:
                if angulo > 90:
                    angulo = 180 - angulo
                historico_angulos.append(angulo)
                media_angulo = int(sum(historico_angulos) / len(historico_angulos))
                stats_str = f"ALVO FIXADO (MONO) -> Angulo: {angulo:.1f} deg | Med: {media_angulo:.1f} deg"
                stats_color = (0, 255, 0)
                x1, y1, x2, y2 = linha
                cv2.line(hough_vis, (x1, y1), (x2, y2), (0, 255, 0), 3)
                cv2.line(bin_vis, (x1, y1), (x2, y2), (0, 255, 0), 3)
                send_ang_serial(media_angulo)

            cv2.circle(hough_vis, (cx, cy), roi_radius, (0, 255, 255), 2)
            cv2.circle(bin_vis, (cx, cy), roi_radius, (0, 255, 255), 1)

            cv2.putText(hough_vis, f"Cam {cam_ativa.name} (MONO)", (10, out_size[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(bin_vis, "Binarizacao", (10, out_size[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Só duas colunas (sem painel de profundidade, que exige as duas cameras)
            display = cv2.hconcat([hough_vis, bin_vis])

        else:
            # -------------------------------------------------------------
            # MODO ESTEREO: pipeline original, sem alteracoes
            # -------------------------------------------------------------
            f0 = cam0.read()
            f1 = cam1.read()
            if f0 is None or f1 is None:
                time.sleep(0.005)
                continue
            data = scale_mgr.get()
            out_size = data["size"]
            map1x, map1y, map2x, map2y, Q, focal, baseline = data["maps"]
            if (f0.shape[1], f0.shape[0]) != out_size:
                f0 = cv2.resize(f0, out_size)
            if (f1.shape[1], f1.shape[0]) != out_size:
                f1 = cv2.resize(f1, out_size)
            rect_l = cv2.remap(f0, map1x, map1y, cv2.INTER_LINEAR)
            rect_r = cv2.remap(f1, map2x, map2y, cv2.INTER_LINEAR)
            gl = cv2.cvtColor(rect_l, cv2.COLOR_BGR2GRAY)
            gr = cv2.cvtColor(rect_r, cv2.COLOR_BGR2GRAY)
            dl = left_m.compute(gl, gr)
            if use_wls:
                dr = right_m.compute(gr, gl)
                disp = wls.filter(dl, rect_l, None, dr).astype(np.float32) / 16.0
            else:
                disp = dl.astype(np.float32) / 16.0
            mask = disp > sgbm.min_disp
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

            hough_vis = rect_l.copy()
            cx, cy = out_size[0] // 2, out_size[1] // 2
            roi_radius = int(160 * SCALES[scale_key])

            angulo, linha, dist_alvo, roi_bin = detectar_linha_mais_proxima(
                rect_l, disp, focal, baseline, cx, cy, roi_radius
            )

            bin_vis = cv2.cvtColor(roi_bin, cv2.COLOR_GRAY2BGR)
            if angulo is not None:
                if angulo > 90:
                    angulo = 180 - angulo
                historico_angulos.append(angulo)
                media_angulo = int(sum(historico_angulos) / len(historico_angulos))
                stats_str = f"ALVO FIXADO -> Dist: {dist_alvo:.2f}m | Angulo: {angulo:.1f} deg | Med: {media_angulo:.1f} deg"
                stats_color = (0, 255, 0)
                x1, y1, x2, y2 = linha

                cv2.line(disp_vis, (x1, y1), (x2, y2), (0, 255, 0), 3)
                cv2.line(hough_vis, (x1, y1), (x2, y2), (0, 255, 0), 3)
                cv2.line(bin_vis, (x1, y1), (x2, y2), (0, 255, 0), 3)

                # Envia o ângulo processado pela câmera via cabo USB para a Black Pill
                send_ang_serial(media_angulo)

            cv2.circle(disp_vis, (cx, cy), roi_radius, (255, 255, 255), 1)
            cv2.circle(hough_vis, (cx, cy), roi_radius, (0, 255, 255), 2)
            cv2.circle(bin_vis, (cx, cy), roi_radius, (0, 255, 255), 1)

            step = int(35 * SCALES[scale_key])
            for dy in range(-roi_radius + 15, roi_radius, step):
                for dx in range(-roi_radius + 15, roi_radius, step):
                    if dx**2 + dy**2 <= (roi_radius - 10) ** 2:
                        px, py = cx + dx, cy + dy
                        if mask[py, px] and disp[py, px] > 0:
                            d = (focal * baseline) / disp[py, px]
                            cv2.circle(disp_vis, (px, py), 2, (0, 255, 0), -1)
                            cv2.putText(
                                disp_vis,
                                f"{d:.1f}",
                                (px + 4, py - 4),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.4,
                                (255, 255, 255),
                                1,
                            )

            cv2.putText(hough_vis, "Cam Esquerda", (10, out_size[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(bin_vis, "Binarizacao", (10, out_size[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(disp_vis, "Profundidade", (10, out_size[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Junta as três matrizes redimensionadas horizontalmente (Lado a Lado)
            display = cv2.hconcat([hough_vis, bin_vis, disp_vis])
        
        # Se usar o streamer do FFmpeg ativo, envie os bytes aqui:
        # try:
        #     streamer.stdin.write(display.tobytes())
        # except Exception as e:
        #     escala_atual = SCALES[scale_key]
        #     streamer = iniciar_streamer(FRAME_W, FRAME_H, escala_atual)
            
        now = time.time()
        fps = 0.9 * fps + 0.1 / max(now - t_last, 1e-6)
        t_last = now
        
        draw_mini_hud(display, fps, stats_str, stats_color)
        cv2.imshow("Stereo Profundidade", display)
        
        k = cv2.waitKey(1) & 0xFF
        if k in [ord("q"), 27]:
            break
            
        # -----------------------------------------------------------------
        # WLS
        # -----------------------------------------------------------------
        elif k in [
            ord("w"),
            ord("W")
        ]:

            use_wls = not use_wls


        # -----------------------------------------------------------------
        # L298N
        # -----------------------------------------------------------------
        elif k in [
            ord("i"),
            ord("I")
        ]:

            send_cmd_serial("w")


        elif k in [
            ord("k"),
            ord("K")
        ]:

            send_cmd_serial("s")


        elif k in [
            ord("o"),
            ord("O")
        ]:

            send_cmd_serial("x")


        # -----------------------------------------------------------------
        # BTS7960
        # -----------------------------------------------------------------
        elif k in [
            ord("j"),
            ord("J")
        ]:

            send_cmd_serial("a")


        elif k in [
            ord("l"),
            ord("L")
        ]:

            send_cmd_serial("d")


        elif k in [
            ord("p"),
            ord("P")
        ]:

            send_cmd_serial("c")


        # -----------------------------------------------------------------
        # AS5600
        # -----------------------------------------------------------------
        elif k in [
            ord("e"),
            ord("E")
        ]:

            send_cmd_serial("e")


        # -----------------------------------------------------------------
        # ESCALA
        # -----------------------------------------------------------------
        elif k in SCALES:

            scale_key = k

            scale_mgr.set(
                SCALES[k]
            )


        # -----------------------------------------------------------------
        # SGBM - DISPARIDADE
        # -----------------------------------------------------------------
        elif k in [
            ord("+"),
            ord("=")
        ]:

            sgbm.inc_disp()

            left_m, right_m, wls = sgbm.build()


        elif k == ord("-"):

            sgbm.dec_disp()

            left_m, right_m, wls = sgbm.build()


        # -----------------------------------------------------------------
        # SGBM - MIN DISP
        # -----------------------------------------------------------------
        elif k == ord("A"):

            sgbm.inc_min_disp()

            left_m, right_m, wls = sgbm.build()


        elif k == ord("S"):

            sgbm.dec_min_disp()

            left_m, right_m, wls = sgbm.build()


        # -----------------------------------------------------------------
        # SGBM - BLOCO
        # -----------------------------------------------------------------
        elif k in [
            ord("b"),
            ord("B")
        ]:

            sgbm.cycle_block()

            left_m, right_m, wls = sgbm.build()


        # -----------------------------------------------------------------
        # SGBM - RESET
        # -----------------------------------------------------------------
        elif k == ord("r"):

            sgbm.reset()

            left_m, right_m, wls = sgbm.build()


        # -----------------------------------------------------------------
        # RESET FÍSICO - GPIO24
        # -----------------------------------------------------------------
        elif k == ord("R"):

            resetar_stm32()

            
    # Se usar o streamer, fecha os descritores de pipe ao sair
    # streamer.stdin.close()
    # streamer.wait()
    
    cam0.stop()
    cam1.stop()
    if arduino is not None:
        arduino.close()
    cv2.destroyAllWindows()
    print("[OK] Encerrado graciosamente")

if __name__ == "__main__":
    main()
