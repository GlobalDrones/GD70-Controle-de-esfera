
from video_streamer import *
from serial_communication import *
from camera_and_image_manipulation import *
from configs import *
from pwm_input import iniciar_pwm_control


# Teclas que vao diretamente para o STM32.
TECLAS_STM = set("wsxadcekypzt0F")

# Controle de transmissão de ângulo pela visão.
visao_atualiza_angulo = True


# Inicializa a thread de telemetria.
if arduino is not None:
    threading.Thread(
        target=thread_leitura_telemetria,
        daemon=True
    ).start()


# ===========================================================================
# MAIN
# ===========================================================================

def main():
    global visao_atualiza_angulo, GUI_ENABLED

    cmtx0, dist0, cmtx1, dist1, R_rel, T_rel = load_params()

    cam0 = AsyncCamera(
        CAM0_ID,
        "cam0-esq",
        FRAME_W,
        FRAME_H
    )

    cam1 = AsyncCamera(
        CAM1_ID,
        "cam1-dir",
        FRAME_W,
        FRAME_H
    )

    ok0 = cam0.start()
    ok1 = cam1.start()

    if not ok0 and not ok1:
        print("[ERRO] Nenhuma camera abriu.")

        if arduino is not None:
            arduino.close()

        sys.exit(1)

    # Modo mono: apenas uma câmera disponível.
    modo_mono = not (ok0 and ok1)

    cam_ativa = cam0 if ok0 else cam1

    scale_mgr = None

    if not modo_mono:
        scale_mgr = ScaleManager(
            cmtx0,
            dist0,
            cmtx1,
            dist1,
            R_rel,
            T_rel,
            FRAME_W,
            FRAME_H
        )
    else:
        print(
            f"[AVISO] So {cam_ativa.name} abriu. "
            "Rodando em modo MONO."
        )

    # -----------------------------------------------------------------------
    # Aguarda os primeiros frames.
    # -----------------------------------------------------------------------

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
        print(
            "[ERRO] Timeout aguardando as cameras. "
            "Verifique o RTSP."
        )

        cam0.stop()
        cam1.stop()

        if arduino is not None:
            arduino.close()

        sys.exit(1)

    print("[OK] Loop de tempo real iniciado!")

    # -----------------------------------------------------------------------
    # Estado inicial.
    # -----------------------------------------------------------------------

    scale_key = ord("3")

    if not modo_mono:
        scale_mgr.set(SCALES[scale_key])

    fps = 0.0
    t_last = time.time()
    t_debug_last = 0.0

    historico_angulos = deque(maxlen=1)
    media_angulo = 0

    if GUI_ENABLED:
        try:
            cv2.namedWindow(
                "Stereo Reconstrucao 3D",
                cv2.WINDOW_NORMAL
            )

        except Exception as e:
            print(
                f"[AVISO] Nao foi possivel abrir janela OpenCV: {e}. "
                "Desativando GUI local."
            )

            GUI_ENABLED = False

    if not GUI_ENABLED:
        print(
            "[INFO] GUI local desativada (headless): "
            "stream ativo em http://<IP>:8080/supervisorio_esfera.html"
        )

    gui_period = 1.0 / max(GUI_MAX_FPS, 0.1)
    t_gui_last = 0.0

    iniciar_pwm_control()

    # =========================================================================
    # LOOP PRINCIPAL
    # =========================================================================

    while True:

        stats_str = "SEM LINHA 3D VALIDA"
        stats_color = (255, 255, 0)

        # =====================================================================
        # MODO MONO
        # =====================================================================

        if modo_mono:

            f0 = cam_ativa.read()

            if f0 is None:
                time.sleep(0.005)
                continue

            escala = SCALES[scale_key]

            out_size = (
                max(2, int(FRAME_W * escala)),
                max(2, int(FRAME_H * escala))
            )

            if (f0.shape[1], f0.shape[0]) != out_size:
                f0 = cv2.resize(
                    f0,
                    out_size,
                    interpolation=cv2.INTER_AREA
                )

            rect_l = f0

            hough_vis = rect_l.copy()

            cx = out_size[0] // 2
            cy = out_size[1] // 2

            roi_radius = int(160 * escala)

            angulo, linha, dist_alvo, roi_bin = detectar_linha_mono(
                rect_l,
                cx,
                cy,
                roi_radius
            )

            bin_vis = cv2.cvtColor(
                roi_bin,
                cv2.COLOR_GRAY2BGR
            )

            if angulo is not None:

                historico_angulos.append(angulo)

                media_angulo = int(
                    sum(historico_angulos)
                    / len(historico_angulos)
                )

                stats_str = (
                    f"ALVO MONO | "
                    f"Bearing: {angulo:+.1f} deg | "
                    f"Med: {media_angulo:+.1f} deg | "
                    f"SEM DISTANCIA"
                )

                stats_color = (0, 255, 0)

                x1, y1, x2, y2 = linha

                cv2.line(
                    hough_vis,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    3
                )

                cv2.line(
                    bin_vis,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    3
                )

                if visao_atualiza_angulo:
                    send_ang_serial(media_angulo, -1.0)

            cv2.circle(
                hough_vis,
                (cx, cy),
                roi_radius,
                (0, 255, 255),
                2
            )

            cv2.circle(
                bin_vis,
                (cx, cy),
                roi_radius,
                (0, 255, 255),
                1
            )

            cv2.putText(
                hough_vis,
                f"Cam {cam_ativa.name} (MONO)",
                (10, out_size[1] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2
            )

            cv2.putText(
                bin_vis,
                "Binarizacao",
                (10, out_size[1] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

            display = cv2.hconcat([
                hough_vis,
                bin_vis
            ])

        # =====================================================================
        # MODO ESTEREO: INTERSECAO DOS PLANOS
        # =====================================================================

        else:

            f0 = cam0.read()
            f1 = cam1.read()

            if f0 is None or f1 is None:
                time.sleep(0.005)
                continue

            data = scale_mgr.get()

            out_size = data["size"]
            geometry = data["geometry"]

            if (f0.shape[1], f0.shape[0]) != out_size:
                f0 = cv2.resize(
                    f0,
                    out_size,
                    interpolation=cv2.INTER_AREA
                )

            if (f1.shape[1], f1.shape[0]) != out_size:
                f1 = cv2.resize(
                    f1,
                    out_size,
                    interpolation=cv2.INTER_AREA
                )

            # Retificação.
            rect_l = cv2.remap(
                f0,
                geometry["map1x"],
                geometry["map1y"],
                cv2.INTER_LINEAR
            )

            rect_r = cv2.remap(
                f1,
                geometry["map2x"],
                geometry["map2y"],
                cv2.INTER_LINEAR
            )

            hough_vis_l = rect_l.copy()
            hough_vis_r = rect_r.copy()

            cx = out_size[0] // 2
            cy = out_size[1] // 2

            roi_radius = int(160 * SCALES[scale_key])

            (
                reconstruction,
                roi_left,
                roi_right,
                edges_left,
                edges_right
            ) = detectar_e_reconstruir(
                rect_l,
                rect_r,
                geometry,
                cx,
                cy,
                roi_radius,
                SCALES[scale_key]
            )
            
            # -----------------------------------------------------------------
            # Diagnóstico da associação estéreo.
            # -----------------------------------------------------------------

            debug = getattr(detectar_e_reconstruir, "debug", {})

            now_debug = time.time()

            if now_debug - t_debug_last >= 1.0:
                t_debug_last = now_debug

                
            bin_vis_l = cv2.cvtColor(
                roi_left,
                cv2.COLOR_GRAY2BGR
            )

            bin_vis_r = cv2.cvtColor(
                roi_right,
                cv2.COLOR_GRAY2BGR
            )

            # -----------------------------------------------------------------
            # Resultado geométrico válido.
            # -----------------------------------------------------------------

            if reconstruction is not None:

                angulo = reconstruction["bearing"]
                dist_alvo = reconstruction["distance"]

                historico_angulos.append(angulo)

                media_angulo = int(
                    sum(historico_angulos)
                    / len(historico_angulos)
                )

                stats_str = (
                    f"LINHA 3D | "
                    f"D MEDIA: {dist_alvo:.2f}m | "
                    f"Bearing: {angulo:+.1f} deg | "
                    f"Plano: {reconstruction['plane_sine']:.3f}"
                )

                stats_color = (0, 255, 0)

                left_candidate = reconstruction["left"]
                right_candidate = reconstruction["right"]

                desenhar_segmento(
                    hough_vis_l,
                    left_candidate,
                    (0, 255, 0),
                    3
                )

                desenhar_segmento(
                    hough_vis_r,
                    right_candidate,
                    (0, 255, 0),
                    3
                )

                desenhar_segmento(
                    bin_vis_l,
                    left_candidate,
                    (0, 255, 0),
                    3
                )

                desenhar_segmento(
                    bin_vis_r,
                    right_candidate,
                    (0, 255, 0),
                    3
                )

                # Envio serial: mantém a interface atual.
                if visao_atualiza_angulo:
                    send_ang_serial(
                        media_angulo,
                        dist_alvo
                    )

            else:
                stats_str = (
                    "SEM RECONSTRUCAO 3D VALIDA "
                    "| Verifique as duas deteccoes"
                )

            # -----------------------------------------------------------------
            # ROI e informações de imagem.
            # -----------------------------------------------------------------

            for frame in (
                hough_vis_l,
                hough_vis_r,
                bin_vis_l,
                bin_vis_r
            ):
                cv2.circle(
                    frame,
                    (cx, cy),
                    roi_radius,
                    (0, 255, 255),
                    1
                )

            cv2.putText(
                hough_vis_l,
                "Cam Esquerda",
                (10, out_size[1] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2
            )

            cv2.putText(
                hough_vis_r,
                "Cam Direita",
                (10, out_size[1] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2
            )

            cv2.putText(
                bin_vis_l,
                "Bin Esquerda",
                (10, out_size[1] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

            cv2.putText(
                bin_vis_r,
                "Bin Direita",
                (10, out_size[1] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

            # Exibe as duas câmeras e as respectivas binarizações.
            display = cv2.hconcat([
                hough_vis_l,
                hough_vis_r,
                bin_vis_l,
                bin_vis_r
            ])

        # =====================================================================
        # FPS E TRANSMISSÃO
        # =====================================================================

        now = time.time()

        fps = (
            0.9 * fps
            + 0.1 / max(now - t_last, 1e-6)
        )

        t_last = now

        draw_mini_hud(
            display,
            fps,
            stats_str,
            stats_color,
            visao_atualiza_angulo
        )

        bridge.update_frame(display)

        # =====================================================================
        # GUI
        # =====================================================================

        k = -1

        if GUI_ENABLED:

            if (now - t_gui_last) >= gui_period:

                t_gui_last = now

                disp_local = display

                if GUI_SCALE != 1.0:
                    disp_local = cv2.resize(
                        display,
                        None,
                        fx=GUI_SCALE,
                        fy=GUI_SCALE,
                        interpolation=cv2.INTER_AREA
                    )

                try:
                    cv2.imshow(
                        "Stereo Reconstrucao 3D",
                        disp_local
                    )

                except Exception as e:
                    print(
                        f"[AVISO] Erro na GUI: {e}. "
                        "Desativando GUI local."
                    )

                    GUI_ENABLED = False

            if GUI_ENABLED:
                try:
                    k = cv2.waitKey(1) & 0xFF
                except Exception:
                    k = -1

        else:
            time.sleep(0.001)

        if k in (-1, 255):
            continue

        ch = chr(k) if k < 128 else ""

        # =====================================================================
        # TECLADO
        # =====================================================================

        if k in (ord("q"), 27):
            break

        elif ch in TECLAS_STM:
            send_cmd_serial(ch)

        elif ch == "v":
            visao_atualiza_angulo = not visao_atualiza_angulo

            status_txt = (
                "ATIVADO"
                if visao_atualiza_angulo
                else "PAUSADO"
            )

            print(
                ">> [VISAO] Envio automatico de angulo: "
                f"{status_txt}"
            )

        elif k in SCALES:

            scale_key = k

            if scale_mgr is not None:
                scale_mgr.set(SCALES[k])

            print(
                f">> [ESCALA] {SCALES[k] * 100:.0f}%"
            )

        elif ch == "!":
            resetar_stm32()

    # =========================================================================
    # ENCERRAMENTO
    # =========================================================================

    cam0.stop()
    cam1.stop()

    if arduino is not None:
        arduino.close()

    if GUI_ENABLED:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

    print("[OK] Encerrado graciosamente")


if __name__ == "__main__":
    main()
