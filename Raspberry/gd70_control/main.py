from video_streamer import *
from serial_communication import *
from camera_and_image_manipulation import *
from configs import *

# Inicializa a Thread se houver conexão com o microcontrolador
if arduino is not None:
    threading.Thread(target=thread_leitura_telemetria, daemon=True).start()


# ===========================================================================
# MAIN
# ===========================================================================
def main():

    # -----------------------------------------------------------------------
    # GPIO DO RESET
    # Pino físico 18 da Raspberry Pi = GPIO24 (BCM)
    # -----------------------------------------------------------------------
    RESET_GPIO = 24

    try:
        import RPi.GPIO as GPIO

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(RESET_GPIO, GPIO.OUT, initial=GPIO.LOW)

        reset_gpio_ok = True
        print("[OK] GPIO24 (pino físico 18) configurado para RESET.")

    except Exception as e:
        GPIO = None
        reset_gpio_ok = False
        print(f"[AVISO] Não foi possível configurar GPIO24: {e}")
    resetar_stm32()

    # -----------------------------------------------------------------------
    # FUNÇÃO DE RESET
    # -----------------------------------------------------------------------
    def acionar_reset():

        if not reset_gpio_ok:
            print("[ERRO] GPIO24 não está disponível.")
            return

        print("[RESET] Acionando reset pelo GPIO24...")

        GPIO.output(RESET_GPIO, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(RESET_GPIO, GPIO.LOW)

        print("[RESET] Pulso de reset enviado.")


    # -----------------------------------------------------------------------
    # CARREGA CALIBRAÇÃO
    # -----------------------------------------------------------------------
    cmtx0, dist0, cmtx1, dist1, R_rel, T_rel = load_params()

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

    sgbm = SGBMParams()
    left_m, right_m, wls = sgbm.build()


    # -----------------------------------------------------------------------
    # CÂMERAS
    # -----------------------------------------------------------------------
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


    if not cam0.start() or not cam1.start():

        if arduino is not None:
            arduino.close()

        if reset_gpio_ok:
            GPIO.cleanup()

        sys.exit(1)


    # -----------------------------------------------------------------------
    # AGUARDA PRIMEIRO FRAME
    # -----------------------------------------------------------------------
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

        if arduino is not None:
            arduino.close()

        if reset_gpio_ok:
            GPIO.cleanup()

        sys.exit(1)


    print("[OK] Loop de tempo real iniciado!")


    # -----------------------------------------------------------------------
    # CONFIGURAÇÕES
    # -----------------------------------------------------------------------
    gray_disp = False
    use_wls = True

    scale_key = ord("3")
    scale_mgr.set(SCALES[scale_key])

    fps = 0.0
    t_last = time.time()

    historico_angulos = deque(maxlen=1)
    media_angulo = 0

    cv2.namedWindow(
        "Stereo Profundidade",
        cv2.WINDOW_NORMAL
    )


    prof = Profiler(print_every=30)


    # =========================================================================
    # LOOP PRINCIPAL
    # =========================================================================
    try:

        while True:

            # -----------------------------------------------------------------
            # CAPTURA
            # -----------------------------------------------------------------
            prof.start("capture")

            f0 = cam0.read()
            f1 = cam1.read()

            if f0 is not None:
                f0 = cv2.flip(f0, -1)

            if f1 is not None:
                f1 = cv2.flip(f1, -1)

            prof.stop()


            if f0 is None or f1 is None:
                time.sleep(0.005)
                continue


            # -----------------------------------------------------------------
            # REMAP / RETIFICAÇÃO
            # -----------------------------------------------------------------
            prof.start("remap")

            data = scale_mgr.get()

            out_size = data["size"]

            map1x, map1y, map2x, map2y, Q, focal, baseline = data["maps"]


            if (f0.shape[1], f0.shape[0]) != out_size:
                f0 = cv2.resize(f0, out_size)

            if (f1.shape[1], f1.shape[0]) != out_size:
                f1 = cv2.resize(f1, out_size)


            rect_l = cv2.remap(
                f0,
                map1x,
                map1y,
                cv2.INTER_LINEAR
            )

            rect_r = cv2.remap(
                f1,
                map2x,
                map2y,
                cv2.INTER_LINEAR
            )

            prof.stop()


            # -----------------------------------------------------------------
            # STATUS
            # -----------------------------------------------------------------
            stats_str = "NUVEM CENTRAL -> Sem dados validos na area"
            stats_color = (255, 255, 0)


            # -----------------------------------------------------------------
            # STEREO MATCHING
            # -----------------------------------------------------------------
            prof.start("stereo_match")

            gl = cv2.cvtColor(
                rect_l,
                cv2.COLOR_BGR2GRAY
            )

            gr = cv2.cvtColor(
                rect_r,
                cv2.COLOR_BGR2GRAY
            )


            dl = left_m.compute(
                gl,
                gr
            )


            if use_wls:

                dr = right_m.compute(
                    gr,
                    gl
                )

                disp = wls.filter(
                    dl,
                    rect_l,
                    None,
                    dr
                ).astype(np.float32) / 16.0

            else:

                disp = dl.astype(np.float32) / 16.0

            prof.stop()


            # -----------------------------------------------------------------
            # PÓS-PROCESSAMENTO DISPARIDADE
            # -----------------------------------------------------------------
            prof.start("disp_postproc")

            mask = disp > sgbm.min_disp

            valid_disp = disp[mask]


            if len(valid_disp) > 0:

                min_d_clip = np.percentile(
                    valid_disp,
                    2
                )

                max_d_clip = np.percentile(
                    valid_disp,
                    98
                )

                disp_clipped = np.clip(
                    disp,
                    min_d_clip,
                    max_d_clip
                )

            else:

                disp_clipped = disp


            disp_norm = cv2.normalize(
                disp_clipped,
                None,
                0,
                255,
                cv2.NORM_MINMAX
            )


            if gray_disp:

                disp_vis = cv2.cvtColor(
                    np.uint8(disp_norm),
                    cv2.COLOR_GRAY2BGR
                )

            else:

                disp_vis = cv2.applyColorMap(
                    np.uint8(disp_norm),
                    cv2.COLORMAP_JET
                )


            disp_vis[~mask] = 0

            prof.stop()


            # -----------------------------------------------------------------
            # DETECÇÃO DA LINHA
            # -----------------------------------------------------------------
            prof.start("hough")

            hough_vis = rect_l.copy()

            cx = out_size[0] // 2
            cy = out_size[1] // 2

            roi_radius = int(
                160 * SCALES[scale_key]
            )


            angulo, linha, dist_alvo, roi_bin = detectar_linha_mais_proxima(
                rect_l,
                disp,
                focal,
                baseline,
                cx,
                cy,
                roi_radius
            )

            prof.stop()


            # -----------------------------------------------------------------
            # VISUALIZAÇÃO
            # -----------------------------------------------------------------
            prof.start("draw_viz")

            bin_vis = cv2.cvtColor(
                roi_bin,
                cv2.COLOR_GRAY2BGR
            )


            if angulo is not None:

                if angulo > 90:
                    angulo = 180 - angulo


                historico_angulos.append(
                    angulo
                )


                media_angulo = int(
                    sum(historico_angulos) /
                    len(historico_angulos)
                )


                stats_str = (
                    f"ALVO FIXADO -> "
                    f"Dist: {dist_alvo:.2f}m | "
                    f"Angulo: {angulo:.1f} deg | "
                    f"Med: {media_angulo:.1f} deg"
                )

                stats_color = (0, 255, 0)


                x1, y1, x2, y2 = linha


                cv2.line(
                    disp_vis,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    3
                )

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


                # Envia ângulo para Black Pill
                send_ang_serial(media_angulo)


            # -----------------------------------------------------------------
            # ROI
            # -----------------------------------------------------------------
            cv2.circle(
                disp_vis,
                (cx, cy),
                roi_radius,
                (255, 255, 255),
                1
            )

            cv2.circle(
                hough_vis,
                (cx, cy),
                roi_radius,
                (0, 255, 0),
                2
            )

            cv2.circle(
                bin_vis,
                (cx, cy),
                roi_radius,
                (255, 255, 255),
                1
            )


            # -----------------------------------------------------------------
            # PONTOS DE PROFUNDIDADE
            # -----------------------------------------------------------------
            step = int(
                35 * SCALES[scale_key]
            )


            for dy in range(
                -roi_radius + 15,
                roi_radius,
                step
            ):

                for dx in range(
                    -roi_radius + 15,
                    roi_radius,
                    step
                ):

                    if dx**2 + dy**2 <= (
                        roi_radius - 10
                    ) ** 2:

                        px = cx + dx
                        py = cy + dy


                        if (
                            mask[py, px]
                            and disp[py, px] > 0
                        ):

                            d = (
                                focal *
                                baseline
                            ) / disp[py, px]


                            cv2.circle(
                                disp_vis,
                                (px, py),
                                2,
                                (0, 255, 0),
                                -1
                            )


                            cv2.putText(
                                disp_vis,
                                f"{d:.1f}",
                                (px + 4, py - 4),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.4,
                                (255, 255, 255),
                                1
                            )


            # -----------------------------------------------------------------
            # TÍTULOS
            # -----------------------------------------------------------------
            cv2.putText(
                hough_vis,
                "Cam Esquerda",
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

            cv2.putText(
                disp_vis,
                "Profundidade",
                (10, out_size[1] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )


            # -----------------------------------------------------------------
            # JUNTA OS VÍDEOS
            # -----------------------------------------------------------------
            display = cv2.hconcat(
                [
                    hough_vis,
                    bin_vis,
                    disp_vis
                ]
            )

            prof.stop()


            # -----------------------------------------------------------------
            # FPS
            # -----------------------------------------------------------------
            now = time.time()

            fps = (
                0.9 * fps +
                0.1 / max(
                    now - t_last,
                    1e-6
                )
            )

            t_last = now


            # -----------------------------------------------------------------
            # HUD
            # -----------------------------------------------------------------
            prof.start("imshow")

            draw_mini_hud(
                display,
                fps,
                stats_str,
                stats_color
            )


            # -----------------------------------------------------------------
            # PAINEL DE CONTROLES
            # -----------------------------------------------------------------
            h, w = display.shape[:2]

            instrucoes = [
                "L298N: [I] Subir   [K] Descer   [O] Parar",
                "BTS7960: [J] Sentido A   [L] Sentido D   [P] Parar",
                "AS5600: [E] Ler angulo",
                "Video: [W] WLS   [D] Cinza   [3/4/5...] Escala",
                "SGBM: [+/-] Disp   [A/S] MinDisp   [B] Bloco   [R] Reset SGBM",
                "Sistema: [Q/ESC] Sair   [SHIFT+R] Reset GPIO24"
            ]


            linha_altura = 24

            painel_altura = (
                len(instrucoes) *
                linha_altura +
                12
            )


            painel = np.zeros(
                (
                    painel_altura,
                    w,
                    3
                ),
                dtype=np.uint8
            )


            for i, texto in enumerate(instrucoes):

                cv2.putText(
                    painel,
                    texto,
                    (
                        10,
                        20 + i * linha_altura
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA
                )


            display_com_instrucoes = cv2.vconcat(
                [
                    display,
                    painel
                ]
            )


            cv2.imshow(
                "Stereo Profundidade",
                display_com_instrucoes
            )

            prof.stop()


            # -----------------------------------------------------------------
            # PROFILER
            # -----------------------------------------------------------------
            prof.tick()


            # -----------------------------------------------------------------
            # TECLADO
            # -----------------------------------------------------------------
            k = cv2.waitKey(1) & 0xFF


            # -----------------------------------------------------------------
            # SAIR
            # -----------------------------------------------------------------
            if k in [
                ord("q"),
                ord("Q"),
                27
            ]:

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


    # =========================================================================
    # ENCERRAMENTO
    # =========================================================================
    finally:

        print("[INFO] Encerrando sistema...")

        try:
            cam0.stop()
        except Exception:
            pass

        try:
            cam1.stop()
        except Exception:
            pass

        if arduino is not None:

            try:
                arduino.close()
            except Exception:
                pass

        if reset_gpio_ok:

            try:
                GPIO.output(
                    RESET_GPIO,
                    GPIO.LOW
                )

                GPIO.cleanup()

            except Exception:
                pass

        cv2.destroyAllWindows()

        print("[OK] Sistema encerrado.")


if __name__ == "__main__":
    main()
