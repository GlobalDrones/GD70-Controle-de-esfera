import time
import threading
import statistics
from collections import deque

from gpiozero import DigitalInputDevice

from serial_communication import send_cmd_serial


# ===========================================================================
# CONFIGURAÇÃO DOS GPIOs PWM (Air Unit CH12-CH15)
# ===========================================================================
#
# Air Unit CH12 -> GPIO5  -> W
# Air Unit CH13 -> GPIO6  -> S
# Air Unit CH14 -> GPIO13 -> A
# Air Unit CH15 -> GPIO19 -> D
#
# ATENÇÃO: GPIOs da Raspberry Pi trabalham em 3.3 V, não aplicar 5 V direto.
# ===========================================================================

GPIO_CH12 = 5
GPIO_CH13 = 6
GPIO_CH14 = 13
GPIO_CH15 = 19

COMMANDS = {
    "CH12": "w",
    "CH13": "s",
    "CH14": "a",
    "CH15": "d",
}

# PWM: < 1300 us = solto | 1300-1700 us = zona morta | > 1700 us = pressionado
BUTTON_LOW = 1100
BUTTON_HIGH = 1900
BUTTON_MAX = 1960  # filtra surto de leitura
BUTTON_MIN = 1000

# Quantas leituras SEGUIDAS em zona "pressionado" são exigidas antes de
# considerar o botão realmente apertado (a cada 5 ms de loop, 6 leituras
# = ~30 ms sustentados). Filtra ruído/glitch de bind do rádio.
CONFIRM_READS = 6

pwm_inputs = {
    "CH12": DigitalInputDevice(GPIO_CH12, pull_up=False),
    "CH13": DigitalInputDevice(GPIO_CH13, pull_up=False),
    "CH14": DigitalInputDevice(GPIO_CH14, pull_up=False),
    "CH15": DigitalInputDevice(GPIO_CH15, pull_up=False),
}

pulse_start_ns = {ch: None for ch in pwm_inputs}
pulse_width_us = {ch: 0 for ch in pwm_inputs}

# Histórico curto de larguras de pulso válidas por canal — usamos a
# MEDIANA das últimas leituras em vez do valor bruto. Isso filtra os
# outliers causados por atraso do callback Python quando a CPU está
# ocupada com o pipeline de visão (OpenCV/stereo), que corrompem uma
# leitura isolada mas não deslocam a mediana de uma janela curta.
PULSE_HISTORY_LEN = 5
pulse_history = {ch: deque(maxlen=PULSE_HISTORY_LEN) for ch in pwm_inputs}

button_pressed = {ch: False for ch in pwm_inputs}
press_streak = {ch: 0 for ch in pwm_inputs}

# Um canal só "arma" (passa a aceitar pressão) depois de ser lido pelo
# menos uma vez numa faixa válida de solto. Isso impede que ruído/glitch
# do rádio no boot/bind, antes de existir uma leitura real, seja
# confundido com um botão pressionado.
armed = {ch: False for ch in pwm_inputs}

ad_stop_sent = False
stop_streak = 0


def _rising_edge(channel):
    pulse_start_ns[channel] = time.monotonic_ns()


def _falling_edge(channel):
    start = pulse_start_ns[channel]
    if start is None:
        return

    width_us = (time.monotonic_ns() - start) / 1000.0

    if 700 <= width_us <= 2300:
        pulse_history[channel].append(width_us)
        # Mediana da janela recente, não o valor bruto — absorve leituras
        # isoladas corrompidas por atraso de callback sob carga de CPU.
        pulse_width_us[channel] = statistics.median(pulse_history[channel])


for _channel, _device in pwm_inputs.items():
    _device.when_activated = (lambda ch=_channel: _rising_edge(ch))
    _device.when_deactivated = (lambda ch=_channel: _falling_edge(ch))


def _process_button(channel):
    pwm = pulse_width_us[channel]

    # Solto: confirma baseline (arma o canal) e rearma o botão
    if BUTTON_MIN < pwm < BUTTON_LOW:
        if not armed[channel]:
            armed[channel] = True
            print(f"[PWM] {channel} armado (baseline solto confirmado)")

        press_streak[channel] = 0

        if button_pressed[channel]:
            print(f"[PWM] {channel} = {pwm:.0f} us -> SOLTO / REARMADO")
        button_pressed[channel] = False
        return

    # Pressão: só conta se o canal já foi armado (baseline solto visto
    # ao menos uma vez) e exige CONFIRM_READS leituras seguidas nessa
    # faixa antes de disparar — sem isso, um glitch isolado não aciona nada.
    if BUTTON_HIGH < pwm < BUTTON_MAX:
        if not armed[channel]:
            return

        press_streak[channel] += 1

        if press_streak[channel] >= CONFIRM_READS and not button_pressed[channel]:
            print(f"[PWM] {channel} = {pwm:.0f} us -> {COMMANDS[channel].upper()} (confirmado)")
            send_cmd_serial(COMMANDS[channel])
            button_pressed[channel] = True

        return

    # Zona morta ou fora da faixa plausível: qualquer leitura fora do
    # patamar de pressão quebra a sequência de confirmação.
    press_streak[channel] = 0


def _process_ad_stop():
    global ad_stop_sent, stop_streak

    # Só avalia depois que os dois canais tiverem um baseline solto
    # confirmado — evita mandar "c" em cima de leitura de boot/bind.
    if not (armed["CH14"] and armed["CH15"]):
        return

    ch14_released = pulse_width_us["CH14"] < BUTTON_LOW
    ch15_released = pulse_width_us["CH15"] < BUTTON_LOW

    # Mesmo critério de estabilidade do CONFIRM_READS: exige leituras
    # seguidas "soltas" antes de latchar, e QUALQUER leitura fora disso
    # já reseta a sequência e libera um novo "c" no futuro — sem isso,
    # uma única leitura ruidosa cruzando o limiar já disparava de novo.
    if ch14_released and ch15_released:
        stop_streak += 1
        if stop_streak >= CONFIRM_READS and not ad_stop_sent:
            print(
                f"[PWM] CH14 = {pulse_width_us['CH14']:.0f} us + "
                f"CH15 = {pulse_width_us['CH15']:.0f} us -> PARAR"
            )
            send_cmd_serial("c")
            ad_stop_sent = True
    else:
        # Pelo menos um A/D pressionado: libera novo "c" quando ambos soltarem
        stop_streak = 0
        ad_stop_sent = False


def _loop_pwm():
    print("[OK] Thread de leitura PWM (CH12-CH15) rodando.")
    while True:
        for ch in pwm_inputs:
            _process_button(ch)
        _process_ad_stop()
        time.sleep(0.005)


def iniciar_pwm_control():
    """Inicia, em background, a leitura dos canais PWM (CH12-CH15) do
    rádio e o envio dos comandos w/a/s/d/c pela serial já aberta em
    serial_communication.py. Roda em paralelo ao controle por teclado,
    ambos mandando comando pro mesmo `arduino`. Cada canal só passa a
    aceitar pressão depois de um baseline solto confirmado, e exige
    CONFIRM_READS leituras seguidas em zona de pressão antes de disparar
    qualquer comando — protege contra ruído/glitch no boot/bind do rádio."""
    threading.Thread(target=_loop_pwm, daemon=True).start()
