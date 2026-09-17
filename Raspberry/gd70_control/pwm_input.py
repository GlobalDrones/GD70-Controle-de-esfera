import time
import threading

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

pwm_inputs = {
    "CH12": DigitalInputDevice(GPIO_CH12, pull_up=False),
    "CH13": DigitalInputDevice(GPIO_CH13, pull_up=False),
    "CH14": DigitalInputDevice(GPIO_CH14, pull_up=False),
    "CH15": DigitalInputDevice(GPIO_CH15, pull_up=False),
}

pulse_start_ns = {ch: None for ch in pwm_inputs}
pulse_width_us = {ch: 0 for ch in pwm_inputs}
button_pressed = {ch: False for ch in pwm_inputs}
ad_stop_sent = False


def _rising_edge(channel):
    pulse_start_ns[channel] = time.monotonic_ns()


def _falling_edge(channel):
    start = pulse_start_ns[channel]
    if start is None:
        return

    width_us = (time.monotonic_ns() - start) / 1000.0

    if 700 <= width_us <= 2300:
        pulse_width_us[channel] = width_us


for _channel, _device in pwm_inputs.items():
    _device.when_activated = (lambda ch=_channel: _rising_edge(ch))
    _device.when_deactivated = (lambda ch=_channel: _falling_edge(ch))


def _process_button(channel):
    pwm = pulse_width_us[channel]

    # Pressão: dispara só quando estava solto, e trava até soltar de novo
    if BUTTON_HIGH < pwm < BUTTON_MAX:
        if not button_pressed[channel]:
            print(f"[PWM] {channel} = {pwm:.0f} us -> {COMMANDS[channel].upper()}")
            send_cmd_serial(COMMANDS[channel])
            button_pressed[channel] = True

    # Solto: rearma o botão
    elif BUTTON_MIN < pwm < BUTTON_LOW:
        if button_pressed[channel]:
            print(f"[PWM] {channel} = {pwm:.0f} us -> SOLTO / REARMADO")
        button_pressed[channel] = False


def _process_ad_stop():
    global ad_stop_sent

    ch14_released = pulse_width_us["CH14"] < BUTTON_LOW
    ch15_released = pulse_width_us["CH15"] < BUTTON_LOW

    if ch14_released and ch15_released:
        if not ad_stop_sent:
            print(
                f"[PWM] CH14 = {pulse_width_us['CH14']:.0f} us + "
                f"CH15 = {pulse_width_us['CH15']:.0f} us -> PARAR"
            )
            send_cmd_serial("c")
            ad_stop_sent = True
    else:
        # Pelo menos um A/D pressionado: libera novo "c" quando ambos soltarem
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
    ambos mandando comando pro mesmo `arduino`."""
    threading.Thread(target=_loop_pwm, daemon=True).start()