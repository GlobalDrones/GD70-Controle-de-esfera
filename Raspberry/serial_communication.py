"""Link com o STM (firmware v2) + ponte TCP para o twin no PC.

Fluxo: reset físico (GPIO18) → busca automática na USB → thread de leitura.
Cada linha lida alimenta `telemetria_dados` (formato TEL v2, mesmas chaves de
sempre — HUD/main.py seguem funcionando) e é retransmitida pela ponte TCP
(``bridge.py``, porta 9999) ao twin/wizard. Comandos vindos por TCP viram
escrita na serial — o app de visão é o ÚNICO dono da porta.

O formato da telemetria é definido UMA vez em ``twin/protocol.py`` (repo
clonado na Rasp; rodar a partir da raiz do repo). gpiozero é opcional: sem
ele (PC de testes), o reset físico vira aviso e o resto funciona.
"""

import os
import sys
import time
import threading

import serial

# Repo na raiz (para importar twin.protocol quando rodado de qualquer cwd)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from twin import protocol  # noqa: E402

try:
    from gpiozero import LED  # Controle seguro do pino de reset na Raspberry Pi 5
except ImportError:  # PC de teste: sem GPIO, o reset físico fica indisponível
    LED = None

from bridge import Bridge  # noqa: E402

# ===========================================================================
# CONFIGURAÇÕES FÍSICAS E DE RESET DA STM32 (BLACK PILL)
# ===========================================================================
PINO_RESET_STM = 18  # GPIO 18 (Pino físico 12 da Raspberry Pi)

# Dicionário de telemetria atualizado continuamente pela Thread em background
# (chaves preservadas da versão anterior + novos campos da v2)
telemetria_dados = {
    "yaw": 0.0,
    "yaw_des": 0.0,
    "rate_z": 0.0,
    "pwm_bf": 1500,
    "pwm_as": 1500,
    "pwm_tr": 1500,
    "erro": 0.0,
    "theta": 0.0,       # |inclinação| total (hypot(sx, sy))
    "sx": 0.0,
    "sy": 0.0,
    "bearing": 0.0,     # último bearing recebido da visão (via STM, quando fresco)
    "D": -1.0,          # distância esfera↔cabo (m; -1 = sem visão)
    "L": 1.0,
    "swing_ativo": 0,
    "yaw_ativo": 0,
}


def resetar_stm32():
    print("[INFO] Enviando sinal de RESET físico para a STM32 (Black Pill)...")
    if LED is None:
        print("[AVISO] gpiozero ausente — reset físico indisponível (PC de teste?).")
        return
    try:
        stm_reset = LED(PINO_RESET_STM, active_high=True, initial_value=True)
        stm_reset.off()
        time.sleep(0.1)
        stm_reset.on()
        print("[OK] STM32 liberada! Aguardando estabilização...")
        stm_reset.close()
        time.sleep(1.5)
    except Exception as e:
        print(f"[AVISO] Falha ao gerenciar pino GPIO de Reset: {e}")


# ===========================================================================
# UART (USB) — BUSCA AUTOMÁTICA
# ===========================================================================
BAUDRATE = 115200
PORTAS_TENTATIVA = [
    "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2",
    "/dev/ttyACM0", "/dev/ttyACM1",
]

# Executa o reset físico ANTES de varrer as portas seriais (boot limpo do STM)
resetar_stm32()

print("[INFO] Procurando STM32 nas portas USB...")
arduino = None
for porta in PORTAS_TENTATIVA:
    try:
        arduino = serial.Serial(porta, BAUDRATE, timeout=1)
        arduino.flush()
        print(f"[OK] Sucesso! Conectado à STM32 na porta: {porta}")
        break
    except Exception:
        pass

if arduino is None:
    print("[AVISO] STM32 não encontrada. O código vai rodar, mas sem telemetria.")

# Ponte TCP: linhas do STM vão ao twin; comandos do twin voltam pela serial
bridge = Bridge(on_client_line=lambda line: send_cmd_serial(line))
if arduino is not None:
    bridge.start()


def send_ang_serial(bearing, D=-1.0):
    """Envia a visão ao STM (contrato V do firmware v2): bearing COM SINAL
    (graus, no corpo) + distância D (m; -1 = sem estereoscopia). O bearing é
    o ERRO de apontamento direto — as câmeras estão fixas ao sensor."""
    if arduino is not None and arduino.is_open:
        bearing = max(-90.0, min(90.0, float(bearing)))
        msg = f"V{bearing:.2f} {float(D):.2f}\n"
        try:
            arduino.write(msg.encode("utf-8"))
        except Exception as e:
            print(f"[ERRO SERIAL] Falha ao enviar visão: {e}")


def send_cmd_serial(cmd):
    if arduino is not None and arduino.is_open:
        msg = f"{cmd}\n"
        try:
            arduino.write(msg.encode("utf-8"))
            if not cmd.startswith("V"):
                print(f"[SERIAL] Comando enviado: {cmd}")
        except Exception as e:
            print(f"[ERRO SERIAL] Falha ao enviar comando '{cmd}': {e}")


def processar_linha_telemetria(linha):
    """Uma linha do STM: TEL → atualiza telemetria_dados; GAIN/logs → só print.
    Toda linha é retransmitida pela ponte (twin vê tudo)."""
    frame = protocol.parse_line(linha)
    if frame is not None:
        d = telemetria_dados
        d["yaw"] = frame.yaw_deg
        d["yaw_des"] = frame.yaw_des_deg
        d["rate_z"] = frame.rate_z_dps
        d["pwm_bf"], d["pwm_as"], d["pwm_tr"] = frame.pwm
        d["sx"], d["sy"] = frame.sx_deg, frame.sy_deg
        d["theta"] = frame.tilt_mag_deg
        d["bearing"] = frame.bearing_deg
        d["D"] = frame.D_m
        d["L"] = frame.L_m
        d["swing_ativo"] = 1 if frame.swing_on else 0
        d["yaw_ativo"] = 1 if frame.yaw_on else 0
        erro = (frame.yaw_deg - frame.yaw_des_deg + 180.0) % 360.0 - 180.0
        d["erro"] = erro
    elif isinstance(frame, tuple):  # GAIN,idx,nome,valor (resposta ao 'R')
        print(f"[STM32] {linha}")
    else:
        print(f"[STM32] {linha}")
    bridge.broadcast(linha)


# Loop assíncrono que consome os dados enviados pelo cabo USB da Black Pill
def thread_leitura_telemetria():
    if arduino is None:
        return
    print("[OK] Thread paralela de leitura de telemetria rodando.")
    while arduino.is_open:
        try:
            if arduino.in_waiting > 0:
                linha = arduino.readline().decode("utf-8", errors="ignore").strip()
                if linha:
                    processar_linha_telemetria(linha)
        except Exception:
            time.sleep(0.05)
        time.sleep(0.005)
