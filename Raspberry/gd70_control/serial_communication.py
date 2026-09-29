import cv2
import numpy as np
import os
import sys
import time
import threading
from collections import deque
import serial
import subprocess
from gpiozero import LED # Controle seguro do pino de reset na Raspberry Pi 5

# ===========================================================================
# CONFIGURAÇÕES FÍSICAS E DE RESET DA STM32 (BLACK PILL)
# ===========================================================================
PINO_RESET_STM = 18  # GPIO 18 (Pino físico 12 da Raspberry Pi)

# Dicionário de telemetria atualizado continuamente pela Thread em background
telemetria_dados = {
    "yaw": 0.0, "rate_z": 0.0, "pwm_bf": 1500, "pwm_as": 1500, "erro": 0.0, "pid_out": 0.0
}

def _varrer_portas(ser, timeout):
    """Tenta abrir alguma das portas até dar timeout (USB pode demorar a reenumerar)."""
    t_fim = time.time() + timeout
    while True:
        for porta in PORTAS_TENTATIVA:
            try:
                ser.port = porta
                ser.open()
                ser.reset_input_buffer()
                print(f"[OK] Conectado à STM32 na porta: {porta}")
                return True
            except Exception:
                pass
        if time.time() >= t_fim:
            return False
        time.sleep(0.5)


def resetar_stm32():
    print("[INFO] Enviando sinal de RESET físico para a STM32 (Black Pill)...")

    # Fecha antes: o USB CDC vai sumir e voltar
    if arduino is not None and arduino.is_open:
        arduino.close()

    try:
        stm_reset = LED(PINO_RESET_STM, active_high=True, initial_value=True)
        stm_reset.off()
        time.sleep(0.1)
        stm_reset.on()
        stm_reset.close()
    except Exception as e:
        print(f"[AVISO] Falha ao gerenciar pino GPIO de Reset: {e}")

    # Reabre na mesma instância (o main.py guarda referência a esse objeto)
    if arduino is not None:
        if not _varrer_portas(arduino, 10):
            print("[ERRO] STM32 não reapareceu após o reset.")


# ===========================================================================
# CONFIGURACOES DE UART (SERIAL) - BUSCA AUTOMATICA E ASYNC THREAD
# ===========================================================================
BAUDRATE = 115200
PORTAS_TENTATIVA = [
    "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2",
    "/dev/ttyACM0", "/dev/ttyACM1"
]

arduino = None

# Executa o reset físico sincronizado de hardware ANTES de varrer as portas seriais
resetar_stm32()

print("[INFO] Procurando STM32 nas portas USB...")
for porta in PORTAS_TENTATIVA:
    try:
        # Abrimos com timeout para evitar que chamadas de leitura congelem a execução
        arduino = serial.Serial(porta, BAUDRATE, timeout=1)
        arduino.flush()
        print(f"[OK] Sucesso! Conectado à STM32 na porta: {porta}")
        break  
    except Exception as e:
        pass  

if arduino is None:
    print("[AVISO] STM32 não encontrada. O código vai rodar, mas sem telemetria.")

def send_ang_serial(angulo):
    if arduino is not None and arduino.is_open:
        angulo = max(0, min(90, angulo))
        msg = f"{angulo:.1f}\n"
        try:
            arduino.write(msg.encode("utf-8"))
        except Exception as e:
            pass  
            
# Nova função para enviar comandos de texto
def send_cmd_serial(cmd):
    if arduino is not None and arduino.is_open:
        msg = f"{cmd}\n"
        try:
            arduino.write(msg.encode("utf-8"))
            print(f"[SERIAL] Comando enviado: {cmd}")
        except Exception as e:
            pass

# Loop assíncrono que consome os dados enviados pelo cabo USB da Black Pill
def thread_leitura_telemetria():
    global telemetria_dados
    if arduino is None:
        return
        
    print("[OK] Thread paralela de leitura de telemetria rodando.")
    while true:
         if not arduino.is_open:      # durante o reset
            time.sleep(0.2)
            continue
        try:
            if arduino.in_waiting > 0:
                linha = arduino.readline().decode('utf-8', errors='ignore').strip()
                if not_linha := not linha:
                    continue
                
                dados = linha.split(',')
                if len(dados) == 6:
                    telemetria_dados["yaw"]     = float(dados[0])
                    telemetria_dados["rate_z"]  = float(dados[1])
                    telemetria_dados["pwm_bf"]  = int(dados[2])
                    telemetria_dados["pwm_as"]  = int(dados[3])
                    telemetria_dados["erro"]    = float(dados[4])
                    telemetria_dados["pid_out"] = float(dados[5])
        except Exception as e:
            time.sleep(0.1)
        time.sleep(0.01)
