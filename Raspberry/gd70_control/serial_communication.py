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

def resetar_stm32():
    print("[INFO] Enviando sinal de RESET físico para a STM32 (Black Pill)...")
    try:
        # Garante que o pino começa em HIGH (3.3V)
        stm_reset = LED(PINO_RESET_STM, active_high=True, initial_value=True)
        
        # Puxa o pino R (NRST) para LOW (GND) por 100ms para forçar o reset
        stm_reset.off()
        time.sleep(0.1)
        
        # Solta o pino de volta para HIGH
        stm_reset.on()
        print("[OK] STM32 liberada! Aguardando estabilização do setup e calibração...")
        stm_reset.close() 
        time.sleep(1.5) # Tempo necessário para a calibração do MPU em bancada
    except Exception as e:
        print(f"[AVISO] Falha ao gerenciar pino GPIO de Reset: {e}")


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
    while arduino.is_open:
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
