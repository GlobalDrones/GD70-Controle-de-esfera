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
    "yaw": 0.0,
    "rate_z": 0.0,
    "pwm_bf": 1500,
    "pwm_as": 1500,
    "pwm_tr": 1500,
    "erro": 0.0,
    "pid_out": 0.0,
    "fim_cima": 0,
    "fim_baixo": 0,
    "theta": 0.0,
    "theta_ref": 0.0,
    "f_swing": 0.0,
    "swing_ativo": 0,
    "yaw_ativo": 1
}

def resetar_stm32():
    print("[INFO] Enviando sinal de RESET físico para a STM32 (Black Pill)...")

    try:
        stm_reset = LED(
            PINO_RESET_STM,
            active_high=True,
            initial_value=True
        )

        stm_reset.off()
        time.sleep(0.1)
        stm_reset.on()

        print("[OK] STM32 liberada! Aguardando estabilização...")

        stm_reset.close()

        time.sleep(1.5)

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
            
# Função para enviar comandos de texto
def send_cmd_serial(cmd):
    if arduino is not None and arduino.is_open:
        msg = f"{cmd}\n"
        try:
            arduino.write(msg.encode("utf-8"))
            if cmd != 'c':
                print(f"[SERIAL] Comando enviado: {cmd}")
        except Exception as e:
            print(f"[ERRO SERIAL] Falha ao enviar comando '{cmd}': {e}")

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
                if not linha:
                    continue
                
                # Mensagens de texto / logs da STM32
                if linha.startswith("[") or linha.startswith(">>") or linha.startswith("==="):
                    print(f"[STM32]: {linha}")
                    continue

                dados = [d.strip() for d in linha.split(',')]
                
                # Formato completo de 13 campos (com pwm_tr e anti-balanco)
                # [0]=yaw, [1]=rate_z, [2]=bf, [3]=as, [4]=tr, [5]=erro, [6]=pid_out,
                # [7]=fim_cima, [8]=fim_baixo, [9]=theta, [10]=theta_ref, [11]=f_swing, [12]=swing_ativo
                if len(dados) >= 14:
                    try:
                        telemetria_dados["yaw"]         = float(dados[0])
                        telemetria_dados["rate_z"]      = float(dados[1])
                        telemetria_dados["pwm_bf"]      = int(float(dados[2]))
                        telemetria_dados["pwm_as"]      = int(float(dados[3]))
                        telemetria_dados["pwm_tr"]      = int(float(dados[4]))
                        telemetria_dados["erro"]        = float(dados[5])
                        telemetria_dados["pid_out"]     = float(dados[6])
                        telemetria_dados["fim_cima"]    = int(float(dados[7]))
                        telemetria_dados["fim_baixo"]   = int(float(dados[8]))
                        telemetria_dados["theta"]       = float(dados[9])
                        telemetria_dados["theta_ref"]   = float(dados[10])
                        telemetria_dados["f_swing"]     = float(dados[11])
                        telemetria_dados["swing_ativo"] = int(float(dados[12]))
                        telemetria_dados["yaw_ativo"]   = int(float(dados[13]))
                    except (ValueError, IndexError):
                        print(f"[STM32 Log]: {linha}")
                elif len(dados) == 13:
                    try:
                        telemetria_dados["yaw"]         = float(dados[0])
                        telemetria_dados["rate_z"]      = float(dados[1])
                        telemetria_dados["pwm_bf"]      = int(float(dados[2]))
                        telemetria_dados["pwm_as"]      = int(float(dados[3]))
                        telemetria_dados["pwm_tr"]      = int(float(dados[4]))
                        telemetria_dados["erro"]        = float(dados[5])
                        telemetria_dados["pid_out"]     = float(dados[6])
                        telemetria_dados["fim_cima"]    = int(float(dados[7]))
                        telemetria_dados["fim_baixo"]   = int(float(dados[8]))
                        telemetria_dados["theta"]       = float(dados[9])
                        telemetria_dados["theta_ref"]   = float(dados[10])
                        telemetria_dados["f_swing"]     = float(dados[11])
                        telemetria_dados["swing_ativo"] = int(float(dados[12]))
                        telemetria_dados["yaw_ativo"]   = 1
                    except (ValueError, IndexError):
                        print(f"[STM32 Log]: {linha}")
                
                # Formato de 12 campos (legado sem pwm_tr)
                elif len(dados) == 12:
                    try:
                        telemetria_dados["yaw"]         = float(dados[0])
                        telemetria_dados["rate_z"]      = float(dados[1])
                        telemetria_dados["pwm_bf"]      = int(float(dados[2]))
                        telemetria_dados["pwm_as"]      = int(float(dados[3]))
                        telemetria_dados["pwm_tr"]      = 1500
                        telemetria_dados["erro"]        = float(dados[4])
                        telemetria_dados["pid_out"]     = float(dados[5])
                        telemetria_dados["fim_cima"]    = int(float(dados[6]))
                        telemetria_dados["fim_baixo"]   = int(float(dados[7]))
                        telemetria_dados["theta"]       = float(dados[8])
                        telemetria_dados["theta_ref"]   = float(dados[9])
                        telemetria_dados["f_swing"]     = float(dados[10])
                        telemetria_dados["swing_ativo"] = int(float(dados[11]))
                    except (ValueError, IndexError):
                        print(f"[STM32 Log]: {linha}")

                # Formato basico de 8 campos (legado)
                elif len(dados) == 8:
                    try:
                        telemetria_dados["yaw"]         = float(dados[0])
                        telemetria_dados["rate_z"]      = float(dados[1])
                        telemetria_dados["pwm_bf"]      = int(float(dados[2]))
                        telemetria_dados["pwm_as"]      = int(float(dados[3]))
                        telemetria_dados["erro"]        = float(dados[4])
                        telemetria_dados["pid_out"]     = float(dados[5])
                        telemetria_dados["fim_cima"]    = int(float(dados[6]))
                        telemetria_dados["fim_baixo"]   = int(float(dados[7]))
                    except (ValueError, IndexError):
                        print(f"[STM32 Log]: {linha}")
                else:
                    print(f"[STM32]: {linha}")
                
        except Exception as e:
            time.sleep(0.05)
        time.sleep(0.005)
