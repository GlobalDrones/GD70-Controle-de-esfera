
# GD70 - Controle de Esfera 🎾

![Status](https://img.shields.io/badge/Status-Em%20Desenvolvimento-yellow)
![Tech](https://img.shields.io/badge/Hardware-Arduino%20%2F%20ESP32-blue)
![Tech](https://img.shields.io/badge/-RaspberryPi-C51A4A?style=for-the-badge&logo=Raspberry-Pi)

Este repositório contém o firmware e os algoritmos de processamento de imagem do **GD70**, um sistema desenvolvido pela **Global Drones** para o alinhamento automático de um gimbal com base em referências lineares detectadas por vídeo.

## 🛠️ Guia Prático: Configuração Raspberry Pi

Para operar o sistema em campo, siga os passos de configuração de rede e visualização.

### 1. Configuração de Rede
Defina um IP estático na interface Ethernet (`eth0`) para comunicação direta com as câmeras.

| Comando | Descrição |
| :--- | :--- |
| `sudo ip addr add 192.168.144.100/24 dev eth0` | Atribui o IP `192.168.144.100`. |
| `sudo ip link set eth0 up` | Ativa a interface de rede. |
| `vlc rtsp://192.168.144.26:8554/main.264` | Testa stream 1 (opcional)|
| `vlc rtsp://192.168.144.2:8554/main.264` | Testa stream 2 (opcional)

<img width="958" height="530" alt="image" src="https://github.com/user-attachments/assets/a4ee5a9d-f3ec-4ccd-8239-9e731bbafc30" />

### 2. Download
Clone o repositório na sua Raspberry:
    `git clone https://github.com/GlobalDrones/GD70-Controle-de-esfera.git`

### 3. Cailbração
 Dentro do repositório:
 -  Atualizar a Raspberry:
	 - `sudo apt update && sudo apt install python3-venv -y`
 - Ativar o virtual enviroment
	 - `cd Raspberry/python_stereo_camera_calibrate/`
	 - `python3 -m venv python_stereo_camera_calibrate/env`
	 - `source python_stereo_camera_calibrate/env/bin/activate`
	 - `pip install -r requirements.txt` (se não funcionar, observar mensagem de erros sobre 'externally managed enviroment')
  - Calibrar as cameras para estereoscopia conforme as referencias listadas no fim deste README.
	  - `python3 calib.py`

### 4. Execução
- em `python_stereo_camera_calibrate/`:
-- `python3 depth_map.py`

## Referências
-   O repositório [stereo_vision (OmidAlekasir)](https://github.com/OmidAlekasir/stereo_vision) fornece código de exemplo para testar configurações de visão estéreo, uma técnica de processamento de imagem utilizada para estimativa de profundidade. O projeto enfatiza que uma calibração de alta qualidade é necessária para corrigir a distorções da lente e alinhar as câmeras.
    

- O repositório [python_stereo_camera_calibrate (TemugeB)](https://github.com/TemugeB/python_stereo_camera_calibrate) fornece um script para calibração baseado em Python. O objetivo é obter parâmetros 3D através de triangulação, capturando frames pareados das duas câmeras para calcular as matrizes de rotação e translação entre elas.
