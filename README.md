# GD70 - Controle de Esfera

![Status](https://img.shields.io/badge/Status-Em%20Desenvolvimento-yellow)
![Tech](https://img.shields.io/badge/Hardware-Arduino%20%2F%20ESP32-blue)
![Tech](https://img.shields.io/badge/-RaspberryPi-C51A4A?style=for-the-badge\&logo=Raspberry-Pi)

Este repositório contém o firmware e os algoritmos de processamento de imagem do **GD70**, um sistema desenvolvido pela **Global Drones** para o alinhamento automático de um gimbal com base em referências lineares detectadas por vídeo.

# 🛠️ Guia Prático: Configuração Raspberry Pi

## 1. Download

Clone o repositório na Raspberry Pi:

```bash
git clone https://github.com/GlobalDrones/GD70-Controle-de-esfera.git
```
---

## 2. Hardware

Conecte os periféricos à Raspberry conforme as imagens:

<img width="1172" height="652" alt="image" src="https://github.com/user-attachments/assets/7b779344-d682-46d3-a5da-0d9b4a5eca8b" />

---

## 2. Configuração das câmeras USB

O sistema utiliza duas câmeras USB para visão estéreo. Para evitar que a numeração (`/dev/video0`, `/dev/video1`, etc.) mude a cada reinicialização, recomenda-se criar nomes fixos para cada câmera.

### Verificar se as câmeras foram detectadas

```bash
v4l2-ctl --list-devices
```

Exemplo:

```
USB 2.0 Camera: USB Camera (usb-xhci-hcd.0-2):
    /dev/video0
    /dev/video1

USB 2.0 Camera: USB Camera (usb-xhci-hcd.1-2):
    /dev/video2
    /dev/video3
```

### Descobrir o identificador de cada câmera

Execute:

```bash
udevadm info --attribute-walk --name=/dev/video0
```

e

```bash
udevadm info --attribute-walk --name=/dev/video2
```

Procure o atributo `KERNELS`, por exemplo:

```
KERNELS=="1-2:1.0"
```

e

```
KERNELS=="2-2:1.0"
```

### Criar regras do udev

Crie o arquivo:

```bash
sudo nano /etc/udev/rules.d/99-cameras.rules
```

Adicione regras semelhantes às abaixo (substituindo os valores de `KERNELS` pelos encontrados na etapa anterior):

```text
SUBSYSTEM=="video4linux", KERNELS=="1-2:1.0", ATTR{index}=="0", SYMLINK+="camera_esquerda"
SUBSYSTEM=="video4linux", KERNELS=="2-2:1.0", ATTR{index}=="0", SYMLINK+="camera_direita"
```

Recarregue as regras:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Verifique se os links foram criados:

```bash
ls -l /dev/camera_*
```

Resultado esperado:

```
/dev/camera_esquerda -> video0
/dev/camera_direita -> video2
```

O software utiliza esses nomes fixos, portanto a ordem de enumeração dos dispositivos `/dev/video*` deixa de ser relevante.

---

## 3. Configuração do ambiente

Dentro do repositório:

Atualize os pacotes necessários:

```bash
sudo apt update
sudo apt install python3-venv -y
```

Crie e ative o ambiente virtual:

```bash
cd Raspberry/gd70_control/
python3 -m venv env
source env/bin/activate
```

Instale as dependências:

```bash
pip install -r requirements.txt
```

Caso ocorra algum erro relacionado a **externally managed environment**, siga as instruções apresentadas pelo próprio `pip`.

---

## 4. Calibração

Calibre as câmeras para visão estéreo conforme as referências listadas ao final deste README.

```bash
python3 Calibration/calib.py
```

---

## 5. Execução

No diretório `gd70_control`:

```bash
python3 main.py
```

# Referências

* O repositório **stereo_vision (OmidAlekasir)** fornece código de exemplo para testar configurações de visão estéreo, técnica utilizada para estimativa de profundidade. O projeto enfatiza que uma calibração de alta qualidade é necessária para corrigir as distorções das lentes e alinhar corretamente as câmeras.

* O repositório **python_stereo_camera_calibrate (TemugeB)** fornece um script de calibração em Python para obtenção dos parâmetros estereoscópicos através de triangulação, capturando pares de imagens para calcular as matrizes de rotação e translação entre as câmeras.
