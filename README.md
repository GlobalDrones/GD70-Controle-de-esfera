# Esfera

## Descrição

Modelo-simulação física de um sistema "pêndulo simples" cujo controle está na
ponta do pêndulo, no centro de massa da esfera. O controle é feito em 3 graus
de liberdade com 3 motores bidirecionais + hélices tangentes ao raio da
esfera, fixos à esfera:

- **Heading (ψ):** orientação da esfera em torno do eixo vertical, rastreando
  uma direção desejada — no campo, o **cabo da linha de transmissão** visto
  pelas câmeras (que olham para baixo, fixas ao sensor);
- **X e Y (translação):** controle de velocidade com setpoint zero
  (amortecimento ativo) — a esfera pende próxima do vertical em repouso.

A esfera será posicionada sobre linhas de transmissão: estabiliza o heading
alinhado à linha, amortece o balanço, desce pela polia (L entre sessões) e
pega a linha com a pinça. A visão estéreo (na Rasp) fornece o bearing do cabo
e a distância D esfera↔cabo.

**Três componentes em paridade total** (mesmo modelo de atuador, mesmos
ganhos, mesmo protocolo): o simulador Python, o firmware do STM32 e o
**digital twin** — a simulação rodando ao lado do hardware real, com a
esfera real sobreposta na mesma tela.

## Parâmetros do hardware

| Parâmetro | Valor | Onde | Status |
|---|---|---|---|
| Massa total do corpo móvel | **12 kg** | `SphereParams.mass` | medido |
| Raio da esfera | **0,60 m** | `SphereParams.radius` | conforme hardware |
| Braço eixo→propulsores | **0,70 m** | `ActuatorParams.arm_radius` | conforme hardware |
| Momento de inércia em ψ | (2/5)·m·r² = **1,728 kg·m²** | `SphereParams.inertia_z` | estimativa (`--inertia` medido rescala ganhos) |
| Comprimento do cabo L | 1 m (0,5–6 m, **só entre sessões**) | `RopeParams.length`, `--length` | medido |
| Torção do cabo | **0** (heading livre, por gyro) | `RopeParams` | conforme hardware |
| Força por hélice @ ±500 µs | **18 N** (curva quadrática, ré 60%, dz 30 µs) | `ActuatorParams` | **PROVISÓRIO — wizard calibra por motor** |
| Faixa de PWM | **1500 ± 500 µs (faixa completa do ESC)** | `ActuatorParams.pwm_limit_us` / `PWM_LIMIT_US` | decisão da equipe — **verificar corrente/térmica dos F45A na 1ª sessão** |
| Travamento direcional do ESC (latch) | **~±60 µs** (PWM ~1440–1560): comando pequeno trava o sentido, só comando válido no oposto destrava | `MotorCalib.latch_us` (Python) / `ESC_LATCH_US` (firmware) | observação de bancada — **wizard medirá por ESC** |
| MPU6050 | topo da esfera, junto ao pino | firmware | inclinômetro quase ideal |

**Autoridade resultante** (com os defaults provisórios): envelope de força
≈ 15,5 N (13% do peso) e torque de heading ≈ 25 N·m. A faixa completa foi
liberada em substituição ao clamp conservador de ±250 µs (que deixava o
envelope em 3,2 N — acomodação de perturbações demorava 15+ s; agora ~2,6 s).

## Interface de hardware (motores)

- Motores **a**, **b**, **c** = BF(PA0), AS(PA15), TR(PA1); o motor **a
  empurra no +X do sensor** (verificado fisicamente), b/c a ±120°.
- PWM bidirecional: **1500 µs = parado, 2000/1000 µs = full speed ±**.
- Curva quadrática por motor com zona morta e eficiência de ré própria —
  **nada assume que os 3 motores foram soldados/montados iguais**: o wizard
  mede cada um e o firmware guarda calibração por motor em RAM (comando M).

## Estrutura

```
simulation.py     plantas (não-linear RATTLE + linearizada) e parâmetros
actuators.py      modelo POR MOTOR (curva, sinais, calibração JSON) + lag
pid_controller.py PID com anti-windup, limite de saída e wrap angular
controller.py     cascata heading (perfil de frenagem, µs) + PID velocidade
scenarios.py      cenários (condição inicial, heading, rajadas de força)
main.py           CLI, loop de simulação, métricas, comparação de plantas
interactive.py    modo interativo ao vivo (teclado, HUD, painel PWM, ganhos)
animation.py      animação 3D + painel de séries temporais (batch)
firmware/main.cpp firmware STM32 v2 (controle em paridade com o twin)
twin/             digital twin: protocolo, transportes, STM simulado, janela
wizard.py         identificação por motor (pulsos → modelo calibrado)
rasp/agent.py     agente da Rasp: Serial2 do STM ↔ TCP
tests/            38 testes (física, atuadores, malha fechada, twin, wizard)
```

## Simulador e modo interativo

```bash
python main.py                          # cenário padrão, animação 3D
python main.py --interactive            # simulação ao vivo pelo teclado
python main.py --scenario gust          # rajadas de força externa
python main.py --compare                # linear vs não-linear
python main.py --calib calib.json       # usar calibração medida do wizard
python main.py --length 2.5 --inertia 1.4   # parâmetros da sessão
python -m pytest tests/ -q              # testes
```

Teclas do interativo: setas = força externa · Q/E = torque · 1/2 =
liga/desliga XY/heading · Z/X = ψ desejado · G, +/− = ganhos ao vivo ·
R/P/Esc = reset/pausa/sair.

## Digital Twin

```bash
python -m twin --sim                # contra o STM SIMULADO (testar sem hardware)
python -m twin --serial COM7        # STM plugado no USB do PC
python -m twin --tcp 192.168.0.50   # via agente na Rasp (porta 9999)
```

A janela do twin adiciona à simulação: **esfera fantasma** (estado real:
inclinação, heading, empuxos dos PWMs medidos), **linha de transmissão a
distância D** abaixo da esfera (quando a visão enviar), **curvas real vs
simulada** (inclinação, velocidade, heading, PWM) e HUD do lado real com
conexão e estado dos loops. Os comandos vão aos dois lados: 1/2 (loops),
Z/X (heading), K (parada geral) e +/− (ganhos, com push ao STM via G).

### Arquitetura

```
STM32 (firmware/main.cpp)
  ├── USB ──────────────── PC (twin --serial)
  └── Serial2 → Rasp (rasp/agent.py) → TCP 9999 → PC (twin --tcp)
                                              ↘ wizard --tcp
```

Sem decidir ainda a rede? O transporte é plugável: `--serial` funciona em
bancada sem rede nenhuma; `--tcp` quando o PC alcança a Rasp por IP (ou via
túnel `ssh -L 9999:localhost:9999 pi@<rasp>`).

### Agente na Rasp (deploy)

```bash
scp -r rasp/ pi@<rasp>:esfera/
ssh pi@<rasp> 'sudo cp esfera/esfera-agent.service /etc/systemd/system/ &&
              sudo systemctl enable --now esfera-agent'
```

(requer `pip3 install pyserial` na Rasp; ver cabeçalho de `rasp/agent.py`)

### Contrato da visão (quando as câmeras existirem)

A Rasp envia ao STM (e o STM aceita pela Serial2): `V<bearing_graus> <D_m>`
— bearing do cabo da linha **no referencial da câmera/corpo** (câmeras fixas
ao gyro: o bearing É o erro de apontamento, imune à deriva do gyro) e a
distância estereoscópica ao cabo. O STM usa o bearing como erro direto
enquanto fresco (< 500 ms) e o D vai à telemetria.

## Wizard de identificação por motor

```bash
python wizard.py --sim            # valida contra o STM simulado
python wizard.py --serial COM7    # bancada: esfera pendurada e LIVRE
python wizard.py --tcp <rasp> --push
```

**Mitigação do travamento do ESC já ativa** (paridade Python/firmware): o
menor µs não-nulo comandado nasce FORA da faixa de travamento (latch + 15 µs
de margem) — comandos sub-piso viram neutro, tanto no caminho de força (gate
por motor) quanto no µs final (quantização anti-banda de swing+yaw). O
caminho do wizard não quantiza de propósito: o modelo do ESC fica exposto
para ser medido (rotina de caracterização do latch no wizard: planejada).

Pulsa cada motor em ±100/170/240 µs (600 ms, loops desligados), mede a
resposta pela própria telemetria e devolve **por motor**: direção de empuxo
(com transporte por amostra para o corpo — o pulso gira a esfera), sinal de
yaw, curva µs→N frente/ré e zona morta (com correção analítica do atraso do
atuador e subtração do sinusóide de oscilação residual). Saída: `calib.json`
(mesmo formato do simulador) e push opcional ao STM.

## Firmware (firmware/main.cpp)

Correções vs. firmware anterior: (1) `pos_error` nunca era calculado — PID de
yaw morto; (2) força anti-balanço sem rotação mundo→corpo; (3) taxas no corpo
vs ângulos em mundo; (4) deadzone na integração do yaw; (5) sem gate
gravimétrico no acc; (6) calibração sem verificação de quietude; (7) atuador
global → por motor; (8) I2C 100→400 kHz; (9) parser duplicado → unificado.

Comandos (USB e Serial2): `w/s/x` pinça · `a/d/c` polia · `e` AS5600 ·
`m` teste de motores · `y` yaw on/off · `p|b` anti-balanço on/off · `k`
**parada geral** · `z` calibra MPU (com gate de quietude) · `t` calibra L
pelo período · `0` zera yaw desejado · `L<val>` comprimento · `<num>` degrau
relativo de yaw · `G<idx> <val>` ganho · `R` lê ganhos · `V<b> <D>` visão ·
`W<m> <us> <ms>` pulso do wizard · `M<m> ...` calibração por motor.
Telemetria 25 Hz: `TEL,t,yaw,bearing,yaw_des,rate_z,sx,sy,wx,wy,pwm_a,b,c,L,D,flags`.

Loops de controle **desligados no boot** — 'y'/'p' ligam.

## Checklist de bancada (primeira sessão com o twin)

1. **Flash + smoke test**: boot com loops OFF; 'm' (teste de motores) —
   confirmar a↔PA0/b↔PA15/c↔PA1 e os sentidos.
2. **Eixos do MPU**: inclinar a esfera em direção conhecida → sinais de
   sx/sy na telemetria (TEL); girar a esfera inclinada à mão → sx/sy NÃO
   podem girar junto (senão, inverter `COMP_YAW_SIGN` no firmware).
3. **ESCs**: modo bidirecional 1500 neutro confirmado (calibração de faixa
   já feita); testar refresh > 50 Hz no futuro (reduz o atraso).
4. **Wizard** (`python wizard.py --serial COM7 --push`): substitui TODOS os
   chutes de atuador; conferir o relatório contra o esperado (~18 N/hélice).
5. **'z'** (calibração com quietude) e **'t'** (período→L, cruzar com o L da
   sessão).
6. **Telemetria + fantasma**: anti-balanço OFF, empurrar a esfera — o
   fantasma do twin deve rastrear (valida fusão + telemetria).
7. **Anti-balanço ON** com ganhos conservadores (defaults): confirmar a
   DIREÇÃO do amortecimento; subir ganhos ao vivo pelo twin.
8. **Yaw**: degraus ±15° (referência interna); com a visão, bearing da linha.
9. **Saturação**: comando grande → PWM nos clamps 1250/1750, sem chatter.

## Comportamento esperado (simulador, defaults provisórios, faixa completa do ESC)

| Métrica | Valor |
|---|---|
| Acomodação de velocidade (15° + 0,3 m/s) | ~1,7 s |
| Perturbação forte (20°/−10° + 0,5/−0,4 m/s) | ~2,7 s |
| Rajada de 10 N por 2 s | inclina ~5°, recupera em ~17 s |
| Acomodação de degrau de heading | ~1,8–2,7 s, sobressinal ≤ 1,2° |
| Desvio de energia em oscilação livre (20°, 20 s) | < 0,5% (sem drift) |

## Pré-requisitos e instalação

```bash
python -m venv .venv && .venv\Scripts\activate   # Windows
pip install -r requirements.txt
```

Python 3.10+. Para o firmware: Arduino CLI/IDE com suporte STM32
(STM32Duino/Arduino_Core_STM32), placa Black Pill F401/F411.
