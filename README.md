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
| Comprimento do cabo L | **5,0 m** (0,5–6 m ajustável via `L<val>` ou `t`) | `L_M`, `RopeParams.length` | calibrável |
| Torção do cabo | **0** (heading livre, por gyro) | `RopeParams` | conforme hardware |
| Força por hélice @ ±500 µs | **18 N** (curva quadrática, ré 60%, dz 30 µs) | `ActuatorParams` / `main.cpp` | simétrico de fábrica |
| Faixa de PWM segura | **1500 ± 450 µs** (1050 a 1950 µs, reserva swing 400 µs, yaw 300 µs) | `PWM_LIMIT_US` / `SWING_RESERVE_US` | calibrado |
| Travamento direcional do ESC (latch) | **~±60 µs** (PWM ~1440–1560): sub-piso vai a neutro com rampa suave | `ESC_LATCH_US` / `ESC_MIN_YAW_US` | atenuado |
| MPU6050 | topo da esfera, **DLPF 20 Hz de hardware ativado** | `firmware/main.cpp` | I2C Fast 400 kHz |

**Autoridade resultante**: envelope de força ≈ 15,5 N (13% do peso) e torque de heading ≈ 25 N·m.
O firmware consolidado opera com clamp de segurança em ±450 µs, garantindo resposta rápida sem risco térmico aos ESCs F45A.

## Interface de hardware (motores)

- Motores **a**, **b**, **c** = BF (PA0), AS (PA15), TR (PA1):
  - **Motor a (BF, PA0)**: empurra no **+X do corpo** ($0^\circ$), `yaw_sign = +1.0`.
  - **Motor b (AS, PA15)**: empurra em **+120°** (trás-esquerda), `yaw_sign = +1.0`.
  - **Motor c (TR, PA1)**: montado a **-120°** (trás-direita), **invertido de fábrica** (hélice/ESC CW): opera com `yaw_sign = -1.0` e vetor tangencial compensado `(+0.5, +0.866)`.
- PWM bidirecional: **1500 µs = parado, 1950/1050 µs = full speed ±**.
- Curva quadrática com zona morta e modelo simétrico de fábrica, desacoplando Yaw e Anti-Balanço desde o boot.

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
Raspberry/        app da Rasp: visão estéreo + linha + link STM + ponte TCP
tests/            48 testes (física, atuadores, malha fechada, twin, wizard, Rasp)
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

Teclas do interativo: setas = força externa · `,`/`.` = torque · `p`/`y` =
liga/desliga anti-balanço/heading · `[`/`]` = ψ desejado ∓15° · G, +/− = ganhos
ao vivo · R/espaço/Esc = reset/pausa/sair. **Tabela única de comandos e teclas
(firmware, Rasp, rádio, supervisório, twin): [docs/COMANDOS.md](docs/COMANDOS.md).**

## Digital Twin

```bash
python -m twin --sim                # contra o STM SIMULADO (testar sem hardware)
python -m twin --serial COM7        # STM plugado no USB do PC
python -m twin --tcp 192.168.0.50   # via ponte TCP embutida no app da Rasp
```

A janela do twin adiciona à simulação: **esfera fantasma** (estado real:
inclinação, heading, empuxos dos PWMs medidos), **linha de transmissão a
distância D** abaixo da esfera (quando a visão enviar), **curvas real vs
simulada** (inclinação, velocidade, heading, PWM) e HUD do lado real com
conexão e estado dos loops. Os comandos vão aos dois lados: `p`/`y` (loops),
`[`/`]` (heading), `k` (parada geral), **`z` (calibração da MPU — esfera parada!)**
e +/− (ganhos do controlador Python; os índices 3–7 não valem para o firmware —
ajuste os ganhos do hardware pelo supervisório, ver docs/COMANDOS.md §3).

### Arquitetura

```
STM32 (firmware/main.cpp)
  ├── USB do PC ───────────────────── PC (twin --serial / wizard --serial)
  └── USB da Rasp → app de visão (Raspberry/main.py) ── dono da serial
                       ├── visão estéreo + detecção da linha + HUD local
                       │   (reset físico GPIO18, rádio RC CH12–15)
                       └── ponte TCP embutida (Raspberry/bridge.py, 9999)
                              → PC (twin --tcp / wizard --tcp)
```

Um processo na Rasp é o único dono da serial do STM; twin e wizard
conectam pela ponte TCP (IP direto ou túnel `ssh -L 9999:localhost:9999`).

### Deploy na Rasp

```bash
git clone https://github.com/iNeewbie/esfera.git && cd esfera

# ambiente virtual (o Raspberry Pi OS novo bloqueia pip global) — uma única vez
python3 -m venv .venv --system-site-packages   # enxerga os backends de GPIO do sistema
source .venv/bin/activate
pip install -r Raspberry/requirements.txt      # opencv-CONTRIB (WLS), pyserial, gpiozero…

python Raspberry/main.py                       # a partir da raiz do repo, com o venv ativo
```

Em terminais novos: `source .venv/bin/activate` antes de rodar (ou chame
`~/esfera/.venv/bin/python` direto).

### Checklist de validação na Rasp (após o flash do firmware v2)

1. Sem STM: visão e HUD como antes (mono/estéreo, teclas de escala/SGBM).
2. Com STM v2: HUD alimentado por `TEL`; twin no PC (`--tcp <ip>`) vê o
   fantasma e a linha a distância D.
3. Tecla `v`: envio de V ativo → inclinar o alvo e conferir o SENTIDO do
   bearing (se a esfera girar ao contrário, inverter o sinal no envio —
   a cadeia visão→yaw nunca fechou no firmware antigo, sinal é validação
   de bancada).
4. D vs fita métrica; rádio RC ainda comanda pinça/polia; tecla `p` =
   parada geral de verdade (agora envia `k`).

### Contrato da visão (IMPLEMENTADO em Raspberry/)

A cada frame com linha detectada (tecla `v` liga/desliga o envio), a Rasp
envia ao STM: `V<bearing_graus> <D_m>`
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

**Novo fluxo com parada manual entre motores:**
O wizard guia o operador no terminal para zerar a velocidade da esfera antes de cada motor:
- Exibe `Zere a velocidade...` e monitora em tempo real a quietude (`|rate_z| <= 1.0` °/s e `|w_xy| <= 1.5` °/s por 15 frames seguidos);
- Confirma com `[OK] Velocidade zerada! Solte a esfera.`;
- Realiza contagem regressiva de 3 s mantendo o buffer de telemetria drenado (`flush_input()`);
- Dispara os pulsos em ±100/170/240 µs (600 ms cada) sincronizados pelo bit `FLAG_WIZARD = 16`;
- Aguarda ativamente o repouso do pêndulo entre pulsos (`wait_for_quiet()`) antes do próximo passo.
Saída: `calib.json` saneado e push opcional à RAM do STM.

## Firmware Consolidado (firmware/main.cpp)

> **Revisão de 14/10/2026 (sensoriamento e controle)** — detalhes e justificativas em
> [AUDITORIA_CONTROLE.md](AUDITORIA_CONTROLE.md); comandos em [docs/COMANDOS.md](docs/COMANDOS.md).
> - O anti-balanço tinha **um eixo com realimentação positiva** (gyro espelhado, SC-01):
>   corrigido para a rotação correta; a montagem da MPU fica em `IMU_MOUNT_DEG`/`IMU_Z_UP` e
>   **precisa do teste de eixos da Etapa 3 do BRINGUP antes de ligar o `p`**.
> - Heading controlado em **torque** (N·m) e alocação conjunta força+torque (sem empurrão
>   lateral nas manobras de yaw); ganhos G3–G7 mudaram de unidade.
> - Calibração do gyro de ~9 s com gate real (`z` recusa movimento, `Z` força); malhas não
>   ligam sem calibração; falha da MPU desliga tudo; visão por evento com latência.
> - Estimador opcional da velocidade do CG (`G8 1`) — testar na bancada.
> - `firmware/host_sim/`: o firmware real roda no PC em malha fechada contra um pêndulo
>   simulado (`tests/test_firmware_host.py`, precisa de g++/clang).
>
> O texto abaixo descreve a versão anterior onde não foi atualizado; em conflito, valem os
> dois documentos acima.

Unifica a robustez comprovada do `mainYAWBOM` com o protocolo do Digital Twin:
1. **Filtragem de Ruído Reforçada:**
   - DLPF de hardware no MPU6050 (20 Hz, reg `0x1A = 0x04`): elimina ruídos estruturais das hélices diretamente no conversor AD do sensor;
   - Filtro passa-baixas IIR (~9 Hz) em `wx_est` e `wy_est` (`ALPHA_RATE_XY = 0.22`): elimina >75% do ruído derivativo antes dos ganhos de controle.
2. **Malha de Yaw Estável:**
   - Erro angular calculado em graus com `wrap180` (corrige o bug do `wrap_pi` em graus);
   - PID direto de posição (Kp=5.0, Ki=1.0, Kd=10.0) com termo derivativo em `-rate_z_filt` (sem trancos em degraus de setpoint);
   - Zona morta angular de 2.5° com histerese (3.2°) e **reset imediato do integrador no cruzamento de zero** (elimina overshoot e ciclos-limite).
3. **Anti-Balanço por Velocidade com Zona Morta de Prumo:**
   - Freio viscoso ativo no mundo ($v̂ = L \cdot \omega_{\text{mundo}}$);
   - Zona morta de repouso no prumo (< 0.60° e < 1.50 °/s): motores desligam em neutro (1500 µs) ao atingir estabilização.
4. **Segurança e Mecanismos:**
   - Watchdog de 250 ms na polia (Deadman Switch): desliga se o link cair;
   - Cancelamento imediato do pulso do wizard no comando de emergência `k`.

Comandos (USB e Serial2) — tabela completa em [docs/COMANDOS.md](docs/COMANDOS.md):
`w/s/x` pinça · `a/d/c` polia · `e` AS5600 · `m` teste de motores · `y` heading on/off ·
`p` anti-balanço on/off · `k` **parada geral** · `z` calibra MPU (`Z` força) · `t` L pelo
período · `0` zera yaw desejado · `L<val>` · `<num>` degrau de yaw · `G<idx> <val>` ·
`R` · `V<b> <D> [idade_ms]` · `W<m> <us> <ms>` · `M<m> ...`.
Telemetria 25 Hz (v3): `TEL,t_ms,yaw,bearing,yaw_des,rate_z,sx,sy,wx,wy,pwm_a,b,c,L,D,flags,F_cmd_x,F_cmd_y,Tz,vhat_x,vhat_y`.

Loops de controle **desligados no boot** — 'y'/'p' ligam.

## Checklist de bancada (Bring-up)

- [x] **Etapa 0 — Ensaio no simulador**: paridade validada.
- [x] **Etapa 1 — Gravação do Firmware**: PlatformIO configurado.
- [x] **Etapa 2 — Comunicação e Telemetria**: TEL a 25 Hz sem `%f`.
- [x] **Etapa 3 — Sensores**: MPU com DLPF 20 Hz e calibração por `z`.
- [x] **Etapa 4 — Raspberry Pi**: ponte TCP (porta 9999), VNC ajustado (10 FPS).
- [x] **Etapa 5 — Identificação de Motores (`m`)**: confirmada rotação, Motor A (+X), Motor B (+120°), Motor C (-120°, invertido de fábrica).
- [x] **Etapa 6 — Wizard de Atuadores**: parada manual entre motores executada, calibração salva em `calib.json`.
- [ ] **Refazer Etapa 3 (teste de eixos) com o firmware de 14/10/2026** — obrigatório antes da Etapa 7.
- [ ] **Etapa 7 — Primeira Malha Fechada (Próximo Passo)**:
  1. Ligar anti-balanço (`p`) e empurrar a esfera: confirmar amortecimento rápido em **todas** as direções;
  2. Ligar heading (`y`): aplicar degraus de 15° (`15\n` ou tecla `]`) e observar acomodação suave.

## Comportamento esperado (simulador, defaults provisórios, faixa completa do ESC)

| Métrica | Valor |
|---|---|
| Acomodação de velocidade (15° + 0,3 m/s) | ~1,7 s |
| Perturbação forte (20°/−10° + 0,5/−0,4 m/s) | ~2,7 s |
| Rajada de 10 N por 2 s | inclina ~5°, recupera em ~17 s |
| Acomodação de degrau de heading | ~1,75–2,5 s, sobressinal zero (cenário heading; 1,2° no default) |
| Desvio de energia em oscilação livre (20°, 20 s) | < 0,5% (sem drift) |

## Pré-requisitos e instalação

```bash
python -m venv .venv && .venv\Scripts\activate   # Windows
pip install -r requirements.txt
```

Python 3.10+. Para o firmware: Arduino CLI/IDE com suporte STM32
(STM32Duino/Arduino_Core_STM32), placa Black Pill F401/F411.
