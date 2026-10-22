# Auditoria de código — Esfera

**Data:** 13/10/2026 · **Base:** commit `3095600` + `Raspberry/video_streamer.py` modificado (não commitado)
**Escopo:** repositório inteiro — firmware STM32, app da Raspberry (visão, serial, rádio, calibração), ponte TCP/HTTP, digital twin, wizard, simulador Python e testes.
**Método:** revisão estática linha a linha. Cada achado foi relido no código antes de entrar aqui; os marcados com ✔ foram reconferidos uma segunda vez de forma independente, e alguns foram reproduzidos com script (indicado). Itens que dependem do hardware estão marcados **(bancada)**: o código indica o problema, mas o efeito real precisa ser confirmado na esfera.

**Severidade**

| Nível | Significado |
|---|---|
| 🔴 CRÍTICO | Comportamento errado ou perigoso no uso normal; corrigir antes da próxima sessão com motores |
| 🟠 ALTO | Falha provável em campo, perda de controle/parada, ou bug que invalida uma funcionalidade |
| 🟡 MÉDIO | Falha em condição específica, divergência sim↔hardware, robustez |
| ⚪ BAIXO | Qualidade, manutenção, comentários, performance menor |

**Testes:** `tests/test_plant.py` + `tests/test_twin.py` → **44 passed** (32,6 s, venv à parte com pytest/matplotlib, que não estão no `.venv` do repo). `tests/test_rasp.py` **não foi executado de propósito**: importar `serial_communication` reseta a STM32 e abre a serial/portas reais (ver INF-01), e o app estava rodando na Pi durante a auditoria.

---

## Sumário

| Área | 🔴 | 🟠 | 🟡 | ⚪ |
|---|---|---|---|---|
| Firmware (`firmware/main.cpp`) | 3 | 7 | 10 | 11 |
| Raspberry — app de visão, serial, rádio | 3 | 11 | 9 | 10 |
| Raspberry — `video_streamer.py` | — | 2 | 3 | 5 |
| Ponte, twin e wizard | 1 | 4 | 8 | 5 |
| Simulador / paridade com firmware | — | 2 | 4 | 5 |
| Testes e infraestrutura | — | 1 | 1 | 3 |

### Os 12 que eu corrigiria primeiro

| # | ID | Resumo | Esforço |
|---|---|---|---|
| 1 | FW-01 | `W` maiúsculo na whitelist: o pulso do wizard **sobe a pinça** em vez de pulsar o motor | 1 linha |
| 2 | FW-02 | `wrap_pi` aplicado a erro em **graus**: o yaw sem visão nunca chega no setpoint e pode travar o loop | 1 linha |
| 3 | FW-03 | Anti-balanço gira a força para o referencial errado: com ψ=180° o balanço **aumenta** | pequeno (bancada) |
| 4 | RP-01 | Modo estéreo **nunca** detecta a linha com os parâmetros padrão (Z mínimo medível > 11 m, filtro é 3 m) | médio |
| 5 | TW-01 | Twin manda o ganho no índice errado (`+/-` em `head_rate_kp_us` grava no `head_rate_limit`) | 1 linha |
| 6 | FW-04 / FW-05 | `k` não interrompe o pulso do wizard; MPU falhando → controle segue com dado velho | pequeno |
| 7 | RP-04 / RP-07 | Câmera caída trava a janela (tecla `p` morta); app cai sem mandar `k` | pequeno |
| 8 | VS-01 / VS-02 | Auto-reinício (`os.execv`) **reseta a STM32** no meio da operação e pode derrubar a outra câmera | pequeno |
| 9 | RP-08 | Rádio: perda de sinal com A/D apertado deixa a polia girando; pinça nunca recebe `x` | pequeno |
| 10 | BR-01 | Ponte em 0.0.0.0 sem autenticação + `GET /api/cmd` com CORS `*`: qualquer página na LAN comanda os motores | médio |
| 11 | FW-07 / FW-06 | Sem watchdog (IWDG) e sem timeout de comunicação com a Rasp | pequeno |
| 12 | RP-02 | `calibration_settings.yaml` inválido (`=` em vez de `:`), `calib.py` não roda | 1 linha |

**Padrão por trás de vários bugs:** o simulador do twin (`twin/simstm.py`) processa a linha inteira, usa velocidade/heading verdadeiros de mundo e não reproduz o parser de 1 caractere do firmware. Por isso FW-01, FW-03, FW-12 e FW-13 **passam no twin e falham no hardware**. Um teste que exercite o parser e as constantes reais de `main.cpp` (INF-03) teria pego todos.

---

## 1. Firmware — `firmware/main.cpp`

### 🔴 FW-01 — `W` do wizard executa "subir pinça" ✔
**Onde:** `main.cpp:884` (`WHITELIST`), `:898`, `:721-723`
A whitelist de comandos imediatos inclui `W` maiúsculo (e `0`). Com o buffer vazio, o primeiro caractere de `W0 240 600\n` executa `comando_char('W')` → `moverSubir()`; o `0` seguinte zera `desired_yaw`; `" 240 600"` começa com espaço e é descartado.
**Cenário:** o wizard roda 18 pulsos, **nenhum motor pulsa**, a pinça sobe a cada pulso e o wizard grava calibração a partir de ruído (ver WZ-01). O SimSTM aceita `W` como linha, então `--sim` passa.
**Correção:** só tratar como imediato um caractere que venha **sozinho na linha** (ou trocar o comando do wizard para uma letra fora da whitelist nos três lados). Isso resolve também FW-12.

### 🔴 FW-02 — `wrap_pi` aplicado a erro em graus ✔ (reproduzido)
**Onde:** `main.cpp:551` (`wrap_pi` em `:243`)
`wrap_pi(desired_yaw - yaw_gyro) * (PI/180)` faz o wrap em ±π, mas o argumento está em graus. Um degrau de +10° vira erro de **−2,57°** (sinal invertido), 90° vira 2,04°, 180° vira −2,21°.
**Cenário 1:** sem visão, o yaw nunca chega ao setpoint e pode girar para o lado errado.
**Cenário 2:** um setpoint absurdo (`1e9`, `inf` via serial ou ponte) faz o `while` nunca terminar, porque em float `a - 2π == a`. O loop trava e os ESCs ficam no último PWM, já que o Servo é gerado por timer.
**Correção:** `wrap180(desired_yaw - yaw_gyro) * (PI/180)`, com `wrap180` reescrito com `fmodf`, e rejeitar setpoint que não seja finito ou com |x| > 360.

### 🔴 FW-03 — Anti-balanço: força girada para o referencial errado **(bancada)**
**Onde:** `main.cpp:407-437` (fusão), `:468-471`, `:498-502` (`realizar_controle`), `:1036`
`sx_est/wx_est` vêm de gyro e acelerômetro presos ao corpo, e o termo de transporte (`rate_x + wz·sy`) é a cinemática da inclinação **expressa no corpo**. Mesmo assim, `realizar_controle` aplica Rz(−ψ) com `ψ = yaw_gyro`, como se a velocidade estivesse no mundo.
**Cenário:** depois de um comando de yaw de 90°, a força fica perpendicular à velocidade e não amortece nada. Com 180°, empurra no sentido da velocidade e o balanço cresce. O twin não mostra isso porque usa θ̇ de mundo e o heading verdadeiro (`controller.py:183`, `simstm.py:101-104`).
**Correção:** passar ψ=0 para o canal XY (a força já está no corpo), **ou** girar `wx/wy` para o mundo com +ψ antes do PID. Depois, modelar a medida no corpo no simulador. Validar em bancada com yaw a 0°, 90° e 180°.

### 🟠 FW-04 — `k` (parada geral) não interrompe o pulso do wizard
**Onde:** `main.cpp:707-715` vs `:1021-1032`
`parada_geral()` não zera `wizard_motor` nem `wizard_remaining_ms`. No tick seguinte o motor volta para `wizard_us` por até 3 s.
**Correção:** `wizard_motor = -1; wizard_remaining_ms = 0;` dentro de `parada_geral()`.

### 🟠 FW-05 — MPU falhando: o controle continua com dados velhos
**Onde:** `main.cpp:1016-1037`
Com `s.ok == false` (I2C sem resposta), a fusão é pulada, mas `passo_controle_xy/yaw` continuam com os últimos `wx_est`/`rate_z_filt`, e o integrador acumula sobre um erro constante. Não há flag na telemetria.
**Cenário:** um mau contato na MPU com as malhas ligadas gera empuxo constante indefinidamente.
**Correção:** contar falhas seguidas; depois de cerca de 10 (50 ms), desligar as malhas, ESCs em neutro e pôr uma flag na TEL.

### 🟠 FW-06 — Sem timeout de comunicação com a Rasp
**Onde:** arquivo todo
Se a Rasp cair (crash, RP-07, VS-01, cabo) com as malhas ligadas, o firmware continua atuando e o operador perde o `k`.
**Correção:** heartbeat (as próprias linhas `V`, ou um `H\n` a cada 200 ms). Sem nada por 1-2 s: malhas off e ESCs em neutro.

### 🟠 FW-07 — Sem watchdog de hardware (IWDG)
**Onde:** arquivo todo
Qualquer travamento (FW-02, FW-09, I2C preso) mantém os pulsos de PWM ativos, porque são gerados por timer.
**Correção:** IWDG de ~100-200 ms com refresh no `loop()`. O boot já sai em neutro, então o reset é seguro.

### 🟠 FW-08 — Teste `m` bloqueia ~13-18 s com motores girando
**Onde:** `main.cpp:645-681`, `:734`
São `delay()`s em série: sem leitura serial, sem `k`, sem telemetria. `m` é comando imediato de 1 byte, então um `m` perdido (ruído na Serial2, `m0...` digitado em vez de `M0...`, ou `GET /api/cmd?c=m`) dispara o teste.
**Correção:** máquina de estados não bloqueante (como o wizard) e comando de linha explícito.

### 🟠 FW-09 — `calibrar_mpu` pode entrar em loop infinito
**Onde:** `main.cpp:351`, `:749-750`
`if (!s.ok) { i--; continue; }` fica preso para sempre se a MPU sumir. Mesmo funcionando, `z` bloqueia ~1,6 s com os ESCs no último PWM, porque as malhas não são desligadas antes.
**Correção:** limite de tentativas, `parada_geral()` antes de calibrar e recusar `z` com as malhas ligadas.

### 🟠 FW-10 — Telemetria com `%f` pode sair vazia **(build)**
**Onde:** `main.cpp:916`
No STM32duino o padrão é newlib-nano **sem** float no printf. Sem `-u _printf_float` (ou "Newlib Nano + Float Printf"), os campos saem vazios, `protocol.parse_line` descarta toda TEL e o HUD e o twin ficam "aguardando telemetria". O repo não tem `platformio.ini` para garantir a flag.
**Correção:** documentar ou versionar a configuração de build, ou formatar com inteiros escalados.

### 🟡 FW-11 — Limite do PID de velocidade 5× maior que o do Python ✔
**Onde:** `main.cpp:289-291` vs `controller.py:108`
O firmware tem um fator `3.0` a mais e ignora `REV_RATIO`: **62 N** contra **12,4 N** no Python (o comentário "~4,3 N" está desatualizado).
**Cenário:** numa rajada, a força real satura em ~15 N, mas o anti-windup do firmware só atua acima de 62 N. A integral acumula e gera um sobressinal que o simulador não reproduz.
**Correção:** `0.8f*1.5f*fminf(kf_fwd,kf_rev)*(SWING_RESERVE_US−DB_US)²`, sem o `3.0`.

### 🟡 FW-12 — Setpoints de yaw entre 0 e 1 são corrompidos
**Onde:** `main.cpp:884/898` + `twin/protocol.py:136`
`0.500` → o `0` é imediato (zera `desired_yaw`) e `.500` é ignorado. A esfera volta ao zero antigo em vez de dar um passo de 0,5°. Resolvido junto com FW-01.

### 🟡 FW-13 — Linha com mais de 60 caracteres: o resto vira comando
**Onde:** `main.cpp:902`
O buffer é zerado, mas o resto da mesma linha continua sendo processado: um `k`/`p`/`y`/`m`/`w` vira comando imediato, ou um número vira setpoint com re-zero de yaw. Uma linha `M` com `.6g` chega a 61 caracteres (TW-13).
**Correção:** flag "descartando até `\n`".

### 🟡 FW-14 — Números recebidos sem validação (NaN/inf/faixa)
**Onde:** `main.cpp:787-790`, `:811-812`, `:870`
`toFloat()` aceita `nan`/`inf`:
- `V` com NaN: `fminf` ignora o NaN e `rate_sp` vai a +2,5 rad/s, girando direto enquanto a visão estiver "fresca".
- `G` com NaN: a saída do PID vira NaN e `lroundf(NaN)` gera PWM indefinido.
- Ganhos negativos são aceitos.
- Bearing fora de ±180 não é reduzido.

**Correção:** `isfinite()` e faixas plausíveis em V, G, M e setpoint.

### 🟡 FW-15 — Transição visão → gyro com salto de referência
**Onde:** `main.cpp:546-552`
Quando a visão envelhece (> 500 ms), o erro passa a ser `desired_yaw − yaw_gyro`, que não tem relação com o que a visão vinha seguindo. A linha sai do quadro e a esfera pode girar até 180° atrás de um setpoint antigo.
**Correção:** enquanto a visão estiver fresca, manter `desired_yaw = yaw_gyro + bearing`, ou desligar o yaw quando a visão cair.

### 🟡 FW-16 — `dt_swing` sem limite superior
**Onde:** `main.cpp:1013`
Depois de `m` (13 s) ou `z` (1,6 s), a primeira iteração integra yaw, inclinação e as integrais com dt de segundos. O `dt` com clamp (`:1000-1002`) existe mas não é usado.
**Correção:** `dt_swing = fminf(dt_swing, 0.02f)` ou resetar os PIDs depois de rotinas bloqueantes.

### 🟡 FW-17 — `pid_vy = pid_vx` copia o estado junto com os ganhos ✔
**Onde:** `main.cpp:294`, `:702`
Qualquer `G` (mesmo de heading) sobrescreve `integral`/`last_meas` do eixo Y com os do X. Com kd≠0, gera um pico na derivada.
**Correção:** copiar só kp, ki, kd e out_limit.

### 🟡 FW-18 — Malhas podem ser ligadas com a calibração falha
**Onde:** `main.cpp:735-747`
`y`/`p` não checam `calib_ok`. Se a esfera estava se mexendo no boot, `gyro_bias` fica 0 e o yaw deriva muito.
**Correção:** recusar a ativação com `calib_ok == false`.

### 🟡 FW-19 — Serial2 aceita os mesmos comandos imediatos
**Onde:** `main.cpp:1004-1005`
Ruído ou cabo solto em PA3 dispara `m`/`w`/`a`/`y`/`p`. O README diz que a Rasp está na USB.
**Correção:** desabilitar a Serial2 se não for usada, ou exigir linha completa lá.

### 🟡 FW-20 — Jitter do loop por TX bloqueante **(bancada)**
**Onde:** `main.cpp:927-928`
A TEL de ~110 B na Serial2 a 115200 com buffer de 64 B bloqueia ~4 ms a cada 40 ms; os `println` de debug também. O CDC USB pode bloquear se o host abrir a porta e não ler.
**Correção:** `availableForWrite()`, buffer maior ou enviar só na porta ativa.

### ⚪ Firmware — baixos
- **FW-21** (`:106-107`, `:505`) — reserva swing 490 + yaw 300 > 500. Na saturação, o clamp por motor distorce a direção e o torque (detalhado em SIM-03).
- **FW-22** (`:499-504`) — a quantização anti-latch zera |yaw_us| < 75. O PI acumula e salta, gerando ciclo-limite em erros pequenos (paridade com o Python, mas vale saber).
- **FW-23** — o wizard aceita |us| < 75 de propósito, o que pode deixar o ESC travado num sentido sem lógica de destravar. Reversões +500 → −500 sem passar pelo neutro também não são tratadas **(bancada)**.
- **FW-24** (`:852`) — o comando `M` não tem limite superior em `dz`/`kf`. Com dz > 490, o `cap` (`:492`) perde o sentido.
- **FW-25** (`:736`, `:742`) — `y`/`p` alternam (toggle). Reenviar "liga" desliga. Melhor ter comandos on/off explícitos (ver TW-05).
- **FW-26** (`:383-391`) — o rebias aceita |rate_z| < 0,3°/s como "parado", então absorve rotação lenta real.
- **FW-27** (`:169`) — `L` padrão de 1,0 m. Se o operador esquecer o `L`, o ganho do anti-balanço fica errado em até 6×.
- **FW-28** (`:971-972`, `:1076-1077`) — os fins de curso não são fail-safe (fio rompido equivale a "não bateu"). A polia não tem fim de curso nem timeout.
- **FW-29** (`:892-901`) — `String +=` em execução longa fragmenta a heap. Melhor um `char[]` fixo.
- **FW-30** (`:975`) — `encoder.begin()` depois de `Wire.setClock(400000)` pode resetar o clock, dependendo da versão da lib **(bancada)**.
- **FW-31** (`:1036`) — acelerômetro num pêndulo: a força específica fica alinhada ao cabo, então `sx/sy` tendem a ~0 durante o balanço (ALPHA = 0,88 → o acelerômetro domina). A TEL e o fantasma subestimam a inclinação **(bancada)**.

---

## 2. Raspberry — app de visão, serial, rádio, calibração

### 🔴 RP-01 — Modo estéreo nunca detecta a linha com os parâmetros padrão (reproduzido)
**Onde:** `camera_and_image_manipulation.py:78` (`Z <= 3.0`), `:203-204` (`num_disp=64`, `min_disp=0`), `configs.py:29-47`
Com as matrizes de `configs.py`, `stereoRectify` dá f·B ≈ 714 na escala 0,5. Com 64 disparidades, a disparidade máxima é 63 px, então o Z mínimo medível fica em **11,3 m** (escala 1,0 → 22,7 m; 0,25 → 5,7 m). O filtro exige Z ≤ 3 m, então `profundidades_validas` fica sempre vazia e o modo estéreo **nunca envia `V`**, gastando CPU em SGBM+WLS à toa.
**Correção:** derivar `min_disp`/`num_disp` da faixa de distância real, tirar o 3 m hardcoded e, principalmente, recalibrar (RP-12).

### 🔴 RP-02 — `calibration_settings.yaml` inválido ✔
**Onde:** `Raspberry/Calibration/calibration_settings.yaml:1-2`
`camera0 = "/dev/camera_esquerda"` usa `=` em vez de `:`, e `yaml.safe_load` levanta `ScannerError`. `calib.py` aborta antes de tudo, então hoje não dá para recalibrar.
**Correção:** `camera0: "/dev/camera_esquerda"` (e camera1).

### 🔴 RP-03 — Respostas `GAIN` causam exceção silenciosa ✔
**Onde:** `serial_communication.py:134-154`
`parse_line` devolve uma **tupla** para `GAIN,...`. Como a tupla não é `None`, entra no primeiro ramo e `frame.yaw_deg` gera `AttributeError`, engolido pelo `except` da thread (`:168`). O `bridge.broadcast(linha)` não executa e o `elif isinstance(frame, tuple)` é código morto.
**Cenário:** o wizard ou o twin mandam `R` e nunca recebem os ganhos.
**Correção:** `if isinstance(frame, protocol.TelemetryFrame):` primeiro, e logar exceções na thread.

### 🟠 RP-04 — Câmera caída trava a janela e o teclado (inclusive `p`)
**Onde:** `main.py:126-128` (estéreo) e `:83-85` (mono)
Com o frame `None`, o loop faz `continue` sem `cv2.waitKey`. Durante toda a reconexão (pode passar de 30 s, ver VS-04), a janela congela e **nenhuma tecla é lida**: some a parada de emergência pelo teclado.
**Correção:** chamar `waitKey(1)` e tratar `q`/`p` também nesse caminho, ou manter o último frame e mostrar "CÂMERA CAÍDA".

### 🟠 RP-05 — Frame velho reenviado como visão "fresca" **(bancada)**
**Onde:** `main.py:82`, `:124-125`, `:183-184`
O main não sabe se o frame é novo. Se o `read` do V4L2 ficar bloqueado (timeout de select de ~10 s no OpenCV), o main reprocessa o mesmo frame e manda `V` a cada volta, anulando o timeout de 500 ms do firmware.
**Correção:** `AsyncCamera.read()` devolver timestamp/contador; só enviar `V` com frame novo e com menos de ~200 ms.

### 🟠 RP-06 — Crash no modo mono ao apertar 1-4
**Onde:** `main.py:372-378`
`scale_mgr` é `None` no mono, e `scale_mgr.set(...)` gera `AttributeError`. O app cai, e junto caem o rádio e a ponte (RP-07).
**Correção:** `if scale_mgr is not None:`.

### 🟠 RP-07 — Saída/crash sem mandar `k` e sem `try/finally`
**Onde:** `main.py:74-458`
Qualquer exceção (RP-06, `cv2.error`), Ctrl+C ou `q` encerra sem parada geral. A thread do rádio e a ponte morrem junto. A STM segue no último estado (polia girando com `a`/`d`, pinça, malhas).
**Correção:** `try/finally` que manda `k`, fecha câmeras e serial, e handler de SIGTERM. Combinar com FW-06.

### 🟠 RP-08 — Rádio: perda de sinal deixa a polia girando; pinça nunca recebe `x` **(bancada)**
**Onde:** `pwm_input.py:33-38`, `:109-169`
- CH12/CH13 mandam `w`/`s`, mas soltar o botão nunca manda `x`: a pinça sempre corre até o fim de curso. Confirmar se é intencional.
- Se o receptor parar de mandar pulso (failsafe "no pulses" ou fio solto) com A/D pressionado, `pulse_width_us` congela em "pressionado" e o `c` nunca sai. Não há detecção de pulso velho.

**Correção:** timestamp do último pulso por canal; acima de ~100 ms, tratar como solto e mandar `c`/`x`.

### 🟠 RP-09 — Desconexão USB silenciosa e sem reconexão
**Onde:** `serial_communication.py:162-170`, `:111-128`
Depois de desconectar, `is_open` continua `True`, `in_waiting` levanta exceção a cada volta (engolida) e a thread entra em loop. O HUD mostra a última telemetria como se fosse atual, porque não há timestamp.
**Correção:** detectar `SerialException`, reabrir com backoff, e guardar o horário da última linha para mostrar "TELEMETRIA VELHA".

### 🟠 RP-10 — `write` sem `write_timeout` **(bancada)**
**Onde:** `serial_communication.py:91`
Se o buffer de TX do CDC encher (STM travada, FW-07), `send_ang_serial` bloqueia a thread principal: janela, teclado e `p` congelam.
**Correção:** `write_timeout=0.05` e tratar `SerialTimeoutException`.

### 🟠 RP-11 — Várias threads escrevem na serial sem lock
**Onde:** `serial_communication.py:107-128`
Escrevem na mesma porta o loop de visão (`V` a cada frame), o rádio, os handlers TCP/HTTP da ponte e o teclado. Bytes podem intercalar (`V12.0c\n0 0.85\n`), corrompendo o `V` ou perdendo o `c` (parar a polia).
**Correção:** `threading.Lock` em volta de `arduino.write`.

### 🟠 RP-12 — Intrínsecos implausíveis e calibração que não é usada
**Onde:** `configs.py:29-47`
`cmtx0` tem cx=276 e cy=257 numa imagem de 1280×720 (o centro esperado é ~640/360). `cmtx1` tem fy/fx = 1,47 e cy = 91. Parecem ser de outra resolução ou da época RTSP. `calib.py` grava em `camera_parameters/*.dat`, mas nada lê esses arquivos.
**Correção:** recalibrar a 1280×720 (depois de RP-02) e carregar os `.dat` com caminho absoluto via `__file__`.

### 🟠 RP-13 — Ordem `/dev/video0` / `/dev/video2` não é estável
**Onde:** `configs.py:18-19`
A numeração muda com o boot ou ao replugar, e esquerda/direita trocam em silêncio (disparidade lixo). O YAML de calibração já usa symlinks udev, o main não.
**Correção:** usar `/dev/camera_esquerda`/`direita` (udev) ou `/dev/v4l/by-path/...`.

### 🟠 RP-14 — Tecla `R` reseta a STM em operação sem confirmação
**Onde:** `main.py:444-446`
Fica a um Shift de distância de `r` (reset do SGBM). No firmware, `R` significa "ler ganhos". Depois do reset, a porta CDC cai e não reconecta (RP-09).
**Correção:** confirmação ou outra combinação, e reabrir a serial depois.

### 🟡 RP-15 — Bearing truncado: zona morta de ±1° e viés
**Onde:** `main.py:65`, `:102`, `:172`
`int(...)` trunca em direção a zero (0,9° vira 0). O `deque(maxlen=1)` torna a "média" inútil, mas o HUD mostra "Med" como se fosse filtrada.
**Correção:** manter float; se quiser filtro, maxlen 3-5 ou mediana.

### 🟡 RP-16 — Porta serial sem identificação do dispositivo
**Onde:** `serial_communication.py:89-96`
Conecta na primeira porta que abrir, que pode ser outro adaptador USB-serial. `flush()` não limpa a entrada.
**Correção:** checar VID/PID ou esperar uma linha `TEL,`; `reset_input_buffer()`.

### 🟡 RP-17 — Faixas de PWM do rádio estreitas e comentário errado **(bancada)**
**Onde:** `pwm_input.py:42-46`
"Solto" exige 1000 < pw < 1100 e "pressionado" exige 1900 < pw < 1960, mas o comentário diz 1300/1700. Rádios que saem com 988 ou 2012 µs nunca armam ou nunca disparam.

### 🟡 RP-18 — Medição de pulso em callback Python **(bancada)**
**Onde:** `pwm_input.py:86-101`
O timestamp é lido na entrega do callback, não na borda, e com a visão pesada os eventos atrasam. `CONFIRM_READS=6` a 5 ms equivale a ~1-2 pulsos reais.
**Correção:** `lgpio.callback`, que entrega o timestamp do kernel.

### 🟡 RP-19 — CPU desperdiçada no Pi 5
- SGBM 3WAY + matcher direito + WLS no quadro inteiro, quando só a ROI circular é usada; recortar uma faixa economiza muito.
- Laço Python ponto a ponto por linha de Hough (`camera_and_image_manipulation.py:64-85`); dá para vetorizar.
- Dois `np.percentile` sobre o array inteiro (`main.py:149-150`); um só com `[2, 98]` resolve.
- Captura a 720p e reduz para 0,5; melhor pedir a resolução final direto à câmera.
- O main reprocessa o mesmo frame (RP-05).

### 🟡 RP-20 — Calibração estéreo com pares não sincronizados
**Onde:** `Calibration/calib.py:61`, `:189-232`
Usa o backend padrão, sem V4L2 e sem MJPG (o mesmo problema que o `video_streamer` corrigiu), com uma thread por câmera. O "par" pode ser de instantes diferentes.
**Correção:** reaproveitar o `_open_camera` do `video_streamer` e capturar com o tabuleiro parado.

### 🟡 RP-21 — Imagens velhas e pareamento frágil na calibração
**Onde:** `calib.py:340-351`
Imagens de sessões anteriores ficam em `frames/`. Se faltar um arquivo, o corte por `min()` desalinha todos os pares seguintes.
**Correção:** limpar as pastas no início e parear pelo índice.

### 🟡 RP-22 — `requirements.txt` incompleto
**Onde:** `Raspberry/requirements.txt`
Falta `numpy` e `lgpio`/`rpi-lgpio` (necessário para o gpiozero no Pi 5 dentro de um venv). Não há versões fixadas. `opencv-contrib-python` via pip pode conflitar com o `python3-opencv` do apt.

### 🟡 RP-23 — Baseline e unidades
**Onde:** `camera_and_image_manipulation.py:35`, `configs.py:48-51`
`abs(T_rel[0][0])` = 0,3656 m, mas a baseline retificada real (`-P2[0,3]/P2[0,0]`) é 0,3720 m: ~1,7% de erro em D. A heurística `> 2.0 → /100` resolve centímetros, mas não milímetros.

### ⚪ Raspberry — baixos
- **RP-24** — o app não roda headless (`namedWindow`/`imshow`); por SSH sem X ou via systemd, cai antes de iniciar o rádio.
- **RP-25** (`camera_and_image_manipulation.py:222-226`) — `inc/dec_min_disp` sem limite.
- **RP-26** (`:103-108`) — bearing descontínuo em ±90°.
- **RP-27** — o modo mono usa imagem não retificada (possivelmente da câmera direita) e o estéreo a esquerda retificada: o bearing muda alguns graus ao trocar de modo.
- **RP-28** (`main.py:132-135`) — câmera entregando outra resolução: o resize distorce o aspecto em silêncio.
- **RP-29** — `camera_and_image_manipulation.py:11` e `configs.py:10` importam `gpiozero` sem fallback, o que anula o fallback de `serial_communication`. `pwm_input` cria GPIOs na importação.
- **RP-30** — imports `*` em cascata: a origem de `cv2`, `np`, `sys`, `deque` e `telemetria_dados` fica implícita (ver VS-06).
- **RP-31** — o HUD lê `telemetria_dados` campo a campo enquanto outra thread escreve (só visual). As fontes Hershey não desenham "°", e `pwm_*` aparece como "1500.0".
- **RP-32** (`calib.py:291`, `:395`) — o timeout de 30 s do `waitKey` aceita o frame sozinho; `scipy.linalg` é importado sem uso; as pastas são relativas ao cwd.
- **RP-33** — comentários desatualizados: `main.py:442` diz GPIO24 (é 18); `main.py:51` e `calib.py:60` falam em RTSP.

---

## 3. Raspberry — `video_streamer.py` (versão modificada, não commitada)

Forçar V4L2 + MJPG e validar que a câmera entrega frames foram boas decisões. O ponto fraco é o auto-reinício por `os.execv`.

### 🟠 VS-01 — O auto-reinício reseta a STM32 no meio da operação
**Onde:** `video_streamer.py:184` + `serial_communication.py:84`
O `execv` reimporta `serial_communication`, que faz `resetar_stm32()` na importação: reset físico pelo GPIO18 com a esfera operando. As malhas desligam e a serial e a ponte somem por 2-3 s. Se a câmera abrir e cair de novo, entra em ciclo e reseta a cada volta.
**Correção:** antes do `execv`, definir `os.environ["ESFERA_RESTART"]="1"` e pular o reset quando existir. Somar um contador de reinícios e desistir depois de N.

### 🟠 VS-02 — O `execv` pode deixar a câmera boa "ocupada" no processo novo **(bancada)**
**Onde:** `video_streamer.py:176-184`
Só a câmera que falhou é liberada. O OpenCV abre `/dev/videoX` sem `O_CLOEXEC`, então o descritor da outra câmera (com streaming ativo) passa para o processo novo, e o novo `VideoCapture` tende a falhar com EBUSY. O reinício que deveria consertar uma câmera derruba as duas, e o main termina em `sys.exit(1)`.
**Correção:** registro de instâncias na classe e `release()` de todas antes do `execv`.

### 🟡 VS-03 — Corrida no `stop()`
**Onde:** `video_streamer.py:190-198`
O `join(timeout=1.0)` volta enquanto a thread ainda está em `read()` (até ~10 s). O `release()` concorrente pode dar segfault no OpenCV, ou `AttributeError` na linha 168. Acontece ao apertar `q`.

### 🟡 VS-04 — Detecção de câmera travada lenta **(bancada)**
O `cap.read()` do V4L2 bloqueia até o timeout interno (~10 s). `_open_camera` lê 5 frames, então "3 reconexões" pode levar 30-40 s. Há também `sleep` dobrado entre falhas (linhas 174, 132 e 143).

### 🟡 VS-05 — Câmera que falha no `start()` nunca é retentada
**Onde:** `video_streamer.py:104-105`
Sem thread, não há retry. Depois de um reinício, o sistema pode ficar em mono para sempre.

### ⚪ `iniciar_streamer` (hoje comentado no main, mas quebra se for ativado)
- Falta `-pix_fmt yuv420p` na saída: o libx264 gera yuv444p, que a maioria dos players e navegadores não decodifica.
- A largura fixa de 3 painéis não bate com o modo mono (2 painéis) nem com a troca de escala: imagem embaralhada.
- O x264 exige dimensões pares; uma escala nova (ex.: 0,3) quebra.
- `stdin.write` bloqueante: um ffmpeg ou RTSP lento trava o loop de visão inteiro, inclusive o envio de `V`.
- O processo ffmpeg antigo não é encerrado no `except` do main (zumbi).

### ⚪ Outros
- **VS-06** — **não remover** os imports "não usados" (`np`, `deque`, `serial`, `threading`): o `main.py` depende deles via `from video_streamer import *`.
- **VS-07** — `_open_camera` aceita stream morrendo: se o 1º frame vem e o 2º falha, conta como sucesso. Exigir sucesso na última leitura.
- **VS-08** — MJPG não verificado: logar `cap.get(cv2.CAP_PROP_FOURCC)` para ver se a câmera aceitou.
- **VS-09** — flags do interpretador (`python -u`) se perdem no `execv`.

---

## 4. Ponte TCP/HTTP, twin e wizard

### 🔴 BR-01 — Ponte sem autenticação, em 0.0.0.0, com CSRF via GET
**Onde:** `Raspberry/bridge.py:187` (TCP), `:197` (HTTP), `:107-121` (`/api/cmd`)
Qualquer host da rede manda qualquer comando para a serial da STM. `/api/cmd?c=…` aceita **GET** com `Access-Control-Allow-Origin: *`, então qualquer página aberta num navegador da mesma LAN dispara um comando com um simples `<img src="http://rasp:8080/api/cmd?c=m">`. Dá para mandar `m` (teste bloqueante), `p`/`y` (liga as malhas), `G5 1e9` ou `V` sem o clamp de ±90°.
**Correção:**
- bind em 127.0.0.1 ou na interface do laboratório (com opção explícita para abrir);
- token;
- `POST` com checagem de `Origin`, sem `ACAO: *`;
- whitelist e validação de faixa por comando na ponte.

### 🟠 TW-01 — Twin envia o ganho com o índice deslocado ✔
**Onde:** `twin/window.py:163-166` vs `interactive.py:94-108` e `protocol.GAIN_NAMES`
Faltou `head_decel` na lista de ajustáveis, mas o twin manda `idx = selected-1`. Efeitos:
- ajustar `head_rate_limit` grava em `head_decel`;
- ajustar `head_rate_kp_us` (~900) grava em `head_rate_limit`, que vira ~990 rad/s e deixa de limitar;
- ajustar `head_rate_ki_us` grava em `head_rate_kp_us`.

A sim e o hardware divergem e o yaw pode oscilar violentamente.
**Correção:** `protocol.GAIN_NAMES.index(nome)` em vez da posição.

### 🟠 BR-02 — Cliente lento trava a thread da serial
**Onde:** `bridge.py:162-170`, `:174-183`
`broadcast` roda na thread de leitura da serial e faz `sendall`/`wfile.write` bloqueantes, sem timeout, **com o lock seguro**. Se o PC do twin suspender ou o Wi-Fi cair, o buffer de envio enche. A leitura da serial para, a telemetria e o HUD congelam e novos clientes ficam presos no lock.
**Correção:** fila por cliente com thread escritora (descartando linhas antigas) ou `settimeout` curto; nunca fazer I/O com o lock seguro.

### 🟠 WZ-01 — Wizard aceita ruído como calibração e faz `--push` sem confirmação
**Onde:** `wizard.py:177`, `:280-286`
O limiar `norm < 1e-3 °/s` está muito abaixo do ruído (~0,5 °/s). O wizard não confere `FLAG_WIZARD` (16) na TEL para saber se o pulso aconteceu. Com FW-01, nenhum pulso ocorre, e direções e `yaw_sign` aleatórios vão para a STM via `M`.
**Correção:** exigir `flags & 16` durante o pulso, limiar por SNR (k·σ da linha de base) e confirmação antes do push.

### 🟠 WZ-02 — Wizard trava sem telemetria e o Ctrl+C não manda parada
**Onde:** `wizard.py:85-97`, `:271`
`next_frame` não tem timeout, não há `try/finally` com `k`, e o wizard começa sem confirmação. Com FW-01, um Ctrl+C deixa a pinça subindo.
**Correção:** timeout com erro claro, `input()` de confirmação e `finally: write_line("k")`.

### 🟡 BR-03 — TCP da ponte não remonta linhas entre `recv`s
**Onde:** `bridge.py:36-37`
`recv(256)` → `split(b"\n")` sem buffer, e cada pedaço ganha `\n`. Por exemplo, `G6 900` partido vira `G6 9` (ganho 9) mais `00`, que entra como dois comandos imediatos `0`. Isso acontece com certeza com mais de 256 B em rajada (ex.: `--push` com 3 linhas `M` mais outros comandos).
**Correção:** buffer por conexão, como o `TcpTransport` já faz.

### 🟡 BR-04 — Threads SSE vazam
**Onde:** `bridge.py:88-89`
O handler fica em `while ...: sleep(0.5)` e nunca percebe que o cliente saiu. Cada recarga da página deixa uma thread viva.
**Correção:** sair quando o `wfile` sai de `_sse_clients`, e mandar heartbeat `: ping`.

### 🟡 BR-05 — Bind TCP sem `try` derruba o app de visão
**Onde:** `bridge.py:187` + `serial_communication.py:104`
Com a porta 9999 ocupada (outra instância rodando), o `OSError` acontece no import e o `main.py` não sobe. Além disso, `stop()` não chama `server_close()` nem fecha os clientes.

### 🟡 BR-06 — Servidor estático expõe a raiz do repositório
**Onde:** `bridge.py:63-66`, `:124-127`
O `supervisorio_esfera.html` não existe no repo nem na pasta pai, então `/` dá 404. Em compensação, `/.git/` (com listagem), `.venv/`, todo o código e o `calib.json` ficam baixáveis por qualquer um. O `..` é bloqueado pelo stdlib, mas symlinks são seguidos.
**Correção:** servir só um diretório dedicado (`Raspberry/web/`), sem listagem, e versionar o HTML.

### 🟡 TW-02 — Semântica do yaw diferente entre firmware e twin
**Onde:** `firmware/main.cpp:871`, `:918`; `twin/simstm.py:113`, `:197-202`
O firmware publica `yaw_gyro`, re-zerado a cada setpoint numérico e sem normalização. O SimSTM publica o heading absoluto `%360`. `0` zera o setpoint no firmware, mas significa "heading atual" no sim. O painel real × sim deixa de ser comparável depois de qualquer degrau.

### 🟡 TW-03 — Malhas do twin começam ligadas, as do hardware desligadas
**Onde:** `twin/window.py:147-150`, `controller.py:98-99`
Como as teclas mandam toggles, apertar 1 desliga a sim e **liga** o hardware.
**Correção:** começar desligado e sincronizar pelas flags da TEL; ou comandos on/off absolutos.

### 🟡 TW-04 — Comando com o twin desconectado (inclusive `K`) some sem aviso
**Onde:** `twin/window.py:101-107`, `:120-121`
Depois de qualquer `OSError` (basta um timeout de 10 ms no `sendall`), `connected = False` para sempre, sem reconexão. O K para a sim, mas não a esfera.
**Correção:** reconexão automática, aviso visual forte e reenvio do K.

### 🟡 TW-05 — `TcpTransport.read_line` gasta 10 ms por linha
**Onde:** `twin/transport.py:67-78`
Chama `recv` (timeout de 0,01 s) antes de olhar o buffer. Com 100 linhas acumuladas, congela a GUI por ~1 s.
**Correção:** consumir `_buf` primeiro e usar `recv` não bloqueante.

### ⚪ Ponte/twin — baixos
- **TW-06** — a linha `M` com `.6g` pode chegar a 61 caracteres e estourar o buffer de 60 do firmware (FW-13). Usar `.4g`.
- **TW-07** (`protocol.py:71-106`) — `parse_line` aceita `nan`/`inf`, não conta linhas descartadas e não há versão de protocolo. Se o formato mudar, o twin fica em "aguardando" sem diagnóstico.
- **TW-08** (`transport.py:40`) — `SerialTransport` com timeout de 10 ms pode devolver linha parcial. Bufferizar até `\n`.
- **TW-09** (`simstm.py:131-204`) — o SimSTM levanta `ValueError` em entrada malformada (o firmware vira 0), aceita `nan`/`.5` como yaw e dá outra semântica a `z`/`t`.
- **TW-10** (`bridge.py:103`) — `/api/telemetria` diz "conectado" para sempre depois da primeira linha. Usar o timestamp.

---

## 5. Simulador Python e paridade com o firmware

### 🟠 SIM-01 — `set_pointing_error` acumula o setpoint a cada `V` ✔ (reproduzido)
**Onde:** `controller.py:139`
`_heading_setpoint += error` roda a cada mensagem. 50 × `V10` deixam o setpoint em 140°, e quando a visão expira a esfera simulada gira até 140°. O firmware não altera `desired_yaw` com a visão.
**Correção:** `setpoint = wrap(heading_atual + error)`, ou não mexer no setpoint.

### 🟠 SIM-02 — Paridade quebrada nos pontos que importam
FW-02, FW-03 e FW-11 são divergências firmware × Python em que **o Python está certo**. Corrigir no firmware e acrescentar o teste de paridade INF-03.

### 🟡 SIM-03 — As reservas de PWM não reservam nada (Python e firmware) (reproduzido)
**Onde:** `actuators.py:328-339`, `:429`; `main.cpp:484-509`
Swing 490 + yaw 300 dá até 790 µs, mas o clamp é ±500 por motor. Fy = 12 N com yaw = 300 µs entrega (−2,9; 15,8) N: **14° de erro de direção** e +32% de módulo.
**Correção:** swing ≤ 500 − |yaw| ou um λ comum sobre swing+yaw.

### 🟡 SIM-04 — Alocação `(2/3)·Tᵀ·F` só vale para geometria ideal (reproduzido)
**Onde:** `actuators.py:330`; `main.cpp:476-480`
O wizard calibra `thrust_dir` livremente. Com o motor b a 80°, pedir 5 N em X entrega (4,27; 2,01) N (~25° de erro) mais 1,57 N·m de torque espúrio.
**Correção:** pseudo-inversa de [T; yaw_signs] calculada ao aplicar o `M`, nos dois lados.

### 🟡 SIM-05 — Latch por motor só existe no Python
**Onde:** `actuators.py:105`, `:365` vs `main.cpp:132-137`, `:456`
O Python salva `latch_us` por motor no JSON. O firmware usa `ESC_LATCH_US = 60` global, e o `M` não transmite o latch.
**Correção:** acrescentar `latch_us` ao `MotorCalib` do firmware e ao comando M.

### 🟡 SIM-06 — Simulação mais otimista que o hardware
Cenários e modo interativo a 100 Hz com velocidade e taxa ideais. O firmware roda a 200 Hz com `rate_z_filt` (τ ≈ 14 ms) e v̂ vindo da fusão. As margens de fase no sim ficam otimistas.
**Correção:** dt = 0,005 e o mesmo filtro no caminho de medida.

### ⚪ Simulador — baixos
- **SIM-07** (`interactive.py:166`) — o painel mostra PWM recalculado do empuxo pós-lag, não `act.pwm_us`. Com ESC travado, mostra 1500.
- **SIM-08** (`interactive.py:103`) — passo de `vel_kd` é `0.1 × 0`, então nunca sai de 0.
- **SIM-09** — `controller.py:108`: o limite do PID não é recalculado após `apply_motor_calib` (o firmware também não).
- **SIM-10** — código morto: `latch_band` em `actuators.py:366`, modo `angle_wrap` em `pid_controller.py`, `wrap_to_pi` duplicado, `fig` sem uso em `main.py:322`.
- **SIM-11** — `main.py:103-106`: `--thrust` é ignorado em silêncio quando há `--calib`.

---

## 6. Testes e infraestrutura

### 🟠 INF-01 — `test_rasp.py` mexe em hardware real
**Onde:** `tests/test_rasp.py:272` + `serial_communication.py` (nível de módulo)
Importar `serial_communication` **reseta a STM32**, abre `/dev/ttyACM0` e sobe a ponte em 9999/8080. Rodar `pytest` na Pi (o README sugere) reseta a esfera em operação, disputa a serial com o app e falha com "address in use".
**Correção:** mover os efeitos para `init()`/`main`, e no teste fazer monkeypatch de `serial.Serial`/`LED` e usar `http_port=0`. Isso também resolve VS-01, BR-05 e RP-29 de uma vez.

### 🟡 INF-02 — Ambiente de testes não reproduzível
O `.venv` do repo não tem pytest nem matplotlib, e `requirements.txt` não lista pytest. `docs/build/build_pdf.py` precisa de reportlab, Pillow e pypdf (não listados) e tem caminho Windows fixo (`C:\Users\guilh\...`, linha 34).

### ⚪ Testes — baixos
- **INF-03** — nenhum teste lê `main.cpp`. Um teste que extraia constantes e ganhos do firmware e compare com o Python, mais um que reproduza o parser de 1 caractere, teria pego FW-01, FW-02, FW-11, FW-12, SIM-05 e TW-01.
- **INF-04** (`test_plant.py:341-342`) — `act_n`/`act_i` calculados e nunca verificados. `test_pointing_error_from_vision` só checa o sinal (não pegaria SIM-01). Não há teste direto de `PIDController`.
- **INF-05** (`test_plant.py:598`, `test_twin.py:250`) — testes gravam `out/*.png` no cwd. Usar `tmp_path`.

---

## 7. Pontos positivos

- **Formato TEL único**: 16 campos idênticos em firmware, `protocol.py` e SimSTM, `GAIN_NAMES` na mesma ordem, e parse defensivo (contagem de campos + `ValueError`).
- **Boot seguro no firmware**: malhas desligadas, ESCs em 1500 logo após o attach, aritmética de `micros()`/`millis()` à prova de overflow e desbloqueio do I2C no boot.
- **PID bem feito**, idêntico nos dois lados: anti-windup condicional e derivada na medida.
- **Modelo de atuador cuidadoso**: zona morta com inversa exata, gate anti-latch e quantização no µs final.
- **Simulação de qualidade**: integrador RATTLE (restrição |r| = L em 1e-9, energia limitada a < 0,5%).
- **Wizard com limites no firmware**: pulso ≤ 3 s e |us| ≤ 500, malhas desligadas durante o pulso.
- **Visão com proteções**: expira em 500 ms no firmware, `send_ang_serial` limita o bearing a ±90° e usa D = −1 explícito no modo mono, e o modo mono existe como degradação.
- **Rádio filtrado**: mediana, arming e confirmação por leituras.
- **`video_streamer`**: V4L2 + MJPG + validação de frames resolvem um problema real de banda USB.

---

## 8. Plano sugerido

1. **Antes da próxima sessão com motores (correções curtas):**
   - FW-01 e FW-12 (parser);
   - FW-02 (wrap);
   - FW-04 (`k` no wizard);
   - FW-17 (cópia do PID);
   - TW-01 (índice de ganho);
   - RP-02 (YAML);
   - RP-03 (GAIN);
   - RP-06 (mono);
   - `try/finally` com `k` (RP-07).
2. **Segurança em campo:**
   - FW-06/FW-07 (heartbeat + IWDG);
   - FW-05 (MPU);
   - RP-04 (teclado durante a queda de câmera);
   - RP-08 (rádio);
   - RP-11 (lock da serial);
   - VS-01/VS-02 (`execv`) — ou remover o auto-reinício até o reset sair do import (INF-01);
   - BR-01 (ponte).
3. **Validar em bancada:** FW-03 (anti-balanço com yaw a 0/90/180°), FW-10 (float no printf), RP-05 e VS-04 (cronometrar a queda de câmera), RP-08.
4. **Visão:** RP-02 → recalibrar (RP-12, RP-20, RP-21) → RP-13 (udev) → RP-01 (faixa de disparidade).
5. **Paridade e testes:** INF-03 (teste de paridade firmware × Python), SIM-01, FW-11, SIM-03/04/05, INF-01.
