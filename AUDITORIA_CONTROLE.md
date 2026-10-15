# Auditoria de sensoriamento e controle — Esfera

**Data:** 14/10/2026 · **Base:** `firmware/main.cpp` consolidado (YAWBOM + twin), `wizard.py`, `calib.json`, `supervisorio_esfera.html`, docs.
**Escopo:** só o que afeta a medição e as malhas (anti-balanço e heading). Complementa o `AUDITORIA.md` de 13/10 — não repete o que lá já está, exceto para dizer o que continua aberto.
**Método:** revisão linha a linha + modelo linear do pêndulo duplo em `docs/analise_pendulo_duplo.py` (roda em 1 s, use para refazer as contas com os valores medidos). Itens **(bancada)** precisam de confirmação na esfera.

Severidade igual à do `AUDITORIA.md` (🔴 crítico · 🟠 alto · 🟡 médio · ⚪ baixo).

---

## Resumo

O anti-balanço instável tem uma causa principal bem identificada e uma segunda que deve aparecer logo depois de corrigir a primeira:

1. **SC-01 🔴** — O mapeamento giroscópio → velocidade está **espelhado** (troca de eixos sem a troca de sinal). Uma reflexão não é uma rotação, então, qualquer que seja a orientação de montagem da MPU, **existe sempre uma direção em que o amortecimento vira realimentação positiva**. No modelo, o modo do pêndulo nessa direção fica com ζ ≈ −0,4 a −0,7: a oscilação cresce em ~2 s até saturar. É exatamente o sintoma relatado.
2. **SC-02 🟠** (rebaixado após a geometria informada em 14/10: CG logo abaixo do pino, MPU junto ao pino, hélices **abaixo** do CG) — O conceito "v̂ = L·ω_esfera" supõe que a atitude da esfera acompanha o cabo. Quem prende a esfera ao ângulo do cabo é o torque m·g·d (d = pino→CG); com o CG perto do pino esse acoplamento é **fraco**, e a esfera passa a inclinar 1,2× a 3× o cabo (ou mais, se d < 5 cm). Com as hélices abaixo do CG a malha **não explode** depois do SC-01, mas o empuxo gira a própria esfera e o controle passa a amortecer o bamboleio dela, não o cabo: no modelo, o balanço do cabo fica com ζ ≈ 0,00–0,12 (quase sem amortecimento). Realimentando a **velocidade do CM** (acelerômetro + giroscópio), ζ ≈ 0,6 em todas as geometrias.

O heading funciona, mas está "no braço": o PID manda **µs** direto (o ganho real varia ~15× com a amplitude, porque o empuxo é quadrático), a mistura com o anti-balanço é feita em µs (acopla as duas malhas), a integração do yaw ignora rotações < 0,35 °/s e a visão entra como um degrau segurado entre quadros, sem compensar a latência.

O `calib.json` não é usado porque **nenhum caminho aplica os parâmetros** (o firmware descarta tudo do `M` exceto o `yaw_sign`; o supervisório só carrega no simulador local) e porque os números do wizard **não podem dar certo** com o firmware atual (direção espelhada + girada por um yaw arbitrário; força calculada com o modelo de pêndulo simples). O `calib.json` atual é idêntico aos defaults.

### Ordem sugerida

| # | Item | Tipo | Esforço |
|---|---|---|---|
| 1 | Teste de eixos à mão (sem motores) → corrigir SC-01 | bancada + 4 linhas | 30 min |
| 2 | Teste de oscilação livre (60 s de TEL, malhas off) → FFT de wx/wy: existe pico em 0,5–1,5 Hz? | bancada | 15 min |
| 3 | Fechar a malha XY com `vel_kp` = 10 e observar | bancada | 15 min |
| 4 | Se aparecer bamboleio (ou de qualquer forma, como solução definitiva): estimador de velocidade do CM (SC-02) | firmware | médio |
| 5 | Alocação em força para XY + yaw juntos (SC-03) e heading em torque (SC-09) | firmware | médio |
| 6 | Calibração do giro com janela ≥ 2 períodos e gate de verdade (SC-04); tirar a zona morta de 0,35 °/s (SC-05) | firmware | pequeno |
| 7 | Visão por evento com compensação de latência (SC-06) | firmware | pequeno |
| 8 | Empuxo medido em dinamômetro; `M` aplicando de fato; supervisório enviando `M` (WZ-01..05) | bancada + código | médio |

---

## 1. Anti-balanço

### 🔴 SC-01 — Eixos do giroscópio espelhados: um eixo tem realimentação positiva
**Onde:** `passo_fusao()` (`main.cpp:464-465`, `:491-492`), `calibrar_mpu()` (`:429-430`), `passo_controle_xy()` (`:544-549`).

Convenção usada pelo resto do sistema (twin, supervisório, sim): `sx > 0` = esfera deslocada para **+X**, `wx = d(sx)/dt`, `v = L·(wx, wy)`.

Cinemática (MPU com Z para cima, X alinhado ao motor a): uma rotação **positiva em torno de +Y** leva o CM para **−X**; uma rotação positiva em torno de +X leva o CM para **+Y**. Logo:

```
v_x = −L·g_y      v_y = +L·g_x        (rotação de 90°: det = +1)
```

O firmware faz `rate_x = +g_y`, `rate_y = +g_x` (troca de eixos sem sinal: **det = −1, uma reflexão**). O eixo Y está certo; o eixo X está com o sinal trocado, e o PID `F = −kp·v̂` vira `F = +kp·v` nesse eixo. O acelerômetro (`atan2(−ax, az)`) está coerente com o giroscópio, por isso a fusão parece "funcionar" — o erro só aparece na relação com os motores.

Detalhe importante: **não existe orientação de montagem da MPU (girada em Z) que torne uma reflexão correta.** Se a MPU estiver girada, os dois eixos ficam misturados, mas sempre sobra uma direção com ganho negativo. Por isso o sintoma é "às vezes amortece, às vezes cresce", dependendo da direção do empurrão.

Modelo (`docs/analise_pendulo_duplo.py`, kp = 18): no eixo errado o modo de 0,17 Hz fica com **ζ entre −0,35 e −0,69** em todas as geometrias. Tempo de crescimento ~2 s.

**Correção** (supondo MPU com Z para cima e X para o motor a — **confirmar com o teste abaixo antes**):

```cpp
// passo_fusao(): velocidade de inclinação no CORPO, convenção sx>0 = deslocado para +X
float rate_x = -(s.gy - gyro_bias[1]) * DEG2RAD;   // era +gy
float rate_y =  (s.gx - gyro_bias[0]) * DEG2RAD;
...
float th_acc_x = atan2f(s.ax, s.az) - theta_offset_x;   // era atan2f(-s.ax, s.az)
// calibrar_mpu():
theta_offset_x = atan2f(ax0, az0);                       // era atan2f(-ax0, az0)
```

Melhor ainda: uma matriz `IMU_TO_BODY[2][2]` no topo do arquivo (com o comentário do teste que a determinou), para que uma montagem diferente seja corrigida em uma linha.

**Teste de eixos (malhas desligadas, sem bateria nos ESCs):** com o supervisório mostrando `wx`/`wy`, empurre a esfera devagar com a mão:
- na direção em que o **motor a empurra a esfera** (+X do corpo) → `wx` tem que ficar **positivo** durante o empurrão e `wy` ≈ 0;
- 90° anti-horário visto de cima (+Y) → `wy` positivo e `wx` ≈ 0.

Se der trocado ou invertido, ajuste a matriz. Use o empurrão lento à mão e **não** um pulso de motor: com o pêndulo duplo (SC-02) o giroscópio pode responder no sentido contrário nos primeiros instantes de um pulso.

> Nota: o `BRINGUP.md` (Etapa 3.1) pede "conferir se as curvas fazem sentido", mas não define o sentido certo em relação aos motores. Foi por aí que o espelhamento passou.

### 🔴 SC-02 — A rotação da esfera não é a velocidade do cabo (pêndulo duplo) **(bancada)**
**Onde:** `passo_controle_xy()` — `vhat = L_M * w_est`; premissa repetida no guia (§3.2: "MPU no topo, junto ao pino … inclinômetro quase ideal").

A esfera está presa ao cabo por um pino no topo. Ela não é uma massa pontual na ponta do cabo: é um corpo rígido que pode girar em torno do pino. São dois graus de liberdade por eixo (ângulo do cabo θ₁ e atitude da esfera θ₂), com dois modos:
- **lento** (~0,2 Hz, θ₂ ≈ θ₁): o balanço que se quer amortecer;
- **rápido** (~0,5–1,5 Hz, conforme `d` = pino→CM e a inércia): a esfera "bamboleia" em torno do pino.

O giroscópio mede θ̇₂ (atitude da esfera), não θ̇₁ (cabo). O que obriga a esfera a inclinar junto com o cabo é o torque da tração no pino, m·g·d·(θ₁ − θ₂). Quanto **menor** `d`, mais frouxo esse vínculo — no limite d → 0 (pino no CG) a esfera balança sem inclinar e o giroscópio não vê o balanço. Por isso "CG perto do pino" não torna o pêndulo duplo irrisório; torna a atitude **menos** representativa do cabo. A exceção é se o cabo sai por uma guia rígida/longa (rigidez de flexão): aí o vínculo é mecânico e vale medir.

Modo livre (sem controle), m = 12 kg, L = 5 m:

| d (pino→CG) | I = 1 kg·m²: bamboleio · atitude/cabo | I = 2 kg·m²: bamboleio · atitude/cabo |
|---|---|---|
| 0,05 m | 0,39 Hz · 1,5× | 0,28 Hz · 2,8× |
| 0,10 m | 0,55 Hz · 1,2× | 0,39 Hz · 1,5× |
| 0,20 m | 0,79 Hz · 1,1× | 0,56 Hz · 1,2× |

Malha fechada (kp = 18), hélices 0,2–0,4 m **abaixo** do CG (ζ < 0 = instável):

| Realimentação | balanço do cabo (~0,2 Hz) | bamboleio |
|---|---|---|
| firmware atual, eixo X (SC-01) | **ζ ≈ −0,4 a −0,7** | — |
| giroscópio com sinal corrigido | ζ ≈ 0,00 a +0,12 (quase não amortece) | ζ ≈ +0,01 a +0,34 |
| velocidade do CM (estimador) | ζ ≈ +0,61 a +0,66 | ζ ≈ 0,00 (não excitado) |

(Com hélices **acima** do CG, caso que a doc antiga sugeria, o giroscópio corrigido desestabiliza o bamboleio: ζ até −0,6. Fica registrado caso a montagem mude.)

Ou seja: **corrigir só o SC-01 deve tirar a explosão, mas o amortecimento do cabo tende a ficar fraco**, porque o empuxo abaixo do CG gira a esfera e o controle gasta a ação "endireitando" a esfera. O sintoma esperado é: o PWM trabalha, o `wx/wy` cai rápido, mas o deslocamento real da esfera (visto de fora) demora a parar.

**Diagnóstico (bancada, 15 min):** (1) malhas desligadas, solte a esfera de ~5° e grave 60 s de TEL; compare a amplitude de `sx` com a amplitude real do cabo (vídeo ou marca no chão) — a razão é a "atitude/cabo" da tabela. (2) Segure o cabo firme logo acima do pino, dê um toque na esfera e meça o período da oscilação da atitude pelo giroscópio: é o bamboleio. Bamboleio > ~1,5 Hz → a esfera acompanha bem o cabo e o giroscópio serve; < ~0,8 Hz → precisa do estimador.

**Correção (definitiva):** realimentar a velocidade do CM, estimada assim:

1. O acelerômetro está no pino, e o pino é a ponta do cabo. A força específica medida no corpo é `a_bx ≈ ẍ_pino + g·θ₂`.
2. θ₂ (atitude da esfera) vem do giroscópio integrado — é o `sx_est` atual, com o sinal corrigido.
3. `ẍ_pino = a_bx − g·sx_est`; integra com vazamento (τ ≈ 8 s) → `v_pino`.
4. `v_CM = v_pino + d·θ̇₂` (θ̇₂ = `rate_x` filtrado).

```cpp
// por eixo, dentro de passo_fusao(), depois de sx_est atualizado (sinais conforme SC-01)
float ax_mps2 = (s.ax - acc_bias_x) * (G_ACC / 16384.0f);   // acc_bias_x: média na calibração
float apin_x  = ax_mps2 - G_ACC * sx_est;                    // aceleração horizontal do pino
vpin_x += (apin_x - vpin_x / TAU_VPIN) * dt;                 // integrador com vazamento, TAU_VPIN ~ 8 s
vcm_x   = vpin_x + D_PINO_CM * wx_est;                       // D_PINO_CM medido (m)
// passo_controle_xy(): vhat_x = vcm_x  (em vez de L_M * wx_est)
```

No modelo, isso deixa o bamboleio neutro (ζ ≈ 0, a esfera nunca o excita) e o pêndulo com ζ ≈ 0,4–0,65 para `kp` entre 12 e 18, mesmo errando `d` em ±50%. Bônus: `L` deixa de multiplicar o ganho (o erro de `L` some da malha). Validar primeiro no simulador Python, depois de acrescentar o pino ao modelo (`simulation.py` hoje é pêndulo simples — por isso o twin nunca mostrou nada disso).

Uma alternativa mais completa é um filtro de Kalman do pêndulo duplo, com o empuxo comandado como entrada. Só vale a pena depois de identificar `d`, `h` e `I`.

### 🟠 SC-03 — Yaw somado em µs: acopla heading e balanço e distorce a saturação
**Onde:** `realizar_controle()` (`main.cpp:691-697`).

O anti-balanço é convertido força → µs pela curva quadrática, e depois o yaw (já em µs) é **somado em µs**. Com F ∝ (u − dz)², somar em µs não é somar em força:

- **Força lateral espúria vinda do yaw.** Um yaw puro joga os motores a e b para um lado da curva e o c para o outro (o c é invertido). Com `REV_RATIO = 0,6`, a soma das três forças tangenciais deixa de ser zero: sobra **0,4·F** na direção (0,5; 0,87). Com yaw = 300 µs (F ≈ 5,9 N/motor), são **~2,4 N de empurrão lateral** a cada manobra de heading — 2% do peso, ≈1,2° de deflexão estática. Toda manobra de yaw bate na esfera.
- **Termo cruzado.** Com o anti-balanço ativo, o mesmo δµs de yaw rende mais força no motor que já está alto na curva do que no que está perto do zero. O torque de yaw muda com o balanço, e o yaw muda a força XY.
- **Saturação torta.** Reserva 400 (swing) + 300 (yaw) > clamp de 450: o corte por motor muda a direção da força e o torque (SIM-03 do `AUDITORIA.md` continua aberto).

**Correção:** alocar **força e torque juntos** e só no fim converter cada motor para µs (com `kf_fwd`/`kf_rev` do próprio motor):

```cpp
// Fb = força no corpo [N], Tz = torque de yaw [N·m]
for (int i = 0; i < 3; i++)
  f[i] = (2.0f/3.0f) * (motores[i].tx*Fbx + motores[i].ty*Fby)
       + motores[i].yaw_sign * Tz / (3.0f * R_P);
// um único lambda <= 1 para todos (preserva direção e razão força/torque),
// ou prioridade explícita: escalar primeiro o swing, depois o yaw.
for (int i = 0; i < 3; i++) us[i] = ESC_NEUTRAL + lroundf(us_from_force(i, lam * f[i]));
```

Para geometria ideal isso é a inversa exata de [T; torque]. Se um dia as direções medidas forem diferentes, troque por a inversa da matriz 3×3 calculada uma vez (SIM-04).

### 🟠 SC-04 — Calibração do giroscópio curta demais para um pêndulo; gate não bloqueia nada
**Onde:** `calibrar_mpu()` (`main.cpp:397-439`).

A média do bias usa 600 amostras × ~2,4 ms ≈ **1,4 s**, contra um período de **4,5 s**. Qualquer balanço residual não se cancela na média: com 1° de amplitude, o bias sai errado em **até ~1 °/s**. Esse erro vira `v̂ = L·δ` ≈ 0,09 m/s → **~1,6 N de empurrão constante** (kp = 18) em uma direção fixa do corpo, e uma deriva de heading de dezenas de graus por minuto (mascarada pelo SC-05).

O gate de quietude virou só um aviso: `calib_ok = true` sempre (o `BRINGUP.md` ainda diz que a calibração "se recusa"). E o limiar de 4,5 °/s (soma de três eixos) deixa passar um balanço de vários graus.

**Correção:**
- janela de ≥ 2 períodos (~10 s a L = 5 m), ou um número inteiro de períodos usando `periodo_medido`;
- gate real: desvio-padrão por eixo < ~0,2 °/s, senão `calib_ok = false` e as malhas não ligam (FW-18);
- `parada_geral()` antes de calibrar e limite de tentativas (FW-09 continua aberto);
- guardar também o bias do acelerômetro em repouso (necessário para SC-02).

### 🟡 SC-07 — Termo de "transporte" de yaw no caminho da velocidade
**Onde:** `main.cpp:479-480`, `COMP_YAW_SIGN`.

Depois do FW-03, a força é calculada e aplicada **no corpo** (ψ = 0). A velocidade no corpo sai do giroscópio **diretamente**: `v_b = L·(−ω_by, ω_bx)`, sem termo de transporte (a rotação em Z comuta com o eixo vertical). O termo `± wz·s` é a derivada do *ângulo expresso no corpo*, que gira quando a esfera gira mesmo parada. Com 2° de inclinação estática e 30 °/s de yaw, ele inventa 0,09 m/s → ~1,6 N **perpendicular** à inclinação, o que tende a criar balanço cônico durante manobras de heading.

O Etapa 3.2 do `BRINGUP.md` ("θx/θy não podem girar junto") pede o comportamento de mundo, mas a fusão inteira (acelerômetro incluído) é de corpo: no corpo, as curvas **devem** girar.

**Correção:** tirar o termo do caminho da velocidade. Se quiser mostrar inclinação no mundo no supervisório, gire `sx/sy` por `yaw` só na telemetria.

### 🟡 SC-08 — O acelerômetro não é inclinômetro num pêndulo
**Onde:** guia §3.2, comentários de `passo_fusao`, `FW-31`.

Na esfera pendurada, o acelerômetro mede a força do cabo dividida pela massa (mais o empuxo), não a gravidade. Ele dá o ângulo do **cabo em relação à esfera** (θ₁ − θ₂), que é ~0 no balanço lento. Com `ALPHA_FUSAO = 0,9995` (τ ≈ 10 s) o efeito é pequeno: um passa-altas suave na atitude (≈4° de avanço de fase a 0,22 Hz). Mas três coisas dependem disso:
- `sx/sy` na telemetria **subestimam** a amplitude real do balanço nos ciclos longos (o fantasma do supervisório fica menor);
- a zona de repouso no prumo (`theta_total_deg < 0,6°`) usa esse ângulo;
- é exatamente esse "defeito" que permite o estimador do SC-02 (o acelerômetro mede a aceleração do pino).

**Correção:** corrigir o texto do guia. Para mostrar o ângulo real do cabo no supervisório, use a posição integrada do estimador do SC-02 (`x_pino/L`), não `sx_est`.

### ⚪ Anti-balanço — baixos
- **SC-13** — `PID::update` zera a integral quando o erro troca de sinal. Num balanço a velocidade troca de sinal a cada meio período, então `vel_ki` nunca acumula nada. Hoje `ki = 0`, mas o ganho existe na interface e não faz o que parece. O `kd` deriva uma velocidade já derivada (ruído); manter em 0.
- **SC-14** — `dt_swing` sem limite (FW-16 aberto): depois de `m` (13 s) ou `z` (1,4 s), uma única iteração integra yaw, inclinação e PID com dt de segundos. `dt_swing = fminf(dt_swing, 0.02f)`.
- **SC-15** — MPU falhando (`s.ok == false`): a fusão pula e o controle segue com o último valor (FW-05 aberto). Com o estimador do SC-02 isso fica pior (integrador). Contar falhas e desligar.
- **SC-16** — Medição de período (`t`) usa `|ω|`, que nunca zera num balanço elíptico/cônico. Usar o cruzamento por zero de cada eixo. Ainda assim, **prefira `t` a digitar `L`**: o período dá o comprimento *efetivo* (pino + d), que é o que importa para `v̂ = L·ω` enquanto o SC-02 não sai.
- **SC-17** — `wx/wy` são chamados de "MUNDO" nos comentários e na TEL, mas são do corpo. O supervisório desenha o fantasma como se fosse mundo.

---

## 2. Heading (yaw)

### 🟠 SC-05 — Rotações lentas são invisíveis ao heading
**Onde:** `GYRO_Z_DEADZONE = 0,35 °/s` (`main.cpp:473`), `talvez_rebias()` (`:448`).

A integração do yaw descarta `|rate_z_filt| < 0,35 °/s`: até **21°/min** de rotação real não aparecem no heading, então a malha não corrige. O auto-rebias aceita `|rate_z| < 0,4 °/s` como "parado" e absorve rotação real lenta no bias (FW-26). Os dois juntos escondem justamente a deriva lenta que o heading existe para segurar. Com o bias mal calibrado (SC-04), a zona morta tapa o erro em bancada e piora em campo.

**Correção:** calibrar bem (SC-04) e reduzir a zona morta para ~0,05 °/s (ou zero). Rebias só com motores em neutro **e** visão confirmando o cabo parado no quadro, quando houver visão.

### 🟠 SC-06 — Visão como degrau segurado, sem compensar a latência
**Onde:** `passo_controle_yaw()` (`main.cpp:566-577`).

A cada iteração de 5 ms o firmware faz `desired_yaw = yaw_gyro + vision_bearing`, com o **último** bearing recebido. Entre dois quadros (~33–100 ms), a esfera gira, mas o bearing não muda: o setpoint anda junto com a esfera e o erro fica congelado no valor velho. Isso equivale a um segurador de ordem zero mais a latência da câmera e do processamento (tipicamente 80–150 ms), dentro de uma malha de posição: dá sobressinal e ciclo-limite quando a visão for usada para valer.

**Correção:** tratar o `V` como evento. Atualizar o setpoint **só quando chega um V novo**, referido ao instante da foto, e fechar a malha sempre com o giroscópio:

```cpp
// no parser do V (frame novo):
float yaw_no_quadro = yaw_gyro - rate_z_filt * VISION_LATENCY_S;   // medir a latência na bancada
desired_yaw = wrap180(yaw_no_quadro + vision_bearing);
// em passo_controle_yaw(): sempre
pos_error_yaw = wrap180(desired_yaw - yaw_gyro);
```

O ideal é a Rasp mandar a idade do quadro no `V` (por exemplo, `V<b> <D> <idade_ms>`). Junto com isso, o RP-05 (não reenviar quadro velho) e o RP-15 (bearing truncado em inteiro) do `AUDITORIA.md` ficam mais importantes.

### 🟡 SC-09 — PID em µs sobre um atuador quadrático: ganho varia ~15×
**Onde:** `passo_controle_yaw()` (`main.cpp:608-640`).

O PID entrega µs. O torque é ∝ (u − 30)², então o ganho incremental do atuador vai de ~0 (perto da zona morta) a ~0,04 N·m/µs (perto de 300 µs). Com os ganhos atuais, na faixa média, a malha equivale a Kp ≈ 7,8 N·m/rad e Kd ≈ 16 N·m·s/rad: com Iz = 1,73, ζ ≈ 2,2 (muito sobreamortecida, lenta). Perto do alvo o ganho vai a ~0. Daí os remendos:

- a zona morta de 2,5° com histerese e o reset da integral;
- o piso de 45 µs, que fica **dentro** do travamento do ESC (~60 µs pelo README) e da zona morta de 30 µs: gera ~0,02 N por motor, ou seja, nada. O piso não move a esfera; só faz barulho no PWM.

**Correção:** controlar em **torque** (N·m) e deixar a alocação do SC-03 linearizar:

```cpp
float Tz = KP_T * err_rad - KD_T * rate_rad;        // N·m
Tz = constrain(Tz, -TZ_MAX, TZ_MAX);                 // TZ_MAX pela reserva de yaw
```

Ponto de partida para Iz = 1,73 kg·m² (ωn ≈ 2,4 rad/s, ζ ≈ 0,85): **KP_T ≈ 10 N·m/rad, KD_T ≈ 7 N·m·s/rad**. Dá para medir Iz com um degrau de torque conhecido (inclinação de `rate_z` = Tz/Iz). Com a malha linear, a zona morta pode cair para ~0,5–1° e o piso deve ficar acima do travamento medido do ESC, não abaixo. Isso muda a unidade dos ganhos G3–G7: atualizar `GAIN_NAMES`, `GAIN_STEPS` e as descrições no supervisório junto.

### 🟡 SC-10 — Degraus de heading menores que 1° chegam corrompidos
**Onde:** `WHITELIST` (`main.cpp:1060`) contém `0`; `supervisorio_esfera.html` `stepHeading()`/`setHeadingAbsolute()`.

O supervisório manda o degrau relativo como `delta.toFixed(3)`. Um `0.250` vira: `0` imediato (zera `desired_yaw`!) + `.250` descartado. A esfera volta para o zero antigo em vez de andar 0,25°. FW-12 do `AUDITORIA.md`, ainda aberto e agora exercitado pelo slider do supervisório. **Correção:** tirar o `0` da whitelist e só tratar como imediato o caractere que vier sozinho na linha.

### ⚪ Heading — baixos
- **SC-18** — Cada setpoint numérico faz `yaw_gyro = 0`. O heading publicado perde a referência a cada degrau, e o supervisório precisa reconstruir o absoluto. Preferível manter o `yaw_gyro` contínuo e mandar setpoints absolutos (ou um comando `H<abs>` separado).
- **SC-19** — `wrap180` com `while` + setpoint sem validação: `inf`/`1e30` travam o loop (FW-02/FW-14). Usar `fmodf` e rejeitar valores não finitos.
- **SC-20** — Zona morta do yaw zera a integral sempre que entra. Hoje não há torque constante para a integral corrigir (cabo sem torção), então tudo bem — mas, com o SC-03 corrigido, o Ki pode até ir a zero.

---

## 3. Por que o `calib.json` "não serve"

### 🟠 WZ-01 — Nenhum caminho aplica a calibração
- **Firmware:** o comando `M` lê só o `yaw_sign` e **chama `init_motores_padrao()`**, que reescreve `kf`/`dz` dos três motores com o default. `tx`, `ty`, `kf_fwd`, `kf_rev` e `dz` enviados são descartados (`main.cpp:1021-1040`).
- **Supervisório:** "Carregar calib.json" (`loadCalibJson`) altera só o modelo do simulador da página; não envia nada ao STM.
- O `calib.json` atual é **exatamente o default** (kf = 18/470² = 8,148e-5, direções ideais): já foi sobrescrito.

### 🟠 WZ-02 — Direção de empuxo medida no referencial errado
**Onde:** `wizard.py:273-279`, `:302-310`.

1. O wizard gira `(wx, wy)` por `Rz(−yaw)` como se fosse mundo, mas desde o FW-03 o firmware publica no **corpo**. Como o `yaw_gyro` é zerado a cada setpoint numérico, o ângulo dessa rotação é arbitrário: as três direções saem giradas juntas por um ângulo qualquer.
2. Com o SC-01, o vetor sai **espelhado**: o motor b aparece a −120° do a (sentido horário) em vez de +120°.

Assinatura esperada no relatório do wizard: as três direções com ~120° entre si, mas na ordem trocada e/ou giradas. Se foi isso que vocês viram, está explicado.

### 🟠 WZ-03 — A força é calculada com o modelo de pêndulo simples
**Onde:** `force_of()` (`wizard.py:318-327`).

`F = m·L·ω²·T·mean(θ̇)/(1 − cos ωT)` vale para massa pontual. No pêndulo duplo (SC-02), nos 600 ms do pulso o giroscópio é dominado pela rotação da esfera em torno do pino. Pelo modelo, a razão "medido / esperado" vai de **−2,3 a +3,2** conforme `d` e `h`: o kf sairia de 0,1× a 10× o real e, com empuxo acima do CM, **com o sinal invertido**. Isso explica os "fit inconsistente → mantendo default" e os valores estranhos.

### 🟡 WZ-04 — O que fazer com a calibração
- **Direções:** são geometria. Fixe pelo desenho e confirme com o teste de eixos (SC-01) e com o `m`. Não vale a pena identificar.
- **Curva µs → N por motor e sentido:** meça **estaticamente**, com a esfera presa a um dinamômetro de mão ou uma balança de bagagem, a 4–5 níveis de PWM por sentido. São 20 minutos e dá `kf_fwd`, `kf_rev`, zona morta e travamento reais, sem modelo de pêndulo no meio. Confira também se o motor c (invertido) tem o lado eficiente no PWM negativo: nesse caso o `kf_fwd`/`kf_rev` dele é trocado em relação aos outros.
- **yaw_sign:** o wizard já mede bem (resposta de `rate_z`, sem pêndulo envolvido). A razão entre motores também pode vir da aceleração de yaw de um pulso (ψ̈ = f·R/Iz), que é limpa.
- **Firmware:** o `M` deve aplicar `kf_fwd`, `kf_rev`, `dz` (e um `latch_us`) com validação de faixa (`isfinite`, kf entre 0,3× e 3× o default, dz ≤ 150), e responder com os valores aplicados.
- **Supervisório:** o botão do `calib.json` deve mostrar os valores, pedir confirmação e mandar as três linhas `M` (≤ 60 caracteres cada — usar `.4g`).
- O wizard por pulsos pode voltar depois do SC-02, usando a velocidade do CM estimada em vez de `wx/wy`.

---

## 4. Supervisório (`supervisorio_esfera.html`) — pontos que tocam o controle

- O parser rejeita qualquer TEL que não tenha **exatamente** 16 campos (`p.length !== 16`). Antes de acrescentar campos ao firmware (recomendado: `F_cmd_x`, `F_cmd_y`, `Tz`, `v̂_x`, `v̂_y`, flags de repouso e de falha da MPU), troque para `>= 16` e leia os extras se existirem. Sem `F_cmd` na tela, um sinal trocado como o do SC-01 só aparece quando a esfera já está balançando.
- `p`/`y` são toggles e o estado local (`controller.xy_enabled`) não é sincronizado pelas flags da TEL (TW-03). Depois de uma queda de conexão, o botão pode mostrar "ligado" com a malha desligada (e vice-versa). Use as flags da TEL como verdade, ou crie comandos on/off absolutos.
- O fantasma é desenhado como se `sx/sy` fossem de mundo; são de corpo (SC-17) e hoje estão espelhados em X (SC-01).

---

## 5. Status dos itens do `AUDITORIA.md` ligados a sensoriamento e controle

| ID | Status | Observação |
|---|---|---|
| FW-01 | ✅ corrigido | `W` saiu da whitelist |
| FW-02 | ⚠️ parcial | `wrap180` certo, mas sem validação de entrada (SC-19) |
| FW-03 | ⚠️ parcial | ψ = 0 está certo, mas o problema real era o espelhamento (SC-01) e o termo de transporte ficou (SC-07) |
| FW-04 | ✅ corrigido | |
| FW-05 | ❌ aberto | SC-15 |
| FW-09 / FW-18 | ❌ pior | `calib_ok` agora é sempre `true` (SC-04) |
| FW-10 | ✅ corrigido | formatação inteira + flag no `platformio.ini` |
| FW-11 | ✅ ok | limite = 16,7 N, coerente com a reserva |
| FW-12 | ❌ aberto | agora atinge o supervisório (SC-10) |
| FW-15 | ⚠️ mudou | sincroniza o setpoint, mas sem tratar latência (SC-06) |
| FW-16 | ❌ aberto | SC-14 |
| FW-17 | ✅ corrigido | |
| FW-21 / SIM-03 | ❌ aberto | 400 + 300 > 450 (SC-03) |
| FW-22 | ⚠️ mudou | piso de 45 µs abaixo do travamento (SC-09) |
| FW-26 | ❌ aberto | SC-05 |
| FW-31 | ⚠️ mitigado | α = 0,9995 reduz o efeito; conceito do guia continua errado (SC-08) |

---

## Anexo — reproduzir as contas

```bash
python docs/analise_pendulo_duplo.py
```

Imprime o ζ dos dois modos para as três estratégias de realimentação e a razão do pulso do wizard. Depois de medir `d` (pino → CM), `h` (altura das hélices − altura do CM) e a frequência do bamboleio, troque os valores varridos pelos medidos.
