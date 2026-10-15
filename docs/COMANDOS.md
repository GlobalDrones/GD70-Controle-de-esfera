# Comandos — tabela única

Revisado em 14/10/2026. **Regra:** cada ação do STM tem **uma** letra, e essa letra é a mesma em todo lugar: na serial do STM, na janela da Rasp, no supervisório, no twin e no rádio. Não existe tradução de tecla. Antes, por exemplo, `i` na Rasp mandava `w`, `p` na Rasp mandava `k` e `1` no twin mandava `p`.

As teclas que só existem num programa (visão, SGBM, simulador) usam letras ou símbolos que **não** são comando do STM.

`tests/test_comandos.py` lê os fontes e falha se alguém reintroduzir um alias ou uma tecla divergente.

## 1. STM (firmware) — o que a serial aceita

Todos os programas do projeto mandam cada comando terminado em `\n`. Os comandos marcados como **imediatos** também funcionam num monitor serial sem newline: executam no primeiro byte.

| Comando | Ação | Imediato | Observações |
|---|---|---|---|
| `k` (ou `K`) | **PARADA GERAL**: malhas off, ESCs em 1500 µs, polia e pinça paradas, pulso do wizard cancelado | sim | |
| `w` | pinça sobe | sim | para sozinha no fim de curso |
| `s` | pinça desce | sim | |
| `x` | pinça para | sim | |
| `a` | polia, sentido A | sim | **watchdog de 250 ms**: para sozinha se o `a` não for repetido. O rádio repete a cada 100 ms; no teclado, cada toque é um "jog" de 250 ms |
| `d` | polia, sentido D | sim | idem |
| `c` | polia para | sim | |
| `e` | lê o encoder AS5600 | sim | |
| `p` | liga/desliga o **anti-balanço** | não | recusado sem calibração válida da MPU. Antes também aceitava `b`, `B` e `P` |
| `y` | liga/desliga o **heading** | não | recusado sem calibração válida da MPU |
| `z` | calibra a MPU (~9 s, esfera **parada**) | não | faz parada geral antes; **recusa** se houver movimento |
| `Z` | calibra a MPU **forçando** | não | aceita mesmo com movimento (bias pode sair errado) |
| `t` | calcula L pelo período medido em balanço livre | não | |
| `0` | zera o setpoint de heading | não | só sozinho na linha. Antes era imediato e corrompia `0.5` |
| `m` | teste dos 3 motores (±200 µs, 2 s cada) | não | **bloqueia o STM ~18 s, sem `k`**. Só pelo supervisório, com confirmação |
| `<número>` | degrau **relativo** de heading [graus] | — | ex.: `15`, `-30`, `0.5`; aceita \|x\| ≤ 360 |
| `L<m>` | comprimento do cabo [m] | — | 0,5 a 6,0 |
| `G<i> <v>` | ajusta o ganho `i` | — | faixa validada por ganho (seção 3) |
| `R` | devolve os ganhos (`GAIN,i,nome,valor`) | — | |
| `V<b> <D> [idade_ms]` | visão: bearing [°], distância [m], idade do quadro (opcional) | — | o setpoint muda **uma vez** por V recebido, compensando a latência |
| `W<m> <us> <ms>` | pulso do wizard | — | |
| `M<m> <tx> <ty> <ys> <kf> <kr> <dz>` | calibração do motor | — | `tx`/`ty` são **ignorados** (direção = geometria); `kf ≤ 0` ou `dz < 0` = manter o atual; responde com os valores aplicados |

**Removidos:**
- `b`, `B`, `P` (aliases do anti-balanço);
- as maiúsculas `W S X A D C E Y T`;
- o espaço, que parava a pinça.

## 2. Teclas por programa

| Tecla | Rasp (janela da visão) | Supervisório | Twin / interativo Python |
|---|---|---|---|
| `k` | → `k` | → `k` (sempre, também no simulador) | → `k` + para o sim |
| `p` | → `p` | sim: liga/desliga XY · real: → `p` | → `p` + sim |
| `y` | → `y` | sim: heading · real: → `y` | → `y` + sim |
| `0` | → `0` | sim: ψ = 0 · real: → `0` | — |
| `[` / `]` | — | ψ_des −15° / +15° (real: degrau `-15.000` / `15.000`) | idem |
| `w s x` · `a d c` · `e` | → mesma letra | real: → mesma letra | — |
| `z` | → `z` | real: → `z` | twin: → `z` |
| `t` | → `t` | real: → `t` | — |
| `R` | — | → `R` (ler ganhos) | — |
| `g` · `+` · `-` | — | seleciona / ajusta ganho (real: envia `G`) | idem |
| `v` | liga/desliga o envio de V | — | — |
| `f` | filtro WLS liga/desliga (era `w`) | — | — |
| `1`–`4` | escala da imagem | — | — |
| `+` / `-` | número de disparidades | (ganho) | (ganho) |
| `>` / `<` | min_disp + / − (era `A` / `S`) | — | — |
| `n` · `r` | bloco SGBM · reset SGBM | `r`: reset do simulador | `r`: reset do sim |
| `!` | **reset físico** do STM via GPIO18 (era `R`) | — | — |
| setas | — | força externa (sim) | idem |
| `,` / `.` | — | torque externo (sim) (era `q` / `e`) | idem |
| espaço | — | pausa (sim) (era `p`) | idem |
| `q` / Esc | sai | — | Esc sai |

Regra do supervisório: em modo **simulador**, as teclas e botões mexem só no simulador; em modo **real**, só mandam o comando, e o estado ligado/desligado vem das flags da telemetria. Antes as duas coisas aconteciam sempre: uma tecla no simulador podia ligar a malha da esfera real se houvesse conexão aberta, e os toggles podiam ficar trocados.

## 3. Ganhos (`G<i> <v>`, `R`)

| i | Nome | Unidade | Default | Faixa aceita |
|---|---|---|---|---|
| 0 | `vel_kp` | N/(m/s) | 18 | 0–80 |
| 1 | `vel_ki` | — | 0 | 0–20 |
| 2 | `vel_kd` | — | 0 | 0–20 |
| 3 | `yaw_kp` | N·m/rad | 10 | 0–60 |
| 4 | `yaw_ki` | N·m/(rad·s) | 0 | 0–20 |
| 5 | `yaw_kd` | N·m·s/rad | 7 | 0–40 |
| 6 | `yaw_tz_max` | N·m | 8 | 0–20 |
| 7 | `yaw_int_max` | rad·s | 0,5 | 0–5 |
| 8 | `vel_mode` | 0 = L·gyro, 1 = estimador do CG | 0 | 0–1 |
| 9 | `d_pino_cg` | m | 0,10 | 0–1 |

Os índices 3–7 mudaram de significado em 14/10/2026: o heading agora é controlado em torque. O twin Python (`twin/window.py`) ainda manda os ganhos do controlador Python nesses índices. O firmware **recusa** esses valores por estarem fora da faixa, então use o supervisório para ajustar os ganhos do hardware.

## 4. Rádio (Rasp, `Raspberry/pwm_input.py`)

| Canal | Comando | Ao soltar |
|---|---|---|
| CH12 | `w` | `x` (novo: antes corria até o fim de curso) |
| CH13 | `s` | `x` (novo) |
| CH14 | `a`, repetido a cada 100 ms (watchdog da polia) | `c` quando CH14 e CH15 estão soltos |
| CH15 | `d`, repetido a cada 100 ms | `c` |
| CH16 (opcional) | `p` (era `b`) | — |

## 5. Telemetria (TEL v3)

```
TEL,t_ms,yaw,bearing,yaw_des,rate_z,sx,sy,wx,wy,pwm_a,pwm_b,pwm_c,L,D,flags,F_cmd_x,F_cmd_y,Tz,vhat_x,vhat_y
```

- `sx`, `sy`, `wx`, `wy` são do **corpo**: `sx > 0` = esfera deslocada para +X, que é a direção em que o motor a empurra.
- `F_cmd` está em N (corpo), `Tz` em N·m e `vhat` em m/s (corpo).
- `flags`: 1 yaw · 2 anti-balanço · 4 calib OK · 8 visão · 16 wizard · 32 MPU em falha · 64 repouso no prumo · 128 yaw no alvo · 256 velocidade pelo estimador.
- O SimSTM continua mandando 16 campos; os parsers aceitam 16 ou 21.
