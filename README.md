# Esfera

## Descrição

Modelo-simulação física de um sistema "pêndulo simples" cujo controle está na
ponta do pêndulo, no centro de massa da esfera. O controle é feito em 3 graus
de liberdade com 3 motores bidirecionais + hélices tangentes ao raio da
esfera, fixos à esfera:

- **Heading (ψ):** orientação da esfera em torno do eixo vertical, rastreando
  uma direção desejada que pode ser atualizada em runtime;
- **X e Y (translação):** controle de velocidade com setpoint zero
  (amortecimento ativo) — a esfera pende próxima do vertical em repouso.

A simulação é a base para um futuro **digital twin** rodando junto com o
modelo real.

## Parâmetros do hardware (valores atuais no código)

| Parâmetro | Valor | Onde | Status |
|---|---|---|---|
| Massa total do corpo móvel | **12 kg** | `SphereParams.mass` | medido |
| Comprimento da corda (pino → CG) | **1 m (variável)** — sobrescreva com `--length` | `RopeParams.length` | medido |
| Tração máxima por hélice | **18 N** (reversível, ±) | `ActuatorParams.max_thrust` | aprox. |
| Raio da esfera (= braço das hélices) | **0,35 m** | `SphereParams.radius` | medido |
| Momento de inércia em ψ | (2/5)·m·r² = 0,588 kg·m² | `SphereParams.inertia_z` | estimativa a calibrar* |
| Rigidez torcional da corda | 0,05 N·m/rad | `RopeParams.torsional_stiffness` | estimativa a calibrar |
| Amortecimento torcional da corda | 0,01 N·m·s/rad | `RopeParams.torsional_damping` | estimativa a calibrar |

\* A estrutura que segura os motores está majoritariamente **acima do CG**
(perto do eixo de rotação do heading) e os motores/hélices, no equador, são
leves — logo não dominam `I_ψ`. A fórmula de esfera maciça é um chute de
trabalho; ao medir o valor real (oscilação livre no heading), passe por
`--inertia` — os ganhos do loop de taxa são rescalados automaticamente
(regra kp/I, ver `gains_for_inertia`).

**Autoridade de controle resultante:** envelope exato de força horizontal =
1,5·f_max = 27 N ≈ 0,23 g — o controle amortece velocidades, mas em
inclinações grandes (≳ 13°, onde m·g·sin θ ≳ 27 N) os atuadores saturam
transitoriamente; o anti-windup cobre isso. Torque de heading máximo =
3·R·f_max = 18,9 N·m.

## Interface de hardware (motores)

- Motores **a**, **b**, **c**, defasados de 120°; a hélice **a empurra no
  sentido +X do sensor** (o sensor está montado no pino, no topo, e gira com
  a esfera — o referencial do corpo na simulação É o referencial do sensor).
- PWM bidirecional: **1500 µs = parado, 2000 µs = full speed em um sentido,
  1000 µs = full speed no outro**. Conversão N → µs em
  `ThrusterCluster.thrust_to_pwm_us` (linear, clipada em [1000, 2000]) — é a
  mesma conversão que o digital twin usará no lado real. O PWM de cada ESC
  aparece no painel de séries temporais, no HUD da animação e no
  `SimulationLog.pwm`.
- Alocação de força/torque: `f_i = (2/3)(t_i·F_b) + τ/(3R)`, com saturação
  por hélice; força comandada no referencial do mundo é girada para o corpo
  (que gira com o heading) — mesmo caminho de software do sistema real.

## Modo interativo (tempo real)

```bash
python main.py --interactive                    # cenário default como estado inicial
python main.py --interactive --scenario heading # outra condição inicial
```

Simulação ao vivo (dt = 0,01 s ritmado pelo relógio de parede) numa única
janela, em zonas separadas:

- **faixa de status** (topo): tempo, estado dos loops **XY** e **HEADING**
  (verde = ligado, vermelho = desligado) e indicação de PAUSADO;
- **cena 3D** (esquerda): corda, esfera, setas de empuxo por hélice, seta
  verde = força externa; arraste com o mouse para girar a vista;
- **painel 2×2** (direita): inclinação, velocidade, heading vs desejado e
  **PWM dos ESCs em µs**;
- **rodapé**: bloco CONTROLES (teclas) e bloco GANHOS (valores vigentes e
  item sendo ajustado).

| Tecla | Ação |
|---|---|
| setas | força externa contínua em ±X/±Y do mundo, enquanto seguras |
| Q / E | torque externo contínuo ±0,05 N·m no heading |
| 1 | liga/desliga **controle pendular (X/Y)** |
| 2 | liga/desliga **controle de heading** |
| Z / X | setpoint de heading −15° / +15° |
| G | seleciona próximo item ajustável (força ext. → vel_kp → … → head_rate_ki) |
| + / − | ajusta o item selecionado (passo = 10% do ganho default) |
| R | reset à condição inicial do cenário (integrais zeradas) |
| P | pausa / retoma |
| Esc | sai |

Desligar um loop zera o comando correspondente: com XY desligado a esfera
pendula livremente enquanto o heading continua controlado (e vice-versa);
com os dois desligados os ESCs ficam em 1500 µs. Ao religar, as integrais
dos PIDs são zeradas para não injetar degrau acumulado.

### Forças externas em cenários (rajadas)

Cenários batch suportam rajadas reprodutíveis via `force_events` — tuplas
`(t_início, t_fim, fx, fy)` em N no referencial do mundo, aplicadas
**diretamente na planta** (o controlador só as sente pelo estado, como no
sistema real):

```bash
python main.py --scenario gust   # rajadas de 20 N e −15/−10 N + mudança de heading
```

## Modelo físico

**Planta não-linear (`SphericalPendulumPlant`)** — pêndulo esférico completo,
válido para grandes ângulos. O centro de massa é integrado em coordenadas
cartesianas com a restrição de corda rígida `|r| = L` imposta pelo método
RATTLE (integrador simplético para sistemas com restrição): o erro de energia
fica limitado (~0,1%) e não dissipa artificialmente — importante para não
"ajudar" o controlador. O heading segue `I·ψ̈ = τ − k_t·ψ − b_t·ψ̇` (mola e
amortecimento torcionais da corda, referenciados ao pino).

**Planta linearizada (`LinearizedPendulumPlant`)** — para pequenos ângulos,
dois pêndulos lineares desacoplados em X e Y (`ẍ = −(g/L)x + Fx/m`) + o mesmo
heading. Serve para sintonia rápida dos controladores.

**Controlador (`controller.py`)** — três loops independentes:

- X e Y: PIDs sobre a velocidade (setpoint 0) com anti-windup por clamping;
- Heading: cascata ângulo → taxa (limitada) → torque, com erro envolto em
  [−π, π] e **feedforward** do torque torcional conhecido da corda.

Os ganhos são sintonizados para os parâmetros padrão (12 kg, 0,048 kg·m²):
a regra do rescale é manter kp/m e kp/I — ao mudar massa/inércia, escale os
ganhos de força/torque proporcionalmente.

### Hipóteses e limitações

- A esfera é ponto material para a translação; o empuxo age no CG efetivo
  (L mede pino → CG).
- As hélices empurram sempre no plano horizontal do mundo (o efeito da
  inclinação da esfera sobre a direção do empuxo é de 2ª ordem em θ e fica
  para a próxima iteração do digital twin).
- Sem arrasto aerodinâmico nem dinâmica de motor (atraso de 1ª ordem) —
  pontos naturais de refinamento.
- Corda sem massa e axialmente rígida.
- O controle translacional zera **velocidade** (conforme requisito): a
  posição final fica próxima do vertical, mas não exatamente nele (offset
  típico < 1°) — a posição final depende da trajetória, não há servo de
  posição.

## Estrutura

```
simulation.py     plantas (não-linear RATTLE + linearizada) e parâmetros
actuators.py      cluster de 3 hélices: rótulos a/b/c, alocação, PWM, saturação
pid_controller.py PID com anti-windup, limite de saída e wrap angular
controller.py     3 loops de controle + feedforward torcional + toggles
scenarios.py      cenários (condição inicial, agenda de heading, rajadas)
main.py           CLI, loop de simulação, métricas, comparação de plantas
interactive.py    modo interativo ao vivo (teclado, HUD, painel PWM, ganhos)
animation.py      animação 3D + painel de séries temporais (batch)
tests/            testes de sanidade física e de malha fechada
```

## Pré-requisitos

- Python 3.10+

## Instalação

```bash
python -m venv .venv
.venv\Scripts\activate        # Windows
# source .venv/bin/activate   # Linux/Mac
pip install -r requirements.txt
```

## Execução

```bash
python main.py                          # cenário padrão, animação 3D
python main.py --interactive            # modo interativo ao vivo (teclado)
python main.py --scenario heading       # sequência de mudanças de heading
python main.py --scenario disturbance   # perturbação translacional forte
python main.py --scenario gust          # rajadas de força externa
python main.py --plant linear           # planta linearizada
python main.py --compare                # linear vs não-linear (métricas + gráfico)
python main.py --no-anim                # só console/métricas (rápido, p/ sintonia)
python main.py --length 2.0             # corda de 2 m (L é variável)
python main.py --mass 12 --thrust 18    # sobrescrever massa/tração
python main.py --inertia 0.4            # I_ψ medido (ganhos rescalados sozinhos)
python main.py --save out/anim.gif      # exporta a animação em GIF
```

Testes:

```bash
python -m pytest tests/ -q
```

## Comportamento esperado (parâmetros padrão: 12 kg, r = 0,35 m, L = 1 m, 18 N/hélice)

| Métrica | Valor |
|---|---|
| Acomodação de velocidade (15° + 0,3 m/s iniciais) | ~1,2 s |
| Acomodação de cada degrau de heading (90°, −45°, 180°) | ~1,7–2,0 s |
| Sobressinal de heading | ~0° |
| Rajada de 20 N por 2 s | inclina ~10°, recupera em ~1 s |
| Perturbação forte (20° + 0,64 m/s) | ~2,1 s (satura transitoriamente) |
| Desvio de energia em oscilação livre (20°, 20 s) | < 0,5% (sem drift) |
