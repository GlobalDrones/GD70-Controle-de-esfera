# Bring-up do Sistema Esfera — guia passo a passo

> **Atualização 14/10/2026:** o firmware mudou no sensoriamento e no controle
> ([AUDITORIA_CONTROLE.md](../AUDITORIA_CONTROLE.md)) e as teclas foram unificadas
> ([COMANDOS.md](COMANDOS.md): a tecla é a própria letra do comando do STM em todo
> programa). **Refaça a Etapa 3 (teste de eixos) antes de ligar qualquer malha** — o
> anti-balanço antigo tinha um eixo com sinal invertido, e o novo depende de a montagem
> da MPU estar declarada certa em `IMU_MOUNT_DEG`.

Este documento descreve como ligar e validar o sistema Esfera pela primeira
vez, em etapas separadas. A regra principal é simples: **os motores só são
ligados na Etapa 5**. Antes disso, tudo o que é testado — comunicação,
sensores, visão, comandos — acontece sem nenhum risco de movimento.

Cada etapa tem quatro partes:

- **Objetivo** — o que aquela etapa entrega para a seguinte;
- **Passo a passo** — o que fazer, na ordem;
- **Como saber que deu certo** — o que você deve ver na tela;
- **O que fazer se der errado** — os problemas mais comuns e o remédio.

Não pule etapas. Cada uma depende da anterior funcionar.

---

## Visão geral das etapas

| Etapa | O que é testado | Bateria dos ESCs | Status de Bancada |
|---|---|---|---|
| 0 | Simulador no computador (ensaio geral) | desligada | Concluído |
| 1 | Compilar e gravar o firmware no STM | desligada | Concluído (v2 consolidada) |
| 2 | Comunicação com o STM (comandos e telemetria) | desligada | Concluído (TEL 25 Hz) |
| 3 | Sensores de inclinação e heading | desligada | Concluído (DLPF 20 Hz + calib 'z') |
| 4 | Raspberry Pi: visão, ponte de rede e rádio | desligada | Concluído (TCP 9999 + VNC 10 FPS) |
| 5 | Primeiro giro dos motores (sem hélices) | **ligada** | Concluído (Motor C invertido identificado) |
| 6 | Calibração dos motores com o wizard | ligada | Concluído (Parada manual executada) |
| 7 | Controle em malha fechada | ligada | **Próximo passo na bancada** |
| 8 | Controle usando a visão | ligada | Pendente de malha fechada estável |

---

## Antes de começar

### A regra de segurança que vale o documento inteiro

Em qualquer momento, você pode parar tudo de duas formas:

1. **Tecla `k`** (no twin, ou o caractere `k` na serial): desliga os dois
   loops de controle, coloca os três ESCs em 1500 µs (parado) e para a polia
   e a pinça.
2. **Desconectar a bateria dos ESCs.** Nos primeiros testes com motores,
   deixe a bateria ao alcance da mão.

Além disso, o firmware já nasce seguro: **ao ligar, os dois loops de
controle estão desligados**. Eles só passam a atuar quando você ligar
(comando `y` para o heading, comando `p` para o anti-balanço).

### O truque que permite testar quase tudo sem girar nada

O STM não sabe se os ESCs estão com bateria ou não: ele calcula os comandos
e escreve os valores de PWM de todo modo — e esses valores aparecem na
telemetria (a linha `TEL,...`, que traz `pwm_a`, `pwm_b`, `pwm_c`).
Isso significa que dá para acompanhar o controle inteiro funcionando,
número por número, sem que nada se mexa. É isso que torna as Etapas 2 a 4
completamente seguras.

### O que você vai precisar

- Um computador com Python 3.10 ou mais novo, com o repositório clonado e
  as dependências instaladas (`pip install -r requirements.txt`);
- Um cabo USB para ligar o STM ao computador na hora de gravar o firmware;
- A Raspberry Pi com acesso à rede (a partir da Etapa 4);
- A partir da Etapa 5: a bateria dos ESCs — **só ligue quando chegar lá**.

### Onde o computador precisa estar (e quando ele se desconecta do STM)

A gravação do firmware (Etapa 1) acontece na mesa do computador, com o STM
ligado por USB. Terminando a gravação, você desconecta o cabo, instala o
STM na esfera e **ele não precisa mais voltar perto do computador**: a
partir daí a placa é alimentada pela bateria da esfera, como vocês já
operam hoje.

O que as etapas seguintes precisam não é do cabo USB, e sim de um **caminho
de dados** até o STM — e existem dois:

1. **USB direto no computador** — só faz sentido se a esfera estiver ao
   alcance de um cabo (3 a 5 m; mais que isso exige cabo ativo). Um detalhe
   útil: a própria USB alimenta a placa e os sensores, então nas Etapas 2 e
   3 a bateria da esfera pode continuar desligada;
2. **Pela Raspberry Pi, pela rede** — o caminho para quando a esfera está
   longe do computador. Neste caso, **faça a Etapa 4 antes das Etapas 2 e
   3**: com a Rasp ligada à USB do STM (o mesmo arranjo elétrico de hoje —
   placa na bateria da esfera, USB só de dados), o twin alcança o STM pela
   rede com `python -m twin --tcp <ip-da-rasp>`, não importa a distância.

Em resumo: a ordem Etapa 2 → 3 → 4 pode ser trocada por Etapa 4 → 2 → 3
quando a esfera ficar longe do PC. As etapas funcionam exatamente igual
nos dois caminhos — só muda como o twin se conecta (`--serial` ou `--tcp`).

---

## Etapa 0 — Ensaio geral no simulador

**Objetivo:** aprender como tudo funciona sem encostar em nenhum hardware.
O simulador contém a mesma física, o mesmo controlador e o mesmo protocolo
do sistema real. Dez minutos aqui economizam tempo de bancada depois.

**Passo a passo:**

```bash
python -m pytest tests/ -q      # roda os 49 testes automatizados
python -m twin --sim            # abre o twin conectado a um STM simulado
```

Com o twin aberto, experimente sem pressa:

- As **setas** do teclado empurram a esfera simulada (força externa);
- **p** e **y** ligam e desligam os loops de amortecimento e de heading;
- **[** e **]** pedem que a esfera gire −15° ou +15°;
- **G** seguido de **+** ou **−** seleciona e ajusta um ganho;
- **K** é a parada geral.

Observe no painel de PWM como os três ESCs se movem juntos em uma manobra
de heading, e como eles reagem a um empurrão quando o amortecimento está
ligado. Esse é o comportamento que vamos procurar no hardware.

Se quiser, rode também `python wizard.py --sim` para ver como será a
calibração dos motores na Etapa 6.

**Como saber que deu certo:** os 49 testes passam e você se sente
confortável com as teclas do twin.

---

## Etapa 1 — Compilar e gravar o firmware

**Objetivo:** gravar o firmware v2 (`firmware/main.cpp`) no STM. Nada do
que vem depois funciona sem esta etapa.

**Passo a passo:**

1. Instale o PlatformIO (a extensão do VS Code é o caminho mais fácil). O
   repositório já traz a configuração pronta em `firmware/platformio.ini` —
   ela inclui as flags que fazem a USB do STM virar porta serial e habilita
   a Serial2, além da biblioteca do encoder AS5600;
2. Abra a pasta `firmware/` como projeto e mande compilar e gravar
   (`pio run -t upload`). A gravação padrão usa o gravador **ST-Link**
   (4 fios: 3V3, SWDIO, SWCLK e GND); o próprio arquivo `platformio.ini`
   traz as instruções para trocar para gravação pela USB (DFU), se
   preferir gravar sem gravador;
3. Depois de gravado, ligue o STM ao computador por USB, abra um monitor
   serial na porta que aparecer, com velocidade 115200.

Ponto de atenção: este firmware foi escrito sem um compilador STM à mão,
então **é normal que a primeira compilação apresente erros** (uma vírgula,
um tipo, o nome de uma função). Não tente consertar no escuro: copie a
saída de erro completa do compilador e envie para a equipe de software —
a correção costuma levar minutos.

**Como saber que deu certo:** ao ligar, o STM imprime o banner de boot na
serial, o LED pisca durante a calibração da MPU e, no fim, os loops aparecem
desligados.

**O que fazer se der errado:**

- *Erros de compilação* — envie a saída do compilador;
- *A porta serial não aparece* — confira o cabo (alguns só carregam
  energia) e o gerenciador de dispositivos do sistema operacional.

---

## Etapa 2 — Primeira conversa com o STM (ESCs sem bateria)

**Objetivo:** confirmar que os comandos chegam ao STM e que a telemetria
volta ao computador — a "conversa" completa, sem nenhum motor ligado.
Mantenha a bateria dos ESCs desconectada durante toda esta etapa.

**Como conectar:** se a esfera estiver perto do computador, use USB direto
(o comando abaixo). Se estiver longe, suba primeiro a Etapa 4 (Rasp) e volte
aqui usando `python -m twin --tcp <ip-da-rasp>` — o resto da etapa é igual.

**Passo a passo:**

```bash
python -m twin --serial COM7    # troque COM7 pela porta do seu STM
```

Com o twin aberto, verifique cada item da lista abaixo. Em todos eles, a
esfera não deve se mover nem um milímetro — o que muda são apenas os
números de PWM na telemetria.

1. A telemetria chega (a faixa no topo mostra o tempo correndo, e o painel
   de PWM mostra três linhas paradas em 1500 µs);
2. Calibre a MPU primeiro (item 6): desde 14/10/2026 o STM **recusa** ligar
   as malhas sem uma calibração válida. Depois pressione **y** (liga o loop
   de heading) e **p** (liga o amortecimento): o quadro de status deve
   mostrar `HEADING: LIGADO` e `XY: LIGADO`;
3. Pressione **]** (girar +15°): os PWMs de a e b vão para um lado e o de c
   para o outro (c é invertido) e voltam a 1500 quando a manobra termina. A
   telemetria nova mostra também o torque comandado (`Tz`, em N·m);
4. Pressione **G** e depois **+**: o valor do ganho selecionado aumenta
   (confira na linha `GANHOS` do rodapé). Com **R** o STM devolve os 8
   ganhos;
5. Pressione **K**: parada geral — os dois loops desligam e os PWMs voltam
   a 1500;
6. Calibre a MPU (comando `z`, **~9 s**) — com a esfera pendurada, parada,
   **sem balanço residual** e ninguém encostando. Onde enviar: tecla **z** no
   supervisório, na janela da Rasp ou no twin, ou `z` + Enter em um
   `nc localhost 9999` aberto na Rasp. A janela longa (≈2 períodos do
   pêndulo) é de propósito: com a antiga (~1,4 s) um balanço de 1° deixava
   ~1 °/s de erro no gyro, que virava ~1,6 N de empurrão constante. Para
   testar a proteção, repita **enquanto alguém mexe na esfera**: o STM deve
   responder `calibracao RECUSADA` e manter as malhas bloqueadas. `Z`
   (maiúsculo) força a calibração mesmo assim — só em último caso.

**Como saber que deu certo:** todos os seis itens acima se comportam como
descrito, sem nenhum movimento físico.

**O que fazer se der errado:**

- *O twin não conecta* — confira o nome da porta (no Windows, gerenciador
  de dispositivos; no Linux, `/dev/ttyUSB*` ou `/dev/ttyACM*`);
- *Conecta, mas não chega telemetria* — o STM pode estar com o firmware
  antigo: o novo envia linhas começando com `TEL,`. Confira o banner de
  boot na serial.

---

## Etapa 3 — Conferir os sensores de inclinação e heading

**Objetivo:** garantir que o sistema entende corretamente para onde a
esfera está inclinada e para onde ela está girada. Se algum eixo estiver
trocado ou com sinal invertido, o controle da Etapa 7 empurraria para o
lado errado — muito melhor descobrir agora, sem motores.

**Passo a passo** (tudo observando os painéis do twin, ainda sem bateria):

1. **Teste de eixos (OBRIGATÓRIO — define o sinal do anti-balanço).**
   Convenção: **+X do corpo** = direção em que o **motor a empurra a
   esfera**; **+Y** = 90° anti-horário a partir de +X, visto de cima. Com a
   esfera pendurada e parada, empurre-a **devagar, com a mão**:
   - na direção **+X** → `wx` fica **positivo** durante o empurrão e `wy` ≈ 0;
     depois, parada deslocada, `sx` positivo;
   - na direção **+Y** → `wy` positivo e `wx` ≈ 0; `sy` positivo.

   **Montagem informada em 15/10/2026:** o BF empurra para o **−Y da MPU**
   (sentido de maior eficiência da hélice) → o firmware já vem com
   `IMU_MOUNT_DEG = 90`. O teste acima confirma. Se o lado eficiente do BF
   for o de **PWM < 1500** (confira no teste `m`: em qual meio-ciclo o
   vento é mais forte), então PWM > 1500 empurra para +Y da MPU → use
   `IMU_MOUNT_DEG = -90` e troque `kf_fwd`/`kf_rev` do motor a (comando `M`).

   Se não bater, ajuste a montagem no topo de `firmware/main.cpp` e regrave
   (as regras abaixo partem de `IMU_MOUNT_DEG = 0`; com 90 já gravado, some
   +90 ao valor que a regra der):
   - empurrão +X aparece como `wy` **negativo** (e +Y como `wx` positivo) →
     `IMU_MOUNT_DEG = 90`;
   - +X aparece como `wy` **positivo** (e +Y como `wx` negativo) →
     `IMU_MOUNT_DEG = -90`;
   - +X aparece como `wx` negativo **e** +Y como `wy` negativo →
     `IMU_MOUNT_DEG = 180`;
   - **só um** dos dois sai invertido (ex.: +X → `wx` negativo, mas +Y →
     `wy` positivo): é um espelhamento, que nenhuma rotação explica — a MPU
     está de cabeça para baixo → `IMU_Z_UP = false`, regrave e repita o
     teste (pode ainda precisar de um ângulo).

   Use o empurrão lento à mão, **não** um pulso de motor: com a esfera presa
   por pino, o gyro pode responder ao contrário nos primeiros instantes de
   um pulso. **Não ligue o `p` antes deste teste passar.**
2. **Inclinação e rotação.** Incline a esfera uns 15° e, mantendo a
   inclinação, gire-a devagar em torno do eixo vertical. Desde 14/10/2026
   `sx/sy` são do **corpo**: as curvas **devem** girar junto (a inclinação
   "anda" de θx para θy conforme o corpo gira), suavemente e sem saltos.
   (A instrução antiga — "não podem girar" — pedia o comportamento de
   mundo, que não é o que a fusão calcula; `COMP_YAW_SIGN` não existe mais.)
3. **Deriva do heading.** Deixe a esfera completamente parada por um minuto
   e observe a curva de heading. Com a calibração nova (9 s) a deriva deve
   ficar bem abaixo de 1°/min. Rotação lenta **real** agora aparece (a zona
   morta de integração caiu de 0,35 para 0,05 °/s); se o heading andar com
   a esfera parada, recalibre (`z`);
4. **Comprimento do cabo.** Afaste a esfera e solte-a para balançar
   livremente, com o amortecimento desligado. Depois de alguns balanços,
   envie o comando `t`: o STM mede o período de oscilação e calcula o
   comprimento do cabo. Compare com o valor real — devem bater em alguns
   centímetros.

**Como saber que deu certo:** inclinações coerentes, θx/θy imunes à rotação
do corpo, deriva pequena e `t` medindo o cabo corretamente.

---

## Etapa 4 — Raspberry Pi: visão, ponte de rede e rádio

**Objetivo:** colocar a Rasp no circuito — o aplicativo de visão dela vira
o dono da serial do STM e serve a ponte de rede para o twin. Continuamos
sem bateria nos ESCs.

**Passo a passo:**

Na Raspberry Pi (por SSH ou no teclado dela):

```bash
git clone https://github.com/iNeewbie/esfera.git && cd esfera

# o Raspberry Pi OS novo bloqueia pip direto no sistema ("externally managed");
# a solução é um ambiente virtual — estes 3 comandos rodam UMA única vez:
python3 -m venv .venv --system-site-packages   # enxerga os backends de GPIO do sistema
source .venv/bin/activate
pip install -r Raspberry/requirements.txt

# para rodar (e em todo terminal novo: source .venv/bin/activate antes):
python Raspberry/main.py
```

**Atenção ao rodar por SSH:** o aplicativo de visão abre uma janela com as
imagens — por SSH puro, sem tela associada, ele aborta com um erro do Qt
(`could not connect to display`). Duas saídas: rodar no terminal da própria
Rasp (teclado e monitor ligados nela), ou continuar no SSH mandando a janela
para o monitor da Rasp:

```bash
DISPLAY=:0 python Raspberry/main.py
```

(Não use `ssh -X` para isso — vídeo por túnel X fica lento demais.)

No seu computador:

```bash
python -m twin --tcp <endereço-IP-da-rasp>
```

O aplicativo de visão abre uma janela própria na tela da Rasp (as imagens
das câmeras, a detecção da linha e um resumo da telemetria). O twin, no seu
computador, passa a enxergar o STM através da Rasp, pela rede.

Agora, prepare um alvo para a visão: **estique uma corda grossa no chão,
abaixo da esfera**, fazendo o papel do cabo da linha de transmissão.
Então:

1. A janela da visão deve detectar a corda e desenhar uma linha verde
   sobre ela, com o valor do ângulo (bearing) e da distância;
2. Meça a distância real da esfera até a corda com uma fita métrica e
   compare com o valor de D mostrado;
3. Incline ou mova a corda: o bearing deve mudar no sentido esperado;
4. A tecla **v** (na janela da visão) liga e desliga o envio da visão ao
   STM. As teclas de comando da janela da Rasp são as **mesmas letras** do
   STM (`k` parada, `p`/`y` malhas, `w/s/x` pinça, `a/d/c` polia, `z`, `t`,
   `0`, `e`); as da visão/SGBM mudaram para não colidir: `f` = WLS (era
   `w`), `>`/`<` = min_disp (eram `A`/`S`), `!` = reset físico do STM (era
   `R`). Tabela em [COMANDOS.md](COMANDOS.md). Como o loop de heading continua desligado, nada acontece além do
   STM passar a receber os valores — seguro;
5. Se houver rádio ligado à Rasp, teste os canais: eles comandam a pinça e
   a polia. **Atenção: este é o primeiro movimento físico do bring-up** —
   se preferir, deixe para depois da Etapa 5;
6. Na Rasp, a parada geral é a tecla **k** (como em todo lugar). A tecla
   **p** liga/desliga o anti-balanço, igual ao comando do STM. (Até
   14/10/2026, `p` na Rasp era a parada e `k` descia a pinça.)

**Como saber que deu certo:** visão detectando a corda com bearing e
distância plausíveis; twin enxergando o STM pela rede; comandos fluindo nos
dois sentidos.

**O que fazer se der errado:**

- *O twin não conecta pela rede* — teste primeiro o ping até a Rasp; se a
  rede for restritiva, use o túnel SSH descrito no README
  (`ssh -L 9999:localhost:9999`);
- *A visão não acha a corda* — ajuste a iluminação e a escala (teclas 1–4
  na janela da visão) e confira o filtro binário no painel do meio.

---

## Etapa 5 — Primeiro giro dos motores (hélices removidas)

**Objetivo:** ligar a bateria dos ESCs pela primeira vez e conferir qual
motor é qual, e para onde cada um gira — **sem hélices**.

Antes de ligar a bateria: **remova as três hélices**, prenda a esfera de
forma que ela não balance livremente (um apoio firme serve) e deixe a tecla
`k` ao alcance da mão.

**Passo a passo:**

1. Ligue a bateria dos ESCs;
2. Envie o comando `m` (na serial ou pelo terminal da Rasp). O STM roda a
   sequência de identificação: cada motor gira por 2 segundos em um
   sentido, para, e gira por 2 segundos no outro. A sequência dura uns 18
   segundos e desliga os loops de controle durante a execução;
3. Anote, para cada motor: ele é o `a`, o `b` ou o `c`? Para que lado ele
   sopra quando o comando é positivo? O teste anuncia qual motor está
   girando a cada momento (os nomes seguem os pinos: a = PA0, b = PA15,
   c = PA1).

**Como saber que deu certo:** os três motores giram, cada um em sua vez,
nos dois sentidos, e a correspondência com a/b/c confere com a fiação.
Nenhum motor "trava" com os passos de ±120 µs do teste — eles já nascem
fora da faixa de travamento.

**O que fazer se der errado:** um motor não gira — confira a conexão do
ESC dele e se o teste aponta o pino certo. Guarde também a impressão sobre
o tempo de resposta (quanto tempo do comando até o giro): isso já dá uma
noção do atraso real do conjunto ESC + hélice, que compararemos com o
modelo na Etapa 6.

---

## Etapa 6 — Calibração dos motores com o wizard

**Objetivo:** substituir todas as estimativas sobre os motores (força,
eficiência, direção, zona morta) por **medições de verdade, motor por
motor**. É a etapa que garante que nada depende de "achismo de montagem".

Antes de começar: **remonte as hélices**, pendure a esfera de forma que ela
possa balançar livremente, afaste pessoas e objetos, e mantenha a tecla `k`
à mão. Durante os pulsos a esfera vai balançar — é assim que o sistema mede
a força; está tudo certo.

**Passo a passo:**

No seu computador (ou na própria Rasp, com `--tcp localhost:9999`):

```bash
python wizard.py --serial COM7 --push
```

> **Desde 14/10/2026:** o `--push` envia só o `yaw_sign` (kf/dz só com
> `--push-kf`, e o firmware valida a faixa). A fórmula de força do wizard
> supõe pêndulo simples; com a esfera presa por pino o kf medido pode sair
> várias vezes errado. **Para kf, meça estaticamente:** prenda a esfera a um
> dinamômetro de mão/balança de bagagem na horizontal e anote a força de cada
> motor em 4–5 níveis de PWM por sentido (`W<m> <us> 3000`). Direção de
> empuxo é geometria: o firmware ignora `tx/ty`.

O procedimento é automático e leva cerca de 42 segundos no total (6 pulsos
por motor, 18 pulsos para os três). Ele:

1. Desliga os loops de controle;
2. Pulsa cada motor em vários níveis (±100, ±170 e ±240 µs, 600 ms cada
   pulso), medindo a resposta da esfera pela telemetria;
3. Calcula, para cada motor: a direção em que ele empurra, o sinal do torque
   de yaw, a curva de força em µs (frente e ré separadamente) e a zona
   morta;
4. Salva o arquivo `calib.json` e, por causa do `--push`, envia tudo ao STM
   (vale enquanto o STM não reiniciar).

Ao final, leia o relatório com calma. O que esperar:

- As direções de empuxo devem ficar próximas da geometria (motor a
  apontando para +X do sensor, b e c a ±120°);
- A força por hélice deve ficar na região dos 18 N (valor provisório);
- Diferenças grandes entre os motores não são erro: são exatamente o que o
  wizard existe para detectar (hélice montada ao contrário, ESC com fiação
  invertida e assim por diante).

**Depois do wizard, faça o commit do `calib.json` no repositório** — ele é
um dado medido do hardware de vocês e passa a valer para o simulador, o
twin e o firmware (`python main.py --calib calib.json` no simulador).

**Como saber que deu certo:** relatório plausível, calib.json salvo e o twin
continua se comunicando normalmente.

**O que fazer se der errado:** valores muito fora do esperado (força muito
baixa, direções trocadas) — não ajuste nada às cegas; salve o relatório e
discuta com a equipe antes da próxima sessão.

---

## Etapa 7 — Primeira malha fechada

**Objetivo:** ligar o controle de verdade, primeiro o amortecimento do
balanço, depois o heading — nessa ordem, porque o amortecimento é o mais
fácil de conferir visualmente e o mais seguro.

Mantenha a esfera pendurada livre e a tecla `k` à mão.

**Passo a passo:**

1. **Amortecimento primeiro.** Ligue com `p` e dê um empurrão suave na
   esfera, em **várias direções** (+X, +Y, diagonal). A oscilação deve
   **diminuir** com o tempo em todas, claramente mais rápido do que sem
   controle. A telemetria mostra `F_cmd` e `v̂`: `F_cmd` tem que apontar
   **contra** `v̂`. Perto do prumo (< ~0,35°) os motores desligam de
   propósito (zona de repouso).

   Velocidade pelo estimador do CG (opcional, `G8 1` ou botão no
   supervisório): repita o teste. Se amortecer melhor (sobretudo se com
   `G8 0` o `wx/wy` cai rápido mas a esfera continua indo e vindo), mantenha;
   ajuste `G9` = distância pino→CG medida;

   **Se a oscilação crescer em vez de diminuir, pressione `k` imediatamente.**
   Isso indica um eixo com sinal invertido — volte à Etapa 3 e repita os
   testes de eixos. Não insista com o controle ligado.

2. **Heading depois.** Com o amortecimento estabilizado, ligue o heading
   com `y` e peça um degrau pequeno: `15` na serial (ou tecla **]** no
   supervisório/twin). Os ganhos agora são em torque (N·m/rad, N·m·s/rad);
   se oscilar, aumente `yaw_kd` (G5); se for lento, aumente `yaw_kp` (G3). A esfera deve girar suavemente até o alvo e parar
   sem passar do ponto;
3. **Ajuste fino (opcional).** Se quiser refinar os ganhos, use as teclas
   **G**, **+** e **−** do **supervisório** (no twin os índices 3–7 são do
   controlador Python e o firmware os recusa): o novo valor vale na hora.
   Ajuste de pouco em pouco, um ganho por vez, e observe a resposta.

**Como saber que deu certo:** o empurrão amortece; o degrau de heading chega
ao alvo sem oscilar; nada satura por tempo demais (os PWMs podem tocar os
extremos em transientes, mas voltam ao meio).

**Pendência desta etapa:** com os motores trabalhando na faixa completa
(±500 µs), aproveite para medir a corrente e a temperatura dos ESCs e dos
motores. Foi decisão da equipe liberar a faixa; os números reais confirmam
se ela é segura.

---

## Etapa 8 — Controle usando a visão

**Objetivo:** fechar o circuito completo — a câmera vê a corda no chão, o
STM gira a esfera até alinhá-la ao centro da imagem. É o ensaio final da
bancada, o mais parecido com a operação real.

**Passo a passo:**

1. Com a visão rodando na Rasp e detectando a corda, ligue o envio com a
   tecla **v**;
2. Ligue o heading (`y`) e o amortecimento (`p`);
3. Observe: a esfera deve girar sozinha até o heading em que a corda fica
   alinhada ao centro da imagem;
4. Mova a corda pelo chão: a esfera deve persegui-la.

**O ponto de atenção desta etapa é o sentido da correção.** Se a esfera
girar para o lado errado (afastando do alvo em vez de se aproximar), o sinal
do bearing está invertido. A correção é uma única linha no envio, em
`Raspberry/main.py` — inverta o sinal, reinicie o aplicativo da Rasp e
repita. Essa validação é necessária porque, no firmware antigo, o controle
de heading nunca funcionou de verdade (havia um erro que deixava o loop
morto), então não existe "jeito antigo que funcionava" para copiar.

**Como saber que deu certo:** a esfera alinha com a corda, persegue a corda
quando você a move, e o twin mostra a linha de transmissão desenhada à
distância D abaixo da esfera.

---

## Resumo das pendências que aparecem no caminho

| Quando | Pendência |
|---|---|
| Etapa 1 | Saída de erros do compilador (se houver) |
| Etapa 3 | Teste de eixos: ajustar `IMU_MOUNT_DEG` / `IMU_Z_UP` se necessário e regravar |
| Etapa 5 | Primeira impressão do atraso real do ESC + hélice |
| Etapa 6 | Commit do `calib.json` medido; conferir se a força real difere muito dos 18 N provisórios (se diferir, os ganhos precisam de revisão) |
| Etapa 7 | Medir corrente e temperatura dos ESCs na faixa completa |
| Etapa 8 | Sentido do bearing (flip de uma linha, se necessário) |

Medição recomendada quando houver tempo: o atraso real do conjunto ESC +
hélice (comando `W0 300 600` no hardware e a mesma coisa no `--sim`, e
comparar as respostas no twin). Esse número entra no modelo e afeta a
exatidão do wizard e a margem dos ganhos.

---

*Documento atualizado até o commit `4b593f6`. Em caso de dúvida sobre
qualquer etapa, os capítulos do `docs/guia_esfera.pdf` detalham o sistema
inteiro.*
