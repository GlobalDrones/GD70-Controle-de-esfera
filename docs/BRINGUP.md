# Bring-up do Sistema Esfera — guia passo a passo

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

| Etapa | O que é testado | Bateria dos ESCs |
|---|---|---|
| 0 | Simulador no computador (ensaio geral) | desligada |
| 1 | Compilar e gravar o firmware no STM | desligada |
| 2 | Comunicação com o STM (comandos e telemetria) | desligada |
| 3 | Sensores de inclinação e heading | desligada |
| 4 | Raspberry Pi: visão, ponte de rede e rádio | desligada |
| 5 | Primeiro giro dos motores (sem hélices) | **ligada** |
| 6 | Calibração dos motores com o wizard | ligada |
| 7 | Controle em malha fechada | ligada |
| 8 | Controle usando a visão | ligada |

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
- Um cabo USB para ligar o STM ao computador;
- A partir da Etapa 4: a Raspberry Pi na mesma rede (ou com cabo próprio);
- A partir da Etapa 5: a bateria dos ESCs — **só ligue quando chegar lá**.

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
- **1** e **2** ligam e desligam os loops de amortecimento e de heading;
- **Z** e **X** pedem que a esfera gire −15° ou +15°;
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

1. Instale o Arduino IDE (ou o `arduino-cli`) com o suporte a placas STM32
   (core STM32Duino) e selecione a placa **Black Pill F401 ou F411**;
2. Abra `firmware/main.cpp` e mande compilar;
3. Ligue o STM ao computador por USB e grave o firmware;
4. Abra um monitor serial na porta do STM, com velocidade 115200.

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

**Passo a passo:**

```bash
python -m twin --serial COM7    # troque COM7 pela porta do seu STM
```

Com o twin aberto, verifique cada item da lista abaixo. Em todos eles, a
esfera não deve se mover nem um milímetro — o que muda são apenas os
números de PWM na telemetria.

1. A telemetria chega (a faixa no topo mostra o tempo correndo, e o painel
   de PWM mostra três linhas paradas em 1500 µs);
2. Pressione **2** (liga o loop de heading) e depois **1** (liga o
   amortecimento): o quadro de status deve mostrar `HEADING: LIGADO` e
   `XY: LIGADO`;
3. Pressione **X** (girar +15°): os três PWMs se movem **juntos** para o
   mesmo lado e voltam a 1500 quando a manobra termina. É a distribuição de
   yaw agindo — tudo visível sem motores;
4. Pressione **G** e depois **+**: o valor do ganho selecionado aumenta
   (confira na linha `GANHOS` do rodapé). Com **R** o STM devolve os 8
   ganhos;
5. Pressione **K**: parada geral — os dois loops desligam e os PWMs voltam
   a 1500;
6. Em um terminal serial qualquer (115200), digite `z` para calibrar a MPU.
   Para testar a proteção contra movimento, repita a calibração **enquanto
   alguém mexe na esfera**: ela deve se recusar e avisar. Esse é o "gate de
   quietude" funcionando.

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

1. **Eixos de inclinação.** Com a mão, incline a esfera devagar para a
   direita: o gráfico de θx ou θy deve responder no eixo correspondente, no
   sentido correspondente. Incline para os quatro lados e confira se as
   curvas fazem sentido (escreva o que observou — vai servir de referência);
2. **Compensação de yaw.** Incline a esfera uns 15° em qualquer direção e,
   mantendo a inclinação, gire-a lentamente em torno do eixo vertical (o
   heading no painel deve mudar). **As curvas de θx/θy não podem girar
   junto** — a inclinação é a mesma, só a esfera girou de direção. Se as
   curvas acompanharem a rotação, a constante `COMP_YAW_SIGN` do firmware
   está com o sinal trocado: inverta-a e regrave (é uma linha, no topo de
   `firmware/main.cpp`);
3. **Deriva do heading.** Deixe a esfera completamente parada por um minuto
   e observe a curva de heading: ela pode subir ou descer devagar (alguns
   graus por minuto). Isso é a deriva natural do giroscópio — esperado, e
   corrigido depois pela visão;
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
pip3 install -r Raspberry/requirements.txt
python3 Raspberry/main.py
```

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
   STM. Como o loop de heading continua desligado, nada acontece além do
   STM passar a receber os valores — seguro;
5. Se houver rádio ligado à Rasp, teste os canais: eles comandam a pinça e
   a polia. **Atenção: este é o primeiro movimento físico do bring-up** —
   se preferir, deixe para depois da Etapa 5;
6. Na Rasp, a tecla **p** agora é a parada geral de verdade (foi corrigida
   na migração; antigamente ela desligava o amortecimento sem querer).

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

1. **Amortecimento primeiro.** Ligue com `p` (ou tecla **1** no twin) e
   dê um empurrão suave na esfera. A oscilação deve **diminuir** com o
   tempo, claramente mais rápido do que sem controle;

   **Se a oscilação crescer em vez de diminuir, pressione `k` imediatamente.**
   Isso indica um eixo com sinal invertido — volte à Etapa 3 e repita os
   testes de eixos. Não insista com o controle ligado.

2. **Heading depois.** Com o amortecimento estabilizado, ligue o heading
   com `y` (ou tecla **2**) e peça um degrau pequeno: `15` na serial (ou
   tecla **X** no twin). A esfera deve girar suavemente até o alvo e parar
   sem passar do ponto;
3. **Ajuste fino (opcional).** Se quiser refinar os ganhos, use as teclas
   **G**, **+** e **−** do twin: o novo valor vale na hora para o hardware.
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
| Etapa 3 | Se `COMP_YAW_SIGN` precisar de inversão, regrar o firmware |
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
