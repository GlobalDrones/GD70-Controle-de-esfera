# Bring-up em etapas — do zero à malha fechada, motores por último

Princípio: cada etapa tem critério de SAÍDA claro e nada da etapa seguinte
começa sem a anterior passar. Os ESCs ficam **sem bateria** até a etapa 5 —
até lá, tudo é verificável porque o PWM comandado aparece na telemetria
(`TEL,...pwm_a,pwm_b,pwm_c...`), mesmo sem motor nenhum girar.

Parada de emergência em qualquer etapa: tecla `k` (parada geral: loops off +
ESCs neutro + mecanismos) ou cortar a bateria dos ESCs.

---

## Etapa 0 — PC apenas (zero risco)

```bash
python -m pytest tests/ -q      # 48 passed
python -m twin --sim            # treinar teclas (fantasma + STM simulado)
python wizard.py --sim          # ver o wizard rodando
```

**Saída:** familiaridade com as teclas do twin (1/2, Z/X, G, +/−, K).

## Etapa 1 — Compilar e flashar o firmware

- Arduino IDE/CLI + core STM32Duino, placa Black Pill F401/F411,
  `firmware/main.cpp`. Pode haver erros de compilação (escrito sem toolchain)
  — mande a saída do compilador para corrigir.
- Flashar com a esfera apoiada. Boot: loops OFF, ESCs em 1500 µs.

**Saída:** banner de boot na USB (115200), LED pisca na calibração da MPU.

## Etapa 2 — STM "seco" pela USB do PC (ESCs sem bateria)

```bash
python -m twin --serial COM7
```

- Telemetria a 25 Hz, fantasma em pé, PWMs em 1500.
- `y` / `p`: flags mudam na telemetria (loops on/off) — **nenhum movimento**.
- `k`: parada geral. `R`: ganhos voltam. `G6 950`: valor muda.
- `+15` / `-15`: degrau de yaw → **PWMs variam nos painéis** (a alocação
  inteira é visível sem motores). Confira: os três PWMs se movem juntos
  (distribuição de yaw) e voltam a 1500 ao acomodar.
- `z`: calibração com gate de quietude — teste o ABORTO: mexa a esfera
  durante a calibração e confirme que ela recusa.
- `e`: AS5600 responde.

**Saída:** comando→telemetria fechado no PC, sem nenhum atuador.

## Etapa 3 — Sensores (ainda sem bateria nos ESCs)

- **Eixos da inclinação**: incline a esfera numa direção conhecida → sinais
  de sx/sy coerentes nos painéis do twin.
- **COMP_YAW_SIGN**: gire a esfera inclinada à mão → sx/sy NÃO podem girar
  junto (só mudam se o teste falhar → inverter a constante no firmware).
- **Yaw**: gire e volte → yaw acompanha; meça a deriva em 60 s parado
  (esperado: ~graus/min; a visão corrige isso depois).
- `t` com balanço livre → L medido bate com o L da sessão?

**Saída:** estados confiáveis nos painéis do twin.

## Etapa 4 — Rasp + visão + ponte (ESCs ainda sem bateria)

```bash
# na Rasp (a partir da raiz do repo):
pip3 install -r Raspberry/requirements.txt
python3 Raspberry/main.py
# no PC:
python -m twin --tcp <ip-da-rasp>
```

- HUD da Rasp alimentado por TEL; twin vê o fantasma pela ponte TCP.
- Visão: estique um **cabo de mentira** (corda) no chão sob a esfera →
  linha detectada, bearing e D no HUD; compare D com fita métrica.
- `v`: liga/desliga o envio — com `y` OFF, nada acontece além do STM
  receber os `V<b> <D>` (seguro).
- Rádio RC: canais CH12–15 comandam pinça/polia (mecanismos — primeiro
  movimento físico, se desejar adie para depois da etapa 5).
- Tecla `p` no teclado da Rasp = parada geral (flags off).

**Saída:** visão + telemetria + comandos rodando juntos, motores intactos.

## Etapa 5 — Primeiro giro (bateria ligada, HÉLICES REMOVIDAS)

Segurança: hélices fora, esfera fixa, `k` à mão, ideal com medição de corrente.

- `m`: teste sequencial ±120 µs — confirmar **a↔PA0, b↔PA15, c↔PA1** e os
  sentidos de giro pelos eixos dos motores.
- O tempo de resposta (spin-up/down) dá uma primeira ideia do atraso real τ.

**Saída:** mapeamento físico confirmado; nenhum ESC travando nos passos de
±120 µs (estão acima do piso anti-travamento).

## Etapa 6 — Hélices montadas, esfera pendurada LIVRE → wizard

- Área livre, `k` à mão.
- ```bash
  python wizard.py --serial COM7 --push   # ou --tcp <rasp>
  ```
- Esfera balança durante os pulsos (é a medição). Revisar o relatório:
  direções ≈ geométricas? força ≈ 18 N/hélice? diferenças grandes entre
  motores = montagem detectada (era o objetivo).
- **Commitar o `calib.json` real** (dado medido — este vai para o repo).

**Saída:** atuadores calibrados por motor; suposições substituídas por dados.

## Etapa 7 — Primeira malha fechada

- **Anti-balanço primeiro** (`p`): empurrão gentil → oscilação DECAI.
  Se crescer: `k` imediato e voltar à etapa 3 (sinal de eixo).
- **Heading depois** (`y`): degraus pequenos ±15° → suave, sem sobressinal.
- Ganhos ao vivo pelo twin (G, +/−) — o push vai ao STM.

**Saída:** controle validado com autoridade real.

## Etapa 8 — Visão em malha fechada

- `y` ON + `v` ON: a esfera alinha com o cabo de mentira.
- **Sinal do bearing**: se afastar/flipar em vez de alinhar, inverter o
  sinal no envio (uma linha em `Raspberry/main.py`) — validação empírica,
  a cadeia nunca fechou no firmware antigo.
- Ensaio completo: perturbação + recuperação + heading na linha + D no twin.

---

## Pendências que aparecem naturalmente durante o bring-up

| Quando | O quê |
|---|---|
| Etapa 1 | Erros de compilação → mandar a saída |
| Etapa 5 | Medir τ (degrau real vs `--sim`, sobrepor no twin) e a faixa de travamento real do ESC |
| Etapa 6 | Calibração real → commit; re-checar ganhos se a força diferir muito de 18 N |
| Etapa 7 | Corrente/térmica dos F45A com a faixa ±500 µs |
