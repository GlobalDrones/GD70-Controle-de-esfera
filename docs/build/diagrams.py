"""Diagramas do Guia Esfera (matplotlib, paleta do documento).

Saída: docs/diagrams/*.png (dpi 160, embed A4 ~450pt => fator 0.69; fontes >= 13pt => >= 9pt efetivo)
"""
import os
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

OUT = os.path.join(os.path.dirname(__file__), "..", "diagrams")
os.makedirs(OUT, exist_ok=True)

# paleta (palette.cascade)
CARD = "#e4e7e8"
HEADER = "#334650"
ICON = "#52798c"
ACCENT = "#a63648"
TEXT = "#1a1b1c"
MUTED = "#6f7578"
BORDER = "#b8c8cf"
STRIPE = "#ebedee"

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "axes.edgecolor": MUTED,
    "text.color": TEXT,
})


def box(ax, x, y, w, h, title, lines=(), fill=CARD, edge=ICON, tsize=14, lsize=12.5):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.012,rounding_size=0.014",
                                fc=fill, ec=edge, lw=1.6))
    ax.text(x + w / 2, y + h - 0.052, title, ha="center", va="top",
            fontsize=tsize, fontweight="bold", color=HEADER)
    for i, ln in enumerate(lines):
        ax.text(x + w / 2, y + h - 0.135 - i * 0.075, ln, ha="center", va="top",
                fontsize=lsize, color=TEXT)


def arrow(ax, p1, p2, label=None, color=ICON, style="-|>", lw=2.0, loff=(0, 0.03),
          lsize=12, lcolor=None):
    ax.add_patch(FancyArrowPatch(p1, p2, arrowstyle=style, mutation_scale=18,
                                 color=color, lw=lw))
    if label:
        mx, my = (p1[0] + p2[0]) / 2 + loff[0], (p1[1] + p2[1]) / 2 + loff[1]
        ax.text(mx, my, label, ha="center", va="bottom", fontsize=lsize,
                color=lcolor or MUTED, style="italic")


# ───────────────────────────────────────────── diagrama 1: arquitetura de conexões
fig, ax = plt.subplots(figsize=(10.5, 5.2), dpi=160)
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)
ax.axis("off")

box(ax, 0.03, 0.42, 0.19, 0.28, "STM32F4",
    ["controle 200 Hz", "telemetria 25 Hz", "na esfera, em campo"], tsize=14.5, lsize=12)
box(ax, 0.40, 0.42, 0.21, 0.28, "Raspberry Pi",
    ["agent.py (systemd)", "SSH + remote desktop", "teclado/mouse + monitor"],
    tsize=14.5, lsize=12)
box(ax, 0.77, 0.42, 0.20, 0.28, "PC — operação",
    ["twin --tcp", "wizard --tcp", "terminal SSH"], tsize=14.5, lsize=12)
# bancada: caixa tracejada (USB existe só ali)
bx = FancyBboxPatch((0.335, 0.86), 0.30, 0.115,
                    boxstyle="round,pad=0.010,rounding_size=0.012",
                    fc="#f2f3f4", ec=MUTED, lw=1.3, linestyle=(0, (5, 4)))
ax.add_patch(bx)
ax.text(0.485, 0.930, "PC na bancada — só para flash", ha="center", va="center",
        fontsize=13, fontweight="bold", color=MUTED)
ax.text(0.485, 0.885, "twin/wizard --serial durante a bancada", ha="center", va="center",
        fontsize=11.5, color=MUTED)

arrow(ax, (0.225, 0.62), (0.395, 0.62), "Serial2 (PA2/PA3)", loff=(0.0, 0.025),
      lsize=12, lw=2.6)
arrow(ax, (0.395, 0.50), (0.225, 0.50), lw=2.6)
ax.text(0.31, 0.435, "única porta em campo", fontsize=11, color=ACCENT,
        style="italic", ha="center")

arrow(ax, (0.615, 0.62), (0.765, 0.62), "TCP 9999", loff=(0.0, 0.025), lsize=12, lw=2.6)
arrow(ax, (0.765, 0.50), (0.615, 0.50), lw=2.6)
ax.text(0.69, 0.435, "LAN / hotspot / túnel SSH", fontsize=11, color=MUTED,
        style="italic", ha="center")

ax.add_patch(FancyArrowPatch((0.20, 0.70), (0.36, 0.845), arrowstyle="-|>",
                             mutation_scale=16, color=MUTED, lw=1.5,
                             linestyle=(0, (5, 4))))
ax.text(0.155, 0.735, "USB: só na bancada", fontsize=11, color=MUTED,
        style="italic", ha="left", rotation=30)

ax.text(0.5, 0.03, "Em campo, a única porta do STM é a Serial2 (via Rasp). Operação local: "
        "nc localhost 9999 no terminal da Rasp.", ha="center", fontsize=12,
        color=MUTED, style="italic")
fig.tight_layout(pad=0.3)
fig.savefig(os.path.join(OUT, "arquitetura.png"), facecolor="white")
plt.close(fig)

# ───────────────────────────────────────────── diagrama 2: blocos do loop de controle
fig, ax = plt.subplots(figsize=(10.5, 5.0), dpi=160)
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)
ax.axis("off")

Y1, H1 = 0.56, 0.30
box(ax, 0.02, Y1, 0.17, H1, "MPU6050", ["gyro + acc", "I2C 400 kHz"], tsize=14.5, lsize=12)
box(ax, 0.25, Y1, 0.20, H1, "Fusão de atitude",
    ["gate |acc| ~ 1 g", "taxas em MUNDO", "auto-rebias"], tsize=14.5, lsize=12)
box(ax, 0.51, Y1, 0.22, H1, "Controle (200 Hz)",
    ["XY: PID de v̂ = L·θ̇", "heading: perfil + PI", "anti-windup"], tsize=14.5, lsize=12)
box(ax, 0.79, Y1, 0.19, H1, "Alocação",
    ["3 motores a 120°", "curva F(u)/motor", "clamp ±500 µs"], tsize=14.5, lsize=12)
box(ax, 0.79, 0.06, 0.19, 0.24, "Esfera",
    ["pêndulo 12 kg", "hélices tangentes"], tsize=14.5, lsize=12)

arrow(ax, (0.19, Y1 + H1 / 2), (0.248, Y1 + H1 / 2))
arrow(ax, (0.45, Y1 + H1 / 2), (0.508, Y1 + H1 / 2), "sx, sy, wx, wy, yaw, rate_z",
      loff=(-0.02, 0.03), lsize=11.5)
arrow(ax, (0.73, Y1 + H1 / 2), (0.788, Y1 + H1 / 2), "F [N], yaw_us",
      loff=(-0.02, 0.03), lsize=11.5)
arrow(ax, (0.885, Y1), (0.885, 0.315), "PWM a/b/c", loff=(0.035, -0.02), lsize=11.5)
# retorno: esfera -> MPU, por baixo (sem cruzar caixas)
arrow(ax, (0.785, 0.18), (0.107, 0.18), "resposta física medida pelos sensores",
      color=MUTED, style="-|>", lw=1.6, loff=(0.0, 0.012), lsize=11.5)
ax.add_patch(FancyArrowPatch((0.107, 0.18), (0.107, Y1 - 0.005), arrowstyle="-|>",
                             mutation_scale=18, color=MUTED, lw=1.6))

fig.tight_layout(pad=0.3)
fig.savefig(os.path.join(OUT, "loop_controle.png"), facecolor="white")
plt.close(fig)

# ───────────────────────────────────────────── diagrama 3: timeline do wizard
fig, ax = plt.subplots(figsize=(9.2, 3.6), dpi=160)
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)
ax.axis("off")

# barra segmentada: baseline | pulso | repouso
x0, w_total = 0.055, 0.86
w1, w2 = 0.48 / 2.28 * w_total, 0.60 / 2.28 * w_total
w3 = w_total - w1 - w2
segs = [
    (x0, w1, "baseline", "0,48 s (12 frames)", "#e4e7e8", ICON),
    (x0 + w1, w2, "PULSO", "0,60 s (15 frames)", "#f3dfe2", ACCENT),
    (x0 + w1 + w2, w3, "repouso", "1,20 s (30 frames)", "#e4e7e8", ICON),
]
for x, w, title, sub, fill, edge in segs:
    ax.add_patch(FancyBboxPatch((x, 0.42), w, 0.24, boxstyle="round,pad=0.008,rounding_size=0.012",
                                fc=fill, ec=edge, lw=1.6))
    ax.text(x + w / 2, 0.575, title, ha="center", fontsize=14.5, fontweight="bold", color=HEADER)
    ax.text(x + w / 2, 0.475, sub, ha="center", fontsize=12, color=TEXT)

ax.annotate("mede o resíduo de\noscilação (senoide)", xy=(x0 + w1 / 2, 0.41),
            xytext=(x0 + w1 / 2 - 0.06, 0.16), fontsize=12.5, color=TEXT, ha="center",
            arrowprops=dict(arrowstyle="-|>", color=MUTED, lw=1.4))
ax.annotate("resposta do motor:\nwx, wy, rate_z", xy=(x0 + w1 + w2 / 2, 0.41),
            xytext=(x0 + w1 + w2 / 2, 0.16), fontsize=12.5, color=ACCENT, ha="center",
            arrowprops=dict(arrowstyle="-|>", color=MUTED, lw=1.4))
ax.annotate("esfera volta a\nquase parar", xy=(x0 + w1 + w2 + w3 / 2, 0.41),
            xytext=(x0 + w1 + w2 + w3 / 2 + 0.05, 0.16), fontsize=12.5, color=TEXT,
            ha="center", arrowprops=dict(arrowstyle="-|>", color=MUTED, lw=1.4))

ax.text(0.5, 0.80, "1 pulso = 2,28 s   ·   6 níveis (±240/170/100 µs) × 3 motores ≈ 42 s",
        ha="center", fontsize=14.5, fontweight="bold", color=HEADER)
ax.text(0.5, 0.055, "Comando W<motor> <us> <ms>: loops de controle desligados durante o pulso;",
        ha="center", fontsize=12, color=MUTED, style="italic")
ax.text(0.5, 0.005, "telemetria TEL continua a 25 Hz — é por ela que o wizard mede.",
        ha="center", fontsize=12, color=MUTED, style="italic")
fig.tight_layout(pad=0.3)
fig.savefig(os.path.join(OUT, "wizard_timeline.png"), facecolor="white")
plt.close(fig)

print("diagramas OK:", os.listdir(OUT))
