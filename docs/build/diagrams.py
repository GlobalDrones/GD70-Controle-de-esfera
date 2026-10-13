"""Diagramas do Guia Esfera — layout MEDIDO: texto nunca sai da caixa.

Três garantias, por construção e por verificação:

1. Todo texto é MEDIDO no renderer ANTES de posicionar; cada caixa cresce
   para conter título + linhas com folga (padding), nunca o contrário.
2. Fileiras de caixas são posicionadas sequencialmente com vão fixo — a
   largura medida define o lugar, então caixas nunca se sobrepoem.
3. Após desenhar, ``verify()`` confere no renderer que: nenhum texto sai da
   sua caixa, nenhum texto livre (rótulos de seta, notas) invade caixa
   alguma, e nenhum par de textos se sobrepõe. Se qualquer regra quebrar,
   o build FALHA com a lista de problemas — overflow não passa.

Uso:  python docs/build/diagrams.py   → docs/diagrams/*.png
"""
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "diagrams")
os.makedirs(OUT, exist_ok=True)

# paleta (palette.cascade — mesma do build_pdf.py)
CARD = "#e4e7e8"
HEADER = "#334650"
ICON = "#52798c"
ACCENT = "#a63648"
TEXT = "#1a1b1c"
MUTED = "#6f7578"
BENCH_BG = "#f2f3f4"

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "text.color": TEXT,
})


class Diagram:
    """Canvas 0–1 × 0–1 com caixas medidas e verificação de sobreposição."""

    def __init__(self, figsize, dpi=160):
        self.fig, self.ax = plt.subplots(figsize=figsize, dpi=dpi)
        self.ax.set_xlim(0, 1)
        self.ax.set_ylim(0, 1)
        self.ax.axis("off")
        self.fig.canvas.draw()  # renderer pronto para medir
        self._box_rects = []    # (bbox_display, [artist, ...]) por caixa
        self._loose = []        # artistas sem caixa (rótulos/notas)

    # ── medição ────────────────────────────────────────────────────────────

    def text_size(self, s, size, weight="normal", style="normal"):
        """Largura/altura do texto em fração dos eixos (medido, não estimado)."""
        t = self.ax.text(0.5, 0.5, s, fontsize=size, fontweight=weight,
                         fontstyle=style, ha="center", va="center")
        bb = t.get_window_extent(renderer=self.fig.canvas.get_renderer())
        t.remove()
        inv = self.ax.transAxes.inverted()
        (x0, y0), (x1, y1) = inv.transform([(bb.x0, bb.y0), (bb.x1, bb.y1)])
        return abs(x1 - x0), abs(y1 - y0)

    # ── elementos ──────────────────────────────────────────────────────────

    def box(self, title, lines=(), cx=None, cy=None, tsize=14, lsize=12,
            pad_x=0.022, pad_y=0.020, gap_line=0.012, fill=CARD, edge=ICON,
            lw=1.6, dashed=False):
        """Caixa dimensionada PELO CONTEÚDO. Devolve dict com as bordas."""
        tw, th = self.text_size(title, tsize, weight="bold")
        widths, heights = [tw], [th]
        for ln in lines:
            w, h = self.text_size(ln, lsize)
            widths.append(w)
            heights.append(h)
        w = max(widths) + 2 * pad_x
        h = 2 * pad_y + th + (len(lines) * (heights[0] and 0))  # placeholder
        h = 2 * pad_y + th
        for lh in heights[1:]:
            h += gap_line + lh
        # posição
        if cx is None or cy is None:
            raise ValueError("box precisa de cx/cy")
        x, y = cx - w / 2, cy - h / 2
        style = "round,pad=0.008,rounding_size=0.012"
        patch = FancyBboxPatch(
            (x, y), w, h, boxstyle=style, fc=fill, ec=edge, lw=lw,
            linestyle=(0, (5, 4)) if dashed else "solid",
        )
        self.ax.add_patch(patch)

        arts = []
        arts.append(self.ax.text(
            cx, y + h - pad_y, title, ha="center", va="top",
            fontsize=tsize, fontweight="bold", color=HEADER))
        yy = y + h - pad_y - th
        for ln, lh in zip(lines, heights[1:]):
            yy -= gap_line + lh
            arts.append(self.ax.text(
                cx, yy + lh / 2, ln, ha="center", va="center",
                fontsize=lsize, color=TEXT))
        bb_disp = self._rect_display(x, y, w, h)
        self._box_rects.append((bb_disp, arts))
        return {"x": x, "y": y, "w": w, "h": h, "cx": cx, "cy": cy,
                "left": x, "right": x + w, "bottom": y, "top": y + h}

    def arrow(self, p1, p2, color=ICON, lw=2.2, dashed=False, mutation=18):
        self.ax.add_patch(FancyArrowPatch(
            p1, p2, arrowstyle="-|>", mutation_scale=mutation, color=color,
            lw=lw, linestyle=(0, (5, 4)) if dashed else "solid",
            shrinkA=2, shrinkB=2))

    def note(self, x, y, s, size=12, color=MUTED, weight="normal",
             ha="center", va="center", style="normal"):
        t = self.ax.text(x, y, s, fontsize=size, color=color, ha=ha, va=va,
                         fontweight=weight, fontstyle=style)
        self._loose.append(t)
        return t

    # ── posicionamento auxiliar ────────────────────────────────────────────

    def row(self, boxes_spec, y, gap, tsize=14, lsize=12, **kw):
        """Fileira de caixas: mede todas, posiciona lado a lado com vão
        fixo e centraliza horizontalmente — sobreposição é impossível."""
        # pré-mede larguras
        ws = []
        for title, lines in boxes_spec:
            tw, th = self.text_size(title, tsize, weight="bold")
            w = tw
            for ln in lines:
                lw_, _ = self.text_size(ln, lsize)
                w = max(w, lw_)
            ws.append(w + 2 * 0.022)
        total = sum(ws) + gap * (len(ws) - 1)
        x0 = (1 - total) / 2
        out = []
        for (title, lines), w in zip(boxes_spec, ws):
            cx = x0 + w / 2
            out.append(self.box(title, lines, cx=cx, cy=y,
                                tsize=tsize, lsize=lsize, **kw))
            x0 += w + gap
        return out

    def gap_label(self, a, b, lines, size=11.5, color=MUTED, dy=0.0,
                  max_gap_use=0.86):
        """Rótulo no vão ENTRE duas caixas (verificado contra invasão)."""
        if lines:
            for i, ln in enumerate(lines):
                w, h = self.text_size(ln, size, style="italic")
                if w > (b["left"] - a["right"]) * max_gap_use:
                    raise SystemExit(
                        f"rótulo '{ln}' ({w:.2f}) não cabe no vão "
                        f"({b['left'] - a['right']:.2f}) — quebre em mais linhas")
                self.note((a["right"] + b["left"]) / 2,
                          (a["top"] + a["bottom"]) / 2 + dy + (len(lines) - 1 - i) * (h + 0.004),
                          ln, size=size, color=color, style="italic")

    # ── verificação ────────────────────────────────────────────────────────

    def _rect_display(self, x, y, w, h):
        from matplotlib.transforms import Bbox
        p0 = self.ax.transAxes.transform((x, y))
        p1 = self.ax.transAxes.transform((x + w, y + h))
        return Bbox.from_extents(min(p0[0], p1[0]), min(p0[1], p1[1]),
                                 max(p0[0], p1[0]), max(p0[1], p1[1]))

    def _tbox(self, artist):
        return artist.get_window_extent(renderer=self.fig.canvas.get_renderer())

    def verify(self, name):
        self.fig.canvas.draw()
        problems = []

        def overlaps(b1, b2, tol=1.0):
            return (b1.x0 < b2.x1 - tol and b2.x0 < b1.x1 - tol
                    and b1.y0 < b2.y1 - tol and b2.y0 < b1.y1 - tol)

        def inside(tb, bb, tol=1.0):
            return (tb.x0 >= bb.x0 - tol and tb.x1 <= bb.x1 + tol
                    and tb.y0 >= bb.y0 - tol and tb.y1 <= bb.y1 + tol)

        # 1) todo texto dentro da PRÓPRIA caixa (com folga de 1 px)
        for bb, arts in self._box_rects:
            for a in arts:
                if not inside(self._tbox(a), bb):
                    problems.append(
                        f"[{name}] texto fora da caixa: '{a.get_text()[:28]}'")

        # 2) texto livre (rótulo/nota) não invade NENHUMA caixa
        for a in self._loose:
            tb = self._tbox(a)
            for bb, _ in self._box_rects:
                if overlaps(tb, bb, tol=2.0):
                    problems.append(
                        f"[{name}] rótulo/nota invade caixa: '{a.get_text()[:28]}'")

        # 3) nenhum par de textos se sobrepõe
        all_texts = [(a, self._tbox(a)) for _, arts in self._box_rects for a in arts]
        all_texts += [(a, self._tbox(a)) for a in self._loose]
        for i in range(len(all_texts)):
            for j in range(i + 1, len(all_texts)):
                if overlaps(all_texts[i][1], all_texts[j][1], tol=1.0):
                    problems.append(
                        f"[{name}] textos sobrepostos: "
                        f"'{all_texts[i][0].get_text()[:20]}' × "
                        f"'{all_texts[j][0].get_text()[:20]}'")

        if problems:
            raise SystemExit("Diagrama reprovado na verificação:\n  "
                             + "\n  ".join(problems))

    def save(self, filename):
        path = os.path.join(OUT, filename)
        self.fig.savefig(path, facecolor="white", bbox_inches="tight",
                         pad_inches=0.12)
        plt.close(self.fig)
        return path


# ════════════════════════════════ D1 — arquitetura (pós-mescla da Rasp)


def diagrama_arquitetura():
    d = Diagram((11.0, 6.2))

    # fileira de campo: STM — USB — Rasp — TCP — PC
    stm, rasp, pc = d.row(
        [
            ("STM32F4 (Black Pill)", ["controle 200 Hz", "telemetria 25 Hz",
                                      "loops OFF no boot"]),
            ("Raspberry Pi — app de visão",
             ["visão estéreo + linha", "HUD + rádio RC + reset GPIO18",
              "ÚNICO dono da serial USB", "ponte TCP embutida (9999)"]),
            ("PC — operação", ["twin --tcp", "wizard --tcp", "SSH / túnel"]),
        ],
        y=0.66, gap=0.165,
    )

    # links de campo (bidirecionais, rótulos nos vãos)
    d.arrow((stm["right"] + 0.004, 0.70), (rasp["left"] - 0.004, 0.70))
    d.arrow((rasp["left"] - 0.004, 0.62), (stm["right"] + 0.004, 0.62))
    d.gap_label(stm, rasp, ["USB", "link de campo"], dy=0.085)
    d.arrow((rasp["right"] + 0.004, 0.70), (pc["left"] - 0.004, 0.70))
    d.arrow((pc["left"] - 0.004, 0.62), (rasp["right"] + 0.004, 0.62))
    d.gap_label(rasp, pc, ["TCP 9999", "LAN / túnel"], dy=0.085)

    # faixa da bancada (sem Rasp)
    bench = d.box(
        "Bancada — sem Rasp",
        ["STM na USB do PC: twin/wizard --serial (flash e testes)"],
        cx=0.5, cy=0.22, tsize=13, lsize=11.5, fill=BENCH_BG, edge=MUTED,
        dashed=True)
    d.note(0.5, 0.40, "Serial2 (PA2/PA3) permanece disponível como opção "
                      "alternativa de link", size=11, color=MUTED, style="italic")

    # setas verticais da bancada para STM e PC
    d.arrow((stm["cx"], stm["bottom"] - 0.004), (bench["left"] + 0.02,
             bench["top"] + 0.004), color=MUTED, lw=1.5, dashed=True)
    d.arrow((pc["cx"], pc["bottom"] - 0.004), (bench["right"] - 0.02,
             bench["top"] + 0.004), color=MUTED, lw=1.5, dashed=True)

    d.note(0.5, 0.055,
           "Em campo, o app de visão é o único dono da serial do STM; "
           "twin/wizard conectam à ponte TCP dele.",
           size=12, color=MUTED, style="italic")

    d.verify("arquitetura")
    return d.save("arquitetura.png")


# ════════════════════════════════ D2 — loop de controle


def diagrama_loop():
    d = Diagram((11.2, 5.6))

    mpu, fusao, ctrl, aloc = d.row(
        [
            ("MPU6050", ["gyro + acc", "I2C 400 kHz"]),
            ("Fusão de atitude", ["gate |acc| ≈ 1 g", "taxas em MUNDO",
                                  "auto-rebias"]),
            ("Controle (200 Hz)", ["XY: PID de v̂ = L·θ̇", "heading: perfil + PI",
                                   "anti-windup"]),
            ("Alocação", ["3 motores a 120°", "curva F(u) por motor",
                          "quantização anti-banda"]),
        ],
        y=0.70, gap=0.125,
    )

    esfera = d.box("Esfera", ["pêndulo 12 kg", "hélices tangentes"],
                   cx=(aloc["cx"]), cy=0.22)

    ymid = 0.70
    d.arrow((mpu["right"] + 0.004, ymid), (fusao["left"] - 0.004, ymid))
    d.gap_label(mpu, fusao, ["bruto"], dy=0.075)
    d.arrow((fusao["right"] + 0.004, ymid), (ctrl["left"] - 0.004, ymid))
    d.gap_label(fusao, ctrl, ["sx, sy", "wx, wy", "yaw, rate_z"], dy=0.075)
    d.arrow((ctrl["right"] + 0.004, ymid), (aloc["left"] - 0.004, ymid))
    d.gap_label(ctrl, aloc, ["F [N] +", "yaw_us"], dy=0.075)

    d.arrow((aloc["cx"], aloc["bottom"] - 0.004),
            (esfera["cx"], esfera["top"] + 0.004))
    d.note(aloc["right"] + 0.035, (aloc["bottom"] + esfera["top"]) / 2,
           "PWM a/b/c", size=11.5, color=MUTED, style="italic")

    # retorno: esfera -> MPU por baixo
    yret = esfera["bottom"] - 0.10
    d.arrow((esfera["cx"], esfera["bottom"] - 0.004), (mpu["cx"], yret),
            color=MUTED, lw=1.6)
    d.arrow((mpu["cx"], yret), (mpu["cx"], mpu["bottom"] - 0.004),
            color=MUTED, lw=1.6)
    d.note((mpu["cx"] + esfera["cx"]) / 2, yret - 0.055,
           "resposta física medida pelos sensores (5 ms por volta)",
           size=11.5, color=MUTED, style="italic")

    d.verify("loop_controle")
    return d.save("loop_controle.png")


# ════════════════════════════════ D3 — cronograma do wizard


def diagrama_wizard():
    d = Diagram((9.6, 4.4))

    w_total = 0.86
    dur = {"baseline": 0.48, "pulso": 0.60, "repouso": 1.20}
    total = sum(dur.values())
    segs = [
        ("baseline", "baseline", "0,48 s (12 frames)", CARD, ICON),
        ("pulso", "PULSO", "0,60 s (15 frames)", "#f3dfe2", ACCENT),
        ("repouso", "repouso", "1,20 s (30 frames)", CARD, ICON),
    ]
    # barra com segmentos largos O SUFICIENTE para os rótulos
    x = (1 - w_total) / 2
    ybar, hbar = 0.60, 0.22
    from matplotlib.patches import FancyBboxPatch as FBP
    xs = []
    for key, name, sub, fill, edge in segs:
        w = dur[key] / total * w_total
        t1, _ = d.text_size(name, 14, weight="bold")
        t2, _ = d.text_size(sub, 12)
        w = max(w, t1 + 0.05, t2 + 0.05)
        xs.append((x, w, name, sub, fill, edge))
        x += w
    # recentraliza se os mínimos alargaram a barra
    x0 = (1 - (xs[-1][0] + xs[-1][1])) / 2
    centers = []
    arts_bar = []
    for xo, w, name, sub, fill, edge in xs:
        xo += x0
        patch = FBP((xo, ybar), w, hbar, boxstyle="round,pad=0.006,"
                     "rounding_size=0.010", fc=fill, ec=edge, lw=1.6)
        d.ax.add_patch(patch)
        centers.append(xo + w / 2)
        arts_bar.append(d.ax.text(xo + w / 2, ybar + hbar * 0.66, name,
                                  ha="center", va="center", fontsize=14,
                                  fontweight="bold", color=HEADER))
        arts_bar.append(d.ax.text(xo + w / 2, ybar + hbar * 0.28, sub,
                                  ha="center", va="center", fontsize=12,
                                  color=TEXT))
        d._box_rects.append((
            d._rect_display(xo, ybar, w, hbar),
            arts_bar[-2:]))
    # (segmentos já registrados como “caixas” na verificação, acima)

    d.note(0.5, 0.90, "1 pulso = 2,28 s   ·   6 níveis (±240/170/100 µs) "
           "× 3 motores ≈ 42 s", size=14.5, color=HEADER, weight="bold")

    # notas abaixo, em pista própria, com seta simples apontando o segmento
    notas = [
        (centers[0], "mede o resíduo de\noscilação (senoide)", TEXT),
        (centers[1], "resposta do motor:\nwx, wy, rate_z", ACCENT),
        (centers[2], "esfera volta a\nquase parar", TEXT),
    ]
    ynote = 0.30
    for cx, txt, color in notas:
        d.note(cx, ynote, txt, size=12.5, color=color)
        d.arrow((cx, ynote + 0.055), (cx, ybar - 0.004), color=MUTED, lw=1.4)

    d.note(0.5, 0.055, "Comando W<motor> <µs> <ms>: loops desligados durante o "
           "pulso; telemetria TEL segue a 25 Hz — é por ela que o wizard mede.",
           size=12, color=MUTED, style="italic")

    d.verify("wizard_timeline")
    return d.save("wizard_timeline.png")


if __name__ == "__main__":
    print(diagrama_arquitetura())
    print(diagrama_loop())
    print(diagrama_wizard())
    print("diagramas OK — verificação de sobreposição passou em todos")
