"""Análise linear do anti-balanço num pêndulo DUPLO (cabo + esfera presa por pino).

Acompanha AUDITORIA_CONTROLE.md (SC-01, SC-02, WZ-04). Rode:

    python docs/analise_pendulo_duplo.py

Modelo (por eixo, pequenos ângulos):
    th1 = ângulo do cabo;  th2 = atitude da esfera (o que o giroscópio mede)
    x_pino = L*th1 ;  x_CM = L*th1 + d*th2
    empuxo horizontal F aplicado a uma altura h ACIMA do CM (h<0 = abaixo)
    M q'' + K q = Q F
      M = m*[[L², L d],[L d, d²]] + diag(0, I)
      K = m g diag(L, d)
      Q = [L, d - h]
Malha: atuador de 1ª ordem (tau = 0,12 s) + DLPF 20 Hz + IIR ~8 Hz.

Parâmetros incertos (d = pino→CM, h = altura do empuxo vs CM, I = inércia de
"bamboleio" em torno do CM) são varridos. Troque pelos valores medidos.
"""

import numpy as np
from scipy.linalg import expm

m, g, L = 12.0, 9.80665, 5.0
TAU_ACT = 0.12
F_DLPF, F_IIR = 20.0, 7.9


def planta(d, I, h):
    M = np.array([[m * L * L, m * L * d], [m * L * d, m * d * d + I]])
    K = np.diag([m * g * L, m * g * d])
    Q = np.array([L, d - h])
    Mi = np.linalg.inv(M)
    return Mi @ K, Mi @ Q


def malha(d, I, h, kp, sinal_vel):
    """sinal_vel: vetor (c1, c2) -> v_medida = c1*L*th1' + c2*th2'.

    firmware atual (eixo X refletido): (0, -L)
    giroscópio com sinal corrigido:    (0, +L)
    velocidade real do CM:             (1,  d)
    """
    MiK, MiQ = planta(d, I, h)
    c1, c2 = sinal_vel
    A = np.zeros((7, 7))  # th1 th2 w1 w2 F y_dlpf y_iir
    A[0, 2] = A[1, 3] = 1
    A[2:4, 0:2] = -MiK
    A[2:4, 4] = MiQ
    a2, a1 = 2 * np.pi * F_DLPF, 2 * np.pi * F_IIR
    A[5, 2], A[5, 3], A[5, 5] = a2 * c1 * L, a2 * c2, -a2
    A[6, 5], A[6, 6] = a1, -a1
    A[4, 6], A[4, 4] = -kp / TAU_ACT, -1 / TAU_ACT
    return A


def modos(A):
    ev = np.linalg.eigvals(A)
    return sorted((abs(e.imag) / (2 * np.pi), -e.real / abs(e)) for e in ev if e.imag > 1e-6)


def tabela(kp=18.0):
    print(f"\n=== Amortecimento ζ dos modos (kp = {kp} N/(m/s)); ζ<0 = INSTÁVEL ===")
    casos = {
        "firmware atual (X refletido)": lambda d: (0.0, -L),
        "gyro c/ sinal corrigido": lambda d: (0.0, +L),
        "velocidade do CM (estimador)": lambda d: (1.0, d),
    }
    for nome, f in casos.items():
        print(f"\n{nome}:")
        for d in (0.3, 0.6):
            for h in (-0.15, 0.0, 0.15):
                ms = modos(malha(d, 1.73, h, kp, f(d)))
                txt = ", ".join(f"{fr:.2f} Hz ζ={z:+.2f}" for fr, z in ms)
                print(f"   d={d:.1f} h={h:+.2f}:  {txt}")


def pulso_wizard(T=0.6, F=5.0):
    """Resposta média do GIROSCÓPIO a um pulso do wizard vs a fórmula de pêndulo simples."""
    om = np.sqrt(g / L)
    simples = F * (1 - np.cos(om * T)) / (m * L * om**2 * T)
    print(f"\n=== Pulso do wizard ({F} N, {T} s): mean(θ') medido / modelo do wizard ===")
    for d in (0.3, 0.6):
        for h in (-0.15, 0.0, 0.15):
            MiK, MiQ = planta(d, 1.73, h)
            A = np.zeros((4, 4))
            A[0, 2] = A[1, 3] = 1
            A[2:, :2] = -MiK
            B = np.zeros(4)
            B[2:] = MiQ * F
            dt = 1e-3
            Ad = expm(A * dt)
            Bd = np.linalg.solve(A, (Ad - np.eye(4)) @ B)
            x, w2 = np.zeros(4), []
            for _ in range(int(T / dt)):
                x = Ad @ x + Bd
                w2.append(x[3])
            print(f"   d={d:.1f} h={h:+.2f}:  razão = {np.mean(w2) / simples:+.2f}"
                  f"  (kf sairia ×{(np.mean(w2) / simples) ** 2:.2f}, sinal {'OK' if np.mean(w2) > 0 else 'INVERTIDO'})")


if __name__ == "__main__":
    tabela()
    pulso_wizard()
