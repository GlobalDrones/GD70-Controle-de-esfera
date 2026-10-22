"""Malha fechada do firmware REAL (firmware/main.cpp) contra plantas simuladas.

- harness.cpp: esfera RÍGIDA ao cabo — o anti-balanço tem que amortecer em qualquer
  direção e heading nos 3 modos de velocidade (o firmware antigo divergia — SC-01).
- harness_duplo.cpp: esfera presa por PINO (pêndulo duplo), hélice abaixo do CG, âncora
  (drone) que se move — o modo 2 (cabo relativo + freio do bamboleio, SC-20) tem que
  amortecer e nunca divergir, inclusive com o empuxo real 30% diferente do modelo.
Pulado se não houver compilador C++ no PATH.
"""

import os
import shutil
import subprocess
from pathlib import Path

import pytest

HOST = Path(__file__).resolve().parent.parent / "firmware" / "host_sim"
CXX = shutil.which("g++") or shutil.which("clang++")

pytestmark = pytest.mark.skipif(CXX is None, reason="sem compilador C++ no PATH")


def _build(tmp_path_factory, src):
    exe = tmp_path_factory.mktemp("fw") / src.replace(".cpp", "")
    subprocess.run(
        [CXX, "-std=gnu++17", "-O2", "-w", "-I.", src, "-o", str(exe)],
        cwd=HOST, check=True, capture_output=True, text=True,
    )
    return exe


@pytest.fixture(scope="module")
def harness(tmp_path_factory):
    return _build(tmp_path_factory, "harness.cpp")


@pytest.fixture(scope="module")
def harness_duplo(tmp_path_factory):
    return _build(tmp_path_factory, "harness_duplo.cpp")


def _run(exe, *args, env=None):
    out = subprocess.run([str(exe), *map(str, args)], capture_output=True, text=True,
                         timeout=180, env=env).stdout
    swing = [l.split() for l in out.splitlines() if l.startswith(("SWING", "DRONE"))]
    yaw = [l.split() for l in out.splitlines() if l.startswith("YAW")]
    return swing, yaw


@pytest.mark.parametrize("mode", [0, 1, 2])
def test_rigido_amortece_em_todas_as_direcoes(harness, mode):
    swing, _ = _run(harness, mode)
    assert len(swing) == 9
    for row in swing:
        picos = [float(v) for v in row[5:]]
        # empurrão de 0,3 m/s → 12-17 cm; tem que cair abaixo de 5 cm (zona de repouso ~3 cm)
        assert picos[0] < 20.0, row
        assert max(picos[2:]) < 5.0, row


def test_heading_assenta_sem_empurrao_lateral(harness):
    _, yaw = _run(harness)
    assert abs(float(yaw[3][2])) < 2.0
    assert abs(float(yaw[-1][2])) < 1.5


# (d pino->CG, h hélice-CG, I, rigidez do pino, kf real/modelo, L)
CASOS_DUPLO = [
    (0.15, -0.3, 2.0, 0, 1.0, 5),     # geometria informada: CG perto do pino, pino solto
    (0.10, -0.45, 1.5, 0, 1.3, 10),   # cabo longo, empuxo 30% maior que o modelo
    (0.25, -0.45, 1.5, 0, 1.3, 10),   # caso que divergia com o observador rápido
    (0.15, -0.15, 3.0, 350, 0.7, 5),  # pino rígido, empuxo 30% menor
]


@pytest.mark.parametrize("caso", CASOS_DUPLO)
def test_pendulo_duplo_modo2_amortece_e_nao_diverge(harness_duplo, caso):
    env = dict(os.environ, D_EST="0.15")
    rows, _ = _run(harness_duplo, 2, *caso, env=env)
    assert len(rows) == 3
    for row in rows:
        picos = [float(v) for v in row[2:]]
        assert max(picos[3:]) < 15.0, (caso, row)          # sem ciclo-limite / divergência
        assert picos[-1] <= max(picos[:2]) + 1e-6, (caso, row)


def test_pendulo_duplo_modo2_melhor_que_gyro(harness_duplo):
    env = dict(os.environ, D_EST="0.15")
    caso = (0.15, -0.3, 2.0, 0, 1.0, 5)
    g, _ = _run(harness_duplo, 0, *caso, env=env)
    c, _ = _run(harness_duplo, 2, *caso, env=env)
    drone_g = float(g[-1][-1])
    drone_c = float(c[-1][-1])
    assert drone_c < 0.25 * drone_g, (drone_g, drone_c)
