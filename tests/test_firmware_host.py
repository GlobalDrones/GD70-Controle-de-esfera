"""Malha fechada do firmware REAL (firmware/main.cpp) contra uma planta de pêndulo.

Compila firmware/host_sim/harness.cpp (stubs mínimos do Arduino) com o g++/clang do PC
e confere que o anti-balanço AMORTECE em qualquer direção e heading (o firmware anterior
divergia — espelhamento de eixos, AUDITORIA_CONTROLE.md SC-01) e que o heading assenta.
Pulado se não houver compilador C++ no PATH.
"""

import shutil
import subprocess
from pathlib import Path

import pytest

HOST = Path(__file__).resolve().parent.parent / "firmware" / "host_sim"
CXX = shutil.which("g++") or shutil.which("clang++")

pytestmark = pytest.mark.skipif(CXX is None, reason="sem compilador C++ no PATH")


@pytest.fixture(scope="module")
def harness(tmp_path_factory):
    exe = tmp_path_factory.mktemp("fw") / "harness"
    subprocess.run(
        [CXX, "-std=gnu++17", "-O1", "-w", "-I.", "harness.cpp", "-o", str(exe)],
        cwd=HOST, check=True, capture_output=True, text=True,
    )
    return exe


def _run(exe, *args):
    out = subprocess.run([str(exe), *args], capture_output=True, text=True, timeout=120).stdout
    swing = [l.split() for l in out.splitlines() if l.startswith("SWING")]
    yaw = [l.split() for l in out.splitlines() if l.startswith("YAW")]
    return swing, yaw


@pytest.mark.parametrize("mode", ["", "est"])
def test_anti_balanco_amortece_em_todas_as_direcoes(harness, mode):
    swing, _ = _run(harness, *([mode] if mode else []))
    assert len(swing) == 9
    for row in swing:
        picos = [float(v) for v in row[5:]]
        # empurrão de 0,3 m/s → ~12,5 cm; tem que cair abaixo de 5 cm (zona de repouso ~3-4 cm)
        assert picos[0] < 15.0, row
        assert max(picos[2:]) < 5.0, row


def test_heading_assenta_sem_empurrao_lateral(harness):
    _, yaw = _run(harness)
    erro_final = abs(float(yaw[-1][2]))
    erro_2s = abs(float(yaw[3][2]))
    assert erro_2s < 2.0
    assert erro_final < 1.5
