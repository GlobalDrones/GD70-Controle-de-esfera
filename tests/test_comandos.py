"""Tabela única de comandos (docs/COMANDOS.md): firmware, Rasp, rádio e supervisório
precisam falar exatamente as mesmas letras. Lê os FONTES (sem hardware)."""

import re
from pathlib import Path

from twin import protocol

ROOT = Path(__file__).resolve().parent.parent
FW = (ROOT / "firmware" / "main.cpp").read_text(encoding="utf-8")
SUP = (ROOT / "supervisorio_esfera.html").read_text(encoding="utf-8")
RASP = (ROOT / "Raspberry" / "main.py").read_text(encoding="utf-8")
RADIO = (ROOT / "Raspberry" / "pwm_input.py").read_text(encoding="utf-8")


def _fw_char_commands():
    corpo = FW[FW.index("void comando_char(char c) {"):FW.index("void comando_linha(")]
    return set(re.findall(r"case '(.)'", corpo))


def test_firmware_letras_unicas_sem_alias():
    cmds = _fw_char_commands()
    # um comando, uma letra: sem aliases antigos (b/B/P para anti-balanço, maiúsculas, espaço)
    assert {"b", "B", "P", "W", "S", "X", "A", "D", "C", "E", "Y", "T", " "}.isdisjoint(cmds)
    assert {"w", "s", "x", "a", "d", "c", "e", "m", "y", "p", "k", "z", "Z", "t", "0"} <= cmds


def test_whitelist_sem_digitos_nem_malhas():
    wl = re.search(r'const char WHITELIST\[\] = "([^"]*)"', FW).group(1)
    assert not any(ch.isdigit() for ch in wl)          # SC-10
    assert set(wl) <= set("wsxadcekK")                   # só mecanismos + parada


def test_nomes_de_ganho_iguais_firmware_supervisorio_protocolo():
    fw = re.findall(r'"(\w+)"', FW[FW.index("const char *GAIN_NAMES"):FW.index("const float GAIN_MIN")])
    sup_blk = SUP[SUP.index("const GAIN_NAMES = ["):]
    sup = re.findall(r'"(\w+)"', sup_blk[:sup_blk.index("];")])
    assert fw == sup == list(protocol.FIRMWARE_GAIN_NAMES)


def test_teclas_da_rasp_sao_comandos_do_stm():
    teclas = set(re.search(r'TECLAS_STM = set\("([^"]*)"\)', RASP).group(1))
    assert teclas <= _fw_char_commands()
    # nenhuma tecla local da Rasp pode ser uma letra de comando do STM
    for local in ['ch == "v"', 'ch == "f"', 'ch == ">"', 'ch == "<"', 'ch == "n"', 'ch == "r"', 'ch == "!"']:
        assert local in RASP
        assert local.split('"')[1] not in teclas


def test_radio_usa_letras_do_stm():
    letras = set(re.findall(r'"CH1\d": "(.)"', RADIO)) | set(re.findall(r'COMMANDS\["CH16"\] = "(.)"', RADIO))
    assert letras and letras <= _fw_char_commands()


def test_supervisorio_teclas_diretas_sao_comandos_do_stm():
    blk = re.search(r"TECLAS_STM_DIRETAS = new Set\(\[([^\]]*)\]\)", SUP).group(1)
    assert set(re.findall(r'"(.)"', blk)) <= _fw_char_commands()


def test_tel_v3_parse():
    line = ("TEL,1000,10.00,0.00,0.00,0.10,0.500,-0.200,1.000,0.000,1550,1480,1500,"
            "5.00,-1.00,327,1.25,-0.40,0.80,0.087,0.000")
    f = protocol.parse_line(line)
    assert f.F_cmd_x == 1.25 and f.tz_cmd == 0.8 and f.flags & protocol.FLAG_VEL_ESTIMATOR
    v2 = protocol.parse_line(",".join(line.split(",")[:16]))
    assert v2 is not None and v2.F_cmd_x is None


def test_linha_M_cabe_no_buffer_do_firmware():
    assert len(protocol.cmd_motor_calib(2, 0.5, 0.8660254, -1, 8.148483e-5, 4.889090e-5, 30.0)) <= 60
