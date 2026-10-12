"""Pacote do digital twin: protocolo, transportes, STM simulado e janela.

O twin (``python -m twin``) e o wizard (``python wizard.py``) falam com o
STM32 através de um :class:`~twin.transport.Transport` — serial USB direto,
TCP até o agente na Raspberry Pi, ou o :class:`~twin.simstm.SimSTM` (STM
simulado, mesma planta/atuador/controlador/protocolo) para uso sem hardware.
"""
