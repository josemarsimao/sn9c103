#!/usr/bin/env python3
"""Testa bits de registradores no modo 60 fps (S910 160x120), um caso de cada vez.

Cada caso muda UM ou DOIS bits sobre a configuracao do driver no modo 60 fps (regs_start)
e mede: fps real, se o quadro decodifica e as medias das 4 posicoes do mosaico
com vermelho, verde e azul (como testa_cores.py). Mostre cada cor cobrindo a imagem quando
o script pedir. Pede sudo uma vez (escreve o parametro regs_start). ~5 minutos.
Resultado: bits_resultado.txt   (no fim, o regs_start e' limpo)

Uso:  python3 testa_bits.py            (todos os casos)
      python3 testa_bits.py c2d_b5     (so os casos cujo nome contem o texto)
"""
import os
import re
import statistics
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from sn9c10x_decode import decode  # noqa: E402

from elgin_dev import acha_dev
DEV = acha_dev()
P = '/sys/module/gspca_elgin_fps/parameters/regs_start'
W, H = 160, 120
POS = [(0, 0), (0, 1), (1, 0), (1, 1)]
# (nome, regs_start, o que muda)
CASOS = [
    ('base', '', 'modo 60 fps do driver (COMC=A0 COMH=80 COMJ=C1)'),
    ('c14_b6', 's:0x14=0xe0', '0x14 bit 6 = 1 (reservado)'),
    ('c14_b1', 's:0x14=0xa2', '0x14 bit 1 = 1 (reservado)'),
    ('c14_b0', 's:0x14=0xa1', '0x14 bit 0 = 1 (reservado)'),
    ('c28_b6', 's:0x28=0xc0', '0x28 bit 6 = 1 (BW: preto e branco)'),
    ('c28_b3', 's:0x28=0x88', '0x28 bit 3 = 1 (reservado)'),
    ('c28_b2', 's:0x28=0x84', '0x28 bit 2 = 1 (reservado)'),
    ('c2d_b5', 's:0x2d=0xe1', '0x2D bit 5 = 1 (reservado), com bit 6 = 1'),
    ('c2d_b5s6', 's:0x2d=0xa1', '0x2D bit 5 = 1 e bit 6 = 0 (par do bit 6?)'),
    ('c2d_b3', 's:0x2d=0xc9', '0x2D bit 3 = 1 (reservado)'),
    ('c2d_b1', 's:0x2d=0xc3', '0x2D bit 1 = 1 (reservado)'),
    ('c2d_b0', 's:0x2d=0xc0', '0x2D bit 0 = 0 (reservado; o padrao do sensor tem 1)'),
    ('c16_00', 's:0x16=0x00', '0x16 FSD modo 00 (bits 1:0 = 0, OFF)'),
    ('c18_s10', 'b:0x18=0xaf', 'ponte 0x18 SCAL = 10 (bits 5:4)'),
]
CORES = ['VERMELHO', 'VERDE', 'AZUL']
if len(sys.argv) > 1:
    CASOS[:] = [c for c in CASOS if any(a in c[0] for a in sys.argv[1:])]


class Tee:
    def __init__(self, *fs):
        self.fs = fs

    def write(self, s):
        for f in self.fs:
            f.write(s)

    def flush(self):
        for f in self.fs:
            f.flush()


def regs_start(v):
    subprocess.run(['sudo', '-n', 'tee', P], input=(v + '\n').encode(), capture_output=True)


def v4l2(args, timeout=30):
    try:
        return subprocess.run(['v4l2-ctl', '-d', DEV] + args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return 'TIMEOUT'


FMT = ['--set-fmt-video=width=%d,height=%d,pixelformat=S910' % (W, H), '--set-parm=60', '--stream-mmap']


def mede_fps():
    out = v4l2(FMT + ['--stream-count=150', '--stream-to=/dev/null'], timeout=20)
    if out == 'TIMEOUT':
        return None
    f = [float(x) for x in re.findall(r'<+ ([0-9.]+) fps', out)]
    return statistics.median(f) if f else 0.0


def medias():
    out = v4l2(FMT + ['--stream-skip=8', '--stream-count=1', '--stream-to=/tmp/bits.raw'], timeout=15)
    d = open('/tmp/bits.raw', 'rb').read() if os.path.exists('/tmp/bits.raw') else b''
    if out == 'TIMEOUT' or not d:
        return None
    try:
        bayer, _ = decode(d, W, H)
        q = np.frombuffer(bytes(bayer), np.uint8).reshape(H, W).astype(np.float32)
    except Exception:
        return None
    return [float(q[r::2, c::2].mean()) for r, c in POS]


def main():
    sys.stdout = Tee(sys.__stdout__, open(os.path.join(HERE, 'bits_resultado.txt'), 'w'))
    if subprocess.run(['sudo', '-v']).returncode:
        sys.exit('sudo nao validado')
    print(__doc__)
    res = {c[0]: {} for c in CASOS}
    fps = {}
    try:
        for cor in CORES:
            input('\nMostre %s cobrindo TODA a imagem e aperte Enter (mantenha ate o proximo pedido)... ' % cor)
            for nome, rs, _ in CASOS:
                regs_start(rs)
                if cor == CORES[0]:
                    fps[nome] = mede_fps()
                res[nome][cor] = medias()
                m = res[nome][cor]
                print('  %-9s %-8s %s' % (nome, cor, ('(0,0)=%5.1f (0,1)=%5.1f (1,0)=%5.1f (1,1)=%5.1f' % tuple(m)) if m else 'sem imagem'))
    finally:
        regs_start('')

    print('\n==== RESUMO ====')
    print('%-9s %-48s %-7s %s' % ('caso', 'o que muda', 'fps', 'cor: linhas distintas? (0 = linhas iguais, so 2 cores)  mosaico visto'))
    for nome, rs, desc in CASOS:
        f = fps.get(nome)
        fs = 'travou' if f is None else '%.1f' % f
        r = res[nome]
        if any(r.get(c) is None for c in CORES):
            print('%-9s %-48s %-7s sem imagem em alguma cor' % (nome, desc, fs))
            continue
        tot = sum(sum(r[c]) for c in CORES) or 1.0
        rd = max(abs((r[c][0] + r[c][1]) - (r[c][2] + r[c][3])) for c in CORES) / tot * 4
        quem = []
        for c, letra in (('VERMELHO', 'R'), ('VERDE', 'G'), ('AZUL', 'B')):
            quem.append('%s=%s' % (letra, ''.join(str(i) for i, v in enumerate(r[c]) if v >= 0.7 * max(r[c]))))
        print('%-9s %-48s %-7s %.2f   posicoes mais fortes: %s' % (nome, desc, fs, rd, ' '.join(quem)))
    print('\nPosicoes: 0=(0,0) 1=(0,1) 2=(1,0) 3=(1,1). Um mosaico completo mostra R, G e B em posicoes diferentes e linhas distintas.')


if __name__ == '__main__':
    main()
