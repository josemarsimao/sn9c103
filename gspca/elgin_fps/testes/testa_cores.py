#!/usr/bin/env python3
"""Descobre qual posicao do mosaico Bayer e vermelha, verde ou azul, usando cor SOLIDA,
em seis modos: BA81 e S910 em 160x120, 320x240 e 640x480.

Mostre a cor cobrindo toda a imagem da camera (tela do celular em brilho maximo, papel
colorido bem iluminado, etc.) e aperte Enter. Para cada cor o script captura os seis modos
(~25 s); mantenha a cor ate ele pedir a proxima. Compara as 4 posicoes dentro do mesmo
quadro, entao nao importa o ganho. Nao precisa de sudo. Resultado: cores_resultado.txt

Uso:  python3 testa_cores.py [BA81:320 S910:160 ...]     (sem argumentos: os sete modos)
      python3 testa_cores.py S910:160x240              (modo novo 160x240, janela 320x480)
      FPS60=1 python3 testa_cores.py S910:160           (S910 160x120 no modo de 60 fps)
"""
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from sn9c10x_decode import decode  # noqa: E402

from elgin_dev import acha_dev
DEV = acha_dev()
POS = [(0, 0), (0, 1), (1, 0), (1, 1)]
NOMES = {'BGGR': ['B', 'G', 'G', 'R'], 'GBRG': ['G', 'B', 'R', 'G'],
         'GRBG': ['G', 'R', 'B', 'G'], 'RGGB': ['R', 'G', 'G', 'B']}
V4L2 = {'BGGR': 'SBGGR8 (BA81)', 'GBRG': 'SGBRG8', 'GRBG': 'SGRBG8', 'RGGB': 'SRGGB8'}
MODOS = [('BA81', 160, 120), ('BA81', 320, 240), ('BA81', 640, 480),
         ('S910', 160, 120), ('S910', 320, 240), ('S910', 640, 480), ('S910', 160, 240)]
COR_PEDIDA = ['BRANCO', 'VERMELHO', 'VERDE', 'AZUL']
if len(sys.argv) > 1:                       # ex.: python3 testa_cores.py BA81:320 S910:640
    MODOS[:] = [m for m in MODOS if '%s:%dx%d' % m in sys.argv[1:] or (m[2] != 240 or m[1] != 160) and '%s:%d' % (m[0], m[1]) in sys.argv[1:]]


class Tee:
    """Escreve na tela e num arquivo, para o resultado ficar guardado."""
    def __init__(self, *fs):
        self.fs = fs

    def write(self, s):
        for f in self.fs:
            f.write(s)

    def flush(self):
        for f in self.fs:
            f.flush()


def v4l2(args):
    return subprocess.run(['v4l2-ctl', '-d', DEV] + args, capture_output=True, text=True)


def medias(fmt, w, h):
    """Media das 4 posicoes do mosaico para um modo."""
    opt = ['--set-fmt-video=width=%d,height=%d,pixelformat=%s' % (w, h, fmt), '--stream-mmap']
    if fmt == 'S910' and w == 160 and h == 120 and os.environ.get('FPS60'):    # FPS60=1: testa o modo de 60 fps
        opt.insert(1, '--set-parm=60')
    if fmt == 'BA81':
        n = {160: 100, 320: 40, 640: 10}[w]            # ~3 a 5 s em cada modo
        v4l2(opt + ['--stream-count=%d' % n, '--stream-to=/tmp/cor.raw'])
        d = open('/tmp/cor.raw', 'rb').read()
        fs = w * h
        k = len(d) // fs
        if k < 3:
            return None
        q = np.frombuffer(d[:k * fs], np.uint8).reshape(k, h, w).astype(np.float32)[-min(k, 6):]
    else:
        v4l2(opt + ['--stream-skip=8', '--stream-count=1', '--stream-to=/tmp/cor.raw'])
        d = open('/tmp/cor.raw', 'rb').read()
        if not d:
            return None
        bayer, _ = decode(d, w, h)
        q = np.frombuffer(bytes(bayer), np.uint8).reshape(1, h, w).astype(np.float32)
    return [float(q[:, r::2, c::2].mean()) for r, c in POS]


def padrao(res):
    """Deduz o padrao a partir das respostas a vermelho, verde e azul."""
    r = int(np.argmax(res['VERMELHO']))
    b = int(np.argmax(res['AZUL']))
    g = sorted(int(i) for i in np.argsort(res['VERDE'])[-2:])
    achado = [None] * 4
    achado[r], achado[b] = 'R', 'B'
    for i in g:
        achado[i] = achado[i] or 'G'
    nome = next((k for k, v in NOMES.items() if v == achado), None)
    contr = [max(res[c]) / max(min(res[c]), 1.0) for c in ('VERMELHO', 'VERDE', 'AZUL')]
    return nome, achado, contr


def main():
    sys.stdout = Tee(sys.__stdout__, open(os.path.join(HERE, 'cores_resultado.txt'), 'w'))
    v4l2(['-c', 'gain_automatic=1,brightness=127,red_balance=32,blue_balance=32'])
    print(__doc__)
    res = {m: {} for m in MODOS}
    for cor in COR_PEDIDA:
        input('\nMostre %s cobrindo TODA a imagem e aperte Enter (e mantenha ate o proximo pedido)... ' % cor)
        for m in MODOS:
            r = medias(*m)
            res[m][cor] = r
            print('  %-4s %3dx%-3d  (0,0)=%6.1f  (0,1)=%6.1f  (1,0)=%6.1f  (1,1)=%6.1f' % ((m[0], m[1], m[2]) + tuple(
                r if r else [float('nan')] * 4)))

    print('\n==== RESULTADO POR MODO ====')
    print('%-4s %-8s %-8s %-6s %-28s %s' % ('fmt', 'tamanho', 'mosaico', 'padrao', 'formato V4L2', 'contraste R/G/B'))
    for m in MODOS:
        if any(res[m].get(c) is None for c in COR_PEDIDA):
            print('%-4s %3dx%-3d  sem imagem em alguma cor' % m)
            continue
        nome, achado, contr = padrao(res[m])
        grade = '%s%s/%s%s' % tuple(a or '?' for a in achado)
        print('%-4s %3dx%-3d %-8s %-6s %-28s %.1fx %.1fx %.1fx' % (
            m[0], m[1], m[2], grade, nome or '??', V4L2.get(nome, 'inconsistente'), *contr))
    print('\nEsperado apos a correcao do hstart: BGGR em todos os modos (BA81 anuncia SBGGR8).')


if __name__ == '__main__':
    main()
