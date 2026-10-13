#!/usr/bin/env python3
"""Testa os ganhos R, G e B da ponte (regs 0x05/0x06/0x07, por POSICAO do mosaico) nos 4 formatos Bayer.

Para cada formato (BA81, GBRG, GRBG, RGGB; 160x120, 30 fps) liga AWB e auto-ganho desligados, deixa os 3 ganhos em
0x20 e muda um de cada vez para 0x10 e 0x40. Mede a media dos 4 pontos do mosaico (0,0) (0,1) (1,0) (1,1) e diz
em quais posicoes o ganho agiu. Esperado:
  BA81/RGGB: green_gain age nos dois G; red_gain age em R e B juntos; blue_gain nao age
  GBRG: red_gain -> so R; blue_gain -> so B; green_gain -> os dois G
  GRBG: idem GBRG
Aponte a camera para uma cena PARADA, colorida, sem estourar. Resultado: rganhos_resultado.txt
"""
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import testa_controles as tc  # noqa: E402

W, H = tc.W, tc.H
RAW = '/tmp/rganhos.raw'
BASE = {'white_balance_automatic': 0, 'gain_automatic': 0, 'red_gain': 32, 'green_gain': 32, 'blue_gain': 32}
COR = {'BA81': ['B', 'G', 'G', 'R'], 'GBRG': ['G', 'B', 'R', 'G'], 'GRBG': ['G', 'R', 'B', 'G'],
       'RGGB': ['R', 'G', 'G', 'B']}


def cap(pf, ctrls):
    try:
        os.remove(RAW)
    except OSError:
        pass
    c = dict(tc.PADRAO)
    c.update(BASE)
    c.update(ctrls)
    args = tc.fmt(pf) + ['--set-ctrl=' + ','.join('%s=%s' % kv for kv in c.items())]
    tc.v4l2(args + ['--stream-mmap', '--stream-skip=20', '--stream-count=1', '--stream-to=' + RAW], timeout=40)
    d = open(RAW, 'rb').read() if os.path.exists(RAW) else b''
    if len(d) < W * H:
        return None
    return np.frombuffer(d[:W * H], np.uint8).reshape(H, W).astype(np.float32)


def pos(q):
    return [q[i::2, j::2].mean() for i in (0, 1) for j in (0, 1)]


def main():
    sys.stdout = tc.Tee(sys.__stdout__, open(os.path.join(HERE, 'rganhos_resultado.txt'), 'w'))
    print(__doc__)
    lista = tc.v4l2(['--list-ctrls'])
    for c in BASE:
        print('controle %-24s %s' % (c, 'presente' if c in lista else 'AUSENTE'))
    for pf in ('BA81', 'GBRG', 'GRBG', 'RGGB'):
        print('\n== %s  mosaico (0,0)(0,1)(1,0)(1,1) = %s ==' % (pf, COR[pf]))
        ref = cap(pf, {})
        if ref is None:
            print('SEM IMAGEM (formato nao aceito?)')
            continue
        m0 = pos(ref)
        print('padrao      %s' % ['%.1f' % x for x in m0])
        for nome in ('red_gain', 'green_gain', 'blue_gain'):
            for v in (16, 64):
                q = cap(pf, {nome: v})
                if q is None:
                    print('%s=%d SEM IMAGEM' % (nome, v))
                    continue
                m = pos(q)
                var = [m[k] / max(m0[k], 1e-3) for k in range(4)]
                agiu = [COR[pf][k] + '@%d' % k for k in range(4) if abs(var[k] - 1) > 0.15]
                print('%-10s=%2d  %s  razao %s  agiu em: %s' % (nome, v, ['%.1f' % x for x in m],
                                                              ['%.2f' % x for x in var], agiu or 'nenhuma'))
    tc.v4l2(['--set-ctrl=' + ','.join('%s=%s' % kv for kv in {'red_gain': 32, 'green_gain': 32, 'blue_gain': 32,
                                                              'gain_automatic': 1, 'white_balance_automatic': 1}.items())])
    print('\nfim')


if __name__ == '__main__':
    main()
