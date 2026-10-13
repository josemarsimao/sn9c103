#!/usr/bin/env python3
"""Testa contraste (CNT, reg 0x05 do sensor) e saturacao (SAT, reg 0x03) em BA81 e S910 (160x120 a 30 fps).

Mede, por valor: brilho medio, desvio padrao (contraste) e a "croma" do mosaico
(|R-G| + |B-G|) / media, que cresce com a saturacao. Se o controle nao mudar nada na Bayer crua,
os numeros ficam praticamente iguais em todos os valores.
Aponte a camera para uma cena PARADA, colorida (cartao de cores), sem estourar. Nao precisa de sudo.
Resultado: rcs_resultado.txt ; imagens em cs/*.png

Uso:  python3 testa_cs.py
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import testa_controles as tc  # noqa: E402

tc.OUT = os.path.join(HERE, 'cs')
tc.PADRAO.update({'contrast': 32, 'saturation': 128})


def croma(q):
    b = q[0::2, 0::2].mean()
    g = 0.5 * (q[0::2, 1::2].mean() + q[1::2, 0::2].mean())
    r = q[1::2, 1::2].mean()
    return (abs(r - g) + abs(b - g)) / max(0.5 * (r + g + b) / 1.5, 1e-3), (b, g, r)


def varre(pf, ctrl, valores):
    print('\n-- %s: %s --' % (pf, ctrl))
    for v in valores:
        q = tc.captura({ctrl: v}, pf)
        if q is None:
            print('%s=%-4d SEM IMAGEM' % (ctrl, v), flush=True)
            continue
        c, (b, g, r) = croma(q)
        print('%s=%-4d media=%6.1f dp=%5.1f  B=%5.1f G=%5.1f R=%5.1f  croma=%.3f' % (ctrl, v, q.mean(), q.std(), b, g, r, c),
              flush=True)
        tc.salva(q, '%s_%s_%d' % (pf, ctrl, v))


def main():
    sys.stdout = tc.Tee(sys.__stdout__, open(os.path.join(HERE, 'rcs_resultado.txt'), 'w'))
    print(__doc__)
    lista = tc.v4l2(['--list-ctrls'])
    for c in ('contrast', 'saturation'):
        print('controle %-12s %s' % (c, 'presente' if c in lista else 'AUSENTE (recompile com ./umilmod.sh)'))
    tc.reset()
    for pf in ('BA81', 'S910'):
        varre(pf, 'contrast', (0, 16, 32, 64, 128, 255))
        varre(pf, 'saturation', (0, 64, 128, 192, 255))
    tc.reset()


if __name__ == '__main__':
    main()
