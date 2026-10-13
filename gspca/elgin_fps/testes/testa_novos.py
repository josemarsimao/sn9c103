#!/usr/bin/env python3
"""Testa vertical_flip, alvos do AWB (0x0C/0x0D) e filtro de banda (160x120, 30 fps, BA81 e S910).

Usa as funcoes de testa_controles.py (reseta controles a cada captura).
Vflip: compara a fase do mosaico (B/R deve ficar parecida; se trocar, ajustar VFLIP_VSTART_ADJ para 1)
e mede a correlacao da imagem invertida. AWB: com AWB ligado, varia os alvos e mede a razao B/R.
Banda: compara o desvio padrao/faixas (efeito sutil; so confirma que o controle responde).
Cena parada, colorida. Resultado: rnovos_resultado.txt
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import testa_controles as tc  # noqa: E402

PAD = {'vertical_flip': 0, 'band_stop_filter': 0, 'awb_target_blue': 32, 'awb_target_red': 32}


def cap(extra, pf):
    d = dict(PAD)
    d.update(extra)
    return tc.captura(d, pf)


def vflip(pf):
    print('\n-- vertical_flip em %s --' % pf)
    ref = cap({}, pf)
    v1 = cap({'vertical_flip': 1}, pf)
    tc.linha('%s padrao' % pf, ref)
    tc.linha('%s vflip=1' % pf, v1)
    if ref is None or v1 is None:
        return
    m0, r0 = tc.fase(ref)
    m1, r1 = tc.fase(v1)
    print('fase padrao: %s B/R=%.2f' % (['%.1f' % x for x in m0], r0))
    print('fase vflip : %s B/R=%.2f  (parecida = fase ok; ~1/B/R trocado = ajustar VFLIP_VSTART_ADJ)' % (['%.1f' % x for x in m1], r1))
    esp = ref[::-1, :]
    melhor = max(((tc.corr(esp[max(0, d):tc.H + min(0, d)], v1[max(0, -d):tc.H - max(0, d)]), d) for d in range(-4, 5)))
    print('melhor correlacao da imagem invertida: %.2f no deslocamento %d linhas' % melhor)


def awb(pf):
    print('\n-- alvos do AWB em %s (awb ligado) --' % pf)
    for nome in ('awb_target_blue', 'awb_target_red'):
        for v in (8, 32, 56):
            q = cap({nome: v, 'white_balance_automatic': 1}, pf)
            if q is None:
                print('%s=%d SEM IMAGEM' % (nome, v))
                continue
            m, r = tc.fase(q)
            print('%-16s=%2d media=%6.1f B/R=%.2f fase=%s' % (nome, v, q.mean(), r, ['%.1f' % x for x in m]))


def banda(pf):
    print('\n-- band_stop_filter em %s --' % pf)
    for v in (0, 1):
        q = cap({'band_stop_filter': v}, pf)
        tc.linha('%s banda=%d' % (pf, v), q)


def main():
    sys.stdout = tc.Tee(sys.__stdout__, open(os.path.join(HERE, 'rnovos_resultado.txt'), 'w'))
    print(__doc__)
    lista = tc.v4l2(['--list-ctrls'])
    for c in PAD:
        print('controle %-18s %s' % (c, 'presente' if c in lista else 'AUSENTE (recompile com ./umilmod.sh)'))
    tc.PADRAO.update(PAD)
    for pf in ('BA81', 'S910'):
        vflip(pf)
        awb(pf)
        banda(pf)
    tc.v4l2(['--set-ctrl=' + ','.join('%s=%s' % kv for kv in PAD.items())])
    print('\nfim')


if __name__ == '__main__':
    main()
