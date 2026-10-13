#!/usr/bin/env python3
"""Testa o balanco de branco: automatico do sensor (AWB) e manual (R e B do sensor, G da ponte).

1) mostra os 4 controles e se estao inativos com o AWB ligado;
2) AWB desligado: varre red/blue/green e mede R/G, B/G (cada um deve agir so na sua cor);
3) AWB ligado: mede R/G e B/G depois de 90 quadros (tempo para convergir) e confere que mexer em
   red_balance nao muda nada (controle inativo).
Aponte a camera para uma folha BRANCA ou cinza, parada, sob luz de cor (lampada amarelada, por ex.),
sem estourar. Com o AWB funcionando, R/G e B/G devem ficar mais perto de 1 que com o AWB desligado.
Nao precisa de sudo. Resultado: rawb_resultado.txt ; imagens em awb/*.png

Uso:  python3 testa_awb.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import testa_controles as tc  # noqa: E402

tc.OUT = os.path.join(HERE, 'awb')
tc.PADRAO.update({'white_balance_automatic': 0, 'red_balance': 58, 'blue_balance': 119, 'green_balance': 32})


def razoes(q):
    b = q[0::2, 0::2].mean()
    g = 0.5 * (q[0::2, 1::2].mean() + q[1::2, 0::2].mean())
    r = q[1::2, 1::2].mean()
    return r, g, b


def mede(nome, ctrls, pf, skip=30):
    q = tc.captura(ctrls, pf, skip)
    if q is None:
        print('%-34s SEM IMAGEM' % nome, flush=True)
        return None
    r, g, b = razoes(q)
    print('%-34s R=%5.1f G=%5.1f B=%5.1f   R/G=%.2f B/G=%.2f' % (nome, r, g, b, r / max(g, 1e-3), b / max(g, 1e-3)), flush=True)
    tc.salva(q, '%s_%s' % (pf, nome.replace(' ', '_').replace('=', '').replace(',', '_')))
    return r / max(g, 1e-3), b / max(g, 1e-3)


def main():
    sys.stdout = tc.Tee(sys.__stdout__, open(os.path.join(HERE, 'rawb_resultado.txt'), 'w'))
    print(__doc__)
    tc.reset()
    print('-- controles (com o AWB desligado devem estar ativos) --')
    tc.v4l2(['--set-ctrl=white_balance_automatic=0'])
    for l in tc.v4l2(['--list-ctrls']).splitlines():
        if 'balance' in l:
            print(l.strip())
    tc.v4l2(['--set-ctrl=white_balance_automatic=1'])
    print('-- com o AWB LIGADO (red/blue/green devem estar inactive) --')
    for l in tc.v4l2(['--list-ctrls']).splitlines():
        if 'balance' in l:
            print(l.strip())
    for pf in ('BA81', 'S910'):
        print('\n== %s, AWB desligado: cada ganho age so na sua cor ==' % pf)
        for v in (0, 58, 160, 255):
            mede('red_balance=%d' % v, {'red_balance': v}, pf)
        for v in (0, 119, 255):
            mede('blue_balance=%d' % v, {'blue_balance': v}, pf)
        for v in (0, 32, 80, 127):
            mede('green_balance=%d' % v, {'green_balance': v}, pf)
        print('\n== %s, AWB LIGADO (90 quadros para convergir) ==' % pf)
        a = mede('awb=1', {'white_balance_automatic': 1}, pf, 90)
        b = mede('awb=1 red_balance=0 (ignorado)', {'white_balance_automatic': 1, 'red_balance': 0}, pf, 90)
        if a and b:
            print('diferenca R/G com red_balance=0 e AWB ligado: %.2f (deve ser ~0)' % abs(a[0] - b[0]))
    tc.reset()


if __name__ == '__main__':
    main()
