#!/usr/bin/env python3
"""Janela da exposicao automatica da ponte (0x1C AE_STRX, 0x1D AE_STRY, 0x1E AE_ENDX, 0x1F AE_ENDY).
Unidade: 32 pixels/linhas do sensor (320x240 -> x 0..10, y 0..7).
Para cada janela: grava os 4 registradores (regs_start), espera a exposicao se acomodar, mede a media
total e das 4 metades/quadrantes da imagem, e SEMPRE volta aos valores originais antes da proxima
(com uma medida da base entre elas, para ver se ha deriva).

CENA: aponte a camera para algo com contraste forte e PARADO, por exemplo metade clara (janela, tela
branca) e metade escura (parede escura), sem mexer durante o teste (~6 min). Pede sudo uma vez.
Resultado: varredura_ae_resultado.txt e imagens em varredura_ae/*.png
Uso: python3 varredura_ae.py
"""
import os
import subprocess
import sys

import numpy as np

import varredura as vr
import varredura_ponte as vp

vr.OUT = os.path.join(vr.HERE, 'varredura_ae')
# (nome, strx, stry, endx, endy)
JANELAS = [
    ('original 60fps', 1, 2, 7, 6),
    ('tela cheia', 0, 0, 10, 7),
    ('metade esquerda', 0, 0, 5, 7),
    ('metade direita', 5, 0, 10, 7),
    ('metade de cima', 0, 0, 10, 3),
    ('metade de baixo', 0, 4, 10, 7),
    ('centro pequeno', 4, 3, 6, 5),
    ('canto sup. esq.', 0, 0, 2, 2),
    ('canto inf. dir.', 6, 5, 10, 7),
    ('invertida (ini>fim)', 7, 7, 1, 1),
]


def itens(j):
    return ','.join('b:0x%02x=0x%02x' % (r, v) for r, v in zip((0x1c, 0x1d, 0x1e, 0x1f), j))


def quad(img):
    h, w = img.shape
    return [img[:h // 2, :w // 2].mean(), img[:h // 2, w // 2:].mean(),
            img[h // 2:, :w // 2].mean(), img[h // 2:, w // 2:].mean()]


def linha(nome, c):
    m = c['m']
    if not m:
        return '%-20s sem imagem' % nome
    q = quad(m['img'])
    return '%-20s fps=%-5s media=%5.1f | sup.esq=%5.1f sup.dir=%5.1f inf.esq=%5.1f inf.dir=%5.1f' % (
        nome, 'x' if c['fps'] is None else '%.1f' % c['fps'], m['mean'], *q)


def main():
    sys.stdout = vr.Tee(sys.__stdout__, open(os.path.join(vr.HERE, 'varredura_ae_resultado.txt'), 'w'))
    if subprocess.run(['sudo', '-v']).returncode:
        sys.exit('sudo nao validado')
    print(__doc__)
    vr.regs_start('')
    p = vr.stream_bg(vr.FPS * 30)
    import time
    time.sleep(3)
    orig = [vp.rdb(r) for r in (0x1c, 0x1d, 0x1e, 0x1f)]
    p.terminate()
    p.communicate()
    print('originais lidos (1c 1d 1e 1f): %s' % ' '.join('%02x' % v for v in orig))
    rorig = itens(orig)
    try:
        b = vr.caso(rorig)
        print(linha('BASE', b))
        if b['m']:
            vr.salva_png('base', b['m'])
        for nome, *j in JANELAS:
            c = vr.caso(itens(j))
            print(linha('%s %s' % (nome, tuple(j)), c), flush=True)
            if c['m']:
                vr.salva_png(nome.replace(' ', '_').replace('(', '').replace(')', '').replace('>', ''), c['m'])
            b = vr.caso(rorig)
            print(linha('  base apos', b), flush=True)
    finally:
        vr.regs_start(rorig)
        c = vr.caso(rorig)
        vr.regs_start('')
        print('\nverificacao final: %s' % linha('restaurado', c))


if __name__ == '__main__':
    main()
