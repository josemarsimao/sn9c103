#!/usr/bin/env python3
"""Escrita + leitura de volta na ponte SN9C103, so para ver QUAIS registradores guardam valor.
Durante a captura (S910 160x120), para cada endereco: le o original, grava 0x00 e le, grava 0xFF e le,
grava 0x55 e 0xAA e le, e restaura o original. Sem medir imagem (rapido).
Padrao: 0x40-0x7F. Lista opcional em hexa:  python3 varredura_ponte_escrita.py 31-3f 40-7f
Nunca toca 0x00,0x01,0x08,0x09-0x0e,0x17,0x19. No fim mede fps/brilho como verificacao.
Resultado: varredura_ponte_escrita_resultado.txt
"""
import os
import subprocess
import sys
import time

import varredura as vr
import varredura_ponte as vp

PW = '/sys/module/gspca_elgin_fps/parameters/regw'
ARGS = [a for a in sys.argv[1:] if not a.startswith('--')]
PROIB = {0x00, 0x01, 0x08, 0x17, 0x19} | set(range(0x09, 0x0f))


def wr(reg, val):
    vr.sudo_tee(PW, 'b:0x%02x=0x%02x' % (reg, val))


def lista():
    if not ARGS:
        ARGS.append('40-7f')
    out = []
    for a in ARGS:
        lo, _, hi = a.partition('-')
        out += range(int(lo, 16), int(hi or lo, 16) + 1)
    return [r for r in out if r not in PROIB]


def main():
    sys.stdout = vr.Tee(sys.__stdout__, open(os.path.join(vr.HERE, 'varredura_ponte_escrita_resultado.txt'), 'w'))
    if subprocess.run(['sudo', '-v']).returncode:
        sys.exit('sudo nao validado')
    print(__doc__)
    vr.regs_start('')
    base = vr.caso('')
    print('base: %s' % vr.fmt(base))
    p = vr.stream_bg(vr.FPS * 120)
    time.sleep(3)
    print('reg   orig  =00  =ff  =55  =aa   resultado')
    guardam = []
    try:
        for reg in lista():
            o = vp.rdb(reg)
            if o is None:
                print('0x%02x  leitura falhou' % reg)
                continue
            lido = []
            for v in (0x00, 0xff, 0x55, 0xaa):
                wr(reg, v)
                lido.append(vp.rdb(reg))
            wr(reg, o)
            msk = 0
            for v, l in zip((0x00, 0xff, 0x55, 0xaa), lido):
                if l is not None:
                    msk |= (l ^ 0) if v == 0xff else 0
            res = 'GUARDA' if any(l == v for v, l in zip((0x00, 0xff, 0x55, 0xaa), lido)) else ''
            if lido[1] is not None and lido[1] not in (0, 0xff):
                res = 'GUARDA parcial (mascara 0x%02x)' % lido[1]
            if res:
                guardam.append(reg)
            print('0x%02x  %02x   %s   %s' % (reg, o, '  '.join('--' if l is None else '%02x' % l for l in lido), res), flush=True)
    finally:
        p.terminate()
        p.communicate()
        vr.regs_start('')
        c = vr.caso('')
        print('\nregistradores que guardam valor: %s' % (' '.join('0x%02x' % r for r in guardam) or 'nenhum'))
        print('verificacao final: %s' % vr.fmt(c))


if __name__ == '__main__':
    main()
