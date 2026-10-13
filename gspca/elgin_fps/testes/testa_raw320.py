#!/usr/bin/env python3
"""Investiga os quadros cortados do BA81 320x240 a 10 fps.

Para cada configuracao, captura 6 quadros isolados e mostra o tamanho de cada um
(cheio = 76800 bytes). Usa sudo para escrever os parametros do modulo (pede a senha uma vez).
Resultado tambem em raw320_resultado.txt.

Uso:  python3 testa_raw320.py
"""
import os
import subprocess
import sys

from elgin_dev import acha_dev

DEV = acha_dev()

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS = '/sys/module/gspca_elgin_fps/parameters'
W, H = 320, 240
FS = W * H
PADRAO = dict(top_alt=1, regs_start='', stock_regs=0, stock_sensor=0, sensor_reg11=-1)

CASOS = [
    ('padrao (alt 8, hstart 2, janela de AE ligada)', {}),
    ('alt 7 (top_alt=0)', {'top_alt': 0}),
    ('hstart 1 (como antes da correcao de cor)', {'regs_start': 'b:0x12=0x01'}),
    ('janela de AE zerada (como antes)', {'regs_start': 'b:0x1c=0,b:0x1d=0,b:0x1e=0,b:0x1f=0'}),
    ('alt 7 + hstart 1 + janela de AE zerada (estado de antes)',
     {'top_alt': 0, 'regs_start': 'b:0x12=0x01,b:0x1c=0,b:0x1d=0,b:0x1e=0,b:0x1f=0'}),
    ('registradores do gspca original (stock_regs=1)', {'stock_regs': 1}),
]


class Tee:
    def __init__(self, *fs):
        self.fs = fs

    def write(self, s):
        for f in self.fs:
            f.write(s)

    def flush(self):
        for f in self.fs:
            f.flush()


def setp(p):
    for k, v in p.items():
        r = subprocess.run(['sudo', '-n', 'tee', '%s/%s' % (PARAMS, k)], input=(str(v) + '\n').encode(),
                           capture_output=True)
        if r.returncode:
            sys.exit('nao consegui escrever %s=%s: %s' % (k, v, r.stderr.decode().strip()))


def um_quadro(parm):
    subprocess.run(['v4l2-ctl', '-d', DEV, '--set-fmt-video=width=%d,height=%d,pixelformat=BA81' % (W, H),
                    '--set-parm=%s' % parm, '--stream-mmap', '--stream-skip=4', '--stream-count=1',
                    '--stream-to=/tmp/r320.raw'], capture_output=True, timeout=60)
    return os.path.getsize('/tmp/r320.raw') if os.path.exists('/tmp/r320.raw') else 0


def main():
    sys.stdout = Tee(sys.__stdout__, open(os.path.join(HERE, 'raw320_resultado.txt'), 'w'))
    if subprocess.run(['sudo', '-v']).returncode:
        sys.exit('sudo nao validado')
    subprocess.run(['v4l2-ctl', '-d', DEV, '-c', 'gain_automatic=1'], capture_output=True)
    print('BA81 %dx%d, 6 quadros isolados por caso (cheio = %d bytes)\n' % (W, H, FS))
    print('%-58s %-4s %s' % ('configuracao', 'fps', 'tamanho dos quadros'))
    for nome, extra in CASOS:
        p = dict(PADRAO)
        p.update(extra)
        setp(p)
        for parm in ('10', '8'):
            tam = [um_quadro(parm) for _ in range(6)]
            ok = sum(1 for t in tam if t == FS)
            print('%-58s %-4s %s  -> %d de 6 cheios' % (nome[:58], parm, tam, ok))
    setp(PADRAO)
    print('\nparametros devolvidos ao padrao.')


if __name__ == '__main__':
    main()
