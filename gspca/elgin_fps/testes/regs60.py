#!/usr/bin/env python3
"""Le registradores do SENSOR durante a captura S910 160x120 pedindo 30 e 60 fps e compara.

Mostra se o sensor ficou com COMC=0xa0, COMH=0x80, COMJ=0xc1 (modo de 60 fps) e qual e o fps medido.
Pede o sudo uma vez. Resultado: regs60_resultado.txt

Uso:  python3 regs60.py
"""
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
P = '/sys/module/gspca_elgin_fps/parameters/regr'
REGS = [(0x11, 'CLKRC'), (0x12, 'COMA'), (0x13, 'COMB'), (0x14, 'COMC'), (0x28, 'COMH'), (0x2a, 'FRARH'),
        (0x2b, 'FRARL'), (0x2d, 'COMJ'), (0x10, 'AEC'), (0x00, 'GAIN')]


from elgin_dev import acha_dev

DEV = acha_dev()


class Tee:
    def __init__(self, *fs):
        self.fs = fs

    def write(self, s):
        for f in self.fs:
            f.write(s)

    def flush(self):
        for f in self.fs:
            f.flush()


def rd(reg):
    subprocess.run(['sudo', '-n', 'tee', P], input=('s:0x%02x\n' % reg).encode(), capture_output=True)
    return open(P).read().strip()


def caso(parm):
    cmd = ['v4l2-ctl', '-d', DEV, '--set-fmt-video=width=160,height=120,pixelformat=S910',
           '--set-parm=%d' % parm, '--stream-mmap', '--stream-count=%d' % (parm * 12), '--stream-to=/dev/null']
    pr = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    time.sleep(4)
    vals = {nome: rd(reg) for reg, nome in REGS}
    out = pr.communicate()[0]
    f = re.findall(r'<+ ([0-9.]+) fps', out)	# so as linhas medidas (nao o eco "Frame rate set to")
    return vals, (f[-1] if f else '?')


def main():
    sys.stdout = Tee(sys.__stdout__, open(os.path.join(HERE, 'regs60_resultado.txt'), 'w'))
    if subprocess.run(['sudo', '-v']).returncode:
        sys.exit('sudo nao validado')
    res = {p: caso(p) for p in (30, 60)}
    print('\nregistrador do sensor lido DURANTE a captura (valor bruto devolvido pelo driver)\n')
    print('%-8s %-6s %-26s %-26s' % ('reg', 'nome', 'pedindo 30 fps', 'pedindo 60 fps'))
    for reg, nome in REGS:
        print('0x%02x     %-6s %-26s %-26s' % (reg, nome, res[30][0][nome], res[60][0][nome]))
    print('\nfps medido: pedindo 30 -> %s | pedindo 60 -> %s' % (res[30][1], res[60][1]))
    print('esperado em 60 fps: COMC=0xa0, COMH=0x80, COMJ=0xc1, CLKRC=0x00')


if __name__ == '__main__':
    main()
