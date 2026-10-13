#!/usr/bin/env python3
"""Testa se o fps muda quando o clock mestre do sensor (reg 0x17 da ponte, bits 3:2) muda.

Casos (modo S910 160x120, pedindo 30 e 60 fps, sem recompilar: usa regs_start):
  0x64 = 12 MHz   0x68 = 24 MHz (o do driver hoje)   0x6C = 48 MHz (acima do maximo de 40 MHz do sensor)
Para cada um mede: fps real, se o quadro decodifica, brilho, contraste, bytes por quadro e a taxa de
dados USB (fps x bytes). Se o fps escalar com o clock, o clock manda no fps.

Aponte a camera para uma cena PARADA, com detalhe e sem estourar. ~2 minutos. Pede sudo uma vez.
Resultado: clock_resultado.txt e as imagens em clock/*.png. No fim o regs_start e' limpo.

Uso:  python3 testa_clock.py
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
from elgin_dev import acha_dev  # noqa: E402

DEV = acha_dev()
PS = '/sys/module/gspca_elgin_fps/parameters/regs_start'
W, H = 160, 120
OUT = os.path.join(HERE, 'clock')
RAW = '/tmp/clock.raw'
CLOCKS = [(0x64, '12 MHz'), (0x68, '24 MHz (atual)'), (0x6c, '48 MHz')]
PEDIDOS = (30, 60)


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
    subprocess.run(['sudo', '-n', 'tee', PS], input=(v + '\n').encode(), capture_output=True)


def v4l2(args, timeout=30):
    try:
        return subprocess.run(['v4l2-ctl', '-d', DEV] + args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return 'TIMEOUT'


def fmt(parm):
    return ['--set-fmt-video=width=%d,height=%d,pixelformat=S910' % (W, H), '--set-parm=%d' % parm, '--stream-mmap']


def mede_fps(parm):
    out = v4l2(fmt(parm) + ['--stream-count=%d' % (parm * 5), '--stream-to=/dev/null'], timeout=40)
    if out == 'TIMEOUT':
        return None
    f = [float(x) for x in re.findall(r'<+ ([0-9.]+) fps', out)]
    return statistics.median(f) if f else 0.0


def quadro(parm):
    try:
        os.remove(RAW)
    except OSError:
        pass
    out = v4l2(fmt(parm) + ['--stream-skip=8', '--stream-count=1', '--stream-to=' + RAW], timeout=20)
    if out == 'TIMEOUT' or not os.path.exists(RAW) or os.path.getsize(RAW) == 0:
        return None
    d = open(RAW, 'rb').read()
    try:
        b, _ = decode(d, W, H)
        q = np.frombuffer(bytes(b), np.uint8).reshape(H, W).astype(np.float32)
    except Exception:
        return None
    return dict(mean=float(q.mean()), std=float(q.std()), size=len(d), img=q)


def png(nome, q):
    try:
        from PIL import Image
        os.makedirs(OUT, exist_ok=True)
        Image.fromarray(np.clip(q, 0, 255).astype(np.uint8)).resize((W * 3, H * 3), Image.NEAREST).save(
            os.path.join(OUT, nome + '.png'))
    except Exception:
        pass


def main():
    sys.stdout = Tee(sys.__stdout__, open(os.path.join(HERE, 'clock_resultado.txt'), 'w'))
    if subprocess.run(['sudo', '-v']).returncode:
        sys.exit('sudo nao validado')
    print(__doc__)
    res = []
    try:
        for val, nome in CLOCKS:
            for parm in PEDIDOS:
                subprocess.run(['sudo', '-v'])
                regs_start('b:0x17=0x%02x' % val)
                fps = mede_fps(parm)
                q = quadro(parm)
                if q:
                    png('r17_%02x_pede%d' % (val, parm), q['img'])
                res.append((val, nome, parm, fps, q))
                f = 'travou' if fps is None else '%.1f' % fps
                if q:
                    kb = (fps or 0) * q['size'] / 1024.0
                    print('0x17=0x%02x %-15s pede %2d fps -> %-7s media=%5.1f dp=%5.1f bytes=%5d  USB~%4.0f KB/s' % (
                        val, nome, parm, f, q['mean'], q['std'], q['size'], kb), flush=True)
                else:
                    print('0x17=0x%02x %-15s pede %2d fps -> %-7s sem imagem' % (val, nome, parm, f), flush=True)
    finally:
        regs_start('')
        print('\nregs_start limpo.')

    print('\n==== RESUMO ====')
    print('%-10s %-15s %-10s %-8s %s' % ('reg 0x17', 'clock', 'pedido', 'fps', 'leitura'))
    base = {p: f for v, n, p, f, q in res if v == 0x68 and f}
    for val, nome, parm, fps, q in res:
        if not q or not fps:
            leitura = 'SEM IMAGEM / TRAVOU'
        elif parm in base and base[parm]:
            leitura = 'fps x %.2f em relacao a 24 MHz' % (fps / base[parm])
        else:
            leitura = ''
        print('0x%02x       %-15s %-10d %-8s %s' % (val, nome, parm, 'travou' if fps is None else '%.1f' % fps, leitura))
    print('\nSe o fps escalou com o clock (x0,5 / x1 / x2) o clock manda no fps. Se a imagem saiu estragada ou sem')
    print('imagem em 48 MHz, o sensor (maximo 40 MHz) ou a ponte nao acompanha. A taxa USB ~ fps x bytes deve ficar')
    print('abaixo de ~900 KB/s; acima disso o quadro e cortado.')


if __name__ == '__main__':
    main()
