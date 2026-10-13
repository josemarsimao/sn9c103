#!/usr/bin/env python3
"""Testa se o ruido a 48 MHz vem da borda de captura (reg 0x18 bit 0) ou do clock invertido (reg 0x17 bit 6).

No modo S910 160x120 pedindo 60 fps o driver grava 0x17=0x68 (24 MHz) e 0x18=0x9F. Casos (via regs_start):
  controle 24 MHz:  0x17=0x68/0x18=0x9F (atual) · 0x68/0x9E (captura na descida) · 0x28/0x9F (clock do sensor sem inversao)
  48 MHz:           0x6C/0x9F (ja medido: ruido) · 0x6C/0x9E · 0x2C/0x9F · 0x2C/0x9E
Para cada caso mede fps, bytes por quadro, taxa USB e a CORRELACAO entre pixels vizinhos de mesma cor
(imagem de verdade: perto de 1; ruido: perto de 0). Veredito IMAGEM se a correlacao passar de 0,6.

Aponte a camera para uma cena PARADA, com detalhe e sem estourar. ~1,5 minuto. Pede sudo uma vez.
Resultado: clock48_resultado.txt e as imagens em clock48/*.png. No fim o regs_start e' limpo.

Uso:  python3 testa_clock48.py
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
OUT = os.path.join(HERE, 'clock48')
RAW = '/tmp/clock48.raw'
# (rotulo, reg 0x17, reg 0x18)
CASOS = [
    ('24 MHz atual', 0x68, 0x9f),
    ('24 MHz capt. descida', 0x68, 0x9e),
    ('24 MHz clock sem inv.', 0x28, 0x9f),
    ('48 MHz', 0x6c, 0x9f),
    ('48 MHz capt. descida', 0x6c, 0x9e),
    ('48 MHz clock sem inv.', 0x2c, 0x9f),
    ('48 MHz sem inv. + descida', 0x2c, 0x9e),
]
FMT = ['--set-fmt-video=width=%d,height=%d,pixelformat=S910' % (W, H), '--set-parm=60', '--stream-mmap']


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


def mede_fps():
    out = v4l2(FMT + ['--stream-count=300', '--stream-to=/dev/null'], timeout=40)
    if out == 'TIMEOUT':
        return None
    f = [float(x) for x in re.findall(r'<+ ([0-9.]+) fps', out)]
    return statistics.median(f) if f else 0.0


def corr(a, b):
    a = a.ravel() - a.mean()
    b = b.ravel() - b.mean()
    d = float(np.sqrt((a * a).sum() * (b * b).sum()))
    return float((a * b).sum() / d) if d > 1e-6 else 0.0


def quadro():
    try:
        os.remove(RAW)
    except OSError:
        pass
    out = v4l2(FMT + ['--stream-skip=8', '--stream-count=1', '--stream-to=' + RAW], timeout=20)
    if out == 'TIMEOUT' or not os.path.exists(RAW) or os.path.getsize(RAW) == 0:
        return None
    d = open(RAW, 'rb').read()
    try:
        b, _ = decode(d, W, H)
        q = np.frombuffer(bytes(b), np.uint8).reshape(H, W).astype(np.float32)
    except Exception:
        return None
    return dict(mean=float(q.mean()), std=float(q.std()), size=len(d), img=q,
                cor=0.5 * (corr(q[:, :-2], q[:, 2:]) + corr(q[:-2, :], q[2:, :])))


def png(nome, q):
    try:
        from PIL import Image
        os.makedirs(OUT, exist_ok=True)
        Image.fromarray(np.clip(q, 0, 255).astype(np.uint8)).resize((W * 3, H * 3), Image.NEAREST).save(
            os.path.join(OUT, nome + '.png'))
    except Exception:
        pass


def main():
    sys.stdout = Tee(sys.__stdout__, open(os.path.join(HERE, 'clock48_resultado.txt'), 'w'))
    if subprocess.run(['sudo', '-v']).returncode:
        sys.exit('sudo nao validado')
    print(__doc__)
    linhas = []
    try:
        for nome, r17, r18 in CASOS:
            subprocess.run(['sudo', '-v'])
            regs_start('b:0x17=0x%02x,b:0x18=0x%02x' % (r17, r18))
            fps = mede_fps()
            q = quadro()
            if q:
                png('r17_%02x_r18_%02x' % (r17, r18), q['img'])
            linhas.append((nome, r17, r18, fps, q))
            f = 'travou' if fps is None else '%.1f' % fps
            if q:
                print('%-26s 0x17=0x%02x 0x18=0x%02x  %-7s media=%5.1f dp=%5.1f bytes=%5d USB~%4.0f KB/s  corr=%.2f  %s' % (
                    nome, r17, r18, f, q['mean'], q['std'], q['size'], (fps or 0) * q['size'] / 1024.0, q['cor'],
                    'IMAGEM' if q['cor'] > 0.6 else 'RUIDO'), flush=True)
            else:
                print('%-26s 0x17=0x%02x 0x18=0x%02x  %-7s sem imagem' % (nome, r17, r18, f), flush=True)
    finally:
        regs_start('')
        print('\nregs_start limpo.')

    print('\n==== RESUMO ====')
    boas = [(n, f) for n, r17, r18, f, q in linhas if q and q['cor'] > 0.6 and f]
    for n, r17, r18, f, q in linhas:
        v = 'SEM IMAGEM' if not q else ('IMAGEM' if q['cor'] > 0.6 else 'RUIDO')
        print('%-26s fps=%-7s %s' % (n, 'travou' if f is None else '%.1f' % f, v))
    ok48 = [(n, f) for n, f in boas if n.startswith('48')]
    if ok48:
        print('\n48 MHz com imagem limpa: %s' % ', '.join('%s (%.1f fps)' % x for x in ok48))
    else:
        print('\nNenhum caso a 48 MHz devolveu imagem limpa: o limite parece ser o sensor (maximo 40 MHz), nao a captura.')


if __name__ == '__main__':
    main()
