#!/usr/bin/env python3
"""Testa o balanco de branco manual (red_balance / blue_balance) do gspca_elgin_fps.

Captura BA81 160x120 e, a cada segundo, muda um dos ganhos; mede a media de cada canal
(R, G, B do mosaico Bayer BGGR) e mostra a razao R/G e B/G. Nao precisa de sudo.
Uso:  python3 testa_wb.py
"""
import subprocess
import time

import numpy as np

from elgin_dev import acha_dev
DEV = acha_dev()
W, H = 160, 120
FS = W * H
# (controle, valor) aplicado a cada segundo; o primeiro e' a referencia
# sensor OV7630: RED (reg 0x02) padrao 0x3A=58, BLUE (reg 0x01) padrao 0x77=119, faixa 0 a 255
PASSOS = ([('red_balance', v) for v in (58, 0, 32, 100, 160, 255, 58)] +
          [('blue_balance', v) for v in (0, 60, 119, 180, 255, 119)])


def v4l2(args):
    return subprocess.run(['v4l2-ctl', '-d', DEV] + args, capture_output=True, text=True)


def main():
    v4l2(['-c', 'gain_automatic=0,gain=31,exposure=66,red_balance=58,blue_balance=119'])
    dur = len(PASSOS) + 3
    proc = subprocess.Popen(['v4l2-ctl', '-d', DEV, '--set-fmt-video=width=%d,height=%d,pixelformat=BA81' % (W, H),
                             '--set-parm=30', '--stream-mmap', '--stream-count=%d' % (30 * (dur + 2)),
                             '--stream-to=/tmp/wb.raw'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.5)
    t0 = time.monotonic()
    marcas = []
    for i, (ctl, val) in enumerate(PASSOS):
        v4l2(['-c', '%s=%d' % (ctl, val)])
        marcas.append((time.monotonic() - t0, ctl, val))
        time.sleep(max(0.0, (i + 1) - (time.monotonic() - t0)))
    proc.terminate()
    proc.wait()
    d = open('/tmp/wb.raw', 'rb').read()
    n = len(d) // FS
    q = np.frombuffer(d[:n * FS], np.uint8).reshape(n, H, W).astype(np.float32)
    fps = 30.0
    off = 1.5                                   # os quadros comecam ~1,5 s antes de t0
    print('%-13s %5s | %7s %7s %7s | %6s %6s' % ('controle', 'valor', 'R', 'G', 'B', 'R/G', 'B/G'))
    for t, ctl, val in marcas:
        a = int((t + off + 0.4) * fps)         # janela de 0,5 s, depois da escrita assentar
        b = a + 15
        if b > n:
            break
        s = q[a:b]
        B = s[:, 0::2, 0::2].mean()                 # BGGR: B em (0,0)
        G = (s[:, 0::2, 1::2].mean() + s[:, 1::2, 0::2].mean()) / 2
        R = s[:, 1::2, 1::2].mean()                 # R em (1,1)
        print('%-13s %5d | %7.1f %7.1f %7.1f | %6.2f %6.2f' % (ctl, val, R, G, B, R / G, B / G))
    v4l2(['-c', 'red_balance=58,blue_balance=119'])


if __name__ == '__main__':
    main()
