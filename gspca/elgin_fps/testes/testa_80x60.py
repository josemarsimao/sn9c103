#!/usr/bin/env python3
"""Testa o modo 80x60 (BA81 cru e S910 comprimido) a 30 e a 60 fps.

Para cada caso mede o fps real, decodifica um quadro e confere se e' imagem (correlacao entre pixels
vizinhos de mesma cor: perto de 1 = imagem, perto de 0 = ruido), alem de brilho e tamanho do quadro.
Imagens em 80x60/*.png (ampliadas 6x). Aponte a camera para uma cena PARADA, com detalhe, sem estourar.
Nao precisa de sudo. Resultado: r80x60_resultado.txt  (~1 minuto)

Uso:  python3 testa_80x60.py
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
W, H = 80, 60
OUT = os.path.join(HERE, '80x60')
RAW = '/tmp/r80x60.raw'
CASOS = [('BA81', 30), ('BA81', 60), ('S910', 30), ('S910', 60)]


class Tee:
    def __init__(self, *fs):
        self.fs = fs

    def write(self, s):
        for f in self.fs:
            f.write(s)

    def flush(self):
        for f in self.fs:
            f.flush()


def v4l2(args, timeout=30):
    try:
        return subprocess.run(['v4l2-ctl', '-d', DEV] + args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return 'TIMEOUT'


def fmt(pf, parm):
    return ['--set-fmt-video=width=%d,height=%d,pixelformat=%s' % (W, H, pf), '--set-parm=%d' % parm, '--stream-mmap']


def corr(a, b):
    a = a.ravel() - a.mean()
    b = b.ravel() - b.mean()
    d = float(np.sqrt((a * a).sum() * (b * b).sum()))
    return float((a * b).sum() / d) if d > 1e-6 else 0.0


def main():
    sys.stdout = Tee(sys.__stdout__, open(os.path.join(HERE, 'r80x60_resultado.txt'), 'w'))
    print(__doc__)
    print(v4l2(['--list-formats-ext']).split('Index')[0][:0], end='')
    anuncia = v4l2(['--list-formats-ext'])
    print('80x60 anunciado pelo driver:', 'SIM' if '80x60' in anuncia else 'NAO (recompile com ./umilmod.sh)')
    for pf, parm in CASOS:
        out = v4l2(fmt(pf, parm) + ['--stream-count=%d' % (parm * 4), '--stream-to=/dev/null'], timeout=40)
        f = [float(x) for x in re.findall(r'<+ ([0-9.]+) fps', out)]
        fps = statistics.median(f) if f else 0.0
        try:
            os.remove(RAW)
        except OSError:
            pass
        v4l2(fmt(pf, parm) + ['--stream-skip=8', '--stream-count=1', '--stream-to=' + RAW], timeout=20)
        d = open(RAW, 'rb').read() if os.path.exists(RAW) else b''
        if not d:
            print('%s pede %2d fps -> %.1f fps   SEM IMAGEM' % (pf, parm, fps))
            continue
        try:
            if pf == 'BA81':
                q = np.frombuffer(d[:W * H], np.uint8).reshape(H, W).astype(np.float32)
            else:
                b, _ = decode(d, W, H)
                q = np.frombuffer(bytes(b), np.uint8).reshape(H, W).astype(np.float32)
        except Exception as e:
            print('%s pede %2d fps -> %.1f fps   bytes=%d  FALHA AO DECODIFICAR (%s)' % (pf, parm, fps, len(d), e))
            continue
        c = 0.5 * (corr(q[:, :-2], q[:, 2:]) + corr(q[:-2, :], q[2:, :]))
        print('%s pede %2d fps -> %5.1f fps  bytes=%5d  media=%5.1f dp=%5.1f  corr=%.2f  %s' % (
            pf, parm, fps, len(d), q.mean(), q.std(), c, 'IMAGEM' if c > 0.6 else 'RUIDO'), flush=True)
        try:
            from PIL import Image
            os.makedirs(OUT, exist_ok=True)
            Image.fromarray(np.clip(q, 0, 255).astype(np.uint8)).resize((W * 6, H * 6), Image.NEAREST).save(
                os.path.join(OUT, '%s_%d.png' % (pf, parm)))
        except Exception:
            pass


if __name__ == '__main__':
    main()
