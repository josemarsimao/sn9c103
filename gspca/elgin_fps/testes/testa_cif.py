#!/usr/bin/env python3
"""Testa os modos CIF 352x288 e QCIF 176x144 (BA81 cru e S910 comprimido).

Para cada caso: confere se o modo e' anunciado, mede o fps real, decodifica um quadro, calcula a
correlacao entre vizinhos (imagem x ruido) e LOCALIZA a janela dentro de um quadro BA81 640x480 da
mesma cena. Isso confirma as unidades de H_START/V_START: o esperado e' x=144, y=96 (janela
centralizada). Aponte a camera para uma cena PARADA, com detalhe, sem estourar.
Nao precisa de sudo. Resultado: rcif_resultado.txt  (~1-2 minutos). Imagens em cif/*.png

Uso:  python3 testa_cif.py
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
OUT = os.path.join(HERE, 'cif')
RAW = '/tmp/rcif.raw'
# (formato, largura, altura, fps pedido, escala da janela 352x288 -> 1 ou 2)
CASOS = [('BA81', 352, 288, 4, 1), ('BA81', 352, 288, 2, 1), ('BA81', 176, 144, 12, 2), ('BA81', 176, 144, 8, 2),
         ('S910', 352, 288, 15, 1), ('S910', 352, 288, 30, 1), ('S910', 176, 144, 30, 2), ('S910', 176, 144, 15, 2)]


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


def fmt(pf, w, h, parm):
    return ['--set-fmt-video=width=%d,height=%d,pixelformat=%s' % (w, h, pf), '--set-parm=%d' % parm, '--stream-mmap']


def corr(a, b):
    a = a.ravel() - a.mean()
    b = b.ravel() - b.mean()
    d = float(np.sqrt((a * a).sum() * (b * b).sum()))
    return float((a * b).sum() / d) if d > 1e-6 else 0.0


def captura(pf, w, h, parm):
    try:
        os.remove(RAW)
    except OSError:
        pass
    v4l2(fmt(pf, w, h, parm) + ['--stream-skip=4', '--stream-count=1', '--stream-to=' + RAW], timeout=40)
    d = open(RAW, 'rb').read() if os.path.exists(RAW) else b''
    if not d:
        return None, 0
    if pf == 'BA81':
        if len(d) < w * h:
            return None, len(d)
        return np.frombuffer(d[:w * h], np.uint8).reshape(h, w).astype(np.float32), len(d)
    b, _ = decode(d, w, h)
    return np.frombuffer(bytes(b), np.uint8).reshape(h, w).astype(np.float32), len(d)


def bin2(a):
    h, w = a.shape[0] // 2 * 2, a.shape[1] // 2 * 2
    a = a[:h, :w]
    return (a[0::2, 0::2] + a[0::2, 1::2] + a[1::2, 0::2] + a[1::2, 1::2]) / 4.0


def bin_n(a, n):
    h, w = a.shape[0] // n * n, a.shape[1] // n * n
    return a[:h, :w].reshape(h // n, n, w // n, n).mean(axis=(1, 3))


def ncc_busca(ref, tpl, passo):
    """Procura tpl em ref; devolve (x, y, ncc) em pixels de ref (ambos ja' binados por 'passo')."""
    th, tw = tpl.shape
    best = (-2.0, 0, 0)
    t = tpl - tpl.mean()
    tn = float(np.sqrt((t * t).sum()))
    for y in range(ref.shape[0] - th + 1):
        for x in range(ref.shape[1] - tw + 1):
            r = ref[y:y + th, x:x + tw]
            r = r - r.mean()
            d = float(np.sqrt((r * r).sum())) * tn
            v = float((r * t).sum() / d) if d > 1e-6 else 0.0
            if v > best[0]:
                best = (v, x, y)
    return best[1] * passo, best[2] * passo, best[0]


def localiza(ref, q, escala):
    """Acha onde q (CIF ou QCIF) cai em ref (640x480 cru)."""
    t = q if escala == 1 else None
    if escala == 2:
        # QCIF cobre a mesma area do CIF: cada pixel = 2x2 de ref. Compara com ref binado 2x.
        refb = bin2(ref)          # 320x240, cada pixel = 2x2 de ref
        tg = bin2(q)              # 88x72 (QCIF binado): cada pixel = 4x4 de ref
        r4 = bin_n(ref, 4)        # 160x120, mesma escala de tg
        x4, y4, v = ncc_busca(r4, tg, 4)
        return x4, y4, v
    tg = bin_n(t, 4)             # 88x72
    r4 = bin_n(ref, 4)           # 160x120
    x4, y4, v = ncc_busca(r4, tg, 4)
    # refina em passo de 2 px (bin2) ao redor do melhor
    r2 = bin2(ref)
    t2 = bin2(t)
    best = (-2.0, x4, y4)
    for dy in range(-4, 5, 2):
        for dx in range(-4, 5, 2):
            x, y = x4 + dx, y4 + dy
            if x < 0 or y < 0 or x // 2 + t2.shape[1] > r2.shape[1] or y // 2 + t2.shape[0] > r2.shape[0]:
                continue
            r = r2[y // 2:y // 2 + t2.shape[0], x // 2:x // 2 + t2.shape[1]]
            c = corr(r, t2)
            if c > best[0]:
                best = (c, x, y)
    return best[1], best[2], best[0]


def main():
    sys.stdout = Tee(sys.__stdout__, open(os.path.join(HERE, 'rcif_resultado.txt'), 'w'))
    print(__doc__)
    anuncia = v4l2(['--list-formats-ext'])
    for m in ('352x288', '176x144'):
        print('%s anunciado pelo driver: %s' % (m, 'SIM' if m in anuncia else 'NAO (recompile com ./umilmod.sh)'))
    # Referencia em 2 fps tem exposicao longa e estoura; usa S910 nao serve (comprimido). Tenta 3 vezes.
    ref, n = captura('BA81', 640, 480, 2)
    if ref is None:
        print('Sem quadro de referencia 640x480 BA81 (%d bytes). Abortando.' % n)
        return
    print('Referencia 640x480 BA81: media=%.1f dp=%.1f' % (ref.mean(), ref.std()))
    if ref.std() < 8:
        print('CENA PLANA/ESTOURADA: a localizacao nao e confiavel. Aponte para algo com detalhe.')
    os.makedirs(OUT, exist_ok=True)
    try:
        from PIL import Image
        Image.fromarray(np.clip(ref, 0, 255).astype(np.uint8)).save(os.path.join(OUT, 'ref_640x480.png'))
    except Exception:
        Image = None
    for pf, w, h, parm, esc in CASOS:
        out = v4l2(fmt(pf, w, h, parm) + ['--stream-count=%d' % max(6, parm * 3), '--stream-to=/dev/null'], timeout=60)
        f = [float(x) for x in re.findall(r'<+ ([0-9.]+) fps', out)]
        fps = statistics.median(f) if f else 0.0
        try:
            q, nb = captura(pf, w, h, parm)
        except Exception as e:
            print('%s %dx%d pede %2d fps -> %.1f fps  FALHA AO DECODIFICAR (%s)' % (pf, w, h, parm, fps, e))
            continue
        if q is None:
            print('%s %dx%d pede %2d fps -> %.1f fps  SEM IMAGEM (bytes=%d)' % (pf, w, h, parm, fps, nb))
            continue
        c = 0.5 * (corr(q[:, :-2], q[:, 2:]) + corr(q[:-2, :], q[2:, :]))
        x, y, v = localiza(ref, q, esc)
        print('%s %dx%d pede %2d fps -> %5.1f fps  bytes=%6d media=%5.1f dp=%5.1f corr=%.2f %s | janela em x=%d y=%d (esperado 144,96) ncc=%.2f %s' % (
            pf, w, h, parm, fps, nb, q.mean(), q.std(), c, 'IMAGEM' if c > 0.6 else 'RUIDO', x, y, v,
            'OK' if (abs(x - 144) <= 4 and abs(y - 96) <= 4 and v > 0.8) else '??'), flush=True)
        if Image:
            Image.fromarray(np.clip(q, 0, 255).astype(np.uint8)).save(os.path.join(OUT, '%s_%dx%d_%d.png' % (pf, w, h, parm)))


if __name__ == '__main__':
    main()
