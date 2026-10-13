#!/usr/bin/env python3
"""Testa os controles novos: black_level, gamma, hflip e test_pattern (160x120 a 30 fps, BA81 e S910).

Cada captura comeca com TODOS os controles no padrao (os valores ficam gravados no driver entre
execucoes) e muda so o que esta sendo testado. O ganho/exposicao automaticos ficam como estao.
Para o espelho compara a fase do mosaico (B/R) e a imagem invertida, em BA81 e em S910 decodificado,
e depois muda os controles COM o fluxo ligado.
Aponte a camera para uma cena PARADA, colorida, com detalhe, sem estourar. Nao precisa de sudo.
Resultado: rcontroles_resultado.txt ; imagens em controles/*.png

Uso:  python3 testa_controles.py
"""
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from sn9c10x_decode import decode  # noqa: E402
from elgin_dev import acha_dev  # noqa: E402

DEV = acha_dev()
W, H = 160, 120
OUT = os.path.join(HERE, 'controles')
RAW = '/tmp/rctl.raw'
PADRAO = {'black_level': 10, 'gamma': 2, 'horizontal_flip': 0, 'test_pattern': 0}


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


def fmt(pf):
    return ['--set-fmt-video=width=%d,height=%d,pixelformat=%s' % (W, H, pf), '--set-parm=30']


def reset():
    v4l2(['--set-ctrl=' + ','.join('%s=%s' % kv for kv in PADRAO.items())])


def captura(ctrls, pf='BA81', skip=30):
    """Captura um quadro com os controles no padrao, exceto 'ctrls'. Devolve a matriz Bayer."""
    try:
        os.remove(RAW)
    except OSError:
        pass
    c = dict(PADRAO)
    c.update(ctrls)
    args = fmt(pf) + ['--set-ctrl=' + ','.join('%s=%s' % kv for kv in c.items())]
    v4l2(args + ['--stream-mmap', '--stream-skip=%d' % skip, '--stream-count=1', '--stream-to=' + RAW], timeout=40)
    d = open(RAW, 'rb').read() if os.path.exists(RAW) else b''
    if len(d) < 100:
        return None
    try:
        if pf == 'BA81':
            if len(d) < W * H:
                return None
            return np.frombuffer(d[:W * H], np.uint8).reshape(H, W).astype(np.float32)
        b, _ = decode(d, W, H)
        return np.frombuffer(bytes(b), np.uint8).reshape(H, W).astype(np.float32)
    except Exception:
        return None


def corr(a, b):
    a = a.ravel() - a.mean()
    b = b.ravel() - b.mean()
    d = float(np.sqrt((a * a).sum() * (b * b).sum()))
    return float((a * b).sum() / d) if d > 1e-6 else 0.0


def fase(q):
    """Medias dos 4 pontos do mosaico (0,0) (0,1) (1,0) (1,1); razao (0,0)/(1,1) = B/R no BGGR."""
    m = [q[i::2, j::2].mean() for i in (0, 1) for j in (0, 1)]
    return m, m[0] / max(m[3], 1e-3)


def salva(q, nome):
    try:
        from PIL import Image
        os.makedirs(OUT, exist_ok=True)
        Image.fromarray(np.clip(q, 0, 255).astype(np.uint8)).resize((W * 3, H * 3), Image.NEAREST).save(
            os.path.join(OUT, nome + '.png'))
    except Exception:
        pass


def linha(nome, q):
    if q is None:
        print('%-30s SEM IMAGEM' % nome, flush=True)
        return
    print('%-30s media=%6.1f dp=%5.1f' % (nome, q.mean(), q.std()), flush=True)
    salva(q, nome.replace(' ', '_').replace('=', ''))


def espelho(pf):
    print('\n-- horizontal_flip em %s: a fase do mosaico deve se manter --' % pf)
    ref = captura({}, pf)
    h1 = captura({'horizontal_flip': 1}, pf)
    linha('%s padrao' % pf, ref)
    linha('%s horizontal_flip=1' % pf, h1)
    if ref is None or h1 is None:
        return
    m0, r0 = fase(ref)
    m1, r1 = fase(h1)
    print('fase sem espelho: %s  B/R=%.2f' % (['%.1f' % x for x in m0], r0))
    print('fase com espelho: %s  B/R=%.2f  (deve ser parecida)' % (['%.1f' % x for x in m1], r1))
    esp = ref[:, ::-1]
    melhor = max(((corr(esp[:, max(0, d):W + min(0, d)], h1[:, max(0, -d):W - max(0, d)]), d) for d in range(-4, 5)))
    print('melhor correlacao da imagem espelhada: %.2f no deslocamento %d px' % melhor)


def main():
    sys.stdout = Tee(sys.__stdout__, open(os.path.join(HERE, 'rcontroles_resultado.txt'), 'w'))
    print(__doc__)
    lista = v4l2(['--list-ctrls'])
    for c in ('black_level', 'gamma', 'horizontal_flip', 'test_pattern'):
        print('controle %-16s %s' % (c, 'presente' if c in lista else 'AUSENTE (recompile com ./umilmod.sh)'))
    reset()
    print('\n-- referencia (tudo no padrao) --')
    ref = captura({})
    linha('padrao', ref)
    if ref is None:
        return
    print('\n-- black_level (padrao 10): maior = mais escuro --')
    for v in (0, 10, 30, 60):
        linha('black_level=%d' % v, captura({'black_level': v}))
    print('\n-- gamma (0..8; 2 = linear): menor = mais escuro nas sombras --')
    for v in (0, 2, 5, 8):
        linha('gamma=%d' % v, captura({'gamma': v}))
    print('\n-- test_pattern (1 = ponte, 2 = barras do sensor) --')
    for v in (1, 2):
        linha('test_pattern=%d' % v, captura({'test_pattern': v}))
    espelho('BA81')
    espelho('S910')
    reset()
    print('\n-- ao vivo (S910): muda os controles com o fluxo ligado e grava quadros --')
    p = subprocess.Popen(['v4l2-ctl', '-d', DEV] + fmt('S910') + ['--stream-mmap', '--stream-count=300', '--stream-to=/dev/null'],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    time.sleep(2)
    for c in ('gamma=8', 'black_level=60', 'horizontal_flip=1', 'test_pattern=1', 'test_pattern=2', 'test_pattern=0',
              'horizontal_flip=0', 'black_level=10', 'gamma=2'):
        r = v4l2(['--set-ctrl=' + c])
        print('  %-20s %s' % (c, 'ok' if not r.strip() else r.strip()))
        time.sleep(0.5)
    try:
        p.wait(timeout=40)
        print('fluxo terminou normalmente (codigo %s)' % p.returncode)
    except subprocess.TimeoutExpired:
        p.kill()
        print('fluxo TRAVOU com os ajustes ao vivo')
    reset()


if __name__ == '__main__':
    main()
