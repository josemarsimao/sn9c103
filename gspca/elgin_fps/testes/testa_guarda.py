#!/usr/bin/env python3
"""Testa a trava de banda (controle exposure_dynamic_framerate) com o S910 640x480.

Pede 14 fps e, com o controle LIGADO, acompanha a taxa que o driver informa (G_PARM) segundo a
segundo; depois mede a limpeza dos quadros (sobra de bytes no decodificador) com a trava ligada e
desligada. Funciona melhor com a cena escura (quadros grandes). Nao precisa de sudo.

Uso:  python3 testa_guarda.py
"""
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from sn9c10x_decode import decode  # noqa: E402

from elgin_dev import acha_dev
DEV = acha_dev()
W, H = 640, 480


def v4l2(args, **k):
    return subprocess.run(['v4l2-ctl', '-d', DEV] + args, capture_output=True, text=True, **k)


def taxa_informada():
    m = re.search(r'Frames per second:\s*([0-9.]+)', v4l2(['--get-parm']).stdout)
    return float(m.group(1)) if m else None


def controle():
    return v4l2(['-C', 'exposure_dynamic_framerate,gain,exposure']).stdout.replace('\n', ' ')


def quadros_isolados(n=4, skip=70):
    saidas = []
    for _ in range(n):
        v4l2(['--set-fmt-video=width=%d,height=%d,pixelformat=S910' % (W, H), '--set-parm=14', '--stream-mmap',
              '--stream-skip=%d' % skip, '--stream-count=1', '--stream-to=/tmp/g1.raw'], timeout=120)
        d = open('/tmp/g1.raw', 'rb').read() if os.path.exists('/tmp/g1.raw') else b''
        if not d:
            saidas.append(None)
            continue
        _, bits = decode(d, W, H)
        saidas.append((len(d), len(d) - bits / 8))
    return saidas


def fase(rotulo, ligada):
    print('\n=== %s ===' % rotulo)
    v4l2(['-c', 'exposure_dynamic_framerate=%d,gain_automatic=1' % ligada])
    print('controles:', controle())
    proc = subprocess.Popen(['v4l2-ctl', '-d', DEV, '--set-fmt-video=width=%d,height=%d,pixelformat=S910' % (W, H),
                             '--set-parm=14', '--stream-mmap', '--stream-count=400', '--stream-to=/dev/null'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    t0 = time.monotonic()
    serie = []
    while time.monotonic() - t0 < 14:
        time.sleep(1.0)
        serie.append(taxa_informada())
    proc.terminate()
    proc.wait()
    print('taxa informada pelo driver a cada segundo:', serie)
    q = quadros_isolados()
    ok = [x for x in q if x]
    print('quadros apos assentar (bytes / sobra):', ' | '.join('%d / %.1f' % x if x else 'sem quadro' for x in q))
    print('quadros limpos (sobra <= 1,5): %d de %d' % (sum(1 for x in ok if abs(x[1]) <= 1.5), len(q)))


def main():
    inicio = time.time()
    fase('trava de banda LIGADA (exposure_dynamic_framerate=1)', 1)
    fase('trava de banda DESLIGADA (exposure_dynamic_framerate=0): taxa fixa em 14 fps', 0)
    v4l2(['-c', 'exposure_dynamic_framerate=1'])
    print('\nControle devolvido a 1 (padrao).')
    log = subprocess.run('dmesg | grep "trava de banda" | tail -6', shell=True, capture_output=True, text=True).stdout
    print('\nmensagens do driver (dmesg):\n' + (log or '  (nenhuma)'))
    print('duracao: %.0f s' % (time.time() - inicio))


if __name__ == '__main__':
    main()
