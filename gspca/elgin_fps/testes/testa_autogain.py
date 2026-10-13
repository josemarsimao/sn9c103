#!/usr/bin/env python3
"""Testa o ganho automatico do gspca_elgin_fps: liga o automatico, muda o alvo (brilho)
e mede, segundo a segundo, ganho, exposicao, brilho da imagem e FPS.

Nao precisa de sudo (os controles do v4l2-ctl sao livres).
Uso:  python3 testa_autogain.py [FPS_PEDIDO]      (padrao: 30; so BA81 160x120)
"""
import re
import subprocess
import sys
import time

import numpy as np

from elgin_dev import acha_dev
DEV = acha_dev()
W, H = 160, 120
FS = W * H
DUR = 42                                   # segundos de captura
EVENTOS = [(2, 'gain_automatic=1'), (10, 'brightness=60'), (18, 'brightness=200'),
           (26, 'brightness=127'), (34, 'gain_automatic=0')]


def v4l2(args):
    return subprocess.run(['v4l2-ctl', '-d', DEV] + args, capture_output=True, text=True)


def ler_controles():
    r = v4l2(['-C', 'gain,exposure,brightness,gain_automatic'])
    d = dict(re.findall(r'(\w+): (-?\d+)', r.stdout))
    return {k: int(v) for k, v in d.items()}


def main():
    fps_ped = sys.argv[1] if len(sys.argv) > 1 else '30'
    n_quadros = int(float(fps_ped) * (DUR + 4))
    # estado inicial conhecido: manual, brilho e ganho padrao
    v4l2(['-c', 'gain_automatic=0,brightness=127,gain=31,exposure=66'])
    cmd = ['v4l2-ctl', '-d', DEV, '--set-fmt-video=width=%d,height=%d,pixelformat=BA81' % (W, H),
           '--set-parm=%s' % fps_ped, '--stream-mmap', '--stream-count=%d' % n_quadros,
           '--stream-to=/tmp/ag.raw']
    log = open('/tmp/ag.log', 'w')
    t_ini = time.monotonic()
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
    time.sleep(1.0)                                   # a captura precisa estar rodando
    t0 = time.monotonic()
    amostras = []                                      # (t, controles, evento)
    ev = list(EVENTOS)
    while time.monotonic() - t0 < DUR:
        t = time.monotonic() - t0
        evento = ''
        while ev and t >= ev[0][0]:
            evento = ev[0][1]
            v4l2(['-c', ev.pop(0)[1]])
        amostras.append((t, ler_controles(), evento))
        time.sleep(max(0.0, 1.0 - ((time.monotonic() - t0) % 1.0)))
    proc.terminate()
    proc.wait()
    log.close()

    dados = open('/tmp/ag.raw', 'rb').read()
    n = len(dados) // FS
    q = np.frombuffer(dados[:n * FS], np.uint8).reshape(n, H, W).astype(np.float32)
    fps_log = [float(x) for x in re.findall(r'([0-9.]+) fps', open('/tmp/ag.log').read())]
    fps = float(np.mean(fps_log)) if fps_log else float(fps_ped)
    # janela de AE (02 03 0F 0C, em blocos de 32 px de um quadro 640x480) na escala 1/4
    ae = q[:, 24:96, 16:120]
    atraso = 1.0 + (time.monotonic() - t_ini - 1.0 - DUR) * 0   # quadros comecam ~1 s antes de t0
    print('quadros: %d | fps medio: %.1f (pedido %s)' % (n, fps, fps_ped))
    print('%5s %-18s %5s %9s %5s %12s %10s' % ('t(s)', 'evento', 'ganho', 'exposicao', 'brilho', 'media imagem', 'media AE'))
    for t, c, evento in amostras:
        i = int((t + 1.0) * fps)                        # quadro correspondente a este instante
        if i + 5 > n:
            break
        print('%5.0f %-18s %5d %9d %5d %12.1f %10.1f' % (
            t, evento, c.get('gain', -1), c.get('exposure', -1), c.get('brightness', -1),
            q[i:i + 5].mean(), ae[i:i + 5].mean()))
    print('\nfps por segundo (do v4l2-ctl): min %.1f  max %.1f' % (min(fps_log), max(fps_log)) if fps_log else '')
    v4l2(['-c', 'gain_automatic=0,brightness=127,gain=31,exposure=66'])      # deixa no padrao manual


if __name__ == '__main__':
    main()
