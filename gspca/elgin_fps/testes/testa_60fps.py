#!/usr/bin/env python3
"""Tenta 60 fps no S910 160x120, ligando o modo de um so campo do sensor por etapas.

Para cada configuracao (via regs_start, que exige sudo) mede o FPS real, a limpeza do quadro
(sobra de bytes no decodificador), o brilho e a semelhanca com o quadro de referencia (cena parada),
e guarda um PNG em testes/60fps/. Pede a senha do sudo uma vez e devolve os parametros ao padrao.
Resultado: 60fps_resultado.txt

Etapas (sensor OV7630 / ponte SN9C103):
  s:0x2d=0xc1   COMJ bit 6: so o campo impar, ate 60 fps (padrao 0x81)
  s:0x14=0xa0   COMC bit 5: sensor em QVGA (padrao 0x80)
  b:0x15=0x14, b:0x16=0x0f   janela da ponte 320x240 (padrao 640x480 = 0x28, 0x1e)
  b:0x18=0x9f   escala 1/2 (para sair 160x120 de uma janela de 320x240; padrao 0xaf = 1/4)

Uso:  python3 testa_60fps.py [1|2]      (padrao: etapa 2; a etapa 1 ja foi rodada)
"""
import os
import re
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from sn9c10x_decode import decode, bayer_to_ppm  # noqa: E402

from elgin_dev import acha_dev
DEV = acha_dev()
PARAMS = '/sys/module/gspca_elgin_fps/parameters'
W, H = 160, 120
QV = 's:0x14=0xa0'
JAN = 'b:0x15=0x14,b:0x16=0x0f,b:0x18=0x9f'
COMJ = 's:0x2d=0xc1'
CASOS_1 = [
    ('A  referencia (nada)', ''),
    ('B  so COMJ bit 6', COMJ),
    ('C  so sensor em QVGA', QV),
    ('D  QVGA + janela 320x240 e escala 1/2 (30 fps, valida o caminho)', QV + ',' + JAN),
    ('E  QVGA + COMJ bit 6', QV + ',' + COMJ),
    ('F  QVGA + COMJ bit 6 + janela 320x240 e escala 1/2  (alvo: 60 fps)', QV + ',' + COMJ + ',' + JAN),
]
# Etapa 2: o datasheet diz que o COMJ bit 6 so da 60 fps em modo ENTRELACADO (COMH bit 5 = 0).
# O Windows grava COMH = 0xa0 (uma linha + progressivo). Aqui: 0x80 = uma linha + entrelacado.
CASOS_2 = [
    ('A  referencia (nada)', ''),
    ('D  QVGA + janela 320x240 e escala 1/2 (30 fps)', QV + ',' + JAN),
    ('G  D + COMJ bit 6 + COMH 0x80 (entrelacado)  (alvo: 60 fps)', QV + ',' + COMJ + ',s:0x28=0x80,' + JAN),
    ('H  D + COMH 0x80 (entrelacado), sem COMJ', QV + ',s:0x28=0x80,' + JAN),
    ('I  so COMH 0x80 em VGA (entrelacado)', 's:0x28=0x80'),
    ('J  D + COMJ bit 6 + COMH 0x00 (duas linhas; a ponte nao entende)', QV + ',' + COMJ + ',s:0x28=0x00,' + JAN),
]
ETAPA = sys.argv[1] if len(sys.argv) > 1 else '2'
CASOS = CASOS_1 if ETAPA == '1' else CASOS_2


class Tee:
    def __init__(self, *fs):
        self.fs = fs

    def write(self, s):
        for f in self.fs:
            f.write(s)

    def flush(self):
        for f in self.fs:
            f.flush()


def sudo_tee(nome, valor):
    r = subprocess.run(['sudo', '-n', 'tee', '%s/%s' % (PARAMS, nome)], input=(str(valor) + '\n').encode(),
                       capture_output=True)
    if r.returncode:
        sys.exit('nao consegui escrever %s: %s' % (nome, r.stderr.decode().strip()))


def v4l2(args, **k):
    return subprocess.run(['v4l2-ctl', '-d', DEV] + args, capture_output=True, text=True, **k)


def medir():
    fmt = '--set-fmt-video=width=%d,height=%d,pixelformat=S910' % (W, H)
    r = v4l2([fmt, '--set-parm=30', '--stream-mmap', '--stream-count=240', '--stream-to=/dev/null'], timeout=120)
    f = re.findall(r'([0-9.]+) fps', r.stdout + r.stderr)
    fps = float(f[-1]) if f else None
    v4l2([fmt, '--set-parm=30', '--stream-mmap', '--stream-skip=12', '--stream-count=1', '--stream-to=/tmp/f60.raw'],
         timeout=120)
    d = open('/tmp/f60.raw', 'rb').read() if os.path.exists('/tmp/f60.raw') else b''
    if not d:
        return fps, None
    bayer, bits = decode(d, W, H)
    return fps, (len(d), len(d) - bits / 8, np.frombuffer(bytes(bayer), np.uint8).reshape(H, W).astype(np.float32))


def main():
    sys.stdout = Tee(sys.__stdout__, open(os.path.join(HERE, '60fps_resultado_etapa%s.txt' % ETAPA), 'w'))
    if subprocess.run(['sudo', '-v']).returncode:
        sys.exit('sudo nao validado')
    os.makedirs(os.path.join(HERE, '60fps_etapa%s' % ETAPA), exist_ok=True)
    v4l2(['-c', 'gain_automatic=1'])
    print('S910 %dx%d, pedindo 30 fps (n=0); a taxa real vem do v4l2-ctl\n' % (W, H))
    print('%-72s %7s %8s %7s %6s %6s %s' % ('configuracao', 'fps', 'bytes', 'sobra', 'media', 'corr', 'regs_start'))
    ref = None
    for nome, regs in CASOS:
        sudo_tee('regs_start', regs)
        fps, q = medir()
        if q is None:
            print('%-72s %7s  sem quadro            %s' % (nome[:72], fps, regs))
            continue
        tam, sobra, img = q
        if ref is None:
            ref = img
        a = ref.reshape(H // 2, 2, W // 2, 2).mean(axis=(1, 3)).ravel()
        b = img.reshape(H // 2, 2, W // 2, 2).mean(axis=(1, 3)).ravel()
        corr = float(np.corrcoef(a, b)[0, 1]) if a.std() > 0 and b.std() > 0 else float('nan')
        print('%-72s %7s %8d %7.1f %6.1f %6.2f %s' % (nome[:72], fps, tam, sobra, img.mean(), corr, regs))
        pref = os.path.join(HERE, '60fps_etapa%s' % ETAPA, nome.split()[0])
        open(pref + '_bayer.raw', 'wb').write(bytes(img.astype(np.uint8).tobytes()))
        w2, h2 = bayer_to_ppm(img.astype(np.uint8).tobytes(), W, H, pref + '.ppm')
        subprocess.run(['convert', pref + '.ppm', '-filter', 'point', '-resize', '640x480', '-normalize', pref + '.png'],
                       capture_output=True)
    sudo_tee('regs_start', '')
    print('\nregs_start devolvido ao padrao. PNGs em testes/60fps_etapa%s/.' % ETAPA)


if __name__ == '__main__':
    main()
