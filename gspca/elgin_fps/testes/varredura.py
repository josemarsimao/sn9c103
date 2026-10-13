#!/usr/bin/env python3
"""Varredura de registradores NAO documentados do sensor OV7630 (modo S910 160x120).

Fase 1  leitura: le os 256 enderecos do sensor DURANTE a captura, duas vezes (marca com * os que mudam
        sozinhos) -> mostra quais enderecos respondem.
Fase 2  escrita: para cada registrador reservado do datasheet (07-09 0E-0F 1E 34-4B 4E-5F 69-6E 7B 7E-7F)
        grava 0x00 e depois 0xFF (via regs_start, a cada inicio de captura), e mede: se o stream continua,
        fps, brilho, contraste, diferenca entre linhas e colunas, tamanho comprimido e se o valor "gruda"
        (leitura de volta). O valor original e' restaurado antes do caso seguinte.
Fase 3  bits: nos registradores que mudaram algo, testa os 8 bits um a um (valor = 1 << bit).

Mantenha a camera apontada para uma cena PARADA e bem iluminada durante todo o teste (~15 a 20 min).
Pede sudo uma vez. Resultado: varredura_resultado.txt e as imagens de cada caso em varredura/*.png
Se um registrador deixar o sensor preso, o script avisa (desconecte e reconecte a camera).

Uso:  python3 varredura.py                  (fases 1, 2 e 3 nos registradores reservados)
      python3 varredura.py --so-leitura     (so a fase 1)
      python3 varredura.py 34-4b 7b          (so esses enderecos, em hexadecimal)
      python3 varredura.py --ext             (inclui tambem 0x80-0xFF, +30 min)
      python3 varredura.py --30              (usa 30 fps em vez de 60 fps)
"""
import os
import re
import statistics
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from sn9c10x_decode import decode  # noqa: E402
from elgin_dev import acha_dev  # noqa: E402

DEV = acha_dev()
PR = '/sys/module/gspca_elgin_fps/parameters/regr'
PS = '/sys/module/gspca_elgin_fps/parameters/regs_start'
W, H = 160, 120
OUT = os.path.join(HERE, 'varredura')
RAW = '/tmp/varredura.raw'
ARGS = sys.argv[1:]
FPS = 30 if '--30' in ARGS else 60
FMT = ['--set-fmt-video=width=%d,height=%d,pixelformat=S910' % (W, H), '--set-parm=%d' % FPS, '--stream-mmap']
RESERVADOS = [0x07, 0x08, 0x09, 0x0e, 0x0f, 0x1e] + list(range(0x34, 0x4c)) + list(range(0x4e, 0x60)) \
    + list(range(0x69, 0x6f)) + [0x7b, 0x7e, 0x7f]
VALORES = (0x00, 0xff)


def lista_regs():
    sel = [a for a in ARGS if not a.startswith('--')]
    if not sel:
        return RESERVADOS + (list(range(0x80, 0x100)) if '--ext' in ARGS else [])
    out = []
    for a in sel:
        if '-' in a:
            lo, hi = a.split('-')
            out += list(range(int(lo, 16), int(hi, 16) + 1))
        else:
            out.append(int(a, 16))
    return out


class Tee:
    def __init__(self, *fs):
        self.fs = fs

    def write(self, s):
        for f in self.fs:
            f.write(s)

    def flush(self):
        for f in self.fs:
            f.flush()


def sudo_tee(path, txt):
    subprocess.run(['sudo', '-n', 'tee', path], input=(txt + '\n').encode(), capture_output=True)


def regs_start(v):
    sudo_tee(PS, v)


def rd(reg):
    sudo_tee(PR, 's:0x%02x' % reg)
    try:
        m = re.search(r'= 0x([0-9a-fA-F]{2})', open(PR).read())
    except OSError:
        return None
    return int(m.group(1), 16) if m else None


def v4l2(args, timeout=30):
    try:
        return subprocess.run(['v4l2-ctl', '-d', DEV] + args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return 'TIMEOUT'


def stream_bg(count):
    return subprocess.Popen(['v4l2-ctl', '-d', DEV] + FMT + ['--stream-count=%d' % count, '--stream-to=/dev/null'],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def fps_e_leitura(regs):
    """Captura ~4 s medindo fps; no meio le os registradores pedidos. Retorna (fps, {reg: valor})."""
    p = stream_bg(FPS * 4)
    time.sleep(2.5)
    lido = {r: rd(r) for r in regs}
    try:
        out = p.communicate(timeout=20)[0]
    except subprocess.TimeoutExpired:
        p.kill()
        return None, lido
    f = [float(x) for x in re.findall(r'<+ ([0-9.]+) fps', out)]
    return (statistics.median(f) if f else 0.0), lido


def metricas():
    try:
        os.remove(RAW)
    except OSError:
        pass
    out = v4l2(FMT + ['--stream-skip=8', '--stream-count=1', '--stream-to=' + RAW], timeout=15)
    if out == 'TIMEOUT' or not os.path.exists(RAW) or os.path.getsize(RAW) == 0:
        return None
    d = open(RAW, 'rb').read()
    try:
        b, _ = decode(d, W, H)
        q = np.frombuffer(bytes(b), np.uint8).reshape(H, W).astype(np.float32)
    except Exception:
        return None
    mean = float(q.mean()) + 1e-6
    return dict(mean=mean, std=float(q.std()), rowdiff=abs(float(q[0::2].mean() - q[1::2].mean())) / mean,
                coldiff=abs(float(q[:, 0::2].mean() - q[:, 1::2].mean())) / mean, size=len(d), img=q)


def caso(rs, reg=None):
    subprocess.run(['sudo', '-v'])
    regs_start(rs)
    fps, lido = fps_e_leitura([reg] if reg is not None else [])
    m = metricas()
    return dict(fps=fps, lido=lido.get(reg) if reg is not None else None, m=m)


def salva_png(nome, m):
    try:
        from PIL import Image
        os.makedirs(OUT, exist_ok=True)
        Image.fromarray(np.clip(m['img'], 0, 255).astype(np.uint8)).resize((W * 3, H * 3), Image.NEAREST).save(
            os.path.join(OUT, nome + '.png'))
    except Exception:
        pass


def fmt(c):
    m = c['m']
    f = 'travou' if c['fps'] is None else '%.1f' % c['fps']
    if not m:
        return '%-7s sem imagem' % f
    return '%-7s media=%5.1f dp=%5.1f linhas=%.3f colunas=%.3f bytes=%d' % (f, m['mean'], m['std'], m['rowdiff'],
                                                                           m['coldiff'], m['size'])


class Base:
    def __init__(self, cs):
        ok = [c for c in cs if c['m'] and c['fps']]
        self.ok = bool(ok)
        if not ok:
            return
        self.fps = statistics.median(c['fps'] for c in ok)
        self.mean = statistics.median(c['m']['mean'] for c in ok)
        self.std = statistics.median(c['m']['std'] for c in ok)
        self.row = max(c['m']['rowdiff'] for c in ok)
        self.col = statistics.median(c['m']['coldiff'] for c in ok)
        self.size = statistics.median(c['m']['size'] for c in ok)

    def motivos(self, c):
        m = c['m']
        if not m or not c['fps']:
            return ['PAROU']
        r = []
        if abs(c['fps'] - self.fps) > 4:
            r.append('fps')
        if not 0.6 * self.mean < m['mean'] < 1.6 * self.mean:
            r.append('brilho')
        if not 0.6 * self.std < m['std'] < 1.6 * self.std:
            r.append('contraste')
        if m['rowdiff'] > self.row + 0.05:
            r.append('LINHAS')
        if abs(m['coldiff'] - self.col) > 0.05:
            r.append('colunas')
        if not 0.6 * self.size < m['size'] < 1.6 * self.size:
            r.append('tamanho')
        return r


def fase1():
    print('\n==== FASE 1: leitura dos 256 enderecos do sensor (durante a captura) ====')
    regs_start('')
    p = stream_bg(FPS * 240)
    time.sleep(3)
    a = [rd(r) for r in range(256)]
    b = [rd(r) for r in range(256)]
    p.terminate()
    p.communicate()
    print('       ' + ' '.join('+%x ' % i for i in range(16)))
    for lin in range(16):
        cel = []
        for col in range(16):
            i = lin * 16 + col
            if a[i] is None:
                cel.append('-- ')
            else:
                cel.append('%02x%s' % (a[i], '*' if a[i] != b[i] else ' '))
        print('0x%02x:  %s' % (lin * 16, ' '.join(cel)))
    print('(-- = leitura falhou; * = mudou entre as duas leituras)')
    return a


def main():
    sys.stdout = Tee(sys.__stdout__, open(os.path.join(HERE, 'varredura_resultado.txt'), 'w'))
    if subprocess.run(['sudo', '-v']).returncode:
        sys.exit('sudo nao validado')
    print(__doc__)
    orig = fase1()
    if '--so-leitura' in ARGS:
        return
    regs = lista_regs()
    print('\n==== FASE 2: escrita de %s em %d registradores (modo S910 160x120, %d fps) ====' % (
        '/'.join('0x%02x' % v for v in VALORES), len(regs), FPS))
    resto = ''
    try:
        bases = [caso('') for _ in range(3)]
        for i, c in enumerate(bases):
            print('base %d: %s' % (i + 1, fmt(c)))
        base = Base(bases)
        if not base.ok:
            print('A base nao funciona; confira o driver e a camera.')
            return
        salva_png('base', bases[0]['m'])
        if base.mean > 215 or base.std < 8:
            print('\n*** CENA INADEQUADA: media=%.0f dp=%.1f (imagem lisa ou estourada). Aponte a camera para uma cena\n'
                  '*** com detalhe e cores, sem saturar (ex.: varanda de lado, objeto colorido) e rode de novo. ***' % (
                      base.mean, base.std))
            return
        achados = {}
        linhas = []
        n = 0
        for reg in regs:
            for val in VALORES:
                if orig[reg] == val:
                    continue
                rs = ','.join(x for x in (resto, 's:0x%02x=0x%02x' % (reg, val)) if x)
                c = caso(rs, reg)
                resto = 's:0x%02x=0x%02x' % (reg, orig[reg]) if orig[reg] is not None else ''
                mot = base.motivos(c)
                grude = '' if c['lido'] is None else ('gruda' if c['lido'] == val else 'lido=%02x' % c['lido'])
                print('0x%02x=0x%02x  %-10s %s  %s' % (reg, val, grude, fmt(c), ' '.join(mot).upper() if mot else ''), flush=True)
                linhas.append((reg, val, grude, c, mot))
                if mot:
                    achados.setdefault(reg, []).extend(mot)
                    if c['m']:
                        salva_png('r%02x_v%02x' % (reg, val), c['m'])
                n += 1
                if n % 12 == 0:
                    b2 = Base([caso(resto)])
                    if not b2.ok:
                        print('\n*** O SENSOR PAROU DE RESPONDER depois de 0x%02x. Desconecte e reconecte a camera. ***' % reg)
                        return
        print('\n==== FASE 3: bits dos registradores que mudaram algo (%s) ====' % (
            ' '.join('0x%02x' % r for r in sorted(achados)) or 'nenhum'))
        for reg in sorted(achados)[:8]:
            for bit in range(8):
                val = 1 << bit
                rs = ','.join(x for x in (resto, 's:0x%02x=0x%02x' % (reg, val)) if x)
                c = caso(rs, reg)
                resto = 's:0x%02x=0x%02x' % (reg, orig[reg]) if orig[reg] is not None else ''
                mot = base.motivos(c)
                print('0x%02x bit %d (=0x%02x)  %s  %s' % (reg, bit, val, fmt(c), ' '.join(mot).upper() if mot else ''), flush=True)
                if mot and c['m']:
                    salva_png('r%02x_bit%d' % (reg, bit), c['m'])
        print('\n==== RESUMO: registradores que alteraram o comportamento ====')
        if not achados:
            print('nenhum: todos os valores testados deixaram a imagem e os fps como na base.')
        for reg in sorted(achados):
            print('0x%02x  %s' % (reg, ' '.join(sorted(set(achados[reg])))))
        print('\nbase: fps=%.1f media=%.1f dp=%.1f linhas<=%.3f colunas=%.3f bytes=%d' % (
            base.fps, base.mean, base.std, base.row, base.col, base.size))
        print('LINHAS = as linhas pares e impares passaram a diferir (pista de cor completa).')
    finally:
        regs_start(resto)
        c = caso(resto)
        regs_start('')
        print('\nverificacao final (registrador restaurado, regs_start limpo): %s' % fmt(c))


if __name__ == '__main__':
    main()
