#!/usr/bin/env python3
"""Testa 60 fps COM COR COMPLETA: sensor em QVGA progressivo + opcoes de clock (S910 160x120).

Autor: Josemar Simao <josemars@ifes.edu.br>
Co-autor e Revisor: Claude (Anthropic)

Hoje o modo 60 fps usa o sensor ENTRELACADO (COMH 0x28 = 0x80): cada campo so tem linhas de uma
paridade, entao cada quadro tem so duas cores. O datasheet diz que o QVGA de UMA LINHA pega blocos
2x2 (linhas 1-2, 5-6..., colunas 1-2, 5-6...), preservando as 4 cores, e que o formato cru de uma
linha vai ate 60 f/s. Hipotese: QVGA PROGRESSIVO (COMH = 0xA0) + alguma opcao de clock chega a 60 fps
com cor completa. Candidatos: CLKC 0x23 bit 1 (QCLK) e bit 0 (EVEN2), COMI 0x29 bit 6 (clock 2x),
COMJ 0x2D bit 6 (so campo impar).

Partida: modo 60 fps do driver (pede 60 fps no S910 160x120). Cada caso grava registradores por
regs_start no fim do inicio da captura; o reset do sensor no proximo inicio desfaz tudo.
Mede: fps, quadro integro, medias das 4 posicoes do mosaico (comparadas com a referencia de 30 fps,
que tem cor completa) e salva uma imagem colorida de cada caso em testes/qvga60/.

Cena PARADA (a lampada serve; nao precisa de muita luz). Pede sudo uma vez. ~2 min.
Resultado: qvga60_resultado.txt
Uso:  python3 testa_qvga60.py            (todos os casos)
      python3 testa_qvga60.py prog c29   (so os casos cujo nome contem esses textos)
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
PR = '/sys/module/gspca_elgin_fps/parameters/regr'
W, H = 160, 120
OUT = os.path.join(HERE, 'qvga60')
RAW = '/tmp/qvga60.raw'
POS = [(0, 0), (0, 1), (1, 0), (1, 1)]		# posicoes do mosaico (linha, coluna); BGGR: B, G, G, R
LER = (0x14, 0x23, 0x28, 0x29, 0x2d)		# registradores do sensor conferidos em cada caso

# (nome, fps pedido, regs_start, o que muda)
CASOS = [
    ('ref30', 30, '', 'referencia: modo 30 fps normal (VGA progressivo, cor completa)'),
    ('base60', 60, '', 'modo 60 fps atual (QVGA entrelacado: so 2 cores)'),
    ('prog', 60, 's:0x28=0xa0', 'QVGA PROGRESSIVO (esperado: 30 fps, 4 cores)'),
    ('prog_c2d81', 60, 's:0x28=0xa0,s:0x2d=0x81', 'progressivo + COMJ bit 6 = 0'),
    ('prog_c23ec', 60, 's:0x28=0xa0,s:0x23=0xec', 'progressivo + CLKC bit 1 (QCLK) = 0'),
    ('prog_c23ef', 60, 's:0x28=0xa0,s:0x23=0xef', 'progressivo + CLKC bit 0 (EVEN2) = 1'),
    ('prog_c29', 60, 's:0x28=0xa0,s:0x29=0x70', 'progressivo + COMI bit 6 (clock 2x) = 1'),
    ('prog_c29_c2d81', 60, 's:0x28=0xa0,s:0x29=0x70,s:0x2d=0x81', 'progressivo + clock 2x + COMJ bit 6 = 0'),
    ('prog_c29_c23ec', 60, 's:0x28=0xa0,s:0x29=0x70,s:0x23=0xec', 'progressivo + clock 2x + QCLK = 0'),
    ('entr_c23ef', 60, 's:0x23=0xef', 'entrelacado (atual) + EVEN2 = 1'),
]
if len(sys.argv) > 1:
    CASOS[:] = [c for c in CASOS if c[0] == 'ref30' or any(a in c[0] for a in sys.argv[1:])]


class Tee:
    """Escreve na tela e no arquivo de resultado ao mesmo tempo."""

    def __init__(self, *fs):
        self.fs = fs

    def write(self, s):
        for f in self.fs:
            f.write(s)

    def flush(self):
        for f in self.fs:
            f.flush()


def sudo_tee(path, txt):
    """Grava um parametro do modulo (precisa do sudo validado no inicio)."""
    subprocess.run(['sudo', '-n', 'tee', path], input=(txt + '\n').encode(), capture_output=True)


def rd(reg):
    """Le um registrador do sensor pelo parametro regr (vale durante a captura)."""
    sudo_tee(PR, 's:0x%02x' % reg)
    try:
        m = re.search(r'= 0x([0-9a-fA-F]{2})', open(PR).read())
    except OSError:
        return None
    return int(m.group(1), 16) if m else None


def fmt(fps):
    """Argumentos do v4l2-ctl para o S910 160x120 na taxa pedida."""
    return ['--set-fmt-video=width=%d,height=%d,pixelformat=S910' % (W, H), '--set-parm=%d' % fps, '--stream-mmap']


def mede(fps):
    """Captura ~5 s medindo o fps; no meio le os registradores de LER. Retorna (fps, {reg: valor})."""
    p = subprocess.Popen(['v4l2-ctl', '-d', DEV] + fmt(fps) + ['--stream-count=%d' % (fps * 5), '--stream-to=/dev/null'],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        p.wait(timeout=2.5)
    except subprocess.TimeoutExpired:
        pass
    lido = {r: rd(r) for r in LER}
    try:
        out = p.communicate(timeout=25)[0]
    except subprocess.TimeoutExpired:
        p.kill()
        return None, lido
    f = [float(x) for x in re.findall(r'<+ ([0-9.]+) fps', out)]
    return (statistics.median(f) if f else 0.0), lido


def quadro(fps):
    """Captura um quadro (depois de 10 descartados) e devolve o mosaico 160x120 e o tamanho comprimido."""
    try:
        os.remove(RAW)
    except OSError:
        pass
    try:
        subprocess.run(['v4l2-ctl', '-d', DEV] + fmt(fps) + ['--stream-skip=10', '--stream-count=1', '--stream-to=' + RAW],
                       capture_output=True, timeout=20)
    except subprocess.TimeoutExpired:
        return None, 0
    if not os.path.exists(RAW) or os.path.getsize(RAW) == 0:
        return None, 0
    d = open(RAW, 'rb').read()
    try:
        b, _ = decode(d, W, H)
        return np.frombuffer(bytes(b), np.uint8).reshape(H, W).astype(np.float32), len(d)
    except Exception:
        return None, len(d)


def salva_cor(nome, q):
    """Salva uma imagem colorida simples (meia resolucao, BGGR: B=(0,0), G=media, R=(1,1)), ampliada 4x."""
    try:
        from PIL import Image
        os.makedirs(OUT, exist_ok=True)
        rgb = np.dstack([q[1::2, 1::2], (q[0::2, 1::2] + q[1::2, 0::2]) / 2, q[0::2, 0::2]])
        rgb = rgb * (200.0 / max(float(rgb.mean()), 1.0)) * 0.5		# normaliza o brilho para comparar
        Image.fromarray(np.clip(rgb, 0, 255).astype(np.uint8)).resize((W * 2, H * 2), Image.NEAREST).save(
            os.path.join(OUT, nome + '.png'))
    except Exception:
        pass


def main():
    """Roda os casos, imprime cada um e um resumo comparando o mosaico com a referencia de 30 fps."""
    sys.stdout = Tee(sys.__stdout__, open(os.path.join(HERE, 'qvga60_resultado.txt'), 'w'))
    if subprocess.run(['sudo', '-v']).returncode:
        sys.exit('sudo nao validado')
    print(__doc__)
    print('dispositivo:', DEV)
    res = {}
    try:
        for nome, fps_ped, rs, desc in CASOS:
            sudo_tee(PS, rs)
            fps, lido = mede(fps_ped)
            q, tam = quadro(fps_ped)
            pos = [float(q[r::2, c::2].mean()) for r, c in POS] if q is not None else None
            res[nome] = (fps, pos, tam)
            if q is not None:
                salva_cor(nome, q)
            regs = ' '.join('%02x=%s' % (r, '??' if v is None else '%02x' % v) for r, v in lido.items())
            print('%-15s fps=%-6s bytes=%-6d %s  [%s]' % (
                nome, 'travou' if fps is None else '%.1f' % fps, tam,
                ('B,G,G,R = %5.1f %5.1f %5.1f %5.1f' % tuple(pos)) if pos else 'SEM IMAGEM', regs))
    finally:
        sudo_tee(PS, '')

    print('\n==== RESUMO ====')
    ref = res.get('ref30', (None, None, 0))[1]
    print('Cor: as 4 posicoes normalizadas pela media, comparadas com a ref30 (cor completa).')
    print('     "dif" pequena (< 0,10) = mosaico igual ao da referencia = 4 cores. Com so 2 cores, (0,x) e (1,x) se igualam.')
    print('%-15s %-7s %-6s %s' % ('caso', 'fps', 'dif', 'o que muda'))
    for nome, _, _, desc in CASOS:
        fps, pos, _ = res[nome]
        dif = '  -  '
        if pos and ref:
            a = np.array(pos) / (np.mean(pos) + 1e-6)
            b = np.array(ref) / (np.mean(ref) + 1e-6)
            dif = '%.2f' % float(np.abs(a - b).max())
        print('%-15s %-7s %-6s %s' % (nome, 'travou' if fps is None else '%.1f' % fps, dif, desc))
    print('\nImagens em %s/ (compare base60 e os "prog" com ref30).' % OUT)
    print('Sucesso = fps perto de 60 E dif pequena E imagem com as cores da ref30.')


if __name__ == '__main__':
    main()
