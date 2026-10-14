#!/usr/bin/env python3
"""Testa o formato cru de DUAS LINHAS do sensor (COMH 0x28 bit 7 = 0) no QVGA entrelacado.

Autor: Josemar Simao <josemars@ifes.edu.br>
Co-autor e Revisor: Claude (Anthropic)

Ideia: no formato de duas linhas, cada pulso de linha (HREF) do sensor traz duas linhas misturadas
pixel a pixel (datasheet: "BGRG...", clock de saida dobrado). Se no entrelacado esse par for B/G + G/R,
cada campo teria as 4 cores e o 60 fps teria cor completa. O teste "J" de 2026-10-12 (0x28 = 0x00 a
60 fps) deu 59,5 fps e decodificou, mas a cor nao foi verificada.

Parte 1 (modo 60 fps do driver, S910 160x120): compara um-linha (atual) x duas-linhas pelo mosaico.
Parte 2 (janela larga, sem escala): a ponte pega 640 amostras por linha (no SN9C102 o maximo seria
1008; no SN9C103 nao ha mapa: o teste tambem mostra se 0x15 = 0x28 e' aceito com o sensor em QVGA) e
o padrao das colunas de 4 em 4 mostra o que vem em cada pulso: B G B G (uma linha) ou B G R G (duas).
Taxa reduzida (0x11 = 3) para o fluxo caber no USB: aqui importa a estrutura, nao o fps.

Cena PARADA, de preferencia com algo VERMELHO e algo AZUL no quadro. Pede sudo uma vez. ~2 min.
Resultado: duaslinhas_resultado.txt e imagens em testes/duaslinhas/
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import testa_qvga60 as t  # noqa: E402  (reaproveita sudo, leitura de registradores, medida de fps e decodificacao)
from sn9c10x_decode import decode  # noqa: E402

OUT = os.path.join(HERE, 'duaslinhas')
RAW = '/tmp/duaslinhas.raw'
# janela larga: sensor QVGA entrelacado, ponte 640x240 sem escala, campo a 15/s
LARGA = 's:0x14=0xa0,s:0x2d=0xc1,s:0x11=0x03,b:0x15=0x28,b:0x16=0x0f,b:0x18=0x8f'

# (nome, largura e altura pedidas ao v4l2, fps pedido, regs_start, largura e altura do que a ponte envia, o que muda)
CASOS = [
    ('ref30', 160, 120, 30, '', 160, 120, 'referencia: modo 30 fps normal (cor completa)'),
    ('base60', 160, 120, 60, '', 160, 120, '60 fps atual: uma linha (0x28 = 0x80)'),
    ('J', 160, 120, 60, 's:0x28=0x00', 160, 120, '60 fps com DUAS linhas (0x28 = 0x00)'),
    ('larga_1lin', 320, 240, 30, LARGA + ',s:0x28=0x80', 640, 240, 'janela 640 sem escala, UMA linha'),
    ('larga_2lin', 320, 240, 30, LARGA + ',s:0x28=0x00', 640, 240, 'janela 640 sem escala, DUAS linhas'),
]


def quadro(w, h, fps, wd, hd):
    """Captura um quadro (depois de 10 descartados) no formato pedido (w x h) e decodifica com o tamanho
    que a ponte realmente envia (wd x hd). Devolve (matriz, bytes) ou (None, bytes)."""
    try:
        os.remove(RAW)
    except OSError:
        pass
    args = ['--set-fmt-video=width=%d,height=%d,pixelformat=S910' % (w, h), '--set-parm=%d' % fps, '--stream-mmap']
    try:
        t.subprocess.run(['v4l2-ctl', '-d', t.DEV] + args + ['--stream-skip=10', '--stream-count=1',
                                                             '--stream-to=' + RAW], capture_output=True, timeout=30)
    except t.subprocess.TimeoutExpired:
        return None, 0
    if not os.path.exists(RAW) or os.path.getsize(RAW) == 0:
        return None, 0
    d = open(RAW, 'rb').read()
    try:
        b, bits = decode(d, wd, hd)
        return np.frombuffer(bytes(b), np.uint8).reshape(hd, wd).astype(np.float32), len(d)
    except Exception:
        return None, len(d)


def fases(q, nc):
    """Medias normalizadas por (linha par/impar, coluna mod nc): mostra o padrao que se repete."""
    m = np.array([[q[r::2, c::nc].mean() for c in range(nc)] for r in range(2)])
    return m / (m.mean() + 1e-6)


def salva(nome, q):
    """Salva a imagem crua em tons de cinza (o mosaico como chegou), ampliada 2x."""
    try:
        from PIL import Image
        os.makedirs(OUT, exist_ok=True)
        g = np.clip(q * (128.0 / max(float(q.mean()), 1.0)), 0, 255).astype(np.uint8)
        Image.fromarray(g).resize((q.shape[1] * 2, q.shape[0] * 2), Image.NEAREST).save(os.path.join(OUT, nome + '.png'))
    except Exception:
        pass


def main():
    """Roda os casos e imprime, para cada um, fps, bytes, registradores lidos e o padrao do mosaico."""
    sys.stdout = t.Tee(sys.__stdout__, open(os.path.join(HERE, 'duaslinhas_resultado.txt'), 'w'))
    if t.subprocess.run(['sudo', '-v']).returncode:
        sys.exit('sudo nao validado')
    print(__doc__)
    print('dispositivo:', t.DEV)
    t.LER = (0x11, 0x14, 0x28, 0x2d)
    # o ganho automatico regrava o 0x11 a cada ajuste de exposicao: desligado durante o teste
    ag = t.subprocess.run(['v4l2-ctl', '-d', t.DEV, '-C', 'gain_automatic'], capture_output=True, text=True).stdout
    ag = ag.split(':')[-1].strip() or '1'		# valor atual, devolvido no fim
    t.subprocess.run(['v4l2-ctl', '-d', t.DEV, '-c', 'gain_automatic=0'], capture_output=True)
    try:
        for nome, w, h, fps, rs, wd, hd, desc in CASOS:
            t.sudo_tee(t.PS, rs)
            t.W, t.H = w, h			# a medida de fps usa o formato deste caso
            fmed, lido = t.mede(fps)
            q, tam = quadro(w, h, fps, wd, hd)
            regs = ' '.join('%02x=%s' % (r, '??' if v is None else '%02x' % v) for r, v in lido.items())
            print('\n%-11s %s' % (nome, desc))
            print('   fps=%s  bytes=%d  sensor[%s]' % ('travou' if fmed is None else '%.1f' % fmed, tam, regs))
            if q is None:
                print('   SEM IMAGEM')
                continue
            salva(nome, q)
            f2 = fases(q, 2)
            print('   mosaico 2x2 (linha par / impar):  %s  /  %s' % (
                ' '.join('%.2f' % x for x in f2[0]), ' '.join('%.2f' % x for x in f2[1])))
            print('   igualdade das linhas (|par-impar| medio, ~0 = linhas do mesmo tipo): %.3f' % float(np.abs(f2[0] - f2[1]).mean()))
            if wd >= 640:
                f4 = fases(q, 4)
                print('   colunas de 4 em 4 (linha par):   %s' % ' '.join('%.2f' % x for x in f4[0]))
                print('   colunas de 4 em 4 (linha impar): %s' % ' '.join('%.2f' % x for x in f4[1]))
                print('   (uma linha: col 0 = col 2 e col 1 = col 3; duas linhas B G R G: col 0 != col 2)')
    finally:
        t.sudo_tee(t.PS, '')
        t.subprocess.run(['v4l2-ctl', '-d', t.DEV, '-c', 'gain_automatic=' + ag], capture_output=True)
    print('\nImagens (mosaico cru em cinza) em %s/' % OUT)


if __name__ == '__main__':
    main()
