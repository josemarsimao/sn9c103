#!/usr/bin/env python3
"""Linha unica x duas linhas (COMH 0x28 bit 7) com padroes na tela: cores e comprimento da linha.

Autor: Josemar Simao <josemars@ifes.edu.br>
Co-autor e Revisor: Claude (Anthropic)

Perguntas:
 1. Que cor vem em cada amostra da linha, em cada formato do sensor? Com cor solida (branco, vermelho,
    verde, azul) cada posicao (linha par/impar, coluna de 4 em 4) mostra a cor que carrega. Linha unica
    deve dar B G B G / G R G R; se duas linhas trouxer o par misturado, aparece B G R G.
 2. Quantas amostras tem cada linha (HREF)? Com a tela meio branca / meio preta e a janela da ponte
    em 640 sem escala, a borda anda de uma linha para a outra se a linha do sensor tiver outro tamanho:
    comprimento da linha = 640 + deslocamento da borda por linha.

Casos (todos pedem S910 320x240 ao v4l2 e trocam registradores por regs_start; janela da ponte
sem escala para nao descartar amostras):
  ref    sensor VGA progressivo, modo normal (ponte 640x480 com escala 1/2): referencia
  prog1  sensor QVGA progressivo, linha unica
  entr1  sensor QVGA entrelacado, linha unica (o modo 60 fps)
  entr2  sensor QVGA entrelacado, DUAS linhas
  prog2  sensor QVGA progressivo, DUAS linhas
Taxa reduzida pelo 0x11 para caber no USB. Ganho automatico e taxa dinamica ficam desligados (eles
regravam o 0x11) e voltam ao valor anterior no fim.

Padrao (janela no notebook, tela_padrao.py): a camera no suporte (~5 cm) olhando para a janela. O
script abre a janela no padrao "alinhar"; posicione e ajuste o tamanho (+/-) pelo Multiview ate o
contorno ficar inteiro na imagem, FECHE o Multiview e aperte Enter. Depois tudo e' automatico; as
medidas usam so a regiao do padrao (achada na captura do branco de cada caso). ~5 min.
Com --celular: modo manual antigo (tela do celular cobrindo toda a imagem, Enter a cada tela).
Pede sudo uma vez.
Resultado: linha_resultado.txt e imagens em testes/linha/
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import testa_qvga60 as t  # noqa: E402  (sudo, leitura de registradores, medida de fps)
from janela import Janela  # noqa: E402
from sn9c10x_decode import decode  # noqa: E402

OUT = os.path.join(HERE, 'linha')
RAW = '/tmp/linha.raw'
W, H = 320, 240					# formato pedido ao v4l2
QV = 's:0x14=0xa0,b:0x16=0x0f,b:0x18=0x8f'	# sensor QVGA; ponte 240 linhas, escala 1/1
JAN320 = ',b:0x15=0x14,s:0x11=0x01'		# janela de 320 amostras, metade da taxa
JAN640 = ',b:0x15=0x28,s:0x11=0x03'		# janela de 640 amostras, 1/4 da taxa
# (nome, registradores do sensor no caso, descricao)
CASOS = [
    ('ref', None, 'VGA progressivo, modo normal (referencia)'),
    ('prog1', 's:0x28=0xa0', 'QVGA progressivo, linha unica'),
    ('entr1', 's:0x28=0x80,s:0x2d=0xc1', 'QVGA entrelacado, linha unica (60 fps)'),
    ('entr2', 's:0x28=0x00,s:0x2d=0xc1', 'QVGA entrelacado, DUAS linhas'),
    ('prog2', 's:0x28=0x20', 'QVGA progressivo, DUAS linhas'),
]
# (nome, padrao da janela, pedido no modo celular)
TELAS = [('BRANCO', 'branco', 'tela toda BRANCA (255,255,255)'), ('VERMELHO', 'vermelho', 'tela toda VERMELHA (255,0,0)'),
         ('VERDE', 'verde', 'tela toda VERDE (0,255,0)'), ('AZUL', 'azul', 'tela toda AZUL (0,0,255)'),
         ('MEIO', 'meio', 'metade ESQUERDA BRANCA e metade DIREITA PRETA (divisa vertical)')]
CELULAR = '--celular' in sys.argv


def regs(caso, jan):
    """Monta o regs_start do caso: o ref usa o modo normal (com a janela 640 so na medida de borda)."""
    nome, sens, _ = caso
    if sens is None:
        return '' if jan == 320 else 'b:0x16=0x0f,b:0x18=0x8f' + JAN640
    return QV + (JAN320 if jan == 320 else JAN640) + ',' + sens


def captura(wd, hd):
    """Captura um quadro (depois de 10 descartados) e decodifica com o tamanho que a ponte envia."""
    try:
        os.remove(RAW)
    except OSError:
        pass
    args = ['--set-fmt-video=width=%d,height=%d,pixelformat=S910' % (W, H), '--set-parm=30', '--stream-mmap']
    try:
        t.subprocess.run(['v4l2-ctl', '-d', t.DEV] + args + ['--stream-skip=10', '--stream-count=1',
                                                             '--stream-to=' + RAW], capture_output=True, timeout=40)
    except t.subprocess.TimeoutExpired:
        return None
    if not os.path.exists(RAW) or os.path.getsize(RAW) == 0:
        return None
    try:
        b, _ = decode(open(RAW, 'rb').read(), wd, hd)
        return np.frombuffer(bytes(b), np.uint8).reshape(hd, wd).astype(np.float32)
    except Exception:
        return None


def regiao(q):
    """Regiao do padrao na imagem do branco: retangulo das linhas e colunas claras (acima da metade do
    brilho alto), encolhido 10% em cada lado e alinhado ao mosaico (linha par, coluna multipla de 4).
    Sem padrao claro (modo celular), usa a imagem toda menos 8 pixels de borda."""
    alto = float(np.percentile(q, 99))
    claro = q > 0.5 * alto
    lin = np.where(claro.mean(axis=1) > 0.5)[0]
    col = np.where(claro.mean(axis=0) > 0.5)[0]
    if len(lin) < 16 or len(col) < 16:
        return (8, q.shape[0] - 8, 8, q.shape[1] - 8)
    r0, r1, c0, c1 = lin[0], lin[-1] + 1, col[0], col[-1] + 1
    dr, dc = (r1 - r0) // 10, (c1 - c0) // 10
    r0, r1, c0, c1 = r0 + dr, r1 - dr, c0 + dc, c1 - dc
    return (r0 + r0 % 2, r1, c0 + (-c0) % 4, c1)


def grade(q, roi):
    """Media de cada posicao (linha par/impar x coluna de 4 em 4) dentro da regiao do padrao."""
    r0, r1, c0, c1 = roi
    q = q[r0:r1, c0:c1]
    return np.array([[q[r::2, c::4].mean() for c in range(4)] for r in range(2)])


def borda(q):
    """Comprimento real da linha do sensor: as linhas da ponte, juntas na ordem em que chegaram, formam
    um sinal que se repete a cada linha do sensor (a tela meio a meio e' igual em todas as linhas).
    O periodo (pico da autocorrelacao entre 200 e 1400 amostras) e' o comprimento. Devolve
    (periodo - 640, periodo) para manter a forma 'deslocamento por linha'."""
    x = q.reshape(-1).astype(np.float64)
    x = np.convolve(x - x.mean(), np.ones(4) / 4.0, mode='same')	# suaviza o mosaico (blocos de 4)
    n = len(x)
    f = np.fft.rfft(x, 2 * n)
    ac = np.fft.irfft(f * np.conj(f))[:1500]
    per = 200 + int(np.argmax(ac[200:1400]))
    return per - 640.0, per


def salva(nome, q):
    """Salva o mosaico cru em tons de cinza (como chegou)."""
    try:
        from PIL import Image
        os.makedirs(OUT, exist_ok=True)
        g = np.clip(q * (128.0 / max(float(q.mean()), 1.0)), 0, 255).astype(np.uint8)
        Image.fromarray(g).save(os.path.join(OUT, nome + '.png'))
    except Exception:
        pass


def ctrl(nome, valor=None):
    """Le (valor=None) ou grava um controle V4L2 da camera."""
    if valor is None:
        out = t.subprocess.run(['v4l2-ctl', '-d', t.DEV, '-C', nome], capture_output=True, text=True).stdout
        return out.split(':')[-1].strip()
    t.subprocess.run(['v4l2-ctl', '-d', t.DEV, '-c', '%s=%s' % (nome, valor)], capture_output=True)


def main():
    """Pede cada tela, captura os cinco casos e no fim monta a tabela de cores e o comprimento da linha."""
    sys.stdout = t.Tee(sys.__stdout__, open(os.path.join(HERE, 'linha_resultado.txt'), 'w'))
    if t.subprocess.run(['sudo', '-v']).returncode:
        sys.exit('sudo nao validado')
    print(__doc__)
    print('dispositivo:', t.DEV)
    t.W, t.H = W, H
    t.LER = (0x11, 0x14, 0x28, 0x2d)
    antes = {c: ctrl(c) for c in ('gain_automatic', 'exposure_dynamic_framerate')}
    ctrl('gain_automatic', 0)
    ctrl('exposure_dynamic_framerate', 0)
    g = {}						# g[caso][tela] = grade 2x4
    roi = {}						# roi[caso] = regiao do padrao (achada no branco)
    jan_p = None
    try:
        if not CELULAR:
            jan_p = Janela('alinhar')
            input('\n>>> Posicione a janela pelo Multiview (padrao "alinhar", +/- muda o tamanho) ate o contorno\n'
                  '    ficar inteiro na imagem, FECHE o Multiview e aperte Enter... ')
        for tela, padrao, pedido in TELAS:
            t.subprocess.run(['sudo', '-v'])
            if CELULAR:
                input('\n>>> Mostre %s e aperte Enter (mantenha ate o proximo pedido)... ' % pedido)
            else:
                jan_p.mostra(padrao)
                print('\n--- padrao %s' % padrao)
            for caso in CASOS:
                nome = caso[0]
                jan = 640 if tela == 'MEIO' else 320
                t.sudo_tee(t.PS, regs(caso, jan))
                info = ''
                if tela == 'BRANCO':			# fps e registradores lidos de volta, uma vez por caso
                    fps, lido = t.mede(30)
                    info = '  fps=%s  sensor[%s]' % ('travou' if fps is None else '%.1f' % fps,
                                                     ' '.join('%02x=%s' % (r, '??' if v is None else '%02x' % v)
                                                              for r, v in lido.items()))
                q = captura(jan, 240)
                if q is None:
                    print('  %-6s SEM IMAGEM%s' % (nome, info))
                    continue
                salva('%s_%s' % (tela.lower(), nome), q)
                if tela == 'MEIO':
                    a, _ = borda(q)
                    g.setdefault(nome, {})['MEIO'] = a
                    print('  %-6s linha do sensor ~ %.0f amostras' % (nome, 640 + a))
                else:
                    if tela == 'BRANCO':
                        roi[nome] = regiao(q)
                    m = grade(q, roi.get(nome, (8, q.shape[0] - 8, 8, q.shape[1] - 8)))
                    g.setdefault(nome, {})[tela] = m
                    sat = ' SATURADO (diminua o brilho)' if q.max() >= 250 and np.mean(q >= 250) > 0.2 else ''
                    reg = '  regiao linhas %d-%d colunas %d-%d' % roi[nome] if tela == 'BRANCO' and nome in roi else ''
                    print('  %-6s par: %s  impar: %s%s%s%s' % (nome, ' '.join('%5.1f' % x for x in m[0]),
                                                             ' '.join('%5.1f' % x for x in m[1]), sat, info, reg))
    finally:
        if jan_p:
            jan_p.fecha()
        t.sudo_tee(t.PS, '')
        for c, v in antes.items():
            if v:
                ctrl(c, v)

    print('\n==== COR DE CADA AMOSTRA (dividida pelo branco; letra = cor dominante, minuscula = fraca) ====')
    print('%-6s %-27s %-27s %s' % ('caso', 'linha par (col 0 1 2 3)', 'linha impar (col 0 1 2 3)', 'descricao'))
    for nome, _, desc in CASOS:
        d = g.get(nome, {})
        if not all(k in d for k in ('BRANCO', 'VERMELHO', 'VERDE', 'AZUL')):
            print('%-6s (faltou alguma tela)' % nome)
            continue
        br = d['BRANCO'] + 1e-6
        cores = np.stack([d['VERMELHO'] / br, d['VERDE'] / br, d['AZUL'] / br])	# 3 x 2 x 4
        lin = []
        for r in range(2):
            txt = []
            for c in range(4):
                v = cores[:, r, c]
                k = int(np.argmax(v))
                forte = v[k] > 1.8 * np.partition(v, -2)[-2]			# claramente uma cor so
                txt.append(('RGB'[k] if forte else 'rgb'[k]))
            lin.append(' '.join(txt))
        print('%-6s %-27s %-27s %s' % (nome, lin[0], lin[1], desc))
    print('\n==== COMPRIMENTO DA LINHA (tela meio a meio, janela de 640) ====')
    for nome, _, desc in CASOS:
        a = g.get(nome, {}).get('MEIO')
        print('%-6s %s   %s' % (nome, 'sem medida' if a is None else '~%.0f amostras por linha' % (640 + a), desc))
    print('\nEsperado: ref ~640 e B G B G / G R G R. Imagens em %s/' % OUT)


if __name__ == '__main__':
    main()
