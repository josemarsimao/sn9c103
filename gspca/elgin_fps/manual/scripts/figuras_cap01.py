#!/usr/bin/env python3
"""Figuras do capitulo 1 do manual (controles automaticos), desenhadas com pycairo.

Autor: Josemar Simao <josemars@ifes.edu.br>
Co-autor e Revisor: Claude (Anthropic)

Gera PNGs em ../figuras/ (2x de resolucao, para impressao). Dados medidos lidos dos CSVs de ../../testes/.
Uso:  python3 figuras_cap01.py
"""
import csv
import math
import os
import statistics

import cairo

AQUI = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(AQUI, '..', 'figuras')
TESTES = os.path.join(AQUI, '..', '..', 'testes')
FONTE = 'DejaVu Sans'
AZUL, VERDE, LARANJA, CINZA, ESCURO = (0.15, 0.35, 0.65), (0.15, 0.55, 0.3), (0.85, 0.45, 0.1), (0.6, 0.6, 0.6), (0.15, 0.15, 0.15)


def tela(w, h):
    """Superficie branca de w x h pontos (desenhada em 2x)."""
    s = cairo.ImageSurface(cairo.FORMAT_RGB24, w * 2, h * 2)
    c = cairo.Context(s)
    c.scale(2, 2)
    c.set_source_rgb(1, 1, 1)
    c.paint()
    return s, c


def texto(c, x, y, t, tam=11, cor=ESCURO, centro=False, negrito=False):
    """Escreve texto (x, y = linha de base; centro = centralizado em x)."""
    c.select_font_face(FONTE, cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_BOLD if negrito else cairo.FONT_WEIGHT_NORMAL)
    c.set_font_size(tam)
    c.set_source_rgb(*cor)
    for i, linha in enumerate(t.split('\n')):
        e = c.text_extents(linha)
        c.move_to(x - (e.width / 2 + e.x_bearing if centro else 0), y + i * tam * 1.25)
        c.show_text(linha)


def caixa(c, x, y, w, h, t, cor=AZUL, tam=10):
    """Caixa com borda colorida, fundo claro e texto centralizado."""
    c.set_source_rgb(*[0.88 + 0.12 * v for v in cor])
    c.rectangle(x, y, w, h)
    c.fill_preserve()
    c.set_source_rgb(*cor)
    c.set_line_width(1.5)
    c.stroke()
    n = t.count('\n') + 1
    texto(c, x + w / 2, y + h / 2 - (n - 1) * tam * 0.62 + tam * 0.35, t, tam, ESCURO, centro=True)


def seta(c, x1, y1, x2, y2, cor=ESCURO, larg=1.4):
    """Seta de (x1, y1) a (x2, y2)."""
    c.set_source_rgb(*cor)
    c.set_line_width(larg)
    c.move_to(x1, y1)
    c.line_to(x2, y2)
    c.stroke()
    a = math.atan2(y2 - y1, x2 - x1)
    c.move_to(x2, y2)
    c.line_to(x2 - 8 * math.cos(a - 0.4), y2 - 8 * math.sin(a - 0.4))
    c.line_to(x2 - 8 * math.cos(a + 0.4), y2 - 8 * math.sin(a + 0.4))
    c.close_path()
    c.fill()


def salva(s, nome):
    """Grava a figura em ../figuras/<nome>.png."""
    os.makedirs(FIG, exist_ok=True)
    s.write_to_png(os.path.join(FIG, nome + '.png'))
    print('figura', nome)


def fig_cadeia():
    """1.1 Cadeia do sinal: onde cada ajuste (automatico ou manual) age sobre o pixel."""
    s, c = tela(820, 330)
    texto(c, 410, 22, 'Caminho de um pixel: da luz ao USB', 14, centro=True, negrito=True)
    blocos = [('Luz da cena', CINZA), ('Fotodiodo\n(integra luz)', AZUL), ('Amplificador\nanalógico', AZUL),
              ('ADC\n10 bits', AZUL), ('Processamento\ndigital', AZUL), ('Saída Bayer\n8 bits (16–240)', AZUL),
              ('Ponte\nSN9C103', VERDE), ('USB', CINZA)]
    x = 12
    for i, (t, cor) in enumerate(blocos):
        caixa(c, x, 60, 88, 50, t, cor, 9)
        if i:
            seta(c, x - 12, 85, x, 85)
        x += 100
    atuadores = [(1, 'EXPOSIÇÃO\n0x10 + 0x76\n(AEC)'), (2, 'GANHO 0x00\n(AGC), 0x28 b1,\n0x60, ganhos\nR/B 0x01/0x02\n(AWB)'),
                 (3, 'faixa do ADC\n0x69, offsets\n0x78–0x7A,\npreto óptico\n0x7D'), (4, 'brilho 0x06\n(com BRTEN),\noffsets\n0x21/0x2E,\ngama 0x14 b2'),
                 (6, 'offset 0x14,\nganhos\n0x05–0x07,\ncurva\n0x20–0x30')]
    for i, t in atuadores:
        cx = 12 + i * 100 + 44
        seta(c, cx, 200, cx, 114, LARANJA, 1.2)
        texto(c, cx, 214, t, 8.5, LARANJA, centro=True)
    texto(c, 12, 318, 'Laranja: registradores que atuam em cada etapa. Os automáticos (AEC, AGC, AWB) mexem nesses mesmos '
                       'registradores sozinhos, a cada campo.', 9, CINZA)
    salva(s, 'cap01_cadeia')


def fig_malha():
    """1.2 Malha de controle automatico: medir, comparar com o alvo, atuar."""
    s, c = tela(720, 260)
    texto(c, 360, 22, 'Malha de um controle automático (repetida a cada campo ou quadro)', 13, centro=True, negrito=True)
    caixa(c, 40, 80, 150, 60, 'MEDIR\nbrilho da imagem\n(soma, média ou\ncontagem de pixels)', VERDE, 9)
    caixa(c, 285, 80, 150, 60, 'COMPARAR\ncom o alvo\n(referencia)', AZUL, 9)
    caixa(c, 530, 80, 150, 60, 'ATUAR\nexposição, ganho\nou ganhos de cor', LARANJA, 9)
    seta(c, 190, 110, 285, 110)
    seta(c, 435, 110, 530, 110)
    c.set_source_rgb(*ESCURO)
    c.set_line_width(1.4)
    c.move_to(605, 140)
    c.line_to(605, 200)
    c.line_to(115, 200)
    c.stroke()
    seta(c, 115, 200, 115, 140)
    texto(c, 360, 194, 'próximo quadro: a cena já aparece com o novo ajuste', 9, CINZA, centro=True)
    texto(c, 360, 240, 'Zona morta: se a diferença for pequena, não atua (evita oscilar). Passo: quanto mexe por vez (velocidade).',
          9, CINZA, centro=True)
    salva(s, 'cap01_malha')


def fig_aec_hist():
    """1.3 Como o AEC do OV7630 decide: proporcao de pixels claros e escuros (AEW/AEB)."""
    s, c = tela(720, 300)
    texto(c, 360, 22, 'AEC do OV7630: conta pixels claros e escuros e compara com AEW (0x24) e AEB (0x25)', 12, centro=True, negrito=True)
    x0, y0, w, h = 60, 60, 600, 170
    import random
    random.seed(3)
    vals = [max(0, min(255, random.gauss(110, 45))) for _ in range(6000)]
    bins = [0] * 64
    for v in vals:
        bins[int(v) // 4] += 1
    m = max(bins)
    for i, b in enumerate(bins):
        v = i * 4
        cor = (0.25, 0.25, 0.25) if v < 64 else (0.95, 0.8, 0.2) if v > 192 else (0.65, 0.7, 0.8)
        c.set_source_rgb(*cor)
        bh = h * b / m
        c.rectangle(x0 + i * w / 64, y0 + h - bh, w / 64 - 1, bh)
        c.fill()
    c.set_source_rgb(*ESCURO)
    c.set_line_width(1)
    c.move_to(x0, y0 + h)
    c.line_to(x0 + w, y0 + h)
    c.stroke()
    for v in (0, 64, 128, 192, 255):
        texto(c, x0 + v * w / 256, y0 + h + 16, str(v), 9, CINZA, centro=True)
    texto(c, x0 + 64 * w / 256 / 2, y0 + 20, 'pixels escuros\n(abaixo do nível fixo)', 9, ESCURO, centro=True)
    texto(c, x0 + 224 * w / 256, y0 + 20, 'pixels claros\n(acima do nível fixo)', 9, ESCURO, centro=True)
    texto(c, 360, 274, 'Imagem estável quando a razão claros/escuros fica entre AEW (padrão 25%) e AEB (padrão 75%); fora disso,\n'
                       'o sensor aumenta ou diminui a exposição (e o ganho, se o AGC estiver ligado). Ilustrativo.', 9, CINZA, centro=True)
    salva(s, 'cap01_aec_hist')


def fig_janela_ae():
    """1.4 Janela de medida da ponte (0x1C-0x1F) sobre a imagem 640x480, em blocos de 32 pixels."""
    s, c = tela(720, 360)
    texto(c, 360, 22, 'Janela de medida da ponte: 0x1C–0x1F = 02 03 0F 0C (blocos de 32 px)', 12, centro=True, negrito=True)
    x0, y0, k = 180, 40, 0.75				# imagem 640x480 desenhada em 360x270 pontos, centralizada
    c.set_source_rgb(0.93, 0.93, 0.93)
    c.rectangle(x0, y0, 640 * k * 0.75, 480 * k * 0.75)
    c.fill()
    e = 32 * k * 0.75
    c.set_source_rgb(*CINZA)
    c.set_line_width(0.4)
    for i in range(21):
        c.move_to(x0 + i * e, y0)
        c.line_to(x0 + i * e, y0 + 15 * e)
    for j in range(16):
        c.move_to(x0, y0 + j * e)
        c.line_to(x0 + 20 * e, y0 + j * e)
    c.stroke()
    c.set_source_rgba(0.85, 0.45, 0.1, 0.35)
    c.rectangle(x0 + 2 * e, y0 + 3 * e, 13 * e, 9 * e)
    c.fill_preserve()
    c.set_source_rgb(*LARANJA)
    c.set_line_width(2)
    c.stroke()
    texto(c, x0 + 8.5 * e, y0 + 7.8 * e, 'DENTRO: 13 × 9 = 117 blocos\nsoma ÷ 32 → bytes 9–10 do cabeçalho', 9, ESCURO, centro=True)
    texto(c, x0 + 10 * e, y0 + 15 * e + 16, 'FORA da janela: soma ÷ 128 → bytes 11–12. Cada pixel entra como Y = R/4 + 5G/16 + B/8', 9, CINZA, centro=True)
    texto(c, x0 + 2 * e, y0 - 4, 'STRX=2', 8, LARANJA, centro=True)
    texto(c, x0 + 15 * e, y0 - 4, 'ENDX=15', 8, LARANJA, centro=True)
    texto(c, x0 - 34, y0 + 3 * e + 3, 'STRY=3', 8, LARANJA)
    texto(c, x0 - 38, y0 + 12 * e + 3, 'ENDY=12', 8, LARANJA)
    salva(s, 'cap01_janela_ae')


def fig_exposicao():
    """1.5 Tempo de exposicao dentro do quadro: AEC em linhas, limitado pelo tempo do quadro."""
    s, c = tela(760, 250)
    texto(c, 380, 22, 'Exposição = TLINE × AEC[9:0] (linhas); nunca maior que o quadro (33,3 ms a 30 fps)', 12, centro=True, negrito=True)
    x0, w = 40, 680
    for q in range(2):
        xq = x0 + q * w / 2
        c.set_source_rgb(0.9, 0.9, 0.95)
        c.rectangle(xq, 70, w / 2 - 6, 40)
        c.fill()
        texto(c, xq + w / 4, 64, 'quadro %d (~525 linhas, 33,3 ms)' % (q + 1), 9, CINZA, centro=True)
        c.set_source_rgba(0.85, 0.45, 0.1, 0.6)
        c.rectangle(xq + w / 2 - 6 - w * 0.18, 120, w * 0.18, 26)
        c.fill()
        texto(c, xq + w / 2 - 6 - w * 0.09, 138, 'exposição (ex. 0x41)', 8.5, ESCURO, centro=True)
    texto(c, 380, 180, 'Mais exposição = mais luz por pixel (linear), sem ruído extra; acima do tempo do quadro, só baixando a taxa:\n'
                       'divisor 0x11 (30, 15, 10 fps…), ajuste fino 0x2A/0x2B, linhas extras 0x7E/0x7F ou clock da ponte (MCK_SIZE).', 9, CINZA, centro=True)
    salva(s, 'cap01_exposicao')


def fig_atuacao_pixel():
    """1.6 Como cada tipo de ajuste transforma o valor do pixel (entrada x saida)."""
    s, c = tela(820, 260)
    texto(c, 410, 20, 'Como cada ajuste muda o valor de um pixel (entrada → saída, 0–255)', 12, centro=True, negrito=True)
    curvas = [('Ganho ×2\n(multiplica)', lambda x: 2 * x), ('Offset -40\n(subtrai: preto)', lambda x: x - 40),
              ('Brilho +40\n(soma)', lambda x: x + 40), ('Contraste em S\n(apoio em 64)', None)]
    for i, (nome, f) in enumerate(curvas):
        x0, y0, L = 30 + i * 200, 50, 160
        c.set_source_rgb(0.97, 0.97, 0.97)
        c.rectangle(x0, y0, L, L)
        c.fill()
        c.set_source_rgb(*CINZA)
        c.set_line_width(0.8)
        c.set_dash([3, 3])
        c.move_to(x0, y0 + L)
        c.line_to(x0 + L, y0)
        c.stroke()
        c.set_dash([])
        c.set_source_rgb(*LARANJA)
        c.set_line_width(2)
        for k in range(256):
            if f:
                y = f(k)
            else:
				# a curva em S do driver (kq = 514, p = 64)
                p, kq = 64, 514
                y = p * k * 256 / (kq * p - (kq - 256) * k) if k <= p else \
                    255 - (255 - p) * (255 - k) * 256 / (kq * (255 - p) - (kq - 256) * (255 - k))
            y = max(0, min(255, y))
            px, py = x0 + k * L / 255, y0 + L - y * L / 255
            if k == 0:
                c.move_to(px, py)
            else:
                c.line_to(px, py)
        c.stroke()
        texto(c, x0 + L / 2, y0 + L + 18, nome, 9, ESCURO, centro=True)
    salva(s, 'cap01_atuacao_pixel')


def fig_medidas_aec():
    """1.7 Medidas dos automaticos internos do sensor (testa_combina.py, grupo aec e awb)."""
    caminho = os.path.join(TESTES, 'comb_sensor.csv')
    if not os.path.exists(caminho):
        return
    linhas = list(csv.DictReader(open(caminho)))
    refs = [float(r['media']) for r in linhas if r['n'] == 'REF']
    ref = statistics.median(refs)			# mesma conta do resumo do teste
    curtos = ['AEC\ninterno', 'AGC\ninterno', 'AEC + AGC', 'alvo claro\n(0x24/0x25)', 'alvo escuro\n(0x24/0x25)', 'referência\nbaixa (0x68)', 'referência\nalta (0x68)', 'janela central\n(0x29 bit 3)']
    aec = [float(r['media']) for r in linhas if r['grupo'] == 'aec']
    sel = [('referência\n(manual)', ref)] + list(zip(curtos, aec))
    s, c = tela(780, 270)
    texto(c, 390, 22, 'Brilho medido com os automáticos internos do sensor (0x13) — mesmo padrão, ganho 47, exposição 67',
          11, centro=True, negrito=True)
    x0, y0, h = 30, 50, 180
    bw = (780 - 2 * x0) / len(sel) - 8			# barras ajustadas a largura
    m = max(v for _, v in sel)
    for i, (nome, v) in enumerate(sel):
        bh = h * v / m
        c.set_source_rgb(*(CINZA if i == 0 else AZUL))
        c.rectangle(x0 + i * (bw + 8), y0 + h - bh, bw, bh)
        c.fill()
        texto(c, x0 + i * (bw + 8) + bw / 2, y0 + h - bh - 4, '%.1f' % v, 9, ESCURO, centro=True)
        texto(c, x0 + i * (bw + 8) + bw / 2, y0 + h + 14, nome, 8, ESCURO, centro=True)
    salva(s, 'cap01_medidas_aec')


def fig_imagens_aec():
    """1.8 Imagens reais: referencia e automaticos internos (comb_sensor/c25-c27)."""
    pasta = os.path.join(TESTES, 'comb_sensor')
    nomes = [('referencia.png', 'manual (referência)'), ('c25.png', 'AEC interno'), ('c26.png', 'AGC interno'),
             ('c27.png', 'AEC + AGC')]
    if not all(os.path.exists(os.path.join(pasta, n)) for n, _ in nomes):
        return
    s, c = tela(800, 200)
    for i, (n, t) in enumerate(nomes):
        im = cairo.ImageSurface.create_from_png(os.path.join(pasta, n))
        c.save()
        x, y = 10 + i * 198, 10
        esc = 188.0 / im.get_width()
        c.translate(x, y)
        c.scale(esc, esc)
        c.set_source_surface(im, 0, 0)
        c.paint()
        c.restore()
        texto(c, x + 94, 10 + im.get_height() * esc + 16, t, 9, ESCURO, centro=True)
    salva(s, 'cap01_imagens_aec')


def fig_aec_sequencia():
    """1.x Sequencia do AEC do OV7630: pixel -> comparadores -> contadores -> decisao -> 0x10/0x76."""
    s, c = tela(820, 470)
    texto(c, 410, 22, 'AEC do OV7630: da leitura dos pixels à nova exposição (a cada 1, 2 ou 4 campos)', 13, centro=True, negrito=True)
    # coluna 1: pixels
    caixa(c, 20, 60, 150, 60, 'Pixels do campo\n(todos, ou só o quarto\ncentral: 0x29[3] = 1)', AZUL, 9)
    # comparadores
    caixa(c, 225, 45, 165, 40, 'acima do nível fixo?\n→ conta "claro"', AZUL, 9)
    caixa(c, 225, 95, 165, 40, 'abaixo do nível fixo?\n→ conta "escuro"', AZUL, 9)
    seta(c, 170, 82, 225, 65)
    seta(c, 170, 98, 225, 115)
    texto(c, 307, 152, 'comparadores e contadores\ninternos (níveis não documentados)', 8, CINZA, centro=True)
    # proporcoes
    caixa(c, 445, 60, 150, 60, 'Fim do campo:\n% de claros e\n% de escuros', VERDE, 9)
    seta(c, 390, 65, 445, 80)
    seta(c, 390, 115, 445, 100)
    # referencias
    caixa(c, 445, 175, 150, 75, 'REFERÊNCIAS\n0x24[7:0] AEW (claros)\n0x25[7:0] AEB (escuros)\n0x68[7:5] ARL (nível)', LARANJA, 8.5)
    # decisao
    caixa(c, 650, 100, 150, 90, 'DECISÃO\nclaros demais → expõe menos\nescuros demais → expõe mais\ndentro da faixa → mantém', AZUL, 8.5)
    seta(c, 595, 90, 650, 130)
    seta(c, 595, 212, 650, 165)
    # saida
    caixa(c, 650, 260, 150, 70, 'GRAVA a exposição\nAEC[9:0] =\n0x10[7:0] + 0x76[1:0]', LARANJA, 9)
    seta(c, 725, 190, 725, 260)
    # efeito
    caixa(c, 330, 285, 260, 60, 'Próximo campo: cada pixel acumula\nluz por TEX = TLINE × AEC[9:0]\n(no máximo, o tempo do quadro)', VERDE, 9)
    seta(c, 650, 300, 590, 312)
    c.set_source_rgb(*ESCURO)
    c.set_line_width(1.4)
    c.move_to(330, 318)
    c.line_to(95, 318)
    c.stroke()
    seta(c, 95, 318, 95, 120)
    # opcoes
    texto(c, 20, 375, 'Opções que mudam o ciclo:', 9.5, ESCURO, negrito=True)
    texto(c, 20, 393, '• 0x2A[0] = 1: troca a contagem pela média de luminância interna   '
                      '• 0x2D[7] = 1: decide a cada 2 ou 4 campos (0 = a cada campo)', 8.5, ESCURO)
    texto(c, 20, 411, '• 0x15[2] e 0x15[3]: modo rápido e passo grande (limiares em 0x77)   '
                      '• 0x28[4] = 1: congela a exposição e o ganho atuais', 8.5, ESCURO)
    texto(c, 20, 429, '• 0x75[3]: descarta um quadro quando a exposição muda muito   '
                      '• 0x1F[5]: passos mais espaçados abaixo de 8 linhas', 8.5, ESCURO)
    texto(c, 20, 452, 'Liga: 0x13[0] = 1. Com o AGC (0x13[1] = 1), o mesmo ciclo também ajusta o ganho 0x00[5:0].', 9, LARANJA)
    salva(s, 'cap01_aec_sequencia')


if __name__ == '__main__':
    for f in (fig_cadeia, fig_malha, fig_aec_hist, fig_aec_sequencia, fig_janela_ae, fig_exposicao, fig_atuacao_pixel, fig_medidas_aec, fig_imagens_aec):
        f()
