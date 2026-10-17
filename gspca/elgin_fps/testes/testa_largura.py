#!/usr/bin/env python3
"""Comprimento real da linha do sensor no QVGA: varia a largura da janela da ponte (0x15) com a tela
meio branca / meio preta e mede a inclinacao da borda.

Autor: Josemar Simao <josemars@ifes.edu.br>
Co-autor e Revisor: Claude (Anthropic)

Se a janela da ponte cabe na linha do sensor, cada linha da imagem comeca no mesmo ponto e a borda
branco/preto fica vertical (inclinacao 0). Se a janela e' maior que a linha, a ponte atravessa para a
linha seguinte e a borda anda um pouco a cada linha. A maior largura sem inclinacao (e o valor da
inclinacao logo acima dela) da' o comprimento real da linha em cada formato.

Casos: prog1 (QVGA progressivo, linha unica), entr1 (QVGA entrelacado, linha unica), entr2 (QVGA
entrelacado, duas linhas), prog2 (QVGA progressivo, duas linhas); larguras 0x15 = 0x10 a 0x28
(256 a 640 amostras, passo 16). Taxa reduzida (0x11 = 3); ganho automatico e taxa dinamica
desligados. Casos numerados, gravados um a um em largura_dados.csv; se travar, rode de novo com
--continua (retoma no caso interrompido) ou --continua --pula N.

A janela de padroes abre na posicao gravada (tela_padrao.json) no padrao "meio". ~5 min.
Resultado: largura_resultado.txt
"""
import csv
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import testa_linha as tl  # noqa: E402  (captura, controles, sudo)
t = tl.t
from janela import Janela  # noqa: E402

BASE = 's:0x14=0xa0,b:0x16=0x0f,b:0x18=0x8f,s:0x11=0x03'	# sensor QVGA; ponte 240 linhas, escala 1/1
CASOS = [('prog1', 's:0x28=0xa0', 'QVGA progressivo, linha unica'),
         ('entr1', 's:0x28=0x80,s:0x2d=0xc1', 'QVGA entrelacado, linha unica (60 fps)'),
         ('entr2', 's:0x28=0x00,s:0x2d=0xc1', 'QVGA entrelacado, DUAS linhas'),
         ('prog2', 's:0x28=0x20', 'QVGA progressivo, DUAS linhas')]
LARGURAS = list(range(0x10, 0x29))				# x 16 amostras


def inclinacao(q):
    """Quanto o conteudo anda de uma linha para a seguinte (amostras): para cada par de linhas vizinhas,
    o deslocamento que melhor encaixa uma na outra (correlacao, de -w/2 a +w/2). Devolve (mediana dos
    deslocamentos, fracao de pares com encaixe claro). 0 = janela cabe na linha (ou e' multiplo dela)."""
    w = q.shape[1]
    k = np.ones(4) / 4.0
    linhas = [np.convolve(r - r.mean(), k, mode='same') for r in q.astype(np.float64)]
    desl, bons = [], 0
    for a, b in zip(linhas[:-1], linhas[1:]):
        if a.std() < 1 or b.std() < 1:			# linha sem detalhe (toda clara ou escura)
            continue
        c = np.correlate(b, a, mode='full')[w // 2:w // 2 + w]	# deslocamentos -w/2 .. +w/2
        c = c / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-9)
        i = int(np.argmax(c))
        desl.append(i + w // 2 - (w - 1))			# posicao w-1 do 'full' = deslocamento 0
        bons += c[i] > 0.6
    if not desl:
        return None, 0.0
    return float(np.median(desl)), bons / float(len(desl))


def main():
    """Roda os casos numerados (configuracao x largura) e no fim resume por configuracao."""
    pula = set()
    for i, a in enumerate(sys.argv):
        if a == '--pula' and i + 1 < len(sys.argv):
            pula |= {int(x) for x in sys.argv[i + 1].split(',') if x.strip()}
    csv_path = os.path.join(HERE, 'largura_dados.csv')
    atual_path = os.path.join(HERE, 'largura_atual.txt')
    campos = ['n', 'caso', 'largura', 'amostras', 'estado', 'inclinacao', 'nitidas']
    sys.stdout = t.Tee(sys.__stdout__, open(os.path.join(HERE, 'largura_resultado.txt'), 'a', buffering=1))
    if t.subprocess.run(['sudo', '-v']).returncode:
        sys.exit('sudo nao validado')
    print(__doc__)
    print('==== %s  dispositivo: %s' % (time.strftime('%Y-%m-%d %H:%M'), t.DEV))
    lista = [(c, w) for c in CASOS for w in LARGURAS]
    feitos = set()
    continua = '--continua' in sys.argv and os.path.exists(csv_path)
    if continua:
        feitos = {int(r['n']) for r in csv.DictReader(open(csv_path))}
        print('retomando: %d casos ja gravados' % len(feitos))
        try:
            parado = int(open(atual_path).read())
            if parado not in feitos:
                print('*** o caso %d foi interrompido: %s' % (parado, 'sera PULADO' if parado in pula else
                      'sera repetido (se travar de novo, use --pula %d)' % parado))
        except (OSError, ValueError):
            pass
    fcsv = open(csv_path, 'a' if continua else 'w', newline='')
    wr = csv.DictWriter(fcsv, campos)
    if not continua:
        wr.writeheader()

    def grava(linha):
        """Grava a linha no CSV e forca a ida ao disco."""
        wr.writerow(linha)
        fcsv.flush()
        os.fsync(fcsv.fileno())

    antes = {c: tl.ctrl(c) for c in ('gain_automatic', 'exposure_dynamic_framerate')}
    tl.ctrl('gain_automatic', 0)
    tl.ctrl('exposure_dynamic_framerate', 0)
    jan = Janela('alinhar')
    terminou = False
    try:
        input('\n>>> Confira pelo Multiview se o contorno do padrao "alinhar" esta inteiro na imagem (a janela\n'
              '    abre na posicao gravada). FECHE o Multiview e aperte Enter... ')
        jan.mostra('meio', espera=0.6)
        for n, ((nome, sens, desc), larg) in enumerate(lista, 1):
            if n in feitos:
                continue
            amostras = larg * 16
            rotulo = '[%03d/%03d] %-6s 0x15=0x%02x (%3d amostras)' % (n, len(lista), nome, larg, amostras)
            linha = dict(n=n, caso=nome, largura='0x%02x' % larg, amostras=amostras)
            if n in pula:
                grava(dict(linha, estado='PULADO'))
                print('%s PULADO' % rotulo)
                continue
            if n % 20 == 1:
                t.subprocess.run(['sudo', '-v'])
            with open(atual_path, 'w') as fa:
                fa.write('%d\n' % n)
                fa.flush()
                os.fsync(fa.fileno())
            t.sudo_tee(t.PS, '%s,b:0x15=0x%02x,%s' % (BASE, larg, sens))
            q = tl.captura(amostras, 240)
            if q is None:
                grava(dict(linha, estado='SEM IMAGEM'))
                print('%s SEM IMAGEM' % rotulo)
                continue
            tl.salva('largura_%s_%02x' % (nome, larg), q)
            inc, nit = inclinacao(q)
            grava(dict(linha, estado='ok', inclinacao='' if inc is None else '%.1f' % inc, nitidas='%.2f' % nit))
            print('%s inclinacao %s amostras/linha (bordas nitidas %.0f%%)' % (
                rotulo, '  ?  ' if inc is None else '%+5.1f' % inc, nit * 100))
        else:
            terminou = True
    finally:
        fcsv.close()
        jan.fecha()
        t.sudo_tee(t.PS, '')
        for c, v in antes.items():
            if v:
                tl.ctrl(c, v)
    if terminou:
        try:
            os.remove(atual_path)
        except OSError:
            pass
    resumo(csv_path)


def resumo(csv_path):
    """Por configuracao: a primeira largura que inclina (as menores ficam retas) e o comprimento
    estimado = largura + inclinacao. Janelas maiores podem voltar a dar 0 quando forem multiplas da
    linha; por isso vale a primeira que inclina."""
    dados = {}
    for r in csv.DictReader(open(csv_path)):
        if r['estado'] == 'ok' and r['inclinacao'] != '':
            dados.setdefault(r['caso'], []).append((int(r['amostras']), float(r['inclinacao'])))
    print('\n==== COMPRIMENTO DA LINHA ====')
    for nome, _, desc in CASOS:
        pts = sorted(dados.get(nome, []))
        if not pts:
            print('%-6s sem dados' % nome)
            continue
        retas = 0
        for a, inc in pts:					# larguras retas a partir da menor
            if abs(inc) > 1.0:
                break
            retas = a
        torta = next(((a, inc) for a, inc in pts if a > retas and abs(inc) > 1.0), None)
        if torta:
            a, inc = torta
            txt = 'retas ate %d; %d inclina %+.0f/linha -> linha ~%.0f amostras' % (retas, a, inc, a + inc)
        else:
            txt = 'nenhuma largura inclinou (linha >= %d amostras)' % pts[-1][0]
        print('%-6s %s   (%s)' % (nome, txt, desc))
        print('       ' + ' '.join('%d:%+.0f' % (a, inc) for a, inc in pts))


if __name__ == '__main__':
    main()
