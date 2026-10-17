#!/usr/bin/env python3
"""Testes de combinacao dirigidos: grupos de registradores que o datasheet diz que trabalham juntos.

Autor: Josemar Simao <josemars@ifes.edu.br>
Co-autor e Revisor: Claude (Anthropic)

A varredura bit a bit (varre_bits.py) inverte um bit por vez; aqui cada caso grava um conjunto de
registradores de uma vez (ex.: habilitar o offset manual E escrever o valor). Mesmo arranjo: camera no
suporte olhando o padrao "composto" da janela, modo base S910 320x240 a 30 fps, ganho 47 e exposicao 67
fixos, ganho automatico e taxa dinamica desligados. Para cada caso grava fps, brilho, a cor das faixas
(R, G, B nas posicoes do mosaico) e a rampa, e no fim imprime a tabela comparada com a referencia.

Fases: sensor | ponte.  Casos numerados, gravados um a um em comb_<fase>.csv (com fsync); se travar,
rode de novo com --continua (retoma no caso interrompido) ou --continua --pula N.
Resultado: comb_<fase>_resumo.txt e comb_<fase>.html (imagens em testes/comb_<fase>/).
Uso:  python3 testa_combina.py sensor
"""
import csv
import os
import statistics
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import varre_bits as vb  # noqa: E402  (captura, regiao, medidas, salva, recupera, sudo, controles fixos)
from janela import Janela  # noqa: E402

# (grupo, descricao, regs_start). Valores de base lidos nas varreduras: sensor 0x06 = 0x7F, 0x12 = 0x7C (AWB
# ligado), 0x13 = 0x80, 0x15 = 0x34, 0x20 = 0x40, 0x24 = 0x10, 0x25 = 0x8A, 0x26 = 0xA0, 0x27 = 0x9A,
# 0x29 = 0x30, 0x61 = 0x42, 0x68 = 0xAC, 0x21/0x22/0x2E e 0x78-0x7A = 0x80; ponte 0x17 = 0x68, 0x18 = 0x9F,
# 0x19 = 0x20.
OFS = 's:0x26=0xa2,s:0x27=0x9e'			# COMF bit 1 (offset digital) + COMG bit 2 (manual)
SENSOR = [
    ('offset', 'COMF bit 1 so (offset automatico)', 's:0x26=0xa2'),
    ('offset', 'offset manual, valores neutros (0x80)', OFS + ',s:0x21=0x80,s:0x22=0x80,s:0x2e=0x80'),
    ('offset', 'manual G +32 (0x21 = 0x20)', OFS + ',s:0x21=0x20,s:0x22=0x80,s:0x2e=0x80'),
    ('offset', 'manual G -32 (0x21 = 0xa0)', OFS + ',s:0x21=0xa0,s:0x22=0x80,s:0x2e=0x80'),
    ('offset', 'manual B +32 (0x22 = 0x20)', OFS + ',s:0x21=0x80,s:0x22=0x20,s:0x2e=0x80'),
    ('offset', 'manual B -32 (0x22 = 0xa0)', OFS + ',s:0x21=0x80,s:0x22=0xa0,s:0x2e=0x80'),
    ('offset', 'manual R +32 (0x2E = 0x20)', OFS + ',s:0x21=0x80,s:0x22=0x80,s:0x2e=0x20'),
    ('offset', 'manual R -32 (0x2E = 0xa0)', OFS + ',s:0x21=0x80,s:0x22=0x80,s:0x2e=0xa0'),
    ('adc', 'offset do ADC desligado (COMG bit 3 = 0)', 's:0x27=0x92'),
    ('adc', 'ADC G 0x00 (0x78)', 's:0x78=0x00'), ('adc', 'ADC G 0xff (0x78)', 's:0x78=0xff'),
    ('adc', 'ADC B 0x00 (0x79)', 's:0x79=0x00'), ('adc', 'ADC B 0xff (0x79)', 's:0x79=0xff'),
    ('adc', 'ADC R 0x00 (0x7A)', 's:0x7a=0x00'), ('adc', 'ADC R 0xff (0x7A)', 's:0x7a=0xff'),
    ('adc', 'ADC B 0xff com COMG bit 3 = 0', 's:0x27=0x92,s:0x79=0xff'),
    ('brilho', 'BRTEN = 0, BRT 0x00', 's:0x06=0x00'), ('brilho', 'BRTEN = 0, BRT 0xff', 's:0x06=0xff'),
    ('brilho', 'BRTEN = 1, BRT 0x00', 's:0x61=0x4a,s:0x06=0x00'),
    ('brilho', 'BRTEN = 1, BRT 0x40', 's:0x61=0x4a,s:0x06=0x40'),
    ('brilho', 'BRTEN = 1, BRT 0xc0', 's:0x61=0x4a,s:0x06=0xc0'),
    ('brilho', 'BRTEN = 1, BRT 0xff', 's:0x61=0x4a,s:0x06=0xff'),
    ('brilho', 'BRTEN = 1, BRTH = 1, BRT 0xff', 's:0x61=0x4e,s:0x06=0xff'),
    ('brilho', 'BRTEN = 1 + offset digital (COMF bit 1), BRT 0xff', 's:0x61=0x4a,s:0x26=0xa2,s:0x06=0xff'),
    ('aec', 'AEC interno (0x13 bit 0)', 's:0x13=0x81'), ('aec', 'AGC interno (0x13 bit 1)', 's:0x13=0x82'),
    ('aec', 'AEC + AGC internos', 's:0x13=0x83'),
    ('aec', 'AEC + AGC, alvo claro (0x24 = 0x60, 0x25 = 0x50)', 's:0x13=0x83,s:0x24=0x60,s:0x25=0x50'),
    ('aec', 'AEC + AGC, alvo escuro (0x24 = 0x20, 0x25 = 0x90)', 's:0x13=0x83,s:0x24=0x20,s:0x25=0x90'),
    ('aec', 'AEC + AGC, referencia baixa (0x68 = 0x0c)', 's:0x13=0x83,s:0x68=0x0c'),
    ('aec', 'AEC + AGC, referencia alta (0x68 = 0xec)', 's:0x13=0x83,s:0x68=0xec'),
    ('aec', 'AEC + AGC, janela central 1/4 (0x29 bit 3)', 's:0x13=0x83,s:0x29=0x38'),
    ('awb', 'AWB desligado (0x12 bit 2 = 0)', 's:0x12=0x78'), ('awb', 'AWB ligado (= base)', 's:0x12=0x7c'),
    ('awb', 'AWB + modo rapido (0x20 bit 1)', 's:0x12=0x7c,s:0x20=0x42'),
    ('awb', 'AWB + ajuste manual no automatico (0x20 bit 2)', 's:0x12=0x7c,s:0x20=0x44'),
    ('awb', 'AWB + modo esperto (0x20 bit 3)', 's:0x12=0x7c,s:0x20=0x48'),
    ('awb', 'AWB + passo de 1 bit (0x15 bits 5:4 = 00)', 's:0x12=0x7c,s:0x15=0x04'),
]
PONTE = [
    ('clock', 'SEN_RATE = 00, MCK_SIZE 2 (24/2 = 12 MHz)', 'b:0x17=0x60,b:0x19=0x20'),
    ('clock', 'SEN_RATE = 00, MCK_SIZE 3 (8 MHz)', 'b:0x17=0x60,b:0x19=0x30'),
    ('clock', 'SEN_RATE = 00, MCK_SIZE 4 (6 MHz)', 'b:0x17=0x60,b:0x19=0x40'),
    ('clock', 'SEN_RATE = 00, MCK_SIZE 5 (4,8 MHz; Risolia em cru)', 'b:0x17=0x60,b:0x19=0x50'),
    ('clock', 'SEN_RATE = 00, MCK_SIZE 8 (3 MHz)', 'b:0x17=0x60,b:0x19=0x80'),
    ('clock', 'SEN_RATE = 00, MCK_SIZE 15 (1,6 MHz)', 'b:0x17=0x60,b:0x19=0xf0'),
    ('clock', '12 MHz pelo SEN_RATE = 01', 'b:0x17=0x64'),
    ('sync', 'SYNC_OUT + PCK_OUT (0x19 = 0x23)', 'b:0x19=0x23'),
    ('sync', 'SYNC_OUT + PCK_OUT + PCK_2X (0x19 = 0x2b)', 'b:0x19=0x2b'),
    ('sync', 'PCK_OUT + PCK_2X sem SYNC_OUT (0x19 = 0x2a)', 'b:0x19=0x2a'),
    ('sync', 'original do gspca: 0x17 = 0x6c (48 MHz), 0x19 = 0x2b', 'b:0x17=0x6c,b:0x19=0x2b'),
    ('sync', 'original com 24 MHz: 0x17 = 0x68, 0x19 = 0x2b, sensor 0x11 = 1', 'b:0x19=0x2b,s:0x11=0x01'),
    ('polar', '0x18 bordas: PCK descida (0x9e)', 'b:0x18=0x9e'),
    ('polar', '0x18 bordas: HSYNC descida (0x9d)', 'b:0x18=0x9d'),
    ('polar', '0x18 bordas: PCK e HSYNC descida (0x9c)', 'b:0x18=0x9c'),
    ('polar', '0x18 VSYNC descida (0x9b)', 'b:0x18=0x9b'),
    ('polar', '0x18 VSYNC ativo baixo (0x97)', 'b:0x18=0x97'),
]


def main():
    """Roda os casos numerados da fase, gravando cada um antes do seguinte, e escreve o resumo."""
    fase = next((a for a in sys.argv[1:] if a in ('sensor', 'ponte')), 'sensor')
    casos = SENSOR if fase == 'sensor' else PONTE
    pula = set()
    for i, a in enumerate(sys.argv):
        if a == '--pula' and i + 1 < len(sys.argv):
            pula |= {int(x) for x in sys.argv[i + 1].split(',') if x.strip()}
    pasta = os.path.join(HERE, 'comb_%s' % fase)
    csv_path = os.path.join(HERE, 'comb_%s.csv' % fase)
    atual_path = os.path.join(HERE, 'comb_%s_atual.txt' % fase)
    sys.stdout = vb.Tee(sys.__stdout__, open(os.path.join(HERE, 'comb_%s_log.txt' % fase), 'a', buffering=1))
    if subprocess.run(['sudo', '-v']).returncode:
        sys.exit('sudo nao validado')
    print(__doc__)
    print('==== %s  combinacoes do %s: %d casos' % (time.strftime('%Y-%m-%d %H:%M'), fase, len(casos)))
    chaves = sorted(vb.medidas(np.zeros((vb.H, vb.W), np.float32), (0, vb.H, 0, vb.W)))
    campos = ['n', 'grupo', 'descricao', 'regs', 'estado', 'fps', 'bytes'] + chaves
    feitos = set()
    continua = '--continua' in sys.argv and os.path.exists(csv_path)
    if continua:
        feitos = {int(r['n']) for r in csv.DictReader(open(csv_path)) if r['n'] != 'REF'}
        print('retomando: %d casos ja gravados' % len(feitos))
        try:
            parado = int(open(atual_path).read())
            if parado not in feitos:
                print('*** o caso %d foi interrompido: %s' % (parado, 'sera PULADO' if parado in pula else
                      'sera repetido (se travar de novo, use --pula %d)' % parado))
        except (OSError, ValueError):
            pass
    fcsv = open(csv_path, 'a' if continua else 'w', newline='')
    wr = csv.DictWriter(fcsv, campos, extrasaction='ignore')
    if not continua:
        wr.writeheader()

    def grava(linha):
        """Grava a linha no CSV e forca a ida ao disco."""
        wr.writerow(linha)
        fcsv.flush()
        os.fsync(fcsv.fileno())

    dev = vb.acha_dev()
    ctrl0 = {}
    for nome, val in vb.FIXOS:
        out = subprocess.run(['v4l2-ctl', '-d', dev, '-C', nome], capture_output=True, text=True).stdout
        ctrl0[nome] = out.split(':')[-1].strip()
        subprocess.run(['v4l2-ctl', '-d', dev, '-c', '%s=%d' % (nome, val)], capture_output=True)
    jan = Janela('alinhar')
    terminou = False
    try:
        input('\n>>> Confira pelo Multiview se o contorno do padrao "alinhar" esta inteiro na imagem (a janela\n'
              '    abre na posicao gravada). FECHE o Multiview e aperte Enter... ')
        jan.mostra('branco', espera=0.6)
        _, _, qb = vb.captura(dev, '')
        if qb is None:
            sys.exit('sem imagem na referencia: confira a camera')
        roi = vb.regiao(qb)
        jan.mostra('composto', espera=0.6)

        def referencia():
            """Captura a referencia e grava suas medidas no CSV (linha n = REF)."""
            fps, nb, q = vb.captura(dev, '')
            if q is None:
                return None
            linha = dict(n='REF', grupo='REF', descricao='referencia', estado='REF', fps='%.1f' % (fps or 0.0), bytes=nb)
            linha.update({k: '%.2f' % v for k, v in vb.medidas(q, roi).items()})
            grava(linha)
            return q
        qref = None
        for _ in range(2):
            q = referencia()
            qref = q if q is not None else qref
        if qref is None:
            sys.exit('a referencia nao deu imagem')
        vb.salva(pasta, 'referencia', qref)
        for n, (grupo, desc, rs) in enumerate(casos, 1):
            if n in feitos:
                continue
            rotulo = '[%02d/%02d] %-7s %s' % (n, len(casos), grupo, desc)
            linha = dict(n=n, grupo=grupo, descricao=desc, regs=rs)
            if n in pula:
                grava(dict(linha, estado='PULADO'))
                print('%s  ->  PULADO' % rotulo)
                continue
            if n % 12 == 1:
                subprocess.run(['sudo', '-v'])
            with open(atual_path, 'w') as fa:
                fa.write('%d\n' % n)
                fa.flush()
                os.fsync(fa.fileno())
            fps, nb, q = vb.captura(dev, rs)
            linha.update(fps='' if fps is None else '%.1f' % fps, bytes=nb)
            if q is None:
                grava(dict(linha, estado='SEM IMAGEM' if fps else 'PAROU'))
                print('%s  ->  %s' % (rotulo, 'SEM IMAGEM' if fps else 'PAROU'))
                dev = vb.recupera(dev)
                if dev is None:
                    print('a camera nao voltou: reconecte e rode de novo com --continua')
                    break
                continue
            m = vb.medidas(q, roi)
            linha.update({k: '%.2f' % v for k, v in m.items()}, estado='ok')
            vb.salva(pasta, 'c%02d' % n, q)
            grava(linha)
            print('%s  fps=%s media=%.1f' % (rotulo, linha['fps'], m['media']))
        else:
            terminou = True
    finally:
        fcsv.close()
        jan.fecha()
        vb.sudo_tee(vb.PS, '')
        for nome, val in reversed(list(ctrl0.items())):
            if val:
                subprocess.run(['v4l2-ctl', '-d', dev, '-c', '%s=%s' % (nome, val)], capture_output=True)
    if terminou:
        try:
            os.remove(atual_path)
        except OSError:
            pass
    resumo(fase, csv_path)


def resumo(fase, csv_path):
    """Tabela por caso: fps, brilho, cor da faixa branca (B, G, R do mosaico), a cor de cada faixa na sua
    posicao (R na vermelha, G na verde, B na azul) e os extremos da rampa, ao lado da referencia."""
    linhas = list(csv.DictReader(open(csv_path)))
    refs = [r for r in linhas if r['n'] == 'REF']
    f = lambda r, k: float(r[k]) if r.get(k) not in (None, '') else float('nan')
    def med(k):
        v = [f(r, k) for r in refs]
        return statistics.median(v) if v else float('nan')
    col = [('fps', 'fps'), ('media', 'brilho'), ('branco_00', 'brB'), ('branco_01', 'brG'), ('branco_11', 'brR'),
           ('vermelho_11', 'R/verm'), ('verde_01', 'G/verde'), ('azul_00', 'B/azul'), ('rampa_00', 'rampa0'),
           ('rampa_15', 'rampa15')]
    out = ['Combinacoes do %s (%s). Valores medidos; a linha REF e a mediana das referencias.' % (fase, time.strftime('%Y-%m-%d')),
           '', '%-4s %-7s %-58s %s' % ('n', 'grupo', 'caso', ' '.join('%7s' % c for _, c in col)),
           '%-4s %-7s %-58s %s' % ('REF', '', 'referencia', ' '.join('%7.1f' % med(k) for k, _ in col))]
    for r in linhas:
        if r['n'] == 'REF':
            continue
        if r['estado'] != 'ok':
            out.append('%-4s %-7s %-58s %s' % (r['n'], r['grupo'], r['descricao'][:58], r['estado']))
            continue
        out.append('%-4s %-7s %-58s %s' % (r['n'], r['grupo'], r['descricao'][:58], ' '.join('%7.1f' % f(r, k) for k, _ in col)))
    txt = '\n'.join(out)
    print('\n' + txt)
    open(os.path.join(HERE, 'comb_%s_resumo.txt' % fase), 'w').write(txt + '\n')
    with open(os.path.join(HERE, 'comb_%s.html' % fase), 'w') as h:
        h.write('<!doctype html><meta charset="utf-8"><title>Combinacoes %s</title><style>body{font-family:sans-serif;'
                'margin:16px}td{padding:4px;vertical-align:top;border-bottom:1px solid #ddd}img{width:160px}</style>'
                '<h1>Combinacoes do %s</h1><p>Referencia:<br><img src="comb_%s/referencia.png"></p><table>' % (fase, fase, fase))
        for r in linhas:
            if r['n'] == 'REF':
                continue
            img = 'comb_%s/c%02d.png' % (fase, int(r['n']))
            h.write('<tr><td><b>%s</b> %s<br><code>%s</code><br>%s fps=%s brilho=%s</td><td>%s</td></tr>' % (
                r['n'], r['descricao'], r['regs'], r['estado'], r['fps'], r.get('media', ''),
                '<img src="%s">' % img if os.path.exists(os.path.join(HERE, img)) else ''))
        h.write('</table>')
    print('\nResumo: comb_%s_resumo.txt  Relatorio: comb_%s.html' % (fase, fase))


if __name__ == '__main__':
    main()
