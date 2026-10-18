#!/usr/bin/env python3
"""Escala da ponte (0x18 bits 5:4): confere se 11 tambem e' 1/4, como 10.

Autor: Josemar Simao <josemars@ifes.edu.br>
Co-autor e Revisor: Claude (Anthropic)

No formato cru (BA81) nao ha compressao: o tamanho de cada quadro em bytes mostra a escala. Pede BA81
320x240 e troca so o 0x18 (compressao desligada, sincronismo igual ao do driver): 0x0F = 1/1, 0x1F = 1/2,
0x2F = 1/4 (10) e 0x3F (11), em dois modos:
  VGA  (modo normal: sensor 640x480, janela da ponte 640x480): esperado 307200 (nao cabe no buffer),
       76800, 19200 e, se 11 for 1/4, 19200;
  QVGA (sensor 0x14 = 0xA0, janela da ponte 320x240): esperado 76800, 19200, 4800 e, se 11 for 1/4, 4800.
Qualquer cena serve. Pede sudo uma vez.
Casos numerados, gravados um a um em escala_dados.csv. Resultado: escala_resultado.txt
"""
import csv
import os
import re
import statistics
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from elgin_dev import acha_dev  # noqa: E402

PS = '/sys/module/gspca_elgin_fps/parameters/regs_start'
RAW = '/tmp/escala.raw'
ESCALAS = [(0x0f, '00', '1/1'), (0x1f, '01', '1/2'), (0x2f, '10', '1/4'), (0x3f, '11', '?')]
MODOS = [('VGA', ''), ('QVGA', 's:0x14=0xa0,b:0x15=0x14,b:0x16=0x0f,')]
CASOS = [(modo, pre, v, bits, esc) for modo, pre in MODOS for v, bits, esc in ESCALAS]
N = 5						# quadros gravados por caso


def main():
    """Para cada valor do 0x18, captura N quadros BA81 e calcula o tamanho medio por quadro."""
    dev = acha_dev()
    if subprocess.run(['sudo', '-v']).returncode:
        sys.exit('sudo nao validado')
    out = open(os.path.join(HERE, 'escala_resultado.txt'), 'w')
    fcsv = open(os.path.join(HERE, 'escala_dados.csv'), 'w', newline='')
    wr = csv.writer(fcsv)
    wr.writerow(['n', 'modo', 'reg18', 'bits54', 'esperado', 'fps', 'bytes_por_quadro', 'quadros'])

    def mostra(txt):
        """Imprime e grava no resultado."""
        print(txt)
        out.write(txt + '\n')
        out.flush()
    mostra('==== %s  dispositivo %s: BA81 320x240, so o 0x18 muda (VGA e QVGA)' % (time.strftime('%Y-%m-%d %H:%M'), dev))
    try:
        for n, (modo, pre, v, bits, esc) in enumerate(CASOS, 1):
            subprocess.run(['sudo', '-n', 'tee', PS], input=('%sb:0x18=0x%02x\n' % (pre, v)).encode(), capture_output=True)
            try:
                os.remove(RAW)
            except OSError:
                pass
            try:
                r = subprocess.run(['v4l2-ctl', '-d', dev, '--set-fmt-video=width=320,height=240,pixelformat=BA81',
                                    '--set-parm=30', '--stream-mmap', '--stream-skip=5', '--stream-count=%d' % N,
                                    '--stream-to=' + RAW], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   text=True, timeout=30).stdout
            except subprocess.TimeoutExpired:
                r = ''
            f = [float(x) for x in re.findall(r'<+ ([0-9.]+) fps', r)]
            tam = os.path.getsize(RAW) if os.path.exists(RAW) else 0
            por = tam // N if tam else 0
            wr.writerow([n, modo, '0x%02x' % v, bits, esc, '%.1f' % statistics.median(f) if f else '', por, N])
            fcsv.flush()
            os.fsync(fcsv.fileno())
            mostra('[%d/%d] %-4s 0x18=0x%02x (bits 5:4 = %s, %s): %s bytes por quadro%s' % (
                n, len(CASOS), modo, v, bits, esc, por if por else 'nenhum quadro',
                {307200: ' = 640x480', 76800: ' = 320x240', 19200: ' = 160x120', 4800: ' = 80x60'}.get(por, '')))
    finally:
        subprocess.run(['sudo', '-n', 'tee', PS], input=b'\n', capture_output=True)
        fcsv.close()


if __name__ == '__main__':
    main()
