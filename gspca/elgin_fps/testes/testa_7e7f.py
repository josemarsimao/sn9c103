#!/usr/bin/env python3
"""Confirma a formula dos registradores 0x7E/0x7F do sensor (hipotese: contador de 16 bits de LINHAS EXTRAS por
quadro, ~62 us cada) com valores intermediarios, e confirma a restauracao para 0x00.

Modo S910 160x120 a 60 fps (o mesmo da varredura). Para cada valor N de 16 bits (0x7F = byte alto, 0x7E = baixo)
grava via regs_start, mede o fps e calcula o tempo extra por quadro: dT = 1/fps - 1/fps_base; dT/N deve ser
constante (~62 us) se a hipotese vale. No fim restaura 0x00 e mede de novo.

Aponte a camera para uma cena PARADA. Pede sudo uma vez. Resultado: r7e7f_resultado.txt  (~5 min)
Uso:  python3 testa_7e7f.py
"""
import os
import re
import statistics
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import varredura as vr  # noqa: E402

VALORES = [0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080, 0x00C0, 0x00FF,
           0x0100, 0x0180, 0x0200, 0x0300]
NQ = 40		# quadros por medicao


def mede():
    """fps medio (mediana) de NQ quadros, mais as leituras de 0x7E/0x7F no meio da captura."""
    p = subprocess.Popen(['v4l2-ctl', '-d', vr.DEV] + vr.FMT + ['--stream-count=%d' % NQ, '--stream-to=/dev/null'],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    time.sleep(1.5)
    lido = (vr.rd(0x7e), vr.rd(0x7f))
    try:
        out = p.communicate(timeout=120)[0]
    except subprocess.TimeoutExpired:
        p.kill()
        return None, lido
    f = [float(x) for x in re.findall(r'<+ ([0-9.]+) fps', out)]
    return (statistics.median(f) if f else None), lido


def rs(n):
    return 's:0x7e=0x%02x,s:0x7f=0x%02x' % (n & 0xff, n >> 8)


def main():
    sys.stdout = vr.Tee(sys.__stdout__, open(os.path.join(HERE, 'r7e7f_resultado.txt'), 'w'))
    if subprocess.run(['sudo', '-v']).returncode:
        sys.exit('sudo nao validado')
    print(__doc__)
    vr.regs_start('s:0x7e=0x00,s:0x7f=0x00')
    base, lido = mede()
    if not base:
        sys.exit('A base nao funciona; confira o driver e a camera.')
    print('base (0x7E/0x7F = 0x00): %.2f fps  leitura 7E/7F = %s' % (base, lido))
    t0 = 1.0 / base
    res = []
    print('\n    N      7F:7E    fps     dT(ms)   dT/N(us)   lido 7E/7F')
    for n in VALORES:
        vr.regs_start(rs(n))
        f, lido = mede()
        if not f:
            print('0x%04x  SEM MEDIDA (stream parou ou ficou lento demais)' % n)
            continue
        dt = 1.0 / f - t0
        us = dt / n * 1e6
        res.append((n, us))
        print('0x%04x  %02x:%02x  %7.2f  %8.2f  %8.1f   %s' % (n, n >> 8, n & 0xff, f, dt * 1e3, us, lido))
    if res:
        v = [u for _, u in res]
        print('\ndT/N: mediana %.1f us, minimo %.1f, maximo %.1f' % (statistics.median(v), min(v), max(v)))
        print('constante (hipotese confirmada) se minimo e maximo ficam perto da mediana (+-10%)')
    print('\n== restauracao ==')
    vr.regs_start('s:0x7e=0x00,s:0x7f=0x00')
    f, lido = mede()
    print('depois de regs_start 0x00: %s fps  leitura 7E/7F = %s  (base %.2f)' % (f, lido, base))
    vr.regs_start('')
    f, lido = mede()
    print('depois de regs_start vazio: %s fps  leitura 7E/7F = %s  (deve voltar a %.2f; se ficar baixo, o valor '
          'so volta com 0x00 explicito ou reconectando a camera)' % (f, lido, base))
    print('\nfim')


if __name__ == '__main__':
    main()
