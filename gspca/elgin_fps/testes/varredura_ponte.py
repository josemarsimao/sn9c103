#!/usr/bin/env python3
"""Varredura dos registradores da PONTE SN9C103 (modo S910 160x120).
Reaproveita varredura.py (mesmos metodos); so troca o alvo: 's:' (sensor) -> 'b:' (ponte).

Fase 1  leitura dos 256 enderecos da ponte durante a captura, duas vezes (* = mudou sozinho).
Fase 2  grava 0x00 e 0xFF em cada registrador candidato (via regs_start) e mede fps, brilho,
        contraste, linhas/colunas, tamanho comprimido, e se o valor gruda. Restaura o original.
Fase 3  bit a bit nos registradores que mudaram algo.

NUNCA testados por padrao (o kernel/USB ou o fluxo dependem deles):
  0x00 (ID, so leitura), 0x01 (V_TX_EN / power / LED / clock), 0x08 (I2C), 0x17 (clock/teste), 0x19 (MCK/PCK).
Os de I2C (0x09-0x0E) so entram com --i2c (podem desligar o sensor ate reconectar).
Padrao: 0x02-0x04, 0x0F-0x11, 0x1A-0x1F, 0x31-0x7F.  --ext inclui 0x80-0xFF.
Os ja usados pelo driver (0x05-0x07, 0x12-0x16, 0x18, 0x20-0x30) ficam de fora; use a lista
explicita para testa-los:  python3 varredura_ponte.py 1a-1f 31-3f

Cena PARADA e bem iluminada. Pede sudo uma vez. Resultado: varredura_ponte_resultado.txt
Se travar, desconecte e reconecte a camera.

Uso:  python3 varredura_ponte.py --so-leitura | [lista hex] | --i2c | --ext
"""
import os
import re
import statistics
import subprocess
import sys
import time

sys.argv = [sys.argv[0]] + [a for a in sys.argv[1:]]
import varredura as vr  # noqa: E402

ARGS = sys.argv[1:]


def rdb(reg):
    vr.sudo_tee(vr.PR, 'b:0x%02x' % reg)
    try:
        m = re.search(r'= 0x([0-9a-fA-F]{2})', open(vr.PR).read())
    except OSError:
        return None
    return int(m.group(1), 16) if m else None


vr.rd = rdb  # fps_e_leitura() passa a ler a ponte
vr.OUT = os.path.join(vr.HERE, 'varredura_ponte')

NUNCA = {0x00, 0x01, 0x08, 0x17, 0x19}
I2C = set(range(0x09, 0x0f))
USADOS = {0x05, 0x06, 0x07, 0x12, 0x13, 0x14, 0x15, 0x16, 0x18} | set(range(0x20, 0x31))


def lista():
    sel = [a for a in ARGS if not a.startswith('--')]
    if sel:
        out = []
        for a in sel:
            if '-' in a:
                lo, hi = a.split('-')
                out += list(range(int(lo, 16), int(hi, 16) + 1))
            else:
                out.append(int(a, 16))
        return [r for r in out if r not in NUNCA]
    r = [0x02, 0x03, 0x04, 0x0f, 0x10, 0x11] + list(range(0x1a, 0x20)) + list(range(0x31, 0x80))
    if '--i2c' in ARGS:
        r += sorted(I2C)
    if '--ext' in ARGS:
        r += list(range(0x80, 0x100))
    return r


def fase1():
    print('\n==== FASE 1: leitura dos 256 enderecos da ponte (durante a captura) ====')
    vr.regs_start('')
    p = vr.stream_bg(vr.FPS * 240)
    time.sleep(3)
    a = [rdb(r) for r in range(256)]
    b = [rdb(r) for r in range(256)]
    p.terminate()
    p.communicate()
    print('       ' + ' '.join('+%x ' % i for i in range(16)))
    for lin in range(16):
        cel = []
        for col in range(16):
            i = lin * 16 + col
            cel.append('-- ' if a[i] is None else '%02x%s' % (a[i], '*' if a[i] != b[i] else ' '))
        print('0x%02x:  %s' % (lin * 16, ' '.join(cel)))
    print('(-- = leitura falhou; * = mudou entre as duas leituras)')
    return a


def item(reg, val):
    return 'b:0x%02x=0x%02x' % (reg, val)


def main():
    sys.stdout = vr.Tee(sys.__stdout__, open(os.path.join(vr.HERE, 'varredura_ponte_resultado.txt'), 'w'))
    if subprocess.run(['sudo', '-v']).returncode:
        sys.exit('sudo nao validado')
    print(__doc__)
    orig = fase1()
    if '--so-leitura' in ARGS:
        return
    regs = lista()
    print('\n==== FASE 2: 0x00/0xFF em %d registradores da ponte (S910 160x120, %d fps) ====' % (len(regs), vr.FPS))
    resto = ''
    try:
        bases = [vr.caso('') for _ in range(3)]
        for i, c in enumerate(bases):
            print('base %d: %s' % (i + 1, vr.fmt(c)))
        base = vr.Base(bases)
        if not base.ok:
            print('A base nao funciona; confira o driver e a camera.')
            return
        vr.salva_png('base', bases[0]['m'])
        if base.mean > 215 or base.std < 8:
            print('*** CENA INADEQUADA (lisa ou estourada): aponte para algo com detalhe e rode de novo. ***')
            return
        achados = {}
        n = 0
        for reg in regs:
            if orig[reg] is None:
                continue
            for val in vr.VALORES:
                if orig[reg] == val:
                    continue
                rs = ','.join(x for x in (resto, item(reg, val)) if x)
                c = vr.caso(rs, reg)
                resto = item(reg, orig[reg])
                mot = base.motivos(c)
                grude = '' if c['lido'] is None else ('gruda' if c['lido'] == val else 'lido=%02x' % c['lido'])
                print('0x%02x=0x%02x  %-10s %s  %s' % (reg, val, grude, vr.fmt(c), ' '.join(mot).upper() if mot else ''), flush=True)
                if mot:
                    achados.setdefault(reg, []).extend(mot)
                    if c['m']:
                        vr.salva_png('r%02x_v%02x' % (reg, val), c['m'])
                n += 1
                if n % 12 == 0 and not vr.Base([vr.caso(resto)]).ok:
                    print('\n*** A CAMERA PAROU depois de 0x%02x. Desconecte e reconecte. ***' % reg)
                    return
        print('\n==== FASE 3: bits (%s) ====' % (' '.join('0x%02x' % r for r in sorted(achados)) or 'nenhum'))
        for reg in sorted(achados)[:8]:
            for bit in range(8):
                val = 1 << bit
                rs = ','.join(x for x in (resto, item(reg, val)) if x)
                c = vr.caso(rs, reg)
                resto = item(reg, orig[reg])
                mot = base.motivos(c)
                print('0x%02x bit %d (=0x%02x)  %s  %s' % (reg, bit, val, vr.fmt(c), ' '.join(mot).upper() if mot else ''), flush=True)
                if mot and c['m']:
                    vr.salva_png('r%02x_bit%d' % (reg, bit), c['m'])
        print('\n==== RESUMO ====')
        if not achados:
            print('nenhum registrador testado alterou imagem ou fps.')
        for reg in sorted(achados):
            print('0x%02x  %s' % (reg, ' '.join(sorted(set(achados[reg])))))
    finally:
        vr.regs_start(resto)
        c = vr.caso(resto)
        vr.regs_start('')
        print('\nverificacao final (restaurado): %s' % vr.fmt(c))


if __name__ == '__main__':
    main()
