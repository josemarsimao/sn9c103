#!/usr/bin/env python3
"""Aplica o resultado de uma varredura (varre_bits.py) ao MAPA_REGISTRADORES.html.

Autor: Josemar Simao <josemars@ifes.edu.br>
Co-autor e Revisor: Claude (Anthropic)

Le varre_<fase>.csv (bits testados) e varre_<fase>_resumo.txt (bits com efeito e o motivo) e, para cada
registrador: marca o estado de teste das linhas de bits (T✔ efeito, T✖ sem efeito; um T✔ existente nao
e' rebaixado) e acrescenta uma nota com os efeitos medidos. Grava com doc_html (confere a tabela e faz
copia .bak_varre_<fase>).

Uso:  python3 aplica_mapa.py sensor [caminho do MAPA]
"""
import csv
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import doc_html  # noqa: E402

LADO = {'s': 'sensor', 'b': 'ponte'}


def main():
    """Le os resultados da fase e atualiza o MAPA."""
    fase = sys.argv[1] if len(sys.argv) > 1 else 'sensor'
    mapa = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(HERE), 'MAPA_REGISTRADORES.html')
    testados = {}						# (tipo, reg) -> {bit: efeito?}
    for r in csv.DictReader(open(os.path.join(HERE, 'varre_%s.csv' % fase))):
        if r['n'] != 'REF' and r['estado'] in ('ok', 'PAROU', 'SEM IMAGEM'):
            testados.setdefault((r['tipo'], int(r['reg'], 16)), {})[int(r['bit'])] = False
    motivos = {}						# (tipo, reg) -> [(bit, motivo)]
    for linha in open(os.path.join(HERE, 'varre_%s_resumo.txt' % fase)):
        m = re.match(r'([sb]):0x([0-9a-f]{2}) bit (\d) \((0x[0-9a-f]{2}) -> (0x[0-9a-f]{2})\): (.*)', linha.strip())
        if not m:
            continue
        tipo, reg, bit = m.group(1), int(m.group(2), 16), int(m.group(3))
        if (tipo, reg) in testados:
            testados[(tipo, reg)][bit] = True
            motivos.setdefault((tipo, reg), []).append((bit, m.group(4), m.group(5), m.group(6)))
    d = doc_html.Doc(mapa)
    mud = 0
    for (tipo, reg), res in sorted(testados.items()):
        mud += d.marca_bits(LADO[tipo], '0x%02x' % reg, res)
        if (tipo, reg) in motivos:
            txt = '; '.join('bit %d (%s → %s): %s' % (b, v0, v1, mo.replace('->', '→'))
                            for b, v0, v1, mo in sorted(motivos[(tipo, reg)], reverse=True))
            d.nota(LADO[tipo], '0x%02x' % reg, 'sup', 'varredura',
                   'Um bit por vez (varre_bits.py %s, 2026-10-17; referência: média 23,5, 29,8 fps): %s. '
                   'Demais bits sem efeito.' % (fase, txt), t='y', d='o')
        else:
            d.nota(LADO[tipo], '0x%02x' % reg, 'sup', 'varredura',
                   'Um bit por vez (varre_bits.py %s, 2026-10-17): nenhum bit teve efeito no modo base '
                   '(S910 320x240, VGA progressivo).' % fase, t='n', d='o')
    print('registradores: %d, linhas de estado alteradas: %d' % (len(testados), mud))
    print(d.salva('varre_%s' % fase))


if __name__ == '__main__':
    main()
