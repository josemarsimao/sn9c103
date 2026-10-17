#!/usr/bin/env python3
"""doc_html.py: edição segura do MAPA_REGISTRADORES.html e do BITS_ANALISE_60FPS.html.

Autor: Josemar Simão <josemars@ifes.edu.br>
Co-autor e Revisor: Claude (Anthropic)

Os scripts que geravam esses HTML não existem mais; o HTML é o original e é editado no lugar.
Esta ferramenta acrescenta notas no formato da tabela, troca trechos e confere se a tabela
continua íntegra antes de gravar (com cópia .bak_<assunto> na mesma pasta).

Uso como comando:
    python3 doc_html.py checa MAPA_REGISTRADORES.html      # confere a tabela
    python3 doc_html.py checa BITS_ANALISE_60FPS.html

Uso num script curto (exemplo):
    import doc_html
    m = doc_html.Doc('MAPA_REGISTRADORES.html')
    m.nota('sensor', '0x61', 'sup', 'medido', 'BRTEN = 1 faz o 0x06 agir na Bayer.', t='y', d='o')
    m.troca('texto antigo exato', 'texto novo')
    m.salva('brilho')                                  # grava com cópia MAPA_...html.bak_brilho

    b = doc_html.Doc('BITS_ANALISE_60FPS.html')
    b.bloco('Atualização AAAA-MM-DD: assunto', ['item 1', 'item 2'])
    b.salva('assunto')

Fontes (cor e etiqueta da legenda do MAPA): 'ds' datasheet, 'ris' Risolia, 'win' captura do Windows,
'sup' suposição ou medida nossa. Estado: t = 'y' (T✔), 'p' (T◐), 'n' (T✖), 'o' (T○);
d = 'y' (D●), 'p' (D◐), 'o' (D○).
"""
import os
import re
import shutil
import sys

# marcadores de estado, iguais aos que o MAPA já usa (T = teste, D = driver)
_T = {'y': '<span class="t-y" title="confirmado em teste">T✔</span>',
      'p': '<span class="t-p">T◐</span>',
      'n': '<span class="t-n" title="testado, sem efeito / refutado">T✖</span>',
      'o': '<span class="t-o" title="não testado">T○</span>'}
_D = {'y': '<span class="d-y" title="implementado">D●</span>',
      'p': '<span class="d-p">D◐</span>',
      'o': '<span class="d-o" title="não implementado">D○</span>'}
_FONTES = ('ds', 'ris', 'win', 'sup')
# as 4 células vazias de cada lado de uma linha do MAPA (0 = ponte, 1 = sensor)
_VAZIO = ('<td class="f"></td><td class="bits"></td><td class="p "></td><td class="st"></td>',
          '<td class="f2"></td><td class="bits"></td><td class="p "></td><td class="st"></td>')
_LINHA = re.compile(r'<tr[^>]*>.*?</tr>', re.S)


class Doc:
    """Um documento HTML aberto para edição; nada é gravado até salva()."""

    def __init__(self, caminho):
        """Lê o arquivo e guarda o texto original para a checagem final."""
        self.caminho = caminho
        self.h = open(caminho, encoding='utf-8').read()
        self.original = self.h
        self.mudancas = []

    # ---------------------------------------------------------------- MAPA
    def _celulas(self, lado, fonte, rotulo, texto, t, d):
        """Monta as 4 células de uma linha de nota no estilo do MAPA."""
        f = 'f' if lado == 0 else 'f2'
        return ('<td class="%s"></td><td class="bits"><span class="s-%s">—</span></td>'
                '<td class="p bit"><span class="s-%s"><b>%s</b> %s</span> <span class="tg %s">%s</span></td>'
                '<td class="st">%s %s</td>') % (f, fonte, fonte, rotulo, texto, fonte, fonte.upper(), _T[t], _D[d])

    def nota(self, lado, reg, fonte, rotulo, texto, t='o', d='o'):
        """Acrescenta uma nota ao grupo do registrador 'reg' ('0x61') do lado 'ponte' ou 'sensor'.
        Usa uma linha do grupo em que esse lado está vazio; se não houver, cria uma linha no fim do grupo.
        'texto' é HTML: escreva &lt; e &gt; para os sinais < e >."""
        lado = {'ponte': 0, 'sensor': 1}[lado]
        assert fonte in _FONTES, 'fonte deve ser uma de %s' % (_FONTES,)
        reg = reg[:2].lower() + reg[2:].upper()          # o MAPA escreve 0x6B, 0x1C...
        linhas = list(_LINHA.finditer(self.h))
        chave = '<td class="%s">%s</td>' % ('f' if lado == 0 else 'f2', reg)
        inicio = next((i for i, m in enumerate(linhas) if chave in m.group(0) and 'first' in m.group(0)), None)
        if inicio is None:
            raise KeyError('registrador %s não encontrado do lado %s' % (reg, 'ponte' if lado == 0 else 'sensor'))
        fim = inicio + 1
        while fim < len(linhas) and 'first' not in linhas[fim].group(0) and 'class="sec"' not in linhas[fim].group(0):
            fim += 1
        nova = self._celulas(lado, fonte, rotulo, texto, t, d)
        for m in linhas[inicio + 1:fim]:                  # reaproveita uma linha com esse lado vazio
            if _VAZIO[lado] in m.group(0):
                self.h = self.h[:m.start()] + m.group(0).replace(_VAZIO[lado], nova, 1) + self.h[m.end():]
                break
        else:                                             # cria a linha, com a mesma cor de fundo do grupo
            cls = 'g1' if 'class="g1' in linhas[inicio].group(0) else 'g0'
            tds = nova + _VAZIO[1] if lado == 0 else _VAZIO[0] + nova
            pos = linhas[fim - 1].end()
            self.h = self.h[:pos] + '\n<tr class="%s">%s</tr>' % (cls, tds) + self.h[pos:]
        self.mudancas.append('nota %s %s: %s' % (('ponte', 'sensor')[lado], reg, rotulo))

    def marca_bits(self, lado, reg, resultado):
        """Marca o estado de teste (T) das linhas de bits do registrador a partir de uma varredura.
        resultado = {bit: True (teve efeito) ou False (testado, sem efeito)}. Linha de um bit ou de uma
        faixa (ex. "7:4"): T✔ se algum bit testado da linha teve efeito; T✖ se todos os testados ficaram
        sem efeito, mas so a partir de T○: T✔ e T◐ existentes nao sao rebaixados (podem depender de combinacao). A linha do
        registrador (cabecalho) segue a mesma regra com todos os bits. Devolve quantas linhas mudaram."""
        lado = {'ponte': 0, 'sensor': 1}[lado]
        reg = reg[:2].lower() + reg[2:].upper()
        linhas = list(_LINHA.finditer(self.h))
        chave = '<td class="%s">%s</td>' % ('f' if lado == 0 else 'f2', reg)
        inicio = next((i for i, m in enumerate(linhas) if chave in m.group(0) and 'first' in m.group(0)), None)
        if inicio is None:
            raise KeyError('registrador %s nao encontrado' % reg)
        fim = inicio + 1
        while fim < len(linhas) and 'first' not in linhas[fim].group(0) and 'class="sec"' not in linhas[fim].group(0):
            fim += 1
        mudou = 0
        for k in range(fim - 1, inicio - 1, -1):		# de tras para frente: as posicoes nao se deslocam
            m = linhas[k]
            tds = list(re.finditer(r'<td[^>]*>.*?</td>', m.group(0), re.S))
            if len(tds) != 8:
                continue
            txt = re.sub(r'<[^>]+>', '', tds[1 + 4 * lado].group(0)).strip()
            if k == inicio:
                bits = set(range(8))				# cabecalho: o registrador inteiro
            elif re.fullmatch(r'\d', txt):
                bits = {int(txt)}
            elif re.fullmatch(r'\d:\d', txt):
                a, b = int(txt[0]), int(txt[2])
                bits = set(range(min(a, b), max(a, b) + 1))
            else:
                continue					# linhas de nota ("—")
            testados = {b: e for b, e in resultado.items() if b in bits}
            if not testados:
                continue
            st = tds[3 + 4 * lado]
            atual = re.search(r'<span class="t-[a-z]"[^>]*>T.</span>', st.group(0))
            if not atual:
                continue
            if any(testados.values()):
                novo = _T['y']
            elif 'class="t-o"' not in atual.group(0):
                continue					# so T○ vira T✖; T✔ e T◐ ficam (podem depender de combinacao)
            else:
                novo = _T['n']
            if atual.group(0) == novo:
                continue
            novo_st = st.group(0).replace(atual.group(0), novo, 1)
            ini_st = m.start() + st.start()
            self.h = self.h[:ini_st] + novo_st + self.h[ini_st + len(st.group(0)):]
            mudou += 1
        self.mudancas.append('teste %s %s: %d linhas' % (('ponte', 'sensor')[lado], reg, mudou))
        return mudou

    # ---------------------------------------------------------------- BITS
    def bloco(self, titulo, itens):
        """Acrescenta um quadro verde (div.cc) com título e lista, logo depois do último quadro existente
        (no BITS, antes da tabela). Os itens são HTML."""
        fins = [m.end() for m in re.finditer(r'<div class="cc">.*?</div>', self.h, re.S)]
        if not fins:
            raise ValueError('nenhum quadro <div class="cc"> no documento')
        html = '<div class="cc"><b>%s</b><ul>\n%s</ul></div>' % (titulo, '\n'.join('<li>%s</li>' % i for i in itens))
        self.h = self.h[:fins[-1]] + html + self.h[fins[-1]:]
        self.mudancas.append('bloco: %s' % titulo)

    # ---------------------------------------------------------------- geral
    def troca(self, antigo, novo):
        """Troca um trecho que deve existir exatamente uma vez (evita trocar no lugar errado)."""
        n = self.h.count(antigo)
        if n != 1:
            raise ValueError('o trecho aparece %d vezes (precisa ser 1): %r' % (n, antigo[:80]))
        self.h = self.h.replace(antigo, novo, 1)
        self.mudancas.append('troca: %s' % antigo[:50])

    def salva(self, assunto):
        """Confere a tabela; se estiver íntegra, faz a cópia .bak_<assunto> e grava. Devolve o resumo."""
        problemas = checa_texto(self.h, checa_texto(self.original)['fora_do_padrao'])['problemas']
        if problemas:
            raise RuntimeError('não gravado, tabela com problema: %s' % problemas)
        bak = '%s.bak_%s' % (self.caminho, assunto)
        if not os.path.exists(bak):                       # não sobrescreve uma cópia anterior do mesmo assunto
            shutil.copy2(self.caminho, bak)
        with open(self.caminho, 'w', encoding='utf-8') as f:
            f.write(self.h)
        tam = os.path.getsize(self.caminho)
        return 'gravado %s (%d bytes, cópia %s): %s' % (self.caminho, tam, os.path.basename(bak),
                                                         '; '.join(self.mudancas))


def checa_texto(h, aceitas=0):
    """Confere a estrutura: <tr> abertas = fechadas e cada linha com o número de células da tabela
    (8 no MAPA, 5 no BITS; linhas de seção têm 1). 'aceitas' = linhas fora do padrão que já existiam."""
    linhas = _LINHA.findall(h)
    ncel = [len(re.findall(r'<td[\s>]', r)) for r in linhas]
    padrao = max(set(ncel), key=ncel.count) if ncel else 0
    fora = sum(1 for r, n in zip(linhas, ncel) if n not in (padrao, 1) and 'class="sec"' not in r)
    problemas = []
    if h.count('<tr') != h.count('</tr>'):
        problemas.append('<tr> abertas %d, fechadas %d' % (h.count('<tr'), h.count('</tr>')))
    if fora > aceitas:
        problemas.append('%d linhas fora do padrão de %d células (antes: %d)' % (fora, padrao, aceitas))
    for tag in ('table', 'tbody', 'ul', 'div'):
        a, f = len(re.findall(r'<%s[\s>]' % tag, h)), h.count('</%s>' % tag)
        if a != f:
            problemas.append('<%s> abertas %d, fechadas %d' % (tag, a, f))
    return {'linhas': len(linhas), 'celulas': padrao, 'fora_do_padrao': fora, 'problemas': problemas}


def main():
    """Comando: 'checa ARQUIVO' mostra o resultado da conferência."""
    if len(sys.argv) == 3 and sys.argv[1] == 'checa':
        r = checa_texto(open(sys.argv[2], encoding='utf-8').read(), aceitas=10 ** 6)
        print('%s: %d linhas de %d células; %d fora do padrão; %s' % (
            sys.argv[2], r['linhas'], r['celulas'], r['fora_do_padrao'],
            'problemas: %s' % r['problemas'] if r['problemas'] else 'estrutura OK'))
    else:
        print(__doc__)


if __name__ == '__main__':
    main()
