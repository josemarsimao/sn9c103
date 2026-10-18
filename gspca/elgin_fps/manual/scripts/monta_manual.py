#!/usr/bin/env python3
"""Monta o manual (.docx) a partir do modelo da apostila e dos capitulos em Markdown.

Autor: Josemar Simao <josemars@ifes.edu.br>
Co-autor e Revisor: Claude (Anthropic)

Aproveita do modelo (manual/modelo/*.docx) os estilos, o cabecalho, o rodape, a numeracao e o tema; troca
so o conteudo do corpo: capa, sumario e os capitulos de manual/capitulos/*.md (em ordem de nome).

Markdown aceito (simples, linha a linha):
  # Capitulo N — titulo        -> Titulo 2 (capitulo, com quebra de pagina)
  ## N.M titulo                -> Titulo 3 (secao)        ### titulo -> Titulo 4
  paragrafos com **negrito**, *italico* e `codigo`
  - item                       -> lista com marcador
  > **Nota:** texto            -> quadro do modelo (Nota, Atencao, Dados, Nao esquecer, Exemplo, Simulacao,
                                  Projeto, Exercicio)
  Tabela: titulo               -> titulo da tabela seguinte ("Tabela N.K – titulo")
  | a | b |  (tabela Markdown)  -> tabela no formato do modelo
  ![legenda](caminho.png)      -> figura centralizada com "Figura N.K – legenda"
  --- (linha)                  -> ignorada

Saida: manual/Manual_camera_Elgin.docx e, com --pdf, manual/Manual_camera_Elgin.pdf (sumario atualizado
pelo LibreOffice). Uso:  python3 monta_manual.py [--pdf]
"""
import glob
import html
import os
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import time
import zipfile

AQUI = os.path.dirname(os.path.abspath(__file__))
MANUAL = os.path.dirname(AQUI)
SAIDA = os.path.join(MANUAL, 'Manual_camera_Elgin.docx')
LARGURA = 11906 - 1134 - 1134			# largura util da pagina do modelo (dxa)

# cores dos quadros, como no modelo: (borda, fundo)
QUADROS = {'Nota': ('E0A100', 'FFF4D6'), 'Atenção': ('C0392B', 'FDE9E7'), 'Dados': ('2F6DB5', 'E8F1FB'),
           'Não esquecer': ('B8860B', 'FFF9C4'), 'Exemplo': ('5F6B7A', 'EEF1F4'), 'Exercício': ('5F6B7A', 'EEF1F4'),
           'Simulação': ('2E8B57', 'E6F4EA'), 'Projeto': ('6B4FA0', 'EEE9F7'), 'Atividade': ('6B4FA0', 'EEE9F7')}
CAPA = {
    'instituicao': 'INSTITUTO FEDERAL DO ESPÍRITO SANTO – CAMPUS VITÓRIA',
    'curso': 'Projeto do driver Linux gspca_elgin_fps',
    'titulo': 'Manual da Câmera Elgin',
    'subtitulo': 'Ponte SN9C103 · Sensor OV7630 · Registradores, controles e medidas',
    'autor': 'Prof. Josemar Simão',
    'coautor': 'Co-autor e revisor: Claude (Anthropic)',
}
CABECALHO = ('Câmera Elgin – SN9C103 + OV7630', 'Manual de registradores e controles')
MESES = ['janeiro', 'fevereiro', 'março', 'abril', 'maio', 'junho', 'julho', 'agosto', 'setembro', 'outubro',
         'novembro', 'dezembro']


def esc(t):
    """Texto seguro para XML."""
    return html.escape(t, quote=False)


ORDEM_RPR = ('rFonts', 'b', 'bCs', 'i', 'iCs', 'color', 'sz', 'szCs', 'vertAlign')	# ordem exigida pelo Word


def junta_rpr(*partes):
    """Junta propriedades de run (XML) sem repetir elementos: a ultima definicao de cada um vale, e a
    saida segue a ordem do esquema do Word."""
    props = {}
    for parte in partes:
        for m in re.finditer(r'<w:(\w+)(?: [^>]*)?/>', parte or ''):
            props[m.group(1)] = m.group(0)
    return ''.join(props[k] for k in ORDEM_RPR if k in props)


def runs(texto, base=''):
    """Converte **negrito**, *italico* e `codigo` em runs do Word. base = rPr extra para todos os runs
    (o tamanho da base vale tambem para o codigo, como nas tabelas)."""
    out = []
    tam_base = 'sz' in re.findall(r'<w:(\w+)', base or '')
    for parte in re.split(r'(\*\*.+?\*\*|`[^`]+`|(?<!\*)\*[^*\s][^*]*\*(?!\*))', texto):
        if not parte:
            continue
        extra = ''
        if parte.startswith('**') and parte.endswith('**') and len(parte) > 4:
            parte, extra = parte[2:-2], '<w:b/><w:bCs/>'
        elif parte.startswith('`') and parte.endswith('`'):
            # codigo (registradores, comandos): fonte mono, azul escuro, um pouco menor que o texto
            parte, extra = parte[1:-1], ('<w:rFonts w:ascii="Liberation Mono" w:hAnsi="Liberation Mono" w:cs="Liberation Mono"/>'
                                         '<w:color w:val="1F3864"/>' + ('' if tam_base else '<w:sz w:val="20"/><w:szCs w:val="20"/>'))
        elif parte.startswith('*') and parte.endswith('*') and len(parte) > 2:
            parte, extra = parte[1:-1], '<w:i/><w:iCs/>'
        rpr = junta_rpr(base, extra)		# o codigo so traz tamanho proprio quando a base nao tem
        out.append('<w:r>%s<w:t xml:space="preserve">%s</w:t></w:r>' % (
            '<w:rPr>%s</w:rPr>' % rpr if rpr else '', esc(parte)))
    return ''.join(out)


def par(texto, ppr='', base=''):
    """Paragrafo com propriedades ppr (XML de <w:pPr> sem a tag) e texto com formatacao inline."""
    return '<w:p>%s%s</w:p>' % ('<w:pPr>%s</w:pPr>' % ppr if ppr else '', runs(texto, base))


def estilo(nome):
    """pStyle do modelo."""
    return '<w:pStyle w:val="%s"/>' % nome


class Montador:
    """Converte os capitulos em XML do corpo e guarda as figuras a incluir no pacote."""

    def __init__(self):
        self.xml = []
        self.figuras = []				# (rId, caminho)
        self.cap = 0
        self.nfig = 0
        self.ntab = 0
        self.titulo_tab = None
        self.id_draw = 100

    def capa(self):
        """Capa no formato da primeira pagina do modelo, mais o sumario (campo TOC)."""
        hoje = time.localtime()
        versao = 'Versão de trabalho de %d de %s de %d.' % (hoje.tm_mday, MESES[hoje.tm_mon - 1], hoje.tm_year)
        x = self.xml
        x.append(par(CAPA['instituicao'], '<w:spacing w:after="40"/>', '<w:b/><w:bCs/><w:color w:val="555555"/><w:sz w:val="20"/><w:szCs w:val="20"/>'))
        x.append(par(CAPA['curso'], '<w:spacing w:after="40"/>', '<w:color w:val="555555"/>'))
        x.append(par(CAPA['titulo'], '<w:spacing w:before="120" w:after="60"/>', '<w:b/><w:bCs/><w:color w:val="1F3864"/><w:sz w:val="40"/><w:szCs w:val="40"/>'))
        x.append(par(CAPA['subtitulo'], '<w:pBdr><w:bottom w:val="single" w:sz="12" w:space="6" w:color="1F3864"/></w:pBdr><w:spacing w:after="200"/>',
                     '<w:color w:val="444444"/>'))
        x.append(par(CAPA['autor'], '<w:spacing w:after="20"/>'))
        x.append(par(CAPA['coautor'], '<w:spacing w:after="60"/>', '<w:color w:val="444444"/><w:sz w:val="20"/><w:szCs w:val="20"/>'))
        x.append(par(versao, '<w:spacing w:after="240"/>', '<w:i/><w:iCs/><w:color w:val="666666"/><w:sz w:val="20"/><w:szCs w:val="20"/>'))
        x.append(par('Sumário', estilo('Ttuloforadosumrio')))
        x.append('<w:p><w:pPr>%s<w:tabs><w:tab w:val="right" w:leader="dot" w:pos="9628"/></w:tabs></w:pPr>'
                 '<w:r><w:fldChar w:fldCharType="begin" w:dirty="true"/></w:r>'
                 '<w:r><w:instrText xml:space="preserve">TOC \\o "1-3" \\h \\z \\u</w:instrText></w:r>'
                 '<w:r><w:fldChar w:fldCharType="separate"/></w:r>'
                 '<w:r><w:rPr><w:i/><w:color w:val="666666"/></w:rPr><w:t>(Sumário: atualize os campos — no Word, F9.)</w:t></w:r>'
                 '<w:r><w:fldChar w:fldCharType="end"/></w:r></w:p>' % estilo('Sumrio1'))

    def figura(self, legenda, caminho):
        """Figura centralizada, largura ate a da pagina (16 cm), e a legenda numerada."""
        self.nfig += 1
        rid = 'rIdFig%d' % self.nfig
        self.figuras.append((rid, caminho))
        with open(caminho, 'rb') as f:
            cab = f.read(24)
        w, h = struct.unpack('>II', cab[16:24])			# PNG: largura e altura
        larg = min(16.0, w / 2 / 72 * 2.54)				# figuras em 2x: 1 ponto = 2 pixels
        cx = int(larg * 360000)
        cy = int(cx * h / w)
        self.id_draw += 1
        d = self.id_draw
        self.xml.append(
            '<w:p><w:pPr><w:keepNext/><w:spacing w:before="120" w:after="40"/><w:jc w:val="center"/></w:pPr>'
            '<w:r><w:rPr><w:noProof/></w:rPr><w:drawing><wp:inline distT="0" distB="0" distL="0" distR="0">'
            '<wp:extent cx="%d" cy="%d"/><wp:effectExtent l="0" t="0" r="0" b="0"/><wp:docPr id="%d" name="Figura %d"/>'
            '<wp:cNvGraphicFramePr><a:graphicFrameLocks xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" noChangeAspect="1"/></wp:cNvGraphicFramePr>'
            '<a:graphic xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main"><a:graphicData uri="http://schemas.openxmlformats.org/drawingml/2006/picture">'
            '<pic:pic xmlns:pic="http://schemas.openxmlformats.org/drawingml/2006/picture"><pic:nvPicPr><pic:cNvPr id="%d" name="fig%d.png"/><pic:cNvPicPr/></pic:nvPicPr>'
            '<pic:blipFill><a:blip r:embed="%s"/><a:stretch><a:fillRect/></a:stretch></pic:blipFill>'
            '<pic:spPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="%d" cy="%d"/></a:xfrm><a:prstGeom prst="rect"><a:avLst/></a:prstGeom></pic:spPr>'
            '</pic:pic></a:graphicData></a:graphic></wp:inline></w:drawing></w:r></w:p>' % (cx, cy, d, d, d, d, rid, cx, cy))
        self.xml.append(par('Figura %d.%d – %s' % (self.cap, self.nfig, legenda), estilo('Legendadefigura')))

    def tabela(self, linhas):
        """Tabela Markdown no formato do modelo: titulo em negrito antes, cabecalho azul claro."""
        cel = [[c.strip() for c in l.strip().strip('|').split('|')] for l in linhas]
        cel = [r for r in cel if not all(re.fullmatch(r':?-{3,}:?', c) for c in r)]
        n = max(len(r) for r in cel)
        cel = [r + [''] * (n - len(r)) for r in cel]
        comp = [max(min(len(re.sub(r'[*`]', '', r[i])), 60) for r in cel) + 4 for i in range(n)]
        palavra = [max(len(p) for r in cel for p in (re.sub(r'[*`]', '', r[i]).split() or [''])) for i in range(n)]
        larg = [max(900, 115 * palavra[i] + 200, int(LARGURA * c / sum(comp))) for i, c in enumerate(comp)]
        excesso = sum(larg) - LARGURA
        if excesso > 0:				# tira o excesso da coluna mais larga
            k = larg.index(max(larg))
            larg[k] -= excesso
        larg[-1] += LARGURA - sum(larg)
        if self.titulo_tab:
            self.ntab += 1
            self.xml.append(par('Tabela %d.%d – %s' % (self.cap, self.ntab, self.titulo_tab),
                                '<w:keepNext/><w:spacing w:before="140" w:after="60"/>',
                                '<w:b/><w:bCs/><w:sz w:val="20"/><w:szCs w:val="20"/>'))
            self.titulo_tab = None
        borda = ''.join('<w:%s w:val="single" w:sz="4" w:space="0" w:color="8C8C8C"/>' % b for b in ('top', 'left', 'bottom', 'right'))
        x = ['<w:tbl><w:tblPr><w:tblW w:w="%d" w:type="dxa"/><w:jc w:val="center"/><w:tblBorders>%s'
             '<w:insideH w:val="single" w:sz="4" w:space="0" w:color="8C8C8C"/><w:insideV w:val="single" w:sz="4" w:space="0" w:color="8C8C8C"/>'
             '</w:tblBorders><w:tblLayout w:type="fixed"/><w:tblCellMar><w:left w:w="10" w:type="dxa"/><w:right w:w="10" w:type="dxa"/></w:tblCellMar>'
             '<w:tblLook w:val="0000" w:firstRow="0" w:lastRow="0" w:firstColumn="0" w:lastColumn="0" w:noHBand="0" w:noVBand="0"/></w:tblPr>'
             '<w:tblGrid>%s</w:tblGrid>' % (LARGURA, borda, ''.join('<w:gridCol w:w="%d"/>' % l for l in larg))]
        for i, r in enumerate(cel):
            cab = i == 0
            x.append('<w:tr><w:trPr><w:cantSplit/>%s<w:jc w:val="center"/></w:trPr>' % ('<w:tblHeader/>' if cab else ''))
            for j, c in enumerate(r):
                x.append('<w:tc><w:tcPr><w:tcW w:w="%d" w:type="dxa"/><w:tcBorders>%s</w:tcBorders>%s'
                         '<w:tcMar><w:top w:w="40" w:type="dxa"/><w:left w:w="80" w:type="dxa"/><w:bottom w:w="40" w:type="dxa"/><w:right w:w="80" w:type="dxa"/></w:tcMar>'
                         '<w:vAlign w:val="center"/></w:tcPr>' % (larg[j], borda,
                                                                 '<w:shd w:val="clear" w:color="auto" w:fill="D9E2F3"/>' if cab else ''))
                x.append(par(c, '<w:spacing w:before="0" w:after="0" w:line="240" w:lineRule="auto"/><w:jc w:val="%s"/>' % ('center' if cab else 'left'),
                             '<w:sz w:val="18"/><w:szCs w:val="18"/>' + ('<w:b/><w:bCs/>' if cab else '')))
                x.append('</w:tc>')
            x.append('</w:tr>')
        x.append('</w:tbl>')
        self.xml.append(''.join(x))
        self.xml.append(par('', '<w:spacing w:before="0" w:after="60"/>'))

    def quadro(self, tipo, texto):
        """Quadro colorido do modelo (borda esquerda + fundo)."""
        borda, fundo = QUADROS.get(tipo, QUADROS['Nota'])
        ppr = (estilo('Quadro') + '<w:keepLines/><w:pBdr><w:left w:val="single" w:sz="24" w:space="6" w:color="%s"/></w:pBdr>'
               '<w:shd w:val="clear" w:color="auto" w:fill="%s"/><w:spacing w:before="100" w:after="140" w:line="276" w:lineRule="auto"/>'
               '<w:ind w:left="140" w:right="140"/>' % (borda, fundo))
        self.xml.append(par(texto, ppr))

    def capitulo(self, caminho):
        """Converte um arquivo Markdown de capitulo."""
        linhas = open(caminho, encoding='utf-8').read().split('\n')
        i, paragrafo = 0, []

        def fecha():
            if paragrafo:
                self.xml.append(par(' '.join(paragrafo)))
                paragrafo.clear()
        while i < len(linhas):
            l = linhas[i].rstrip()
            if not l.strip() or l.strip() == '---':
                fecha()
            elif l.startswith('# '):
                fecha()
                m = re.match(r'#\s+Cap[ií]tulo\s+(\d+)\s*[—–-]\s*(.*)', l)
                self.cap = int(m.group(1)) if m else self.cap + 1
                self.nfig = self.ntab = 0
                titulo = 'Capítulo %d – %s' % (self.cap, m.group(2)) if m else l[2:]
                self.xml.append(par(titulo, estilo('Ttulo2') + '<w:pageBreakBefore/>'))
            elif l.startswith('## '):
                fecha()
                self.xml.append(par(l[3:].strip(), estilo('Ttulo3')))
            elif l.startswith('### '):
                fecha()
                self.xml.append(par(l[4:].strip(), estilo('Ttulo4')))
            elif l.startswith('Tabela:'):
                fecha()
                self.titulo_tab = l[7:].strip()
            elif l.lstrip().startswith('|'):
                fecha()
                bloco = []
                while i < len(linhas) and linhas[i].lstrip().startswith('|'):
                    bloco.append(linhas[i])
                    i += 1
                self.tabela(bloco)
                continue
            elif re.match(r'!\[.*\]\(.*\)\s*$', l):
                fecha()
                m = re.match(r'!\[(.*)\]\((.*)\)', l)
                self.figura(m.group(1), os.path.normpath(os.path.join(os.path.dirname(caminho), m.group(2))))
            elif l.startswith('> '):
                fecha()
                texto = l[2:].strip()
                while i + 1 < len(linhas) and linhas[i + 1].startswith('> '):
                    i += 1
                    texto += ' ' + linhas[i][2:].strip()
                m = re.match(r'\*\*([^*:]+):\*\*\s*(.*)', texto)
                tipo = m.group(1) if m and m.group(1) in QUADROS else 'Nota'
                self.quadro(tipo, texto)
            elif re.match(r'\s*[-*] ', l):
                fecha()
                self.xml.append(par(re.sub(r'^\s*[-*] ', '', l),
                                    estilo('PargrafodaLista') + '<w:numPr><w:ilvl w:val="0"/><w:numId w:val="2"/></w:numPr>'
                                    '<w:spacing w:after="70" w:line="276" w:lineRule="auto"/>'))
            else:
                paragrafo.append(l.strip())
            i += 1
        fecha()


def monta(pdf):
    """Copia o pacote do modelo, troca o corpo, as figuras, o cabecalho e grava o .docx (e o PDF)."""
    modelo = sorted(glob.glob(os.path.join(MANUAL, 'modelo', '*.docx')))[0]
    tmp = tempfile.mkdtemp(prefix='manual_')
    with zipfile.ZipFile(modelo) as z:
        for n in z.namelist():
            if '..' in n or n.startswith('/'):
                continue					# pacote estranho: ignora
            z.extract(n, tmp)
    w = os.path.join(tmp, 'word')
    doc = open(os.path.join(w, 'document.xml'), encoding='utf-8').read()
    raiz = doc[:doc.index('<w:body>')]
    sect = re.findall(r'<w:sectPr.*?</w:sectPr>', doc, re.S)[-1]

    m = Montador()
    m.capa()
    for cap in sorted(glob.glob(os.path.join(MANUAL, 'capitulos', '*.md'))):
        m.capitulo(cap)
    open(os.path.join(w, 'document.xml'), 'w', encoding='utf-8').write(
        raiz + '<w:body>' + ''.join(m.xml) + sect + '</w:body></w:document>')

    # figuras: tira as do modelo e poe as do manual
    for f in glob.glob(os.path.join(w, 'media', '*')):
        os.remove(f)
    rels_p = os.path.join(w, '_rels', 'document.xml.rels')
    rels = open(rels_p, encoding='utf-8').read()
    rels = re.sub(r'<Relationship [^>]*relationships/image"[^>]*/>', '', rels)
    novas = ''
    for k, (rid, cam) in enumerate(m.figuras, 1):
        shutil.copy(cam, os.path.join(w, 'media', 'fig%d.png' % k))
        novas += '<Relationship Id="%s" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/image" Target="media/fig%d.png"/>' % (rid, k)
    open(rels_p, 'w', encoding='utf-8').write(rels.replace('</Relationships>', novas + '</Relationships>'))

    # cabecalho: troca os dois textos do modelo
    hp = os.path.join(w, 'header1.xml')
    hx = open(hp, encoding='utf-8').read()
    textos = re.findall(r'<w:t(?: [^>]*)?>([^<]+)</w:t>', hx)
    if len(textos) >= 2:
        hx = hx.replace('>' + textos[0] + '<', '>' + esc(CABECALHO[0]) + '<', 1)
        hx = hx.replace('>' + textos[-1] + '<', '>' + esc(CABECALHO[1]) + '<', 1)
    open(hp, 'w', encoding='utf-8').write(hx)

    # o Word atualiza o sumario ao abrir
    sp = os.path.join(w, 'settings.xml')
    sx = open(sp, encoding='utf-8').read()
    if '<w:updateFields' not in sx:
        sx = re.sub(r'(<w:settings[^>]*>)', r'\1<w:updateFields w:val="true"/>', sx, count=1)
    open(sp, 'w', encoding='utf-8').write(sx)

    # propriedades do documento
    cp = os.path.join(tmp, 'docProps', 'core.xml')
    if os.path.exists(cp):
        cx = open(cp, encoding='utf-8').read()
        cx = re.sub(r'<dc:title>.*?</dc:title>', '<dc:title>%s</dc:title>' % esc(CAPA['titulo']), cx)
        open(cp, 'w', encoding='utf-8').write(cx)

    if os.path.exists(SAIDA):
        os.remove(SAIDA)
    with zipfile.ZipFile(SAIDA, 'w', zipfile.ZIP_DEFLATED) as z:
        tipos = os.path.join(tmp, '[Content_Types].xml')
        z.write(tipos, '[Content_Types].xml')		# primeiro no pacote
        for raiz_d, _, arqs in os.walk(tmp):
            for a in arqs:
                p = os.path.join(raiz_d, a)
                if p != tipos:
                    z.write(p, os.path.relpath(p, tmp))
    shutil.rmtree(tmp)
    print('gravado %s (%d capitulos, %d figuras)' % (SAIDA, m.cap, len(m.figuras)))
    if pdf:
        gera_pdf()


def gera_pdf():
    """Abre o .docx no LibreOffice (sem tela), atualiza o sumario e exporta o PDF."""
    script = r'''
import uno, sys, time, subprocess
from com.sun.star.beans import PropertyValue
def prop(n, v):
    p = PropertyValue(); p.Name = n; p.Value = v; return p
proc = subprocess.Popen(["soffice", "--headless", "--norestore", "--accept=socket,host=localhost,port=2004;urp;"])
ctx = None
for _ in range(60):
    try:
        local = uno.getComponentContext()
        res = local.ServiceManager.createInstanceWithContext("com.sun.star.bridge.UnoUrlResolver", local)
        ctx = res.resolve("uno:socket,host=localhost,port=2004;urp;StarOffice.ComponentContext")
        break
    except Exception:
        time.sleep(0.5)
desk = ctx.ServiceManager.createInstanceWithContext("com.sun.star.frame.Desktop", ctx)
doc = desk.loadComponentFromURL(uno.systemPathToFileUrl(sys.argv[1]), "_blank", 0, (prop("Hidden", True),))
idx = doc.getDocumentIndexes()
for i in range(idx.getCount()):
    idx.getByIndex(i).update()
doc.storeToURL(uno.systemPathToFileUrl(sys.argv[2]), (prop("FilterName", "writer_pdf_Export"),))
doc.close(True)
try:
    desk.terminate()
except Exception:
    pass
proc.wait(timeout=30)
'''
    pdf = SAIDA[:-5] + '.pdf'
    subprocess.run([sys.executable, '-c', script, SAIDA, pdf], check=True, timeout=180)
    print('gravado', pdf)


if __name__ == '__main__':
    monta('--pdf' in sys.argv)
