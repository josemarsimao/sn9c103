#!/usr/bin/env python3
"""Janela de imagens padrao para os testes com a camera encostada na tela do notebook.

Autor: Josemar Simao <josemars@ifes.edu.br>
Co-autor e Revisor: Claude (Anthropic)

Janela comum (nao tela cheia), redimensionavel e sempre por cima: posicione-a sob a camera.
O padrao e' desenhado no centro com margem PRETA em volta (+ e - mudam o tamanho do padrao), para
a janela poder ser maior que o campo da camera e o padrao ficar todo enquadrado. O padrao
"alinhar" (contorno, grade em tercos e cruz) ajuda a centralizar pelo Multiview.

Teclado (com a janela em foco):
    espaco / seta direita   proximo padrao        seta esquerda   padrao anterior
    1 a 9, 0                padrao pelo numero    a               padrao "alinhar"
    + / -                   padrao maior/menor    q / Esc         fecha
Comandos (para os scripts de teste): uma linha por comando na entrada padrao, com o nome do padrao
(ex.: "vermelho"); "sair" fecha. A cada troca a janela escreve "ok <nome>" na saida padrao, depois
de desenhar, para o script saber que pode capturar.

Posicao, tamanho e escala sao gravados em tela_padrao.json (nesta pasta) a cada mudanca e
restaurados ao abrir: com a camera fixa no suporte, a janela volta sempre ao mesmo lugar.
Para reposicionar do zero, apague o tela_padrao.json.

Uso:  python3 tela_padrao.py              (comeca no branco)
      python3 tela_padrao.py rampa        (comeca no padrao pedido)
"""
import json
import os
import sys

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import GLib, Gtk  # noqa: E402

CONF = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tela_padrao.json')

# nome -> funcao de desenho (cairo, largura, altura)


def _cheio(r, g, b):
    """Padrao de cor solida (componentes de 0 a 1)."""
    def f(cr, w, h):
        cr.set_source_rgb(r, g, b)
        cr.paint()
    return f


def _meio(cr, w, h):
    """Metade esquerda branca, metade direita preta (divisa vertical): mede o comprimento da linha."""
    cr.set_source_rgb(0, 0, 0)
    cr.paint()
    cr.set_source_rgb(1, 1, 1)
    cr.rectangle(0, 0, w / 2.0, h)
    cr.fill()


def _rampa(cr, w, h):
    """Degrade horizontal do preto ao branco, em 16 degraus: curva de gama, offset e saturacao."""
    for i in range(16):
        v = i / 15.0
        cr.set_source_rgb(v, v, v)
        cr.rectangle(i * w / 16.0, 0, w / 16.0 + 1, h)
        cr.fill()


def _barras(cr, w, h):
    """Oito barras verticais de cor (branco, amarelo, ciano, verde, magenta, vermelho, azul, preto)."""
    cores = [(1, 1, 1), (1, 1, 0), (0, 1, 1), (0, 1, 0), (1, 0, 1), (1, 0, 0), (0, 0, 1), (0, 0, 0)]
    for i, c in enumerate(cores):
        cr.set_source_rgb(*c)
        cr.rectangle(i * w / 8.0, 0, w / 8.0 + 1, h)
        cr.fill()


def _composto(cr, w, h):
    """Quadro composto para medir varias coisas numa captura so: em cima, quatro faixas (branco,
    vermelho, verde, azul); embaixo, a rampa de 16 degraus."""
    for i, c in enumerate([(1, 1, 1), (1, 0, 0), (0, 1, 0), (0, 0, 1)]):
        cr.set_source_rgb(*c)
        cr.rectangle(i * w / 4.0, 0, w / 4.0 + 1, h / 2.0)
        cr.fill()
    for i in range(16):
        v = i / 15.0
        cr.set_source_rgb(v, v, v)
        cr.rectangle(i * w / 16.0, h / 2.0, w / 16.0 + 1, h / 2.0)
        cr.fill()


def _alinhar(cr, w, h):
    """Fundo cinza-escuro com contorno branco na borda do padrao, grade em tercos e cruz central:
    mostra no Multiview onde o padrao comeca e termina, para enquadra-lo inteiro na camera."""
    cr.set_source_rgb(0.15, 0.15, 0.15)
    cr.paint()
    t = max(3.0, min(w, h) / 50.0)		# espessura das linhas
    cr.set_source_rgb(1, 1, 1)
    cr.set_line_width(t)
    cr.rectangle(t / 2, t / 2, w - t, h - t)	# contorno = limite do padrao
    cr.stroke()
    cr.set_line_width(max(1.0, t / 3))
    for i in (1, 2):				# grade em tercos
        cr.move_to(w * i / 3.0, 0)
        cr.line_to(w * i / 3.0, h)
        cr.move_to(0, h * i / 3.0)
        cr.line_to(w, h * i / 3.0)
    cr.stroke()
    c = min(w, h) / 8.0
    cr.set_line_width(t)
    cr.move_to(w / 2.0 - c, h / 2.0)		# cruz central
    cr.line_to(w / 2.0 + c, h / 2.0)
    cr.move_to(w / 2.0, h / 2.0 - c)
    cr.line_to(w / 2.0, h / 2.0 + c)
    cr.stroke()


PADROES = [('branco', _cheio(1, 1, 1)), ('vermelho', _cheio(1, 0, 0)), ('verde', _cheio(0, 1, 0)),
           ('azul', _cheio(0, 0, 1)), ('preto', _cheio(0, 0, 0)), ('cinza', _cheio(0.5, 0.5, 0.5)),
           ('meio', _meio), ('rampa', _rampa), ('barras', _barras), ('composto', _composto),
           ('alinhar', _alinhar)]
NOMES = [n for n, _ in PADROES]


class Tela(Gtk.Window):
    """A janela: desenha o padrao atual e troca por teclado ou por comando na entrada padrao."""

    def __init__(self, inicial):
        """Cria a janela (sempre por cima) na posicao, tamanho e escala gravados (ou 800x600 na
        primeira vez) e comeca a ouvir a entrada padrao."""
        super().__init__(title='tela padrao')
        self.conf = self.le_conf()
        self.set_default_size(self.conf.get('w', 800), self.conf.get('h', 600))
        if 'x' in self.conf:
            self.move(self.conf['x'], self.conf['y'])
        self.set_keep_above(True)
        self.idx = NOMES.index(inicial) if inicial in NOMES else 0
        self.area = Gtk.DrawingArea()
        self.area.connect('draw', self.desenha)
        self.add(self.area)
        self.connect('key-press-event', self.tecla)
        self.connect('destroy', Gtk.main_quit)
        self.avisar = None			# nome a confirmar na saida padrao depois do proximo desenho
        self.escala = self.conf.get('escala', 0.8)	# fracao da janela ocupada pelo padrao (o resto e' margem preta)
        self.grava_id = None			# gravacao adiada (enquanto a janela e' arrastada)
        self.connect('configure-event', self.mudou)
        GLib.io_add_watch(sys.stdin, GLib.IO_IN | GLib.IO_HUP, self.comando)
        self.titulo()

    def le_conf(self):
        """Le a posicao, o tamanho e a escala gravados; {} se ainda nao houver."""
        try:
            return json.load(open(CONF))
        except (OSError, ValueError):
            return {}

    def mudou(self, widget, ev):
        """A janela foi movida ou redimensionada: grava meio segundo depois de parar."""
        if self.grava_id:
            GLib.source_remove(self.grava_id)
        self.grava_id = GLib.timeout_add(500, self.grava)
        return False

    def grava(self):
        """Grava posicao, tamanho e escala em tela_padrao.json."""
        self.grava_id = None
        x, y = self.get_position()
        w, h = self.get_size()
        self.conf = dict(x=x, y=y, w=w, h=h, escala=round(self.escala, 2))
        try:
            json.dump(self.conf, open(CONF, 'w'))
        except OSError:
            pass
        self.titulo()
        return False

    def titulo(self):
        """Mostra o padrao atual e o tamanho da janela no titulo."""
        a = self.get_allocation()
        pos = '  pos %d,%d' % (self.conf['x'], self.conf['y']) if 'x' in self.conf else ''
        self.set_title('tela padrao: %d %s  padrao %d%% da janela (%dx%d)%s  espaco = proximo, +/- tamanho' % (
            self.idx + 1, NOMES[self.idx], round(self.escala * 100), a.width, a.height, pos))

    def desenha(self, widget, cr):
        """Desenha o padrao atual ocupando a area inteira; confirma a troca pedida por comando."""
        a = widget.get_allocation()
        cr.set_source_rgb(0, 0, 0)		# margem preta
        cr.paint()
        pw, ph = a.width * self.escala, a.height * self.escala
        cr.translate((a.width - pw) / 2.0, (a.height - ph) / 2.0)
        cr.rectangle(0, 0, pw, ph)
        cr.clip()
        PADROES[self.idx][1](cr, pw, ph)
        if self.avisar:
            GLib.idle_add(self.confirma, self.avisar)
            self.avisar = None
        return False

    def confirma(self, nome):
        """Escreve "ok <nome>" depois que o desenho foi para a tela."""
        print('ok', nome, flush=True)
        return False

    def troca(self, idx, avisar=False):
        """Muda para o padrao idx e redesenha."""
        self.idx = idx % len(PADROES)
        self.avisar = NOMES[self.idx] if avisar else None
        self.titulo()
        self.area.queue_draw()

    def tecla(self, widget, ev):
        """Espaco/setas trocam o padrao; 1-9 e 0 escolhem pelo numero; q/Esc fecham."""
        k = ev.string
        nome = Gtk.accelerator_name(ev.keyval, 0)
        if nome in ('space', 'Right'):
            self.troca(self.idx + 1)
        elif nome == 'Left':
            self.troca(self.idx - 1)
        elif k and k.isdigit():
            self.troca((int(k) - 1) % 10)
        elif k == 'a':
            self.troca(NOMES.index('alinhar'))
        elif k in ('+', '='):
            self.escala = min(1.0, self.escala + 0.05)
            self.troca(self.idx)
            self.grava()
        elif k == '-':
            self.escala = max(0.2, self.escala - 0.05)
            self.troca(self.idx)
            self.grava()
        elif nome in ('q', 'Escape'):
            Gtk.main_quit()
        return True

    def comando(self, fonte, cond):
        """Le um comando da entrada padrao: nome do padrao ou "sair"."""
        if cond & GLib.IO_HUP:
            return False
        linha = fonte.readline().strip().lower()
        if linha == 'sair':
            Gtk.main_quit()
            return False
        if linha in NOMES:
            self.troca(NOMES.index(linha), avisar=True)
        elif linha:
            print('erro padrao desconhecido: %s (padroes: %s)' % (linha, ' '.join(NOMES)), flush=True)
        return True


def main():
    """Abre a janela no padrao pedido (ou branco)."""
    Tela(sys.argv[1] if len(sys.argv) > 1 else 'branco').show_all()
    Gtk.main()


if __name__ == '__main__':
    main()
