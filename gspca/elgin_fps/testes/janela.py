#!/usr/bin/env python3
"""Controle da janela de padroes (tela_padrao.py) pelos scripts de teste.

Autor: Josemar Simao <josemars@ifes.edu.br>
Co-autor e Revisor: Claude (Anthropic)

Uso:
    from janela import Janela
    j = Janela('alinhar')        # abre a janela (o usuario posiciona e ajusta o tamanho)
    j.mostra('vermelho')         # troca o padrao e espera a confirmacao de desenho
    j.fecha()
"""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))


class Janela:
    """Abre o tela_padrao.py como processo filho e troca o padrao por comando."""

    def __init__(self, inicial='alinhar'):
        """Abre a janela no padrao inicial."""
        self.p = subprocess.Popen([sys.executable, '-I', os.path.join(HERE, 'tela_padrao.py'), inicial],
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        time.sleep(1.0)

    def mostra(self, nome, espera=0.4):
        """Troca para o padrao 'nome', espera o "ok" da janela (desenho feito) e mais 'espera' segundos
        para a tela e a exposicao da camera se acomodarem. Erro se a janela foi fechada ou recusou."""
        if self.p.poll() is not None:
            raise RuntimeError('a janela de padroes foi fechada')
        self.p.stdin.write(nome + '\n')
        self.p.stdin.flush()
        r = self.p.stdout.readline().strip()
        if r != 'ok ' + nome:
            raise RuntimeError('janela respondeu %r ao pedir %r' % (r, nome))
        time.sleep(espera)

    def fecha(self):
        """Fecha a janela (se ainda estiver aberta)."""
        if self.p.poll() is None:
            try:
                self.p.stdin.write('sair\n')
                self.p.stdin.flush()
                self.p.wait(timeout=5)
            except Exception:
                self.p.kill()
