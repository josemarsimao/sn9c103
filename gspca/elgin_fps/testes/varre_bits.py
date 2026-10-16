#!/usr/bin/env python3
"""Varredura bit a bit dos registradores com a camera olhando um padrao na tela (janela do notebook).

Autor: Josemar Simao <josemars@ifes.edu.br>
Co-autor e Revisor: Claude (Anthropic)

Para cada bit da lista, inverte so aquele bit em relacao ao valor de base (lido do chip) e captura o
padrao "composto" (faixas branco, vermelho, verde, azul em cima; rampa de 16 cinzas embaixo). Mede:
fps, bytes do quadro, comprimento da linha, mosaico de cada faixa (B, G, G, R), os 16 degraus da
rampa e se a imagem ainda e' a mesma. Compara com a referencia (sem mudanca), repetida no inicio e a
cada 25 casos, e marca os bits com efeito.

Fases (argumento):
  sensor      bits DOCUMENTADOS do sensor OV7630 (fase 1; ~75 registradores)
  ponte       registradores da ponte SN9C103 0x01-0x1F e 0x3F
  reservados  registradores reservados do sensor
  ou uma lista: s:0x14 s:0x28 b:0x18 (tipo:registrador; todos os bits)
Opcoes: --continua (pula os casos ja feitos no CSV)   --ensaio (so os 10 primeiros casos)

Modo base: S910 320x240 a 30 fps (modo normal, sensor VGA progressivo). Ganho automatico e taxa
dinamica desligados durante o teste (voltam ao valor anterior). O driver reinicia o sensor e regrava
a ponte a cada inicio de captura, entao cada caso parte do mesmo estado.
Bits que nao sao testados (cortam o I2C, desligam o sensor ou so resetam): ver PULA.

Camera no suporte olhando a janela; o script abre a janela no padrao "alinhar" e espera o Enter.
Pede sudo uma vez. Resultado: varre_<fase>.csv, varre_<fase>_resumo.txt e varre_<fase>.html
(imagens em testes/varre_<fase>/).
"""
import csv
import os
import re
import statistics
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from sn9c10x_decode import decode  # noqa: E402
from elgin_dev import acha_dev  # noqa: E402
from janela import Janela  # noqa: E402

PS = '/sys/module/gspca_elgin_fps/parameters/regs_start'
PR = '/sys/module/gspca_elgin_fps/parameters/regr'
W, H = 320, 240
FMT = ['--set-fmt-video=width=%d,height=%d,pixelformat=S910' % (W, H), '--set-parm=30', '--stream-mmap']
RAW = '/tmp/varre_bits.raw'
REF_CADA = 25					# repete a referencia a cada N casos

# registradores documentados do sensor (datasheet OV7630), sem os de leitura (0x0A, 0x0B, 0x1C, 0x1D)
SENSOR_DOC = ([0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x0c, 0x0d] + list(range(0x10, 0x1c)) + [0x1f]
              + list(range(0x20, 0x34)) + [0x4c, 0x4d, 0x60, 0x61, 0x65, 0x66, 0x67, 0x68, 0x69]
              + list(range(0x6f, 0x7b)) + [0x7d])
SENSOR_RES = [0x07, 0x08, 0x09, 0x0e, 0x0f, 0x1e] + list(range(0x34, 0x4c)) + list(range(0x4e, 0x60)) \
    + [0x62, 0x63, 0x64, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x7b, 0x7e, 0x7f]
PONTE = list(range(0x01, 0x20)) + [0x3f]
# (tipo, registrador, bit ou None = todos): nao testar
PULA = {('s', 0x12, 7): 'reset do sensor (desfaz tudo)',
        ('b', 0x08, None): 'controle do I2C (corta a comunicacao com o sensor)',
        ('b', 0x09, None): 'endereco I2C do sensor (corta a comunicacao)',
        ('b', 0x0a, None): 'porta de dados I2C', ('b', 0x0b, None): 'porta de dados I2C',
        ('b', 0x0c, None): 'porta de dados I2C', ('b', 0x0d, None): 'porta de dados I2C',
        ('b', 0x0e, None): 'porta de dados I2C', ('b', 0x0f, None): 'controle/estado I2C',
        ('b', 0x01, 0): 'desliga o sensor', ('b', 0x01, 2): 'desliga a transferencia de video',
        ('b', 0x01, 7): 'modo de teste do ASIC ("nao habilitar")',
        ('b', 0x17, 5): 'desliga o clock do sensor'}


class Tee:
    """Escreve na tela e num arquivo."""

    def __init__(self, *fs):
        self.fs = fs

    def write(self, s):
        for f in self.fs:
            f.write(s)

    def flush(self):
        for f in self.fs:
            f.flush()


def sudo_tee(path, txt):
    """Grava um parametro do modulo."""
    subprocess.run(['sudo', '-n', 'tee', path], input=(txt + '\n').encode(), capture_output=True)


def le_reg(tipo, reg):
    """Le um registrador ('s' sensor ou 'b' ponte) pelo parametro regr; a camera precisa estar capturando
    para o sensor responder. Devolve o valor ou None."""
    sudo_tee(PR, '%s:0x%02x' % (tipo, reg))
    try:
        m = re.search(r'= 0x([0-9a-fA-F]{2})', open(PR).read())
    except OSError:
        return None
    return int(m.group(1), 16) if m else None


def le_base(dev, alvos):
    """Le o valor de base de todos os registradores da lista durante uma captura de referencia."""
    sudo_tee(PS, '')
    p = subprocess.Popen(['v4l2-ctl', '-d', dev] + FMT + ['--stream-count=600', '--stream-to=/dev/null'],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)
    base = {}
    for tipo, reg in alvos:
        base[(tipo, reg)] = le_reg(tipo, reg)
    p.terminate()
    p.wait(timeout=10)
    return base


def captura(dev, rs):
    """Grava o regs_start, captura 25 quadros (descarta 10) e devolve (fps, bytes, matriz) ou (None, 0, None)."""
    sudo_tee(PS, rs)
    try:
        os.remove(RAW)
    except OSError:
        pass
    try:
        out = subprocess.run(['v4l2-ctl', '-d', dev] + FMT + ['--stream-skip=10', '--stream-count=15',
                                                              '--stream-to=' + RAW],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=45).stdout
    except subprocess.TimeoutExpired:
        return None, 0, None
    f = [float(x) for x in re.findall(r'<+ ([0-9.]+) fps', out)]
    fps = statistics.median(f) if f else None
    if not os.path.exists(RAW) or os.path.getsize(RAW) == 0:
        return fps, 0, None
    d = open(RAW, 'rb').read()
    try:
        b, bits = decode(d, W, H)				# o primeiro quadro do arquivo
        q = np.frombuffer(bytes(b), np.uint8).reshape(H, W).astype(np.float32)
    except Exception:
        return fps, 0, None
    return fps, (bits + 7) // 8, q


def regiao(q):
    """Retangulo do padrao na imagem do branco, alinhado ao mosaico (linha par, coluna par)."""
    alto = float(np.percentile(q, 99))
    claro = q > 0.5 * alto
    lin = np.where(claro.mean(axis=1) > 0.5)[0]
    col = np.where(claro.mean(axis=0) > 0.5)[0]
    if len(lin) < 16 or len(col) < 16:
        return (0, H, 0, W)
    r0, r1, c0, c1 = int(lin[0]), int(lin[-1] + 1), int(col[0]), int(col[-1] + 1)
    return (r0 + r0 % 2, r1, c0 + c0 % 2, c1)


def medidas(q, roi):
    """Medidas do padrao composto dentro da regiao: mosaico (B, G, G, R) das 4 faixas de cima, media dos
    16 degraus da rampa de baixo, comprimento da linha e igualdade das linhas par/impar."""
    r0, r1, c0, c1 = roi
    hh, ww = r1 - r0, c1 - c0
    m = {}
    for i, nome in enumerate(('branco', 'vermelho', 'verde', 'azul')):	# faixas: miolo de 60%
        a = q[r0 + int(hh * 0.1):r0 + int(hh * 0.4), c0 + int(ww * (i + 0.2) / 4):c0 + int(ww * (i + 0.8) / 4)]
        a = a[:a.shape[0] // 2 * 2, :a.shape[1] // 2 * 2]
        r_off, c_off = (r0 + int(hh * 0.1)) % 2, (c0 + int(ww * (i + 0.2) / 4)) % 2
        # posicoes absolutas do mosaico (0,0), (0,1), (1,0), (1,1)
        for k, (rr, cc) in enumerate(((0, 0), (0, 1), (1, 0), (1, 1))):
            m['%s_%d%d' % (nome, rr, cc)] = float(a[(rr - r_off) % 2::2, (cc - c_off) % 2::2].mean())
    for i in range(16):						# rampa: miolo de cada degrau
        a = q[r0 + int(hh * 0.6):r0 + int(hh * 0.9), c0 + int(ww * (i + 0.25) / 16):c0 + int(ww * (i + 0.75) / 16)]
        m['rampa_%02d' % i] = float(a.mean()) if a.size else 0.0
    x = q.reshape(-1).astype(np.float64)
    x = np.convolve(x - x.mean(), np.ones(4) / 4.0, mode='same')
    f = np.fft.rfft(x, 2 * len(x))
    ac = np.fft.irfft(f * np.conj(f))[:700]
    m['linha'] = float(100 + int(np.argmax(ac[100:680])))		# periodo do fluxo (= largura se normal)
    m['media'] = float(q.mean())
    m['dp'] = float(q.std())
    return m


def salva(pasta, nome, q):
    """Imagem colorida simples (BGGR em meia resolucao), para o relatorio."""
    try:
        from PIL import Image
        os.makedirs(pasta, exist_ok=True)
        rgb = np.dstack([q[1::2, 1::2], (q[0::2, 1::2] + q[1::2, 0::2]) / 2, q[0::2, 0::2]])
        Image.fromarray(np.clip(rgb, 0, 255).astype(np.uint8)).save(os.path.join(pasta, nome + '.png'))
    except Exception:
        pass


def lista_casos(args):
    """Monta a lista de (tipo, registrador, bit) da fase pedida, tirando os de PULA."""
    sel = [a for a in args if not a.startswith('--')]
    fase = sel[0] if sel else 'sensor'
    if fase == 'sensor':
        regs = [('s', r) for r in SENSOR_DOC]
    elif fase == 'ponte':
        regs = [('b', r) for r in PONTE]
    elif fase == 'reservados':
        regs = [('s', r) for r in SENSOR_RES]
    else:
        fase = 'lista'
        regs = [(a.split(':')[0], int(a.split(':')[1], 16)) for a in sel]
    casos, pulados = [], []
    for tipo, reg in regs:
        if (tipo, reg, None) in PULA:
            pulados.append((tipo, reg, None, PULA[(tipo, reg, None)]))
            continue
        for bit in range(7, -1, -1):
            if (tipo, reg, bit) in PULA:
                pulados.append((tipo, reg, bit, PULA[(tipo, reg, bit)]))
            else:
                casos.append((tipo, reg, bit))
    return fase, regs, casos, pulados


def recupera(dev):
    """Depois de uma falha: limpa o regs_start e testa a referencia; se nao vier imagem, recarrega o
    driver. Devolve o dispositivo (pode mudar) ou None."""
    sudo_tee(PS, '')
    if captura(dev, '')[2] is not None:
        return dev
    print('   ... recarregando o driver')
    subprocess.run(['sudo', '-n', 'modprobe', '-r', 'gspca_elgin_fps'], capture_output=True)
    time.sleep(2)
    subprocess.run(['sudo', '-n', 'modprobe', 'gspca_elgin_fps'], capture_output=True)
    time.sleep(4)
    dev = acha_dev()
    for nome, val in (('gain_automatic', 0), ('exposure_dynamic_framerate', 0)):
        subprocess.run(['v4l2-ctl', '-d', dev, '-c', '%s=%d' % (nome, val)], capture_output=True)
    return dev if captura(dev, '')[2] is not None else None


def main():
    """Roda a varredura, grava o CSV a cada caso e no fim escreve o resumo e o relatorio HTML."""
    fase, regs, casos, pulados = lista_casos(sys.argv[1:])
    if '--ensaio' in sys.argv:
        casos = casos[:10]
    pasta = os.path.join(HERE, 'varre_%s' % fase)
    csv_path = os.path.join(HERE, 'varre_%s.csv' % fase)
    sys.stdout = Tee(sys.__stdout__, open(os.path.join(HERE, 'varre_%s_log.txt' % fase), 'a'))
    if subprocess.run(['sudo', '-v']).returncode:
        sys.exit('sudo nao validado')
    print(__doc__)
    dev = acha_dev()
    print('fase %s: %d registradores, %d casos (%d pulados). Estimativa: ~%d min.' % (
        fase, len(regs), len(casos), len(pulados), len(casos) * 3.5 / 60 + 3))
    feitos = set()
    if '--continua' in sys.argv and os.path.exists(csv_path):
        for row in csv.DictReader(open(csv_path)):
            feitos.add((row['tipo'], int(row['reg'], 16), int(row['bit']) if row['bit'] != '' else -1))

    ctrl0 = {}
    for nome in ('gain_automatic', 'exposure_dynamic_framerate'):
        out = subprocess.run(['v4l2-ctl', '-d', dev, '-C', nome], capture_output=True, text=True).stdout
        ctrl0[nome] = out.split(':')[-1].strip()
        subprocess.run(['v4l2-ctl', '-d', dev, '-c', nome + '=0'], capture_output=True)
    jan = Janela('alinhar')
    try:
        input('\n>>> Posicione a janela pelo Multiview (padrao "alinhar", +/- muda o tamanho) ate o contorno\n'
              '    ficar inteiro na imagem, FECHE o Multiview e aperte Enter... ')
        jan.mostra('branco', espera=0.6)
        _, _, qb = captura(dev, '')
        if qb is None:
            sys.exit('sem imagem na referencia: confira a camera')
        roi = regiao(qb)
        print('regiao do padrao: linhas %d-%d, colunas %d-%d' % roi)
        jan.mostra('composto', espera=0.6)
        base = le_base(dev, regs)
        print('valores de base lidos: %d de %d' % (sum(v is not None for v in base.values()), len(base)))

        refs = []
        def referencia():
            """Captura a referencia e guarda as medidas."""
            fps, nb, q = captura(dev, '')
            if q is not None:
                refs.append(dict(medidas(q, roi), fps=fps or 0.0, bytes=nb))
                return q
            return None
        qref = None
        for _ in range(3):					# 3 referencias: medem a variacao natural
            q = referencia()
            qref = q if q is not None else qref
        if qref is None:
            sys.exit('a referencia nao deu imagem: confira a camera e o padrao')
        salva(pasta, 'referencia', qref)

        novo = not os.path.exists(csv_path) or '--continua' not in sys.argv
        fcsv = open(csv_path, 'w' if novo else 'a', newline='')
        campos = None
        n = 0
        for tipo, reg, bit in casos:
            if (tipo, reg, bit) in feitos:
                continue
            v0 = base.get((tipo, reg))
            if v0 is None:
                print('%s:0x%02x sem valor de base: pulado' % (tipo, reg))
                continue
            if n and n % REF_CADA == 0:
                subprocess.run(['sudo', '-v'])
                referencia()
            n += 1
            v = v0 ^ (1 << bit)
            rs = '%s:0x%02x=0x%02x' % (tipo, reg, v)
            t0 = time.time()
            fps, nb, q = captura(dev, rs)
            lido = None
            linha = dict(tipo=tipo, reg='0x%02x' % reg, bit=bit, base='0x%02x' % v0, valor='0x%02x' % v,
                         fps='' if fps is None else '%.1f' % fps, bytes=nb)
            if q is None:
                linha['estado'] = 'SEM IMAGEM' if fps else 'PAROU'
                print('%s  ->  %s' % (rs, linha['estado']))
                dev = recupera(dev)
                if dev is None:
                    print('a camera nao voltou: desconecte e reconecte, depois rode com --continua')
                    break
            else:
                m = medidas(q, roi)
                linha.update({k: '%.2f' % val for k, val in m.items()})
                linha['estado'] = 'ok'
                salva(pasta, '%s%02x_b%d' % (tipo, reg, bit), q)
            if campos is None:
                campos = ['tipo', 'reg', 'bit', 'base', 'valor', 'estado', 'fps', 'bytes'] + \
                    sorted(k for k in medidas(qref, roi))
                if novo:
                    csv.DictWriter(fcsv, campos).writeheader()
            csv.DictWriter(fcsv, campos, extrasaction='ignore').writerow(linha)
            fcsv.flush()
            if linha['estado'] == 'ok':
                print('%s  fps=%s bytes=%d media=%s linha=%s  (%.1f s)' % (
                    rs, linha['fps'], nb, linha['media'], linha['linha'], time.time() - t0))
        fcsv.close()
    finally:
        jan.fecha()
        sudo_tee(PS, '')
        for nome, val in ctrl0.items():
            if val:
                subprocess.run(['v4l2-ctl', '-d', dev, '-c', '%s=%s' % (nome, val)], capture_output=True)
    resumo(fase, csv_path, refs, pulados, pasta)


def resumo(fase, csv_path, refs, pulados, pasta):
    """Compara cada caso com a referencia (tolerancia = variacao das referencias) e escreve o resumo e o
    relatorio HTML com as imagens dos bits com efeito."""
    if not refs or not os.path.exists(csv_path):
        return
    chaves = [k for k in refs[0] if k not in ('fps', 'bytes')]
    ref = {k: statistics.median(r[k] for r in refs) for k in chaves + ['fps', 'bytes']}
    tol = {k: max(3 * (statistics.pstdev([r[k] for r in refs]) if len(refs) > 1 else 0), 0.08 * abs(ref[k]) + 2)
           for k in chaves}
    tol['linha'] = 2
    linhas = list(csv.DictReader(open(csv_path)))
    efeitos = []
    for row in linhas:
        if row['estado'] != 'ok':
            efeitos.append((row, [row['estado']]))
            continue
        motivo = []
        if row['fps'] and abs(float(row['fps']) - ref['fps']) > 2:
            motivo.append('fps %s' % row['fps'])
        if abs(float(row['linha']) - ref['linha']) > tol['linha']:
            motivo.append('linha %s' % row['linha'])
        # grupos de medidas: cor das faixas, rampa, brilho geral
        for grupo, pref in (('cor', ('branco', 'vermelho', 'verde', 'azul')), ('rampa', ('rampa',))):
            fora = [k for k in chaves if k.startswith(pref) and abs(float(row[k]) - ref[k]) > tol[k]]
            if fora:
                motivo.append('%s (%d medidas)' % (grupo, len(fora)))
        if abs(float(row['media']) - ref['media']) > tol['media']:
            motivo.append('brilho %s (ref %.1f)' % (row['media'], ref['media']))
        if motivo:
            efeitos.append((row, motivo))
    with open(os.path.join(HERE, 'varre_%s_resumo.txt' % fase), 'w') as f:
        f.write('Varredura %s: %d casos, %d com efeito, %d pulados. Referencia: fps %.1f, media %.1f, linha %.0f\n\n' % (
            fase, len(linhas), len(efeitos), len(pulados), ref['fps'], ref['media'], ref['linha']))
        for row, mot in efeitos:
            f.write('%s:%s bit %s (%s -> %s): %s\n' % (row['tipo'], row['reg'], row['bit'], row['base'], row['valor'], '; '.join(mot)))
        f.write('\nPulados:\n')
        for tipo, reg, bit, why in pulados:
            f.write('%s:0x%02x %s: %s\n' % (tipo, reg, 'todos os bits' if bit is None else 'bit %d' % bit, why))
    with open(os.path.join(HERE, 'varre_%s.html' % fase), 'w') as f:
        f.write('<!doctype html><meta charset="utf-8"><title>Varredura %s</title><style>body{font-family:sans-serif;'
                'margin:16px}td{padding:4px;vertical-align:top;border-bottom:1px solid #ddd}img{width:160px;'
                'image-rendering:pixelated}</style><h1>Varredura %s: bits com efeito</h1>' % (fase, fase))
        f.write('<p>Referencia: <img src="varre_%s/referencia.png"></p><table>' % fase)
        for row, mot in efeitos:
            img = 'varre_%s/%s%02x_b%s.png' % (fase, row['tipo'], int(row['reg'], 16), row['bit'])
            f.write('<tr><td><b>%s:%s bit %s</b><br>%s → %s</td><td>%s</td><td>%s</td></tr>' % (
                row['tipo'], row['reg'], row['bit'], row['base'], row['valor'], '<br>'.join(mot),
                '<img src="%s">' % img if os.path.exists(os.path.join(HERE, img)) else ''))
        f.write('</table>')
    print('\nResumo: varre_%s_resumo.txt (%d bits com efeito de %d)  Relatorio: varre_%s.html' % (
        fase, len(efeitos), len(linhas), fase))


if __name__ == '__main__':
    main()
