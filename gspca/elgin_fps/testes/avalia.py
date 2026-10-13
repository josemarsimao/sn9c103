#!/usr/bin/env python3
"""Avaliacao interativa da camera SN9C103 (modulo gspca_elgin_fps).

Para cada caso: ajusta os parametros do modulo, mede o FPS, captura um quadro,
decodifica (se S910) e mostra numa janela OpenCV. Voce responde BOA / RUIM por
clique ou tecla; tudo vai para resultados.csv.

Uso (rode num terminal da sua sessao grafica):
    python3 avalia.py lista            # mostra os conjuntos de casos
    python3 avalia.py basico           # BA81 e S910 160x120
    python3 avalia.py fps320 | fps640 | lq | clock | alt8 | geom15 | taxas | todos
    python3 avalia.py --varre b:0x1a 0x14 0x20      # varre um registrador (ponte b: / sensor s:)
    python3 avalia.py basico --teste   # sem janela e sem mexer nos parametros

Teclas: s = boa   n = ruim   r = repetir   b = realce de brilho   q/ESC = sair
"""
import argparse
import csv
import datetime
import os
import re
import subprocess
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from sn9c10x_decode import decode  # noqa: E402

from elgin_dev import acha_dev
DEV = acha_dev()
PARAMS_DIR = '/sys/module/gspca_elgin_fps/parameters'
CSV_PATH = os.path.join(HERE, 'resultados.csv')
JAN = 'Avaliacao SN9C103'
DEFAULTS = dict(stock_regs=0, stock_sensor=0, sensor_reg11=-1, reg17=-1, reg19=-1, lq_sel=-1, top_alt=1,
                regs_start='')


# ---------------------------------------------------------------- casos de teste
def caso(fmt, w, h, fps=None, **params):
    p = dict(DEFAULTS)
    p.update(params)
    nome = '%s %dx%d' % (fmt, w, h)
    if fps:
        nome += ' fps=%s' % fps
    if params:
        nome += ' ' + ' '.join('%s=%s' % (k, hex(v) if k in ('reg17', 'reg19') else v)
                               for k, v in params.items())
    return dict(fmt=fmt, w=w, h=h, fps=fps, params=p, nome=nome)


def conjuntos():
    c = {}
    c['basico'] = [caso('BA81', 160, 120), caso('S910', 160, 120)]
    c['fps320'] = [caso('S910', 320, 240, sensor_reg11=r) for r in (0, 1, 2, 3)]
    c['fps640'] = [caso('S910', 640, 480, sensor_reg11=r) for r in (0, 1, 2, 3)]
    c['lq'] = [caso('S910', w, h, sensor_reg11=r, lq_sel=lq)
               for (w, h, regs) in ((320, 240, (0, 1, 2)), (640, 480, (1, 2, 3)))
               for r in regs for lq in (0, 1)]
    c['clock'] = [caso('S910', w, h, sensor_reg11=r, reg17=r17, reg19=r19, lq_sel=lq)
                  for (w, h, r) in ((320, 240, 0), (640, 480, 3))
                  for (r17, r19) in ((0x68, 0x20), (0x60, 0x50), (0x6C, 0x2B))
                  for lq in (0, 1)]
    # alt 8 (1003 KB/s): 640x480 a 15 fps precisa de ~907 KB/s; 10 fps e 320x240 sao controles
    c['alt8'] = [caso('S910', 640, 480, sensor_reg11=r, top_alt=1) for r in (1, 2, 3, 1, 0)] + \
                [caso('S910', 320, 240, sensor_reg11=0, top_alt=1)]
    # geometria da linha a 15 fps (causa do cisalhamento): registradores da ponte, reaplicados no fim do start
    c['geom15'] = [caso('S910', 640, 480, sensor_reg11=1, top_alt=1, regs_start=r) for r in (
        '', 'b:0x1a=0x1d,b:0x1b=0x10',
        'b:0x1a=0x1d,b:0x1b=0x10,b:0x1c=0x02,b:0x1d=0x03,b:0x1e=0x0f,b:0x1f=0x0c',
        'b:0x1a=0x1d', 'b:0x1b=0x10', 'b:0x12=0x00', 'b:0x12=0x02', 'b:0x13=0x00', 'b:0x13=0x02')]
    # taxas anunciadas pelo driver, pedidas como um aplicativo pediria (S_PARM), sem sensor_reg11
    c['taxas'] = [caso('S910', 640, 480, fps=f) for f in (14, 13, 10, 8)] + \
                 [caso('S910', 320, 240, fps=f) for f in (30, 15)] + \
                 [caso('S910', 160, 120, fps=30), caso('BA81', 160, 120, fps=30)]
    c['todos'] = c['basico'] + c['fps320'] + c['fps640'] + c['lq'] + c['clock'] + c['alt8']
    return c


# ---------------------------------------------------------------- acesso ao modulo
def checar_modulo():
    if not os.path.isdir(PARAMS_DIR):
        sys.exit('Modulo gspca_elgin_fps nao esta carregado (%s nao existe).' % PARAMS_DIR)


def set_params(p, cache):
    """Escreve so o que mudou. Precisa de sudo (credencial ja validada no inicio)."""
    for k, v in p.items():
        if cache.get(k) == v:
            continue
        r = subprocess.run(['sudo', '-n', 'tee', '%s/%s' % (PARAMS_DIR, k)],
                           input=(str(v) + '\n').encode(), capture_output=True)
        if r.returncode:
            sys.exit('Nao consegui escrever %s=%s: %s' % (k, v, r.stderr.decode().strip()))
        cache[k] = v


def v4l2(args, timeout):
    try:
        return subprocess.run(['v4l2-ctl', '-d', DEV] + args, capture_output=True,
                              text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return None


def opt_fmt(c):
    o = ['--set-fmt-video=width=%d,height=%d,pixelformat=%s' % (c['w'], c['h'], c['fmt'])]
    if c['fps']:
        o.append('--set-parm=%s' % c['fps'])
    return o


def medir_fps(c):
    n = {160: 90, 320: 60, 640: 30}.get(c['w'], 30)      # >= 2 s de captura (o v4l2-ctl so imprime o fps apos 1 s)
    r = v4l2(opt_fmt(c) + ['--stream-mmap', '--stream-count=%d' % n, '--stream-to=/dev/null'], 60)
    if r is None:
        return None
    m = re.findall(r'([0-9.]+) fps', r.stdout + r.stderr)
    return float(m[-1]) if m else None


def salvar_bruto(c, data, ext):
    """Guarda o quadro bruto (S910 comprimido ou Bayer cru) para analise offline."""
    os.makedirs(os.path.join(HERE, 'brutos'), exist_ok=True)
    arq = re.sub(r'[^A-Za-z0-9._=-]+', '_', c['nome'])
    c['arq_bruto'] = '%s_%s.%s' % (datetime.datetime.now().strftime('%H%M%S'), arq, ext)
    open(os.path.join(HERE, 'brutos', c['arq_bruto']), 'wb').write(data)


def capturar(c):
    """Retorna (matriz Bayer HxW ou None, aviso)."""
    path = '/tmp/avalia_quadro.raw'
    if os.path.exists(path):
        os.remove(path)
    if c['fmt'] == 'BA81':
        v4l2(opt_fmt(c) + ['--stream-mmap', '--stream-count=4', '--stream-to=' + path], 60)
        data = open(path, 'rb').read() if os.path.exists(path) else b''
        fs = c['w'] * c['h']
        if len(data) < fs:
            return None, 'sem quadro'
        c['bytes_quadro'] = fs
        salvar_bruto(c, data[-fs:], 'ba81')
        return np.frombuffer(data[-fs:], np.uint8).reshape(c['h'], c['w']), ''
    v4l2(opt_fmt(c) + ['--stream-mmap', '--stream-skip=4', '--stream-count=1',
                       '--stream-to=' + path], 60)
    data = open(path, 'rb').read() if os.path.exists(path) else b''
    if not data:
        return None, 'sem quadro'
    c['bytes_quadro'] = len(data)
    st = {}
    bayer, bits = decode(data, c['w'], c['h'], st)
    c['sobra'] = round(len(data) - bits / 8, 1)   # bytes que o decodificador nao usou (~0 = quadro limpo)
    tot = sum(st.values()) or 1
    c['codigos'] = ' '.join('%s=%.1f%%' % (k, 100.0 * st[k] / tot) for k in sorted(st))
    salvar_bruto(c, data, 's910')
    aviso = ''
    if abs(bits / 8 - len(data)) > 1.01:         # quadro valido consome todos os bytes
        aviso = 'AUTO: quadro S910 inconsistente (%d de %d bytes)' % (round(bits / 8), len(data))
    return np.frombuffer(bytes(bayer), np.uint8).reshape(c['h'], c['w']), aviso


# ---------------------------------------------------------------- imagem e janela
def para_bgr(bayer, normalizar):
    b = bayer[0::2, 0::2].astype(np.float32)
    g = (bayer[0::2, 1::2].astype(np.float32) + bayer[1::2, 0::2]) / 2
    r = bayer[1::2, 1::2].astype(np.float32)
    img = np.dstack([b, g, r])
    if normalizar:
        img = img * (255.0 / max(float(np.percentile(img, 99.5)), 1.0))
    return np.clip(img, 0, 255).astype(np.uint8)


BOTOES = [('BOA (s)', (50, 150, 50), 's'), ('RUIM (n)', (50, 50, 190), 'n'),
          ('REPETIR (r)', (110, 110, 110), 'r'), ('SAIR (q)', (70, 70, 70), 'q')]


def montar(img, linhas):
    ih, iw = img.shape[:2]
    esc = max(1, 640 // iw)
    big = cv2.resize(img, (iw * esc, ih * esc), interpolation=cv2.INTER_NEAREST)
    W = max(big.shape[1], 660)
    cab = 24 * len(linhas) + 8
    canvas = np.full((cab + big.shape[0] + 70, W, 3), 30, np.uint8)
    for i, t in enumerate(linhas):
        cv2.putText(canvas, t, (8, 22 + 24 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (230, 230, 230), 1, cv2.LINE_AA)
    x0 = (W - big.shape[1]) // 2
    canvas[cab:cab + big.shape[0], x0:x0 + big.shape[1]] = big
    rects = []
    bw = W // len(BOTOES)
    y1, y2 = cab + big.shape[0] + 12, cab + big.shape[0] + 58
    for i, (txt, cor, tecla) in enumerate(BOTOES):
        x1, x2 = i * bw + 6, (i + 1) * bw - 6
        cv2.rectangle(canvas, (x1, y1), (x2, y2), cor, -1)
        cv2.putText(canvas, txt, (x1 + 12, y1 + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 2, cv2.LINE_AA)
        rects.append((x1, y1, x2, y2, tecla))
    return canvas, rects


def perguntar(canvas, rects):
    resp = {'v': None}

    def mouse(ev, x, y, flags, param):
        if ev == cv2.EVENT_LBUTTONDOWN:
            for x1, y1, x2, y2, k in rects:
                if x1 <= x <= x2 and y1 <= y <= y2:
                    resp['v'] = k

    cv2.setMouseCallback(JAN, mouse)
    while resp['v'] is None:
        cv2.imshow(JAN, canvas)
        k = cv2.waitKey(30) & 0xFF
        if k == 27:
            resp['v'] = 'q'
        elif k != 255 and chr(k).lower() in 'snrqb':
            resp['v'] = chr(k).lower()
        try:
            if cv2.getWindowProperty(JAN, cv2.WND_PROP_VISIBLE) < 1:
                resp['v'] = 'q'
        except cv2.error:
            resp['v'] = 'q'
    return resp['v']


# ---------------------------------------------------------------- principal
def gravar(linha):
    novo = not os.path.exists(CSV_PATH)
    with open(CSV_PATH, 'a', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        if novo:
            w.writerow(['data_hora', 'caso', 'formato', 'largura', 'altura', 'fps_pedido',
                        'fps_medido', 'parametros', 'media', 'desvio', 'aviso', 'veredito', 'bytes_quadro',
                        'codigos', 'arquivo_bruto', 'sobra_bytes'])
        w.writerow(linha)


def main():
    todos = conjuntos()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('conjunto', nargs='?', default='basico', choices=list(todos) + ['lista'])
    ap.add_argument('--teste', action='store_true', help='sem janela, sem sudo; salva /tmp/avalia_teste_N.png')
    ap.add_argument('--varre', nargs='+', metavar=('ITEM', 'INI'),
                    help='varre um registrador: ITEM INI FIM [PASSO]  (ITEM = b:0x1a ponte ou s:0x28 sensor)')
    ap.add_argument('--fmt', default='S910', help='formato da varredura (padrao S910)')
    ap.add_argument('--tam', default='640x480', help='tamanho da varredura (padrao 640x480)')
    ap.add_argument('--reg11', type=int, default=1, help='sensor_reg11 da varredura (padrao 1 = 15 fps)')
    ap.add_argument('--topalt', type=int, default=1, help='top_alt da varredura (padrao 1)')
    a = ap.parse_args()

    if a.conjunto == 'lista':
        for nome, cs in todos.items():
            print('%-7s %2d casos' % (nome, len(cs)))
            if nome != 'todos':
                for c in cs:
                    print('         -', c['nome'])
        return

    casos = todos[a.conjunto]
    if a.varre:
        item = a.varre[0]
        ini, fim = int(a.varre[1], 0), int(a.varre[2], 0)
        passo = int(a.varre[3], 0) if len(a.varre) > 3 else 1
        w, h = (int(x) for x in a.tam.split('x'))
        casos = [caso(a.fmt, w, h, sensor_reg11=a.reg11, top_alt=a.topalt,
                      regs_start='%s=%s' % (item, hex(v))) for v in range(ini, fim + 1, passo)]
    checar_modulo()
    cache = {}
    if not a.teste:
        print('Validando sudo (para escrever os parametros do modulo)...')
        if subprocess.run(['sudo', '-v']).returncode:
            sys.exit('sudo nao validado.')
        cv2.namedWindow(JAN, cv2.WINDOW_AUTOSIZE)

    normalizar = True
    resumo = {'boa': 0, 'ruim': 0, 'sem_imagem': 0}
    i = 0
    while i < len(casos):
        c = casos[i]
        print('[%d/%d] %s' % (i + 1, len(casos), c['nome']))
        if not a.teste:
            set_params(c['params'], cache)
        fps = medir_fps(c)
        bayer, aviso = capturar(c)
        if bayer is None:
            print('   sem imagem')
            gravar([datetime.datetime.now().isoformat(timespec='seconds'), c['nome'], c['fmt'], c['w'],
                    c['h'], c['fps'] or '', fps or '', c['params'], '', '', aviso, 'sem_imagem', '', '', '', ''])
            resumo['sem_imagem'] += 1
            i += 1
            continue
        media, desvio = float(bayer.mean()), float(bayer.std())
        while True:
            linhas = ['[%d/%d] %s' % (i + 1, len(casos), c['nome']),
                      'FPS medido: %s | brilho medio %.0f | desvio %.0f | realce: %s'
                      % ('%.1f' % fps if fps else '?', media, desvio, 'sim' if normalizar else 'nao'),
                      aviso or 'teclas: s=boa n=ruim r=repetir b=brilho q=sair']
            canvas, rects = montar(para_bgr(bayer, normalizar), linhas)
            if a.teste:
                cv2.imwrite('/tmp/avalia_teste_%d.png' % (i + 1), canvas)
                print('   fps=%s media=%.0f desvio=%.0f %s | codigos: %s' % (fps, media, desvio, aviso, c.get('codigos', '')))
                v = 's'
                break
            v = perguntar(canvas, rects)
            if v == 'b':
                normalizar = not normalizar
                continue
            break
        if v == 'r':
            continue                              # repete o mesmo caso
        veredito = {'s': 'boa', 'n': 'ruim', 'q': 'sair'}[v]
        if not a.teste:
            os.makedirs(os.path.join(HERE, 'imagens'), exist_ok=True)
            arq = re.sub(r'[^A-Za-z0-9._=-]+', '_', c['nome'])
            cv2.imwrite(os.path.join(HERE, 'imagens', '%s_%s_%s.png' % (
                datetime.datetime.now().strftime('%H%M%S'), arq, veredito)), canvas)
            gravar([datetime.datetime.now().isoformat(timespec='seconds'), c['nome'], c['fmt'], c['w'],
                    c['h'], c['fps'] or '', fps or '', c['params'], '%.1f' % media, '%.1f' % desvio,
                    aviso, veredito, c.get('bytes_quadro', ''), c.get('codigos', ''),
                    c.get('arq_bruto', ''), c.get('sobra', '')])
        if v == 'q':
            break
        resumo[veredito] += 1
        i += 1

    if not a.teste:
        set_params(DEFAULTS, cache)               # volta o modulo ao padrao
        cv2.destroyAllWindows()
    print('Resumo:', resumo, '| resultados em', CSV_PATH)


if __name__ == '__main__':
    main()
