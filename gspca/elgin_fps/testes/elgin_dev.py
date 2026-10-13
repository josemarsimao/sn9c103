"""Acha o /dev/videoN da camera Elgin (SN9C103 + OV7630).

O numero muda: a camera interna do laptop costuma ficar com /dev/video0 e /dev/video1.
Os testes usam:   from elgin_dev import acha_dev ;  DEV = acha_dev()

Ordem de procura:
  1. variavel de ambiente ELGIN_DEV (ex.: ELGIN_DEV=/dev/video4 python3 testa_cores.py)
  2. /sys/class/video4linux/videoN/name que contem "Elgin" (menor N)
  3. a saida de `v4l2-ctl --list-devices`
Se nao achar, o teste termina com uma mensagem clara (em vez de medir a camera errada).
"""
import glob
import os
import re
import subprocess
import sys


def nome_dev(dev):
    """Nome da camera (o que o driver informa) para o /dev/videoN dado."""
    try:
        out = subprocess.run(['v4l2-ctl', '-d', dev, '--info'], capture_output=True, text=True).stdout
        m = re.search(r'Card type\s*:\s*(.+)', out)
        if m:
            return m.group(1).strip()
    except OSError:
        pass
    return '?'


def acha_dev():
    dev = _procura()
    print('>>> Camera analisada: %s  (%s)' % (dev, nome_dev(dev)), flush=True)
    return dev


def _procura():
    env = os.environ.get('ELGIN_DEV')
    if env:
        return env
    achados = []
    for d in glob.glob('/sys/class/video4linux/video*'):
        try:
            nome = open(os.path.join(d, 'name')).read()
        except OSError:
            continue
        if 'elgin' in nome.lower():
            achados.append(int(re.sub(r'\D', '', os.path.basename(d))))
    if achados:
        return '/dev/video%d' % min(achados)
    try:
        out = subprocess.run(['v4l2-ctl', '--list-devices'], capture_output=True, text=True).stdout
    except OSError:
        out = ''
    ach = False
    for ln in out.splitlines():
        if ln and not ln.startswith(('\t', ' ')):
            ach = 'elgin' in ln.lower()
        elif ach and ln.strip().startswith('/dev/video'):
            return ln.strip()
    sys.exit('camera Elgin nao encontrada (modulo gspca_elgin_fps carregado? camera conectada?). '
             'Para forcar: ELGIN_DEV=/dev/videoN python3 ...')


if __name__ == '__main__':
    print(acha_dev())
