#!/usr/bin/env python3
"""Decodifica um quadro S910 (SN9C10X comprimido) para Bayer BGGR e gera PNG.

Algoritmo: CHIP_INFO/sn9c102_newdoc.txt, secao 12 (Huffman por diferenca,
levantado por Bertrik Sikken).

Uso:  python3 sn9c10x_decode.py quadro_s910.bin LARGURA ALTURA prefixo_saida
Gera: prefixo_bayer.raw  e  prefixo_cor.png (se o ImageMagick 'convert' existir)
"""
import subprocess
import sys


def build_table():
    t = []
    for i in range(256):
        is_abs, val, ln = 0, 0, 0
        if (i & 0x80) == 0:
            val, ln = 0, 1                      # 0        -> +0
        elif (i & 0xE0) == 0x80:
            val, ln = 4, 3                      # 100      -> +4
        elif (i & 0xE0) == 0xA0:
            val, ln = -4, 3                     # 101      -> -4
        elif (i & 0xF0) == 0xD0:
            val, ln = 11, 4                     # 1101     -> +11
        elif (i & 0xF0) == 0xF0:
            val, ln = -11, 4                    # 1111     -> -11
        elif (i & 0xF8) == 0xC8:
            val, ln = 20, 5                     # 11001    -> +20
        elif (i & 0xFC) == 0xC0:
            val, ln = -20, 6                    # 110000   -> -20
        elif (i & 0xFC) == 0xC4:
            val, ln = 0, 8                      # 110001   -> nao usado
        elif (i & 0xF0) == 0xE0:
            is_abs, val, ln = 1, (i & 0x0F) << 4, 8   # 1110xxxx -> absoluto
        t.append((is_abs, val, ln))
    return t


TABLE = build_table()


def decode(data, w, h, stats=None):
    """Retorna (bayer, bits_usados). Bayer BGGR, w*h bytes."""
    n = len(data)
    out = bytearray(w * h)
    bitpos = 0

    def code_at(bp):
        i, s = bp >> 3, bp & 7
        b0 = data[i] if i < n else 0
        b1 = data[i + 1] if i + 1 < n else 0
        return ((b0 << s) | (b1 >> (8 - s))) & 0xFF

    for row in range(h):
        col = 0
        if row < 2:                              # 2 primeiros pixels: 8 bits crus
            out[row * w] = code_at(bitpos)
            bitpos += 8
            out[row * w + 1] = code_at(bitpos)
            bitpos += 8
            col = 2
        while col < w:
            c = code_at(bitpos)
            is_abs, val, ln = TABLE[c]
            bitpos += ln
            if stats is not None:                # contagem de cada tipo de codigo
                k = 'abs' if is_abs else ('??' if ln == 8 else '%+d' % val)
                stats[k] = stats.get(k, 0) + 1
            idx = row * w + col
            if not is_abs:
                if col < 2:                      # coluna da esquerda: pixel acima
                    val += out[idx - 2 * w]
                elif row < 2:                    # duas primeiras linhas: pixel a esquerda
                    val += out[idx - 2]
                else:                            # demais: media esquerda e acima
                    val += (out[idx - 2] + out[idx - 2 * w]) // 2
            out[idx] = 0 if val < 0 else 255 if val > 255 else val
            col += 1
    return out, bitpos


def bayer_to_ppm(bayer, w, h, path):
    """Demosaico simples 2x2 (BGGR) -> imagem RGB de metade do tamanho."""
    w2, h2 = w // 2, h // 2
    rgb = bytearray()
    for y in range(h2):
        for x in range(w2):
            b = bayer[(2 * y) * w + 2 * x]
            g1 = bayer[(2 * y) * w + 2 * x + 1]
            g2 = bayer[(2 * y + 1) * w + 2 * x]
            r = bayer[(2 * y + 1) * w + 2 * x + 1]
            rgb += bytes((r, (g1 + g2) // 2, b))
    with open(path, 'wb') as f:
        f.write(b'P6\n%d %d\n255\n' % (w2, h2) + bytes(rgb))
    return w2, h2


def main():
    if len(sys.argv) != 5:
        print(__doc__)
        sys.exit(1)
    src, w, h, pref = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4]
    data = open(src, 'rb').read()
    bayer, bits = decode(data, w, h)
    open(pref + '_bayer.raw', 'wb').write(bayer)
    ppm = pref + '_cor.ppm'
    w2, h2 = bayer_to_ppm(bayer, w, h, ppm)
    vals = list(bayer)
    print('entrada: %d bytes | bits consumidos: %d (%.1f bytes) | sobra: %.1f bytes'
          % (len(data), bits, bits / 8, len(data) - bits / 8))
    print('Bayer: min=%d max=%d media=%d' % (min(vals), max(vals), sum(vals) // len(vals)))
    try:
        scale = max(1, 640 // w2)
        subprocess.run(['convert', ppm, '-filter', 'point', '-resize', '%dx%d' % (w2 * scale, h2 * scale),
                        '-normalize', pref + '_cor.png'], check=True)
        print('PNG: %s_cor.png' % pref)
    except Exception as e:                      # ImageMagick ausente
        print('sem PNG (%s); use o .ppm' % e)


if __name__ == '__main__':
    main()
