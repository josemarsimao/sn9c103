# Notas: webcam SN9C103 + OV7630 (USB 0c45:608f)

Última atualização: 2026-10-13 (varredura das fontes, seção no fim) · kernel 7.0.0-38-generic · módulo `gspca_elgin_fps` (esta pasta).

**Legenda de confiança:** `[medido]` testado por você · `[captura]` log do Windows (`../../log_sn9c103.txt`, 2022-12-09) · `[datasheet]` OV7630.pdf v1.7 · `[código]` lido do driver · `[hipótese]` ainda não confirmado.

---

## 0. Critérios do projeto

- **Driver completo:** o objetivo é um driver completo. Funcionalidades padrão do V4L2 devem ser implementadas **mesmo que o kernel em uso não as habilite** (por exemplo, os ganchos `VIDIOC_DBG_*`, que dependem de `CONFIG_VIDEO_ADV_DEBUG`, desligado no kernel 7.0.0-38 desta máquina). Esse código fica sob `#ifdef` e é verificado compilando com `KCFLAGS=-DCONFIG_VIDEO_ADV_DEBUG` num diretório temporário (sem instalar: o layout de `v4l2_ioctl_ops` muda com a opção).
- **Um passo de cada vez**, com respostas curtas e comandos explicados de forma simples.
- **Ordem combinada:** primeiro a melhoria do driver, depois o emulador de câmera.

## 1. Hardware e ambiente

- Bridge SN9C103 + sensor OV7630 (endereço I²C `0x21`). USB 1.1 full-speed, na porta `1-3` (direto no root hub).
- Alt settings da câmera `[medido]` (dmesg, debug=3): alt 1 a 8 = 128000, 256000, 384000, 512000, 680000, 800000, 900000, 1003000 B/s.
- O driver original inicia em **alt 7 (900000 B/s)**, sem recusa do host, em 320x240 e 640x480. `[medido]`
  - Por que não o alt 8? `[hipótese]`: regra do gspca "dispositivo com áudio USB pula o alt mais alto" (vale se o limite passa de 1.000.000). Verificar com `dmesg | grep -i audio` ou `lsusb -v -d 0c45:608f`.
- `min bandwidth` pedido pelo driver: 5.760.000 (640x480: 384.000 B × 15) e 2.880.000 (320x240: 96.000 B × 30). Ambos acima do máximo, então sempre usa o alt mais alto permitido. `[medido]`

## 2. Bridge SN9C103: bloco de 0x30 bytes escrito a partir do reg 0x01

Colunas: valor da captura do Windows (bloco inicial com valores de CIF 352x288, alt 8; **o modo capturado é 320x240 comprimido**, ver "Varredura das fontes") × valor do bloco "original" do gspca (antigo `#if(0)`, hoje `stock_regs=1`).

| Reg | Windows `[captura]` | Original `[código]` | Significado / observações |
|---|---|---|---|
| 0x01 | 0x44 | 0x44 | Seleciona clock de 24 MHz; no fim da init é reescrito para ligar o vídeo |
| 0x02 | 0x44 | 0x44 | |
| 0x03 | 0x00 | 0x00 | |
| 0x04 | 0x1A | 0x00 | |
| 0x05–0x07 | 0x20 | 0x20 | Ganho do bridge R, G, B = 1,0 |
| 0x08 | 0x80 | 0x80 | |
| 0x09 | 0x21 | 0x21 | |
| 0x0A–0x0F | 0x00 | 0x00 | |
| 0x10 | 0x03 | 0x00 | |
| 0x11 | 0x00 | 0x00 | |
| 0x12 | 0x01 | 0x02 | hstart. Sua anotação: em 640x480 com 5 aparece linha escura vertical à direita; com 4 ainda restam resquícios |
| 0x13 | 0x01 | 0x00 | vstart |
| 0x14 | 0x0A | 0x00 | Offset; sua anotação: quanto maior, mais escura a imagem; se clareia, parece mais borrada |
| 0x15 | 0x16 | 0x28 | hsize/16: 0x16 = 352, 0x28 = 640 |
| 0x16 | 0x12 | 0x1E | vsize/16: 0x12 = 288, 0x1E = 480 |
| 0x17 | 0x68 | 0x6C | Clock do sensor (`fixme: ov7630 [17]=68`) |
| 0x18 | 0x8F | 0x8F \| (modo<<4) | bit 7 = compressão (zerado em RAW); bits 6:4 = escala (0 = nenhuma, 1 = ÷2, 2 = ÷4). Nibble baixo 0xF = polaridades de sincronismo (PCK_RIS, HSYNC_RIS, VSYNC_RIS, VSYNC_HIGH), ver 2b. **Correção:** antes estava anotado como "nível de compressão", o que estava errado |
| 0x19 | 0x20 | 0x2B | MCKSIZE |
| 0x1A | 0x1D | 0x14 | HO_SIZE ("makes no sense"). Sua anotação: mudar não alterou nada |
| 0x1B | 0x10 | 0x0F | VO_SIZE (idem) |
| 0x1C–0x1F | 02 03 0F 0C | 00 00 00 00 | Janela de autoexposição (H-start, V-start, H-end, V-end) |
| 0x20 | 0x00 | 0x00 | |
| 0x21–0x30 | 10 20 30 … F0 FF | idem | Tabela gama linear (`i*16`, último = 255) |

**Valores ativos hoje no `elgin_fps` (`stock_regs=0`):** coluna Windows, **exceto** `0x15 = 0x28`, `0x16 = 0x1E` (janela 640x480; com 0x16/0x12 todos os modos saíam deformados `[medido]`) e `0x1A–0x1F = 0x14, 0x0F, 0, 0, 0, 0` (ramo "SIF" forçado no código).

**Sequência do `sd_start`:** grava 0x30 bytes em 0x01 → init do sensor → I²C `0x13 = 0x80` → regs `0x15` (×2), `0x18`, `0x12`, `0x13`, `0x17`, `0x19`, `0x1C` (×4) → `0x01` (liga vídeo) → `0x18` (×2) → espera 20 ms → `apply_fps` / `setfreq`. `[código]`

### 2b. Significado dos campos `[datasheet SN9C102 v0.9]`

Fonte: `CHIP_INFO/sn9c102_registers.pdf` (antes `sn9c102.pdf`). O SN9C103 é parecido, mas **não há mapa oficial dele**; use como referência (bate com o comportamento observado).

| Reg | Campos |
|---|---|
| 0x01 | bit0 S_PWR_DN · bit1 S_PDN_INV · bit2 V_TX_EN (transferência de vídeo) · bit3 LED · bit4 KEY (leitura) · bit6 SYS_SEL_24M (1 = clock do sistema 24 MHz, 0 = 12 MHz) · bit7 TEST_ASIC (não ligar). 0x44 = V_TX_EN + 24 MHz |
| 0x08 | bit0 I2C_HIGH (400 kHz) · bit1 I2C_SEL_RD · bit2 I2C_RDY · bit3 I2C_ERR · bits 6:4 I2C_BYTE_NUM (bytes totais = endereço + dados) · bit7 I2C_DEV (1 = I²C). Por isso 0xA0/B0/C0/D0 = 2/3/4/5 bytes |
| 0x09 | SLAVE_ID (0x21 = OV7630) |
| 0x0A–0x0E | Porta de dados I²C: 5 bytes de uma vez (endereço do registrador do sensor + data0..3) |
| 0x0F | Controle/status (o `0x10` final do vetor de 8 bytes) |
| 0x10, 0x11 | No SN9C102 são os ganhos R/B e G (4 bits). No 103 os ganhos R, G, B ficam em 0x05–0x07 (7 bits, 0–127; hoje 0x20) |
| 0x12, 0x13 | H_START / V_START: primeiro pixel / linha ativos após o sync do sensor. Linha 1 = BGBG, linha 2 = GRGR (Bayer BGGR) |
| 0x14 | OFFSET: compensação do nível de preto |
| 0x15, 0x16 | H_SIZE (6 bits) / V_SIZE (5 bits): janela de saída, em múltiplos de 16 |
| 0x17 | bit0 **LQ_SEL** (1 = compressão de baixa qualidade, 0 = alta) · bits 3:2 SEN_RATE (clock do sensor: 01 = 12, 10 = 24, 11 = 48 MHz, 00 = Fsys/MCK_SIZE) · bit4 TEST_IMG · bit5 SEN_CLK_EN · bit6 SEN_CLK_INV. Windows 0x68 = sensor a 24 MHz; original 0x6C = 48 MHz; ambos com qualidade alta |
| 0x18 | bit0 PCK_RIS · bit1 HSYNC_RIS · bit2 VSYNC_RIS · bit3 VSYNC_HIGH · bits 5:4 SCAL (00 = 1/1, 01 = 1/2, 1x = 1/4) · bit6 SEL_CURVE (curva de companding) · bit7 CMP_MODE (compressão) |
| 0x19 | bit0 SYNC_OUT (1 = bridge gera o sync, sensor escravo) · bit1 PCK_OUT · bit3 PCK_2X · bits 7:4 MCK_SIZE (2–15; clock = Fsys/MCK_SIZE se SEN_RATE = 0). Windows 0x20 = sensor mestre; original 0x2B = SYNC_OUT, PCK_OUT e PCK_2X ligados |
| 0x1A, 0x1B | HO_SIZE / VO_SIZE: tamanho "original" do sensor, em múltiplos de 32 |
| 0x1C–0x1F | AE_STRX (3 bits), AE_STRY (3 bits), AE_ENDX (5 bits), AE_ENDY (4 bits): janela da autoexposição, em múltiplos de 32 |

**Cabeçalho do quadro** `[doc sn9c102]`: 12 bytes no SN9C101/102, **18 bytes no SN9C103** (correção 2026-10-13, documentação do Risolia `sn9c102.txt`; o `find_sof` do driver já usa 18). No 103: bytes 0–5 = `FF FF 00 C4 C4 96`; byte 6 = ganho R e byte 7 = ganho B (bits 6:0, ganho = 1/2 + valor/64); byte 8 = bit 0 compressão, bits 2:1 escala máxima, bit 3 FIFO USB (2 KB) cheio, bit 4 "ganho digital pronto", bit 5 "exposição pronta", bits 7:6 índice do quadro; bytes 9–10 = soma de Y **dentro** da janela de AE, Y = (R/4 + 5G/16 + B/8)/32; bytes 11–12 = soma **fora**, com /128; bytes 13–17 = áudio (nº do quadro de áudio, gravando, soma, contagem e pico). A soma de dentro é a base do ganho automático do driver; a janela de AE hoje usa `02 03 0F 0C` (padrão por modo) e o alvo é escalado pela área (`ae_area`).

**USB** `[datasheet]`: ep0 controle (64 B), ep1 isócrono de vídeo (alts com 0, 128, 256, 384, 512, 680, 800, 900 e 1023 B/ms), ep2 bulk (64 B), ep3 interrupção (botão). SN9C103: "até 30 fps em CIF e 15 fps em VGA", buffer USB de 2 KB, compressor proprietário, janela por hardware e escala 1/2 e 1/4.

### 2c. Referências cruzadas na pasta `CHIP_INFO/` `[datasheets e drivers irmãos]`

O que cada arquivo realmente traz:

| Arquivo | Conteúdo útil |
|---|---|
| `SN9C103_diagram.pdf` (antes `SN9C103_DataSheet.pdf`) | Pinos, diagrama de blocos, lista de recursos. **Sem mapa de registradores.** "Até 30 fps em CIF, 15 fps em VGA"; lista "image quality / frame rate selection" |
| `sn9c102_registers.pdf` (v0.9) e `SN9C101_endpoints.pdf` (antes `sn9c102.pdf` e `SN9C101_Spec_v1[1].3.pdf`) | **Únicos com descrição de registradores** (mapa base da família) |
| `SN9C102_diagram`, `SN9C102P_schematic`, `SN9C105_diagram`, `SN9C201_Spec`, `SN9C202_Spec` (os três primeiros eram `*_DataSheet*`) | Só recursos e pinagem. Taxas: SN9C101 30 fps CIF; SN9C102 30 CIF / **12 VGA**; SN9C103 30 CIF / **15 VGA**; 102P/105/201/202 30 fps VGA (usam compressão **JPEG**) |
| `sn9c1xx-1.48.tar.gz` (Luca Risolia) | Driver que **suporta explicitamente SN9C103 + OV7630**, com a init completa de cada sensor; a melhor referência cruzada |
| `sn9c102-1.24.tar.gz`, `sn9c102*.txt/pdf` | Versão antiga do mesmo driver e a documentação (formato do quadro, cabeçalho de 12 bytes, algoritmo de compressão) |

**Qualidade de compressão no SN9C103 `[driver irmão]`:** o `sn9c102_set_compression()` trata SN9C101/102/**103** pelo **bit 0 do reg `0x17`** (bit = 1 → qualidade baixa; bit = 0 → alta). O bit 6 do `0x18` é só dos SN9C105/120 (JPEG). Isso confirma o `LQ_SEL` também para o 103. A qualidade inicial é a que a init do sensor deixa em `0x17`.

**Init do bridge para SN9C103 + OV7630 no driver irmão** (`sn9c102_ov7630.c`) comparada com as outras duas:

| Reg | Risolia 1.48 | Windows `[captura]` | gspca original |
|---|---|---|---|
| 0x04 | 0x1A | 0x1A | 0x00 |
| 0x05–0x07 | 0x20 | 0x20 | 0x20 |
| 0x10 | 0x03 | 0x03 | 0x00 |
| 0x14 | 0x0A | 0x0A | 0x00 |
| **0x17** | **0x60** | **0x68** | **0x6C** |
| 0x18 | 0x0F (sem compressão, sem escala; o driver ajusta depois) | 0x8F | 0x8F \| (modo<<4) |
| **0x19** | **0x50** | **0x20** | **0x2B** |
| 0x1A, 0x1B | 0x1D, 0x10 | 0x1D, 0x10 | 0x14, 0x0F |
| **0x1C–0x1F** | **02 03 0F 0C** | **02 03 0F 0C** | 00 00 00 00 |
| 0x21–0x30 | gama linear | gama linear | gama linear |

Leitura:
- **Concordam** (Risolia e Windows): `0x04`, `0x10`, `0x14`, `0x1A`, `0x1B` e a **janela de AE `02 03 0F 0C`**. A janela zerada do nosso código é a exceção, não a regra.
- **Divergem nos três** em `0x17` e `0x19`, os registradores do **clock do sensor**: Risolia `SEN_RATE = 00` (clock = Fsys/MCK_SIZE, com MCK_SIZE = 5), Windows 24 MHz, original 48 MHz. Há, portanto, três configurações de clock funcionando para o mesmo sensor; é o espaço a explorar para o FPS.
- Sensor (Risolia, 103): `0x11 = 0x01`, `0x15 = 0x34`, `0x1B = 0x04`, `0x20 = 0x44`, `0x23 = 0xEE`, `0x26..29 = A0 9A 20 30`, `0x2F = 0x3D`, `0x30 = 0x24`, `0x32 = 0x86`, `0x60 = 0xA9`, `0x61 = 0x42`, `0x69 = 0x38`, `0x6F..71 = 88 0B 00`, `0x74 = 0x21`, `0x7D = 0xF7`. Quase igual à captura do Windows; diferenças: `0x28 = 0x20` (Windows 0xA0) e `0x60 = 0xA9` (Windows 0x29).

## 3. Sensor OV7630 (I²C `0x21`): sequência capturada do Windows

Formato do vetor de 8 bytes: `{ctrl, id, subendereço, D0..D3, CRT}`; `ctrl` = 0xA0/0xB0/0xC0/0xD0 para 1/2/3/4 bytes de dados, com subendereço auto-incrementado. `[inferido da tabela]`

```
0x12     = 0x80              (soft reset)
0x12..15 = 78 00 80 34
0x11     = 0x01
0x01..02 = 77 3A
0x1B     = 0x04
0x20     = 0x44
0x23     = 0xEE
0x26..29 = A0 9A A0 30
0x2A..2B = 80 00
0x2F..30 = 3D 24
0x32     = 0x86
0x60..61 = 29 42
0x65     = 0x00
0x69     = 0x38
0x6F..71 = 88 0B 00
0x74..76 = 21 8E 00
0x7D     = 0xF7
0x17..1A = 1C BD 06 F6
0x10     = 0x83
0x76     = 0x00
0x11     = 0x00              (divisor 0 → 30 fps)
0x00     = 0x10
0x2A..2B = 80 60
```
Depois, o driver ainda grava `0x13 = 0x80`.

**Diferenças em relação à tabela original** (`ov7630_sensor_init_stock`, usada com `stock_sensor=1`): `0x12` = 0x78 (Windows) × 0x3C; `0x10` = 0x83 × 0x50; `0x13` = 0x00→0x80 × 0x83; `0x29` = 0x30 × 0x74; `0x2D` = não escrito (padrão 0x81) × 0x81; a original ainda grava `0x14 = 0x80`, `0x15 = 0x2D`, `0x16 = 0x03`.

### Registradores do sensor ligados a FPS e imagem

| Reg | Função | Fonte |
|---|---|---|
| 0x11 CLKRC | Divisor do clock: **fps = 30/(n+1)**. n=0→30, 1→15, 2→10, 3→7,5 | `[medido]` |
| 0x2D COMJ | bit 6 = 1: só campo ímpar, entrelaçado, **até 60 fps**; = 0: todos os campos, máx. 30 fps. Padrão 0x81 (a linha `0x2D=0x81 // QVGA -> 60fps` da tabela antiga grava o padrão, não liga o 60) | `[datasheet]` |
| 0x14 COMC | bit 5 = seleção QVGA do sensor (não usado hoje) | `[datasheet]` |
| 0x28 COMH | Windows grava 0xA0 (raw de uma linha, progressivo); padrão 0x00 (entrelaçado). Sem a escrita você viu "FPS=60" | `[captura]` / `[hipótese]` |
| 0x10 | Exposição fina; `reg10_max = 0x41` | `[código]` |
| 0x00 | Ganho do sensor | `[código]` |
| 0x2B | Frequência da rede (`setfreq`) | `[código]` |
| 0x06 | Brilho (sensor) | `[código]` |

## 4. Resultados dos testes (2026-10-11)

### Driver original `gspca_sonixb` (linha de base)
- 320x240: 15 fps no escuro, 30 fps no claro (autogain sobe a exposição → divisor do 0x11). `[medido]`
- 640x480: 8 fps em qualquer luz. O código força `reg11 ≥ 4` (valor gravado = reg11−1 = 3 → 30/4 = 7,5). Comentário do kernel: abaixo disso a imagem fica instável, porque o bridge entra num modo de compressão mais forte "ainda não decodificado". `[código]`

### `gspca_elgin_fps`, qualidade da imagem × FPS (reg 0x11 forçado)

| Modo | 30 fps | 15 fps | 10 fps | 7,5 fps |
|---|---|---|---|---|
| S910 160x120 | perfeito | perfeito | | |
| S910 320x240 | deformado | perfeito | | perfeito |
| S910 640x480 | deformado | levemente deformado | levemente deformado (menos) | **perfeito** |
| BA81 160x120 | perfeito | perfeito | | |
| BA81 320x240 | deformado | só ~32 linhas | **perfeito** | |
| BA81 640x480 | | | ~2 fps reais, íntegro | |

### Leitura dos resultados
- A causa dos modos maiores é a **banda USB (~900 KB/s)**: quando os dados cabem, a imagem fica íntegra; quando não, o bridge corta o quadro. `[medido]`
- Consumo cru: 160x120 = 19,2 KB/quadro (30 fps = 576 KB/s, cabe); 320x240 = 76,8 KB (30 fps = 2,3 MB/s); 640x480 = 307 KB (30 fps = 9,2 MB/s).
- Estimativa do gspca para quadros comprimidos: 3/8 do tamanho cru. `[hipótese]`
- Janela do bridge: `0x15/0x16 = 0x16/0x12` (352x288) deformou todos os modos; `0x28/0x1E` corrigiu o 160x120. `[medido]`
- BA81 é Bayer 8 bits sem compressão (demosaico no aplicativo); S910 é decodificado pela libv4lconvert, não pelo kernel.

### Varredura automática `testes/avalia.py todos` (2026-10-11 20:09, 34 casos, julgamento seu)

Resultados completos em `testes/resultados_2026-10-11_todos1.csv`.

| Modo | `sensor_reg11` | FPS medido | Resultado | Observação |
|---|---|---|---|---|
| S910 160x120 | padrão | 30,8 | boa | |
| BA81 160x120 | padrão | 29,8 | boa | |
| S910 320x240 | 0 / 1 / 2 / 3 | 29,8* / 14,9 / 9,9 / 7,5 | boa em todos | *medido à mão; o FPS ficou em branco na 1ª versão do script (captura curta demais) |
| S910 640x480 | 0 (30 fps) | | ruim | quadro cortado: chegaram 4,7 KB, o decodificador pedia 39 KB |
| S910 640x480 | 1 (15 fps) | | ruim | quadros de ~55 KB, faltando 2 a 5 bytes |
| S910 640x480 | 2 (10 fps) | 9,9 | **boa** | estável nas 3 repetições (`lq_sel` 0/1 e clock Windows) |
| S910 640x480 | 3 (7,5 fps) | 7,4 | boa | |

Conclusões:
- **fps = 30/(n+1) confirmado:** 14,88, 9,92 e 7,52 fps medidos para n = 1, 2, 3.
- **640x480 comprimido:** o quadro tem cerca de **55 KB**; com ~900 KB/s o teto teórico é ~16 fps, então 15 fps fica na beira e perde bytes. **10 fps é o maior ritmo estável** (33% acima dos 7,5 fps do driver original).
- **`LQ_SEL` não mudou o tamanho do quadro:** 640x480 a 15 fps, 54.999 bytes com `lq_sel=0` e 55.074 com `lq_sel=1`. Hipótese não confirmada neste ajuste.
- **Clock do sensor:** só a combinação do Windows (`reg17=0x68`, `reg19=0x20`) funciona. Risolia (`0x60`/`0x50`): sem imagem em 320x240 e 640x480. Original (`0x6C`/`0x2B`): quadros inconsistentes e ~19 a 20 fps em vez de 30 (320x240, `reg11=0`). Não testado: essas combinações com `stock_sensor=1`/`stock_regs=1`.
- **Brilho sobe quando o FPS cai** (exposição maior): em 320x240 a média foi 13,7 / 19,2 / 25,2 / 32,6 para 30 / 15 / 10 / 7,5 fps.
- **Cobertura:** a varredura testou S910; do BA81 só o 160x120.

### Varredura 2 `testes/avalia.py todos` (2026-10-11 20:32, ganho/exposição aplicados ao iniciar)

Resultados em `testes/resultados.csv` (agora com a coluna `bytes_quadro`).

**Tamanho do quadro S910 `[medido]`:**

| Modo | Bytes por quadro | Observação |
|---|---|---|
| 160x120 | ~3,2 KB | 30 fps = ~96 KB/s |
| 320x240 | ~12,7 KB (12,5 a 13,0) | 30 fps = ~380 KB/s |
| 640x480 a 10 e 7,5 fps | **60,4 a 61,6 KB** | |
| 640x480 a 15 fps | 55,0 a 55,9 KB, **cortado** (~8% a menos) | 60,5 KB × 15 = ~907 KB/s, acima dos 900 KB/s do alt 7 |

- **`LQ_SEL` sem efeito de novo:** 320x240 12.752 × 12.764 bytes; 640x480 a 10 fps 61.632 × 60.646 (diferença de ~1,6%, dentro da variação entre quadros).
- **Pista nova:** o driver usa o alt 7 (900 KB/s) e pula o alt 8 (1.003 KB/s) pela regra do áudio USB. Com o alt 8, 640x480 a 15 fps (907 KB/s) caberia. `[hipótese]`
- **Clock `0x6C`/`0x2B` (original):** FPS ~19,5 a 20,8 tanto com `reg11=0` quanto com `reg11=3`; com `reg19` bit 0 (SYNC_OUT) ligado, quem manda no ritmo é o bridge, não o divisor do sensor. Quadros inconsistentes e brilho alto/ruído (média 36 a 69).
- **Mesmos resultados da varredura 1** para as demais combinações (Windows `0x68`/`0x20` funciona; Risolia `0x60`/`0x50` sem imagem).
- **FPS medido sem lacunas** (captura de 2 s ou mais): 29,8 / 15,0 / 10,0 / 7,5 em 320x240.

**Ganho e exposição ao iniciar `[medido]`:** funcionam e os controles respondem durante a captura (BA81 160x120, 30 fps): ganho 0 ≈ 7, ganho 31 ≈ 11, ganho 47 ≈ 17, exposição 5 ≈ 5 (brilho médio em 255). Com a exposição fixa em 66 ms, o brilho não sobe mais quando o FPS cai (320x240: 12,7 / 11,7 / 8,6 / 7,6 para 30 / 15 / 10 / 7,5 fps). A cena é escura (~4% do fundo de escala); a 30 fps a exposição já está no máximo do tempo de quadro (33 ms).

### Teste do alt 8 (`top_alt=1`, 2026-10-11 20:52)

Resultados em `testes/resultados_2026-10-11_todos2_alt8.csv` (4 últimas linhas).

| Caso (S910) | FPS | Bytes do quadro | Resultado |
|---|---|---|---|
| 640x480 `reg11=1` (15 fps) | 14,9 | 60.701 (completo; antes ~55.000, cortado) | **ruim** |
| 640x480 `reg11=2` (10 fps) | 10,1 | 60.366 | boa |
| 640x480 `reg11=0` (30 fps) | 29,7 | 4.036 (cortado) | ruim |
| 320x240 `reg11=0` (30 fps) | 29,8 | 13.225 | boa |

- **O alt 8 foi aceito:** nenhuma mensagem "not wide enough" no `dmesg` e a mensagem "skipping highest alt" não aparece com `top_alt=1`. O corte do quadro a 15 fps desapareceu (60,7 KB, o mesmo tamanho de 10 fps e o decodificador consome todos os bytes).
- **Mesmo assim o 15 fps continua "ruim".** Nas imagens salvas (`testes/imagens/`), os casos de 15 fps têm uma **rampa de brilho** forte (faixas verticais, esquerda para direita: 152, 110, 71, 43, 25, 22, 30, 32; faixas horizontais, de cima para baixo: 45 a 107), que os de 10 fps não têm (83, 107, 69, 54, 39, 39, 55, 59). Não há cisalhamento (0% das linhas deslocadas). O padrão lembra **deriva acumulada de decodificação**, com erro que cresce pela coluna da esquerda e se espalha pela média esquerda/acima. `[hipótese]`
- Com o alt 8 a causa deixa de ser banda USB; sobra o modo do bridge a 15 fps (o comentário do kernel fala de um "modo de compressão maior ainda não decodificado" abaixo de `reg11 = 4`).
- **Estatística dos códigos de Huffman** (S910 160x120, 30 fps, quadro bom): `+0` 85,4%, `+4` 8,4%, `-4` 4,8%, `+11` 0,6%, `-11` 0,7%, `+20` 0,1%, `-20` 0,1%, `abs` 0,0%. O avaliador agora grava essa contagem e o quadro bruto de cada caso (`testes/brutos/`) para comparar o 15 fps com o 10 fps offline.

### Teste do alt 8 com cena estática (lâmpada, 2026-10-11 21:11) e a causa do 15 fps "ruim"

Resultados em `testes/resultados_2026-10-11_alt8_lampada.csv`; quadros brutos em `testes/brutos/`.

**Correção:** a análise anterior ("rampa de brilho = deriva de decodificação") comparava imagens de **cenas diferentes** (a câmera apontava para a TV, que muda a cada quadro). Está descartada. Com a câmera na lâmpada (cena estática) as comparações valem.

| Caso (S910 640x480, alt 8) | Bytes | Consumidos pelo decodificador | Sobra | Resultado |
|---|---|---|---|---|
| 15 fps (#1) | 62.081 | 62.069 | **12 B** | ruim |
| 15 fps (#2) | 61.912 | 61.900 | **12 B** | ruim |
| 10 fps | 65.416 | 65.415 | 0,6 B | boa |
| 7,5 fps | 65.310 | 65.310 | 0,1 B | boa |

- **Deslocamento horizontal crescente (cisalhamento):** comparando com o quadro de 10 fps, o de 15 fps desloca para a direita de ~0 a 4 px no topo até **~60 a 77 px no fim** (0,13 a 0,16 px por linha, acelerando depois da linha ~150). O de 7,5 fps (controle) fica em 0 a 2 px. É a "deformação sem sincronismo" de 15 fps.
- **Os 12 bytes que sobram batem com o deslocamento:** 12 bytes × 8 bits ÷ ~1,6 bit por pixel ≈ 60 pixels. O fluxo a 15 fps tem **~60 pixels a mais** que 640×480, distribuídos ao longo das linhas; o decodificador lê 640 por linha e vai acumulando o deslocamento. Hipótese: o bridge emite linhas ligeiramente mais longas (ou o sensor entrega pixels extras) quando o sensor roda a 15 fps. `[hipótese]`
- **Não é a tabela de códigos:** variar o valor de `+4`/`-4` só piora (o padrão 4/4 é o melhor). **Não é banda USB:** o quadro chega inteiro e sem marcador de início no meio.
- **Candidatos a investigar** (registradores que definem a geometria da linha): `0x12`/`0x13` (hstart/vstart), `0x14`, `0x1A`/`0x1B` (tamanho "original" do sensor, hoje 0x14/0x0F), `0x17`/`0x19` (clock e sincronismo), e no sensor `0x11`, `0x28` (COMH) e `0x2D` (COMJ). Métrica objetiva: `sobra_bytes` no CSV (0 = limpo; ~12 = cisalhamento a 15 fps).
- **Faixas claras e escuras sutis** nas imagens boas de 640x480: provável *flicker* da lâmpada (rede de 60 Hz); testar `v4l2-ctl -c power_line_frequency=2`. `[hipótese]`

### Teste `geom15` (2026-10-11 21:38): geometria da linha não resolve o 15 fps

S910 640x480, `sensor_reg11=1`, alt 8, com `regs_start`. Todos os 9 casos foram julgados **ruins**.

| `regs_start` | `sobra_bytes` |
|---|---|
| (nenhum) | 7,4 (nas rodadas anteriores: 12) |
| `b:0x1a=0x1d,b:0x1b=0x10` (Windows/Risolia) | 5,2 |
| acima + janela de AE `02 03 0F 0C` | 6,9 |
| `b:0x1a=0x1d` / `b:0x1b=0x10` | 7,4 / 7,2 |
| `b:0x12=0x00` / `b:0x12=0x02` (hstart) | 7,2 / 6,2 |
| `b:0x13=0x00` / `b:0x13=0x02` (vstart) | **−76,5 / −75,1** (quadro mais curto que o esperado; imagem pior) |

- `0x1A`/`0x1B`, `0x12` e a janela de AE **não** corrigem o problema; as diferenças de `sobra` (5 a 7) ficam dentro da variação entre quadros (o mesmo caso deu 12 B antes). `0x13` (vstart) só piora.
- A métrica "deslocamento por faixa" não pôde ser usada nesta rodada: a câmera mudou de posição em relação ao quadro de referência de 10 fps.
- **Nova hipótese:** o 15 fps usa ~93% do alt 8 (62 KB × 15 = ~930 KB/s de 1003 KB/s); perto da saturação o FIFO de 2 KB do bridge pode inserir dados extras. Teste: baixar o ritmo sem trocar o divisor, com o ajuste fino de taxa do sensor (`s:0x2A` bit 7 liga, `s:0x2B` = byte baixo; cada passo diminui a taxa 0,12%; o Windows termina com `0x2A=0x80`, `0x2B=0x60`). Ex.: `s:0x2b=0xA7` (167 passos, −20%) leva 15 fps para ~12 fps. `[datasheet OV7630]`
- Outros candidatos ainda não testados: `b:0x18` nibble baixo (polaridade/borda de PCK, HSYNC e VSYNC; hoje 0xF) e bits do `0x17`/`0x19`.

### Ajuste fino de taxa `s:0x2b` (2026-10-11 22:02): o 640x480 fica limpo a ~13,4 fps

S910 640x480, `sensor_reg11=1` (15 fps), alt 8, `regs_start=s:0x2b=VALOR` (`0x2A` já vem com bit 7 ligado da init). **Os 7 casos foram julgados bons.**

| `s:0x2b` | FPS medido | Bytes do quadro | `sobra_bytes` | Resultado |
|---|---|---|---|---|
| 0x60 | **13,39** | 64.235 | 0,6 | boa |
| 0x70 | 13,16 | 64.344 | 0,1 | boa |
| 0x80 | 12,89 | 64.573 | 0,9 | boa |
| 0x90 | 12,67 | 64.785 | 0,4 | boa |
| 0xa0 | 12,40 | 64.978 | 0,0 | boa |
| 0xb0 | 12,67* | 65.028 | 0,9 | boa |
| 0xc0 | 11,97 | 65.379 | 0,0 | boa |

*valor fora da tendência (provavelmente erro da medição).

- **Hipótese confirmada:** o cisalhamento a 15 fps vem da **saturação do canal**. A 15 fps o quadro de 640x480 usa ~92% do alt 8 (61,5 KB × 14,9 = ~915 KB/s de 1003 KB/s); a 13,4 fps usa ~86% (64,2 KB × 13,4 = ~860 KB/s) e fica limpo; a 10 fps usa ~62%.
- **Ajuste fino do sensor:** `s:0x2b` (byte baixo de 10 bits; `0x2A[7]` liga, `0x2A[6:5]` são os bits altos). Na prática cada passo diminui a taxa em ~0,10% (o datasheet diz 0,12%): 0x60 = −10%, 0xc0 = −20%. O valor 0x60 é o mesmo que o Windows grava no fim da init.
- **Limite entre 86% e 92% de ocupação:** falta achar o ponto exato entre 13,4 e 14,9 fps (`--varre s:0x2b 0x10 0x50 0x10`).
- O quadro fica maior quanto menor a taxa (64,2 a 65,4 KB), enquanto a 15 fps eram 61,5 KB: a diferença de ~4% é consistente com dados que se perdiam/duplicavam na saturação.
- Atenção: o `setfreq` (controle de frequência da rede) também escreve o `0x2B`; ao usar o ajuste fino ele deve ser reaplicado depois.

### Varredura fina de `s:0x2b` de 0x10 a 0x50 (2026-10-11 22:07): todos bons

S910 640x480, `sensor_reg11=1`, alt 8. **Os 5 casos foram julgados bons.**

| `s:0x2b` | FPS medido | Bytes | `sobra_bytes` | Ocupação do alt 8 (bytes × fps) |
|---|---|---|---|---|
| 0x10 | 14,88 | 62.921 | 2,2 (marginal) | 93% |
| 0x20 | 14,39 | 63.307 | 0,8 | 91% |
| 0x30 | 14,62 | 63.550 | 0,6 | 93% |
| 0x40 | 13,89 | 63.800 | 0,4 | 88% |
| 0x50 | 13,65 | 63.953 | 0,4 | 87% |

- O limite fica em torno de **14,5 fps**; 14,9 fps (`0x10`) já é marginal (sobra 2,2 B). A medição de fps tem ±0,2 (0x30 deu mais que 0x20).
- O tamanho do quadro depende da cena; uma cena mais complexa ou ruidosa pede mais banda e deve saturá-lo a taxas menores. Por isso o driver anuncia 14 e 13 fps, mas usa **10 fps como padrão**.

### Ajuste fino implementado no driver (2026-10-11 22:08)

- `apply_fps` agora escreve `reg 0x2A` (bit 7 liga; bits 6:5 = 2 bits altos de N) e `reg 0x2B` (byte baixo de N) quando a taxa pedida fica abaixo da base `30/(n+1)`; N = (base − fps) × 952 / base (cada passo ≈ 0,105%). Sem ajuste fino, reaplica o `setfreq` (devolve o `0x2B` ao controle de frequência da rede). O controle de frequência da rede também chama `apply_fps` depois do `setfreq`.
- S910 640x480 anuncia **14, 13, 10, 8** fps (14 e 13 com `n=1` e N ≈ 63 e 126); padrão 10 fps.
- `top_alt` agora vale **1 por padrão** (usa o alt 8); `0` volta à regra do áudio.
- Avaliador: `DEFAULTS` com `top_alt=1`; conjunto `taxas` pede as taxas anunciadas via `S_PARM` (`--set-parm`), como um aplicativo faria.

### Teste `taxas` (2026-10-11 22:17): as taxas anunciadas funcionam pelo `S_PARM`

Taxas pedidas como um aplicativo pediria (`v4l2-ctl --set-parm`), sem `sensor_reg11`. **Os 8 casos foram julgados bons.**

| Caso | Pedido | Medido | Bytes por quadro | `sobra_bytes` |
|---|---|---|---|---|
| S910 640x480 | 14 | 14,38 | 63.656 | 0,8 |
| S910 640x480 | 13 | 13,39 | 64.226 | 0,9 |
| S910 640x480 | 10 | 10,06 | 63.033 | 0,8 |
| S910 640x480 | 8 | 7,52 | 63.115 | 0,4 |
| S910 320x240 | 30 | 30,75 | 13.737 | 0,8 |
| S910 320x240 | 15 | 15,03 | 13.861 | 0,6 |
| S910 160x120 | 30 | 30,76 | 3.682 | 0,8 |
| BA81 160x120 | 30 | 30,26 | 19.200 | |

- O `dmesg` confirma o caminho: `apply_fps fps=14 ... fino=...` para os de 640x480 e `fino=0` nos demais.
- Sem mensagens "not wide enough", "skipping highest alt", Oops ou BUG.
- Estado final: S910 640x480 saiu de **7,5 fps (driver original) para 14,4 fps**; 320x240 chega a 30 fps; 160x120 a 30 fps.
- **Alt settings da câmera** (`lsusb -v`, interface 0): 0, 128, 256, 384, 512, 680, 800, 900 e **1003** bytes por ms. Não existe alt 9; o alt 8 é o teto (USB 1.1 full-speed, máximo isosíncrono de 1023 bytes por ms). Interfaces 1 e 2 são de áudio (microfone embutido).

### Nomes da ponte e do sensor e ganchos V4L2 padrão (2026-10-11 22:40)

- **`VIDIOC_QUERYCAP`:** o campo `card` passa a ser **"Elgin camera (SN9C103 + OV7630)"** (31 caracteres, o máximo do campo; "USB elgin camera (...)" não cabe). A string do próprio dispositivo (`iProduct`) é "USB camera"; o nome do `lsusb` ("Microdia PC Camera (SN9C103 + OV7630)") vem do banco `usb.ids`, não do dispositivo nem do driver. **`VIDIOC_ENUMINPUT`:** o nome da entrada passa a ser "SN9C103 + OV7630". O sensor continua vindo da tabela de IDs USB (`driver_info`), não é sondado; o `PID` do OV7630 (`0x0A`) vale 0x76 e o `VER` (`0x0B`) 0x30 pelo datasheet, e `regr` permite conferir.
- **`VIDIOC_DBG_G_CHIP_INFO` / `VIDIOC_DBG_G_REGISTER` / `VIDIOC_DBG_S_REGISTER`:** implementados em `sd_chip_info`, `sd_dbg_g_register` e `sd_dbg_s_register` sob `#ifdef CONFIG_VIDEO_ADV_DEBUG`. Chip 0 = ponte ("SN9C103"), chip 1 = sensor ("OV7630"); registradores de 8 bits (0 a 0xFF). Uso, em kernel com a opção ligada e como root: `v4l2-dbg -d /dev/videoN --list-chips` e `v4l2-dbg -d /dev/videoN -c 1 -g 0x0a`.
- **Neste kernel** (`# CONFIG_VIDEO_ADV_DEBUG is not set`) esse código não é compilado no módulo; o equivalente que funciona aqui são `regw`, `regr` e `regs_start`. O código foi verificado compilando com a opção ligada (sem instalar).
- Fica para depois, se necessário: media controller (sensor como entidade `ov7630`).

### Controles V4L2: estado em 2026-10-11 22:50

`v4l2-ctl --list-ctrls` mostra 5 controles. Teste ao vivo (BA81 160x120, brilho médio por janela de 0,5 s):

| Controle | Faixa (padrão) | Registrador | Estado |
|---|---|---|---|
| `gain` | 0 a 47 (31) | sensor `0x00` | **funciona** (ganho 0 ≈ 7, 31 ≈ 11, 47 ≈ 17); fica *inactive* enquanto `gain_automatic=1` |
| `exposure` | 0 a 1023 (66 ms) | sensor `0x10` (e `0x11`) | **funciona** (exposição 5 ≈ 5); *inactive* com `gain_automatic=1` |
| `power_line_frequency` | 0, 1 (50 Hz), 2 (60 Hz) | sensor `0x2B` | **funciona**: 50 Hz subiu o brilho ~15% (taxa 30 → 25 fps); 60 Hz e desligado não ajustam. Nos modos com ajuste fino de taxa (13 e 14 fps) o `0x2B` é do ajuste fino |
| `brightness` | 0 a 255 (127) | sensor `0x06` | **sem efeito medido** em imagem Bayer: 0, 127 e 255 deram a mesma média (~27) |
| `gain_automatic` | 0/1 (1) | — | **incompleto**: o algoritmo de ganho automático está desligado no código (`do_autogain` em `#if(0)`); ligar não faz nada e, de quebra, deixa `gain` e `exposure` inativos por padrão |

O botão da câmera (endpoint de interrupção) já vira um dispositivo de entrada (`input11`). A taxa de quadros é tratada por `S_PARM`, não por controle.

**Atualização 2026-10-11 22:56 (ganho automático implementado, falta validar):**
- `gain_automatic` agora vale **0 por padrão** (manual), então `gain` e `exposure` ficam ativos; `v4l2-ctl -c gain_automatic=1` liga o automático.
- `do_autogain` foi reativado com um algoritmo novo, `elg_expo_autogain`: o mesmo "joelho" do gspca (ganho até o padrão, exposição até o joelho de 200, ganho até 90%, ...), mas **a exposição fica limitada ao tempo de quadro** (`67 × (n+1)` unidades de 0,5 ms) e **a taxa nunca é alterada**. Assim o automático não derruba mais o FPS, que era o defeito original.
- **O controle de brilho vira o alvo do ganho automático:** `alvo = brilho × 13000 / 127` (soma de luminosidade da janela de AE do cabeçalho do quadro). Sem o automático, o brilho não tem efeito visível, como medido.
- **Janela de AE restaurada** (`0x1C`–`0x1F` = `02 03 0F 0C`, os valores do Windows e do Risolia): com a janela zerada o `avg_lum` do cabeçalho não servia.
- Passos para fechar: validar o automático com a câmera (convergir ao alvo, sem oscilar, FPS intacto) e só então trocar o padrão para 1.

**Validação do ganho automático (2026-10-11 22:58, `testes/testa_autogain.py`, BA81 160x120 a 30 fps, câmera na lâmpada):**

| Momento | Ganho | Exposição | Média na janela de AE | FPS |
|---|---|---|---|---|
| Manual (padrão 31/66) | 31 | 66 | 33 | 30 |
| Automático ligado (alvo 127) | 42 | 67 | **51** (estável, sem oscilação) | 30 |
| Alvo 60 | 31 | 54 | 28 | 30 |
| Alvo 200 | 47 (máx.) | 67 (máx.) | 58 (satura: não dá mais ganho) | 30 |

- Converge em ~1 s e fica parado dentro da zona morta (±1500 do alvo de 13000, ~±7 níveis); o FPS ficou entre 29,5 e 30,0 o tempo todo. **Validado.** `gain_automatic` passou a valer **1 por padrão** (2026-10-11 23:00).
- Limite: com a taxa fixa, a exposição máxima é o tempo de quadro (33 ms a 30 fps); em cena muito escura sobra só o ganho. Ganho alto aumenta o ruído e, no S910, o tamanho do quadro comprimido (cuidado com 13 e 14 fps em 640x480).
- Ideia futura (V4L2 padrão): `V4L2_CID_EXPOSURE_AUTO_PRIORITY` ("exposure, dynamic framerate") para o usuário permitir que o automático baixe a taxa em cena escura (o comportamento original), desligado por padrão.
- Ao ligar o automático o driver volta ganho e exposição ao padrão (31/66) antes de convergir (comportamento herdado do gspca).

**Balanço de branco manual (implementado 2026-10-11 23:00, falta validar):** controles `red_balance` e `blue_balance` (0 a 127, padrão 32) = ganhos digitais R e B da ponte (regs `0x05` e `0x07`; o verde `0x06` fica em 0x20). Aplicados no `sd_start` e ao vivo. Teste: `python3 testes/testa_wb.py` (mede R, G, B e as razões R/G e B/G para cada valor).

**Balanço de branco: o mapeamento medido (2026-10-11 23:05, `testes/testa_wb.py`)** não é o que o gspca comenta ("R, G, B"). Média de cada posição do mosaico (linha, coluna) ao variar os registradores da ponte (padrão 0x20):

| Escrita | P00 | P01 | P10 | P11 | Conclusão |
|---|---|---|---|---|---|
| `0x05` = 0 / 32 / 127 | 27 | 21,5 | **16,7 / 31,8 / 69,7** | 26,5 | `0x05` age **só no pixel (1,0)** |
| `0x07` = 0 / 32 / 127 | **13,5 / 27 / 58,8** | 21,3 | 32,7 | **13,3 / 26 / 56,6** | `0x07` age **nos dois pixels da diagonal (0,0) e (1,1)** |
| `0x06` | (não medido) | | | | por exclusão, o pixel (0,1) |

- **O que é robusto (não depende da cena):** a ponte agrupa as posições assim: `0x05` → (1,0); `0x07` → (0,0) e (1,1) juntas. Isso não combina com BGGR (cuja diagonal principal tem B e R, cores diferentes), mas a ponte pode aplicar os ganhos pela fase que ela supõe, que não precisa ser a cor física do sensor.
- **Correção (23:08):** a identificação de R/B por "luz quente da lâmpada" **não tem base**: a câmera apontava direto para a lâmpada e para o teto branco, sem referência de cor. **Não se sabe** qual cor física está em cada posição, nem se o `BA81`/BGGR anunciado está errado. O mapeamento `red_balance` → `0x05` e `blue_balance` → `0x06` é **provisório**.
- **Como resolver:** `python3 testes/testa_cores.py` (cor sólida: branco, vermelho, verde, azul; compara as 4 posições dentro do mesmo quadro e deduz o padrão). Se a fase real não for BGGR, as opções são deslocar o mosaico com a paridade de `hstart` (`reg 0x12`, hoje 1 do Windows; o original usa 2) ou anunciar o formato certo; testes de geometria anteriores (`b:0x12=0x00/0x02`) não quebraram o sincronismo, e `b:0x13` (vstart) sim.

**Cor medida com tela azul (2026-10-11 23:17, última captura do `testa_cores.py`):** posições (0,0)=12,0, **(0,1)=34,1**, (1,0)=6,7, (1,1)=11,4 → o azul está em **(0,1)**, o vermelho (que menos responde ao azul) em **(1,0)** e os verdes na diagonal (0,0) e (1,1), com valores quase iguais. Como o azul fora de (0,0) só é compatível com **GBRG** entre os quatro padrões de Bayer, o mosaico entregue com `hstart = 1` era **GBRG**, e não BGGR como o `BA81` anuncia (cores trocadas nos aplicativos e na decodificação do S910). `[medido, com referência de cor; só o azul foi lido]`
- **Teste completo antes da correção (23:19, BA81 160x120, `hstart = 1`)** — médias por posição (0,0)/(0,1)/(1,0)/(1,1): branco 30,1/32,4/28,2/30,0; **vermelho 23,3/18,8/97,0/20,8**; **verde 52,6/29,1/16,4/52,4**; **azul 12,0/34,1/6,7/11,4**. Contraste 5,2× (R), 3,2× (G), 5,1× (B). Padrão medido: **GBRG** (`G B / R G`). `[medido, com as quatro cores]`
- Isso também confirma a ordem dos ganhos da ponte pela cor física: `0x05` = R (1,0), `0x06` = B (0,1) por exclusão, `0x07` = G (diagonal).
- **Correção aplicada:** `hstart` (`reg 0x12`) de 1 para **2** (um pixel à direita), o que leva GBRG a BGGR; `vstart` fica 1. O cálculo supondo o mosaico nativo GRBG: `hstart` par + `vstart` ímpar = BGGR. **A validar:** `python3 testes/testa_cores.py` de novo (agora grava `cores_resultado.txt`) nos três tamanhos, e conferir se o `testa_wb.py` ainda atua nos mesmos pixels depois do deslocamento.

**Validação da correção de cor (2026-10-11 23:24, `testes/testa_cores.py`, `hstart = 2`):** BGGR confirmado em **5 dos 6 modos**.

| Modo | Mosaico | Padrão | Contraste R/G/B |
|---|---|---|---|
| BA81 160x120 | B G / G R | **BGGR** | 5,0× 2,7× 6,3× |
| BA81 320x240 | inconclusivo (quadros cortados, ver abaixo) | — | 1,1× 1,0× 1,2× |
| BA81 640x480 | B G / G R | **BGGR** | 6,8× 3,2× 12,8× |
| S910 160x120 | B G / G R | **BGGR** | 4,7× 2,6× 4,8× |
| S910 320x240 | B G / G R | **BGGR** | 5,4× 2,7× 5,2× |
| S910 640x480 | B G / G R | **BGGR** | 5,4× 2,8× 5,4× |

A escala 1/2 e 1/4 da ponte preserva a fase, e o S910 (decodificado por `sn9c10x_decode.py`) também sai BGGR. `testa_cores.py` foi trocado para capturar quadros isolados (robusto a quadros cortados).

**Problema novo: BA81 320x240 a 10 fps entrega quadros CORTADOS (2026-10-11 23:30).** Quadros isolados têm 22.913 a 33.869 bytes (30% a 44% de 76.800), todos no formato 589 + 640×k bytes; o conteúdo é imagem normal que termina abruptamente (sem marcador de início dentro). Com `--set-parm=8` (7,5 fps) e 5 fps os quadros vêm **cheios**; os modos BA81 160x120 e 640x480 (2 fps) e todo o S910 estão bons. Não depende de ganho, exposição ou ganho automático (testado ligado e desligado, ganho 0 a 47). Na varredura inicial (alt 7, antes das mudanças de hoje) o BA81 320x240 a 10 fps estava “perfeito”; mudaram desde então `top_alt=1` (alt 8), `hstart` 1→2, janela de AE ligada, ganho/exposição ao iniciar e o ajuste fino de taxa. Para isolar: `python3 testes/testa_raw320.py` (varre essas configurações e mede o tamanho dos quadros).

**Causa dos quadros cortados do BA81 320x240 (2026-10-11 23:40, `testes/testa_raw320.py`):** o corte persistiu em **todas** as configurações testadas a 10 fps (alt 7, `hstart` 1, janela de AE zerada, "estado de antes"), e a 8 fps os quadros sempre vieram cheios. Os quadros têm tamanho `589 + 640×k` (a ponte descarta pares de linhas inteiros). O que mudou hoje não foi nenhum registrador, e sim **a escolha do alt pelo núcleo**: com `get_streamparm` implementado, `which_bandwidth()` passou a usar a taxa real (`sizeimage × fps`) e escolhe o **menor alt que comporta exatamente o fluxo**. BA81 320x240 a 10 fps = 768.000 B/s → alt 6 (800.000) → **96% de ocupação** → corte. Antes, sem `get_streamparm`, o núcleo supunha 30 fps (2,3 MB/s) e pegava sempre o alt mais alto. Os demais modos BA81 nas taxas anunciadas ficam entre 56% e 85% do alt escolhido e estavam bons (160x120: 30, 15, 10, 8 fps; 320x240 a 8 fps; 640x480 a 2 fps); o S910 sempre cai no alt mais alto. **O fluxo cru perde linhas perto de 96%; o comprimido aguentou ~91% do alt 8.**
- **Correção:** `which_bandwidth()` ganhou uma **margem de 30%** quando a taxa vem do `get_streamparm` (BA81 320x240 a 10 fps pede 998.400 → alt 8, 77% de ocupação). Validação: quadros isolados de todos os modos BA81 nas taxas anunciadas (script em linha de comando no histórico; ver `testa_raw320.py` para o 320x240).

**Validação da margem de 30% (2026-10-11 23:41):** BA81, quadros isolados, 6 por taxa: 160x120 a 30, 15, 10 e 8 fps, 320x240 a 10 e 8 fps e 640x480 a 2 fps — **todos cheios (42 de 42)**.

**Achado: o S910 em cena escura infla o quadro e satura o canal (2026-10-11 23:42).** Com a câmera num cenário muito escuro (brilho médio 8 de 255) o ganho automático vai ao máximo (ganho 47) e o quadro comprimido de 640x480 passa de ~60 KB para **92 a 104 KB**. A 10 fps isso dá ~920 KB/s (92% do alt 8) e aparece a sobra de 18 a 24 bytes (cisalhamento); 8 fps (800 KB/s) e 13 fps com quadros de 59 KB ficam limpos; 320x240 a 15 fps teve um quadro com sobra de 39,9 B. Ou seja, **as taxas anunciadas do S910 só valem com a cena em nível normal de ruído**. Opções: limitar o ganho máximo do automático; **trava de banda** que baixa a taxa quando o tamanho médio dos quadros × taxa passa de ~88% do alt (o driver conhece o tamanho de cada quadro); ou apenas documentar.

**Trava de banda do S910 como controle (implementada 2026-10-11 23:46, falta validar):** controle V4L2 padrão `V4L2_CID_EXPOSURE_AUTO_PRIORITY` ("Auto Exposure, Dynamic Framerate", `exposure_dynamic_framerate`, 0/1, **padrão 1**). Ligado, o driver baixa a taxa sozinho quando o quadro comprimido infla (cena escura, ganho alto); desligado, a taxa pedida fica fixa.
- `sd_pkt_scan` guarda o tamanho de cada quadro (`image_len` ao fechar o quadro); `rate_guard()` roda em `do_autogain()` (contexto de processo, com o `usb_lock`), só para o S910, e mantém uma média móvel (1/16). Ocupação = tamanho médio × taxa real ÷ banda do alt em uso (`wMaxPacketSize × 1000`, lida no `sd_start`).
- **Desce um degrau da tabela** de taxas do modo quando a ocupação fica acima de **90%** por 8 avaliações seguidas; **sobe um degrau** (até a taxa pedida) quando a previsão no degrau acima fica abaixo de **86%** por 48 avaliações. O `dmesg` registra "trava de banda: ocupação X% -> N fps".
- Limites calibrados nos testes: 91% do alt 8 ainda limpo, 93% marginal, ~92% com quadros de 92 KB já desliza. `G_PARM` informa a taxa efetiva (já com o limite). Pedir uma taxa nova por `S_PARM`, mudar de modo ou reiniciar a captura apaga o limite.
- Teste: `python3 testes/testa_guarda.py` (pede 14 fps no S910 640x480, acompanha a taxa informada e compara quadros com a trava ligada e desligada).

**Resultado da trava de banda v1 (2026-10-11 23:55, `testes/testa_guarda.py`):** com a trava ligada a taxa informada foi 14, 14, 14, 13, 13, **10 (7 s)**, 8, e os quadros após "assentar" saíram **0 de 4 limpos** (92 KB, sobra 37 a 54 B); com a trava desligada (14 fps fixos) 3 de 4 limpos, e o quadro de 67 KB deslizou (sobra 10,8) enquanto os de 60 KB ficaram limpos. dmesg: 94,0% → 13 fps, 91,6% → 10 fps. **Defeitos:** (1) a média móvel suaviza os quadros grandes, que são os que deformam; (2) exigir 8 avaliações consecutivas acima de 90% quase nunca ocorria com a ocupação oscilando em ~90%; (3) o limite era apagado a cada nova captura (cada `S_PARM`), então o aplicativo voltava a deformar por alguns segundos.

**Trava de banda v2 (2026-10-11 23:58, falta validar):** a decisão usa o **máximo recente** do tamanho dos quadros (decai ~1,5% por quadro) em vez da média; contador que sobe a cada avaliação acima de 90% e desce 1 a cada uma abaixo (6 pontos para descer um degrau); sobe de volta quando a previsão do pico no degrau acima fica < 80% por 48 avaliações; **o limite persiste entre as capturas** (some ao mudar de modo ou desligar o controle).

**Validação da trava de banda v2 (2026-10-11 00:00, `testes/testa_guarda.py`, cena escura, S910 640x480 pedindo 14 fps):**

| Fase | Taxa informada (1 por segundo) | Quadros após assentar |
|---|---|---|
| Trava **ligada** | 14, 14, 13, 10, **8** (estável em 8 a partir do 5º segundo) | **4 de 4 limpos** (sobra 0,0 a 0,6; quadros de 72 a 110 KB) |
| Trava **desligada** (14 fps fixos) | 14 o tempo todo | 3 de 4 limpos (um quadro de 67 KB com sobra 7,5) |

- dmesg: pico 93,0% → 13 fps; 94,2% → 10 fps; 92,3% → 8 fps; depois "folga -> 10 fps" e **de novo** 92,1% → 8 fps (**oscilação**: o pico decaiu rápido demais, a trava subiu e caiu de volta em ~3 s).
- Ajuste da oscilação (2026-10-12): decaimento do pico de 1/64 para **1/256 por quadro** (memória de ~20 s) e subida exigindo **96** avaliações em vez de 48. Falta recarregar e observar o dmesg.
- A trava, com tamanho de quadro oscilando entre 72 e 110 KB na cena escura (o ganho automático muda o ruído), resolveu o essencial: nenhum quadro deformado depois que ela assenta.

**Balanço de branco depois do `hstart = 2` (2026-10-12 00:03, `testes/testa_wb.py`): os ganhos da ponte são por POSIÇÃO.** Com o mosaico em BGGR, `red_balance` (reg `0x05`) continuou agindo só no pixel (1,0) (4,7 → 22,7) e `blue_balance` (reg `0x06`) só em (0,1) (5,2 → 25,2); os pixels B em (0,0) e R em (1,1) não se moveram (7,5–7,8 e 10,8–11,0). Ou seja, a ponte aplica os ganhos supondo sempre a fase **GBRG** (0x05 em (1,0), 0x06 em (0,1), 0x07 na diagonal), independentemente de `hstart`/`vstart`. Em BGGR esses dois pixels são os **dois verdes**, e o `0x07` mexe em B e R juntos. Não dá para ter BGGR (exigido pela libv4lconvert no S910) e balanço de branco independente pela ponte.

**Balanço de branco pelos ganhos do SENSOR (implementado 2026-10-12 00:04, falta validar):** datasheet OV7630: `0x01` BLUE e `0x02` RED (8 bits, padrão 0x80), ganhos de cor que agem na cor física; também existem `0x03` SAT, `0x04` HUE, `0x05` CNT e `0x06` BRT (brilho, padrão 0x80), mas esses só valem na imagem processada (YUV), o que explica o `brightness` sem efeito na Bayer crua. `red_balance` e `blue_balance` agora escrevem o sensor (`0x02` e `0x01`, faixa 0–255, padrões 0x3A e 0x77 da init do Windows) por uma escrita I²C de 2 bytes. Teste: `python3 testes/testa_wb.py` (agora com os valores do sensor): R em (1,1) e B em (0,0) devem responder. Se a Bayer crua ignorar esses ganhos, a alternativa é o `0x07` da ponte (R e B juntos) ou fazer o balanço na aplicação.

**Validação do balanço de branco pelo sensor (2026-10-12 00:10, `testes/testa_wb.py`, BGGR):** cada controle age só na cor certa e as outras ficam paradas.

| Controle (reg do sensor) | Valor | R (1,1) | G (média) | B (0,0) |
|---|---|---|---|---|
| `red_balance` (`0x02`) | 0 / 58 / 100 / 160 / 255 | **9,0 / 10,7 / 12,6 / 15,9 / 20,7** | 10,0 | 7,5–7,7 |
| `blue_balance` (`0x01`) | 0 / 60 / 119 / 180 / 255 | 10,8–11,0 | 10,2 | **5,5 / 6,3 / 7,7 / 9,5 / 11,8** |

Ganho aproximadamente linear (255 ≈ 1,9× a 2,1× o valor padrão). Padrões 58 (vermelho) e 119 (azul) deixam R/G = 1,07 e B/G = 0,76 nesta cena escura. **Validado.** O `brightness` continua sem efeito na imagem crua por ser ajuste da imagem processada; vale como alvo do ganho automático.

**Validação do ajuste da trava de banda (2026-10-12 00:15, `testa_guarda.py`):** com a memória de ~20 s a taxa foi 14, 14, 13, 10, 10, 8 e ficou em 8; **4 de 4 quadros limpos** (sobra 0,5 a 0,9); **uma única sequência de descida** no dmesg (90,4% → 13, 94,4% → 10, 91,0% → 8) e **nenhum** "folga" seguido de nova descida: a oscilação sumiu. Trava desligada: 3 de 4 limpos (o quadro de 67 KB deslizou, sobra 8,9).

**60 fps no S910 160x120: plano de teste (2026-10-12 00:20, `testes/testa_60fps.py`).** Etapas por `regs_start`: A referência; B só `s:0x2d=0xc1` (COMJ bit 6, só o campo ímpar, até 60 fps); C só `s:0x14=0xa0` (sensor em QVGA); D QVGA + janela da ponte 320x240 e escala 1/2 (`b:0x15=0x14,b:0x16=0x0f,b:0x18=0x9f`), para validar o caminho a 30 fps; E QVGA + COMJ; **F tudo junto (alvo 60 fps)**. Mede FPS real, sobra de bytes, brilho e correlação com a referência (cena parada) e guarda PNGs em `testes/60fps/`. Banda: 160x120 comprimido a 60 fps = ~3,8 KB × 60 = ~230 KB/s, folgado.

**60 fps, etapa 1 (2026-10-12 00:19, `60fps_resultado_etapa1.txt`):** S910 160x120 pedindo 30 fps. A referência 29,92 fps, quadro limpo (sobra 0,9). **B** só `s:0x2d=0xc1`: 29,92 fps, sem efeito (corr 1,00). **C** só `s:0x14=0xa0` (QVGA): quadro quebrado (1.729 B, sobra −1.447, corr 0,15) porque a ponte ainda espera 640x480. **D** QVGA + janela 320x240 e escala 1/2 (`b:0x15=0x14,b:0x16=0x0f,b:0x18=0x9f`): **limpo** (3.839 B, sobra 0,9), correlação 0,90 com a referência (mesmo campo de visão; brilho caiu de 43,6 para 25,9) a 29,92 fps: **o caminho QVGA funciona**. **E** QVGA + COMJ sem janela: quebrado. **F** tudo junto: limpo, mas **29,92 fps**: o COMJ bit 6 isolado não dobra a taxa.

**Datasheet OV7630 (leitura 2026-10-12):** `COMJ[6]` (0x2D) = 1 só dá até 60 f/s "read format as interlaced"; **`COMH[5]` (0x28) = 0 é entrelaçado, 1 progressivo** (o Windows grava `0xA0`: uma linha + progressivo); `COMH[7]` = 1 é a saída raw de uma linha que a ponte exige. A anotação antiga "desabilitado produz FPS=60" bate: sem a escrita o `COMH` volta a `0x00` (entrelaçado, mas duas linhas). Etapa 2 testa `COMH = 0x80` (uma linha + entrelaçado). Outros achados do mesmo datasheet: `COMA` (0x12) bit 6 = **espelho** (o Windows grava `0x78`, bit 6 ligado), bit 2 = AWB, bit 1 = **barra de cores de teste**; `FSD` (0x16) bits 1:0 = modo do HREF por campo; janela do sensor em `0x17`–`0x1A`; `COMB[1]`/`[0]` = AGC/AEC internos.

**60 fps, etapa 2 (2026-10-12 00:24, `60fps_resultado_etapa2.txt`): FUNCIONA.** S910 160x120 pedindo 30 fps, taxa medida pelo `v4l2-ctl`:

| Caso | Registradores (`regs_start`) | FPS | Quadro (sobra) | Corr. | Brilho |
|---|---|---|---|---|---|
| A referência | — | 29,92 | limpo (0,1) | 1,00 | 43,3 |
| D QVGA + janela | `s:0x14=0xa0` + `b:0x15=0x14,b:0x16=0x0f,b:0x18=0x9f` | 29,92 | limpo (0,4) | 0,95 | 43,7 |
| **G** | D + `s:0x2d=0xc1` + `s:0x28=0x80` | **59,53** | limpo (0,5) | 0,88 | 21,5 |
| **H** | D + `s:0x28=0x80` (sem COMJ) | **59,52** | limpo (0,0) | 0,92 | 24,6 |
| I | só `s:0x28=0x80` (VGA) | 59,52 | **quebrado** (−1.199,9) | 0,26 | 34,5 |
| J | D + COMJ + `s:0x28=0x00` (duas linhas) | 59,53 | decodifica (0,5) | 0,97 | 24,1 |

- **A chave é `COMH` (0x28) = 0x80 (entrelaçado, uma linha)** com o sensor em QVGA (`COMC` bit 5). Em VGA (I) o entrelaçado entrega só um campo (640x240) e o quadro fica pela metade. O `COMJ` bit 6 (só o campo ímpar) não foi necessário para a taxa; mantido por seguir o datasheet (único campo = posição vertical estável). A queda de brilho (43 → 21–25) se deve ao tempo de quadro de 16,7 ms e ao ganho automático ainda assentando.
- **Implementado no driver (2026-10-12 00:28, falta validar):** `fps_s910_160 = {60, 30, 15, 10, 8}` só no S910 160x120 (padrão continua 30). Com 60 fps: ponte `0x15=0x14`, `0x16=0x0f`, `0x18` com escala 1/2 e janela de AE reduzida (`1c..1f = 01 02 07 06`); sensor `0x14=0xa0`, `0x28=0x80`, `0x2d=0xc1` logo após a init. `sd->hi` fixa o modo no `sd_start`; a exposição máxima e o alvo/zona morta do ganho automático acompanham (tempo de quadro ½; área da janela de AE); a trava de banda usa a taxa real (×2). Pedir 60 fps por `S_PARM` com a captura ligada (ou sair deles) não muda a taxa: exige reiniciar a captura. Teste de cor: `FPS60=1 python3 testes/testa_cores.py S910:160`.

**Hardware sem controle ainda:** ganhos R, G e B da ponte (regs `0x05`–`0x07`, balanço de branco), curva gama (`0x21`–`0x30`), espelhamento do sensor, padrão de teste da ponte (`0x17` bit 4, `TEST_IMG`), deslocamento de preto (`0x14`, OFFSET) e janela/escala por hardware (zoom e panorâmica por `S_CROP`/`S_SELECTION`).

### Taxas anunciadas hoje (tabela por modo)

| Modo | Taxas | Padrão |
|---|---|---|
| BA81 160x120 | 30, 15, 10, 8 | 30 |
| BA81 320x240 | 10, 8 | 10 |
| BA81 640x480 | 2 | 2 |
| S910 160x120 | 30, 15, 10, 8 | 30 |
| S910 320x240 | 30, 15, 10, 8 | 15 |
| S910 640x480 | 14, 13, 10, 8 | 10 |

"8" = 7,5 fps real (o gspca só anuncia inteiros). O 60 fps do 160x120 foi retirado até ser implementado.

## 5. Captura do Windows: o que ela é

- Trecho de uma captura maior (as entradas começam em 010806). Mostra o fim de uma sessão (22:13) e o **início completo da seguinte** (22:15:27), terminando cerca de 1 s depois do início do streaming. Não há tráfego em regime.
- Modo capturado: **CIF 352x288, compressão ligada, sem escala, alt 8, divisor 0 (30 fps)**.
- Atenção: em conversas anteriores esta captura foi descrita como 640x480 com `0x18 = 0x9F`; isso estava **errado**. Os valores corretos estão na tabela da seção 2.
- Consequência: o Windows tem um modo CIF (352x288) que a lista de modos do Linux não tem.

## 6. Parâmetros do módulo e como testar

`/sys/module/gspca_elgin_fps/parameters/` (valem no próximo início de captura):

| Parâmetro | Efeito |
|---|---|
| `sensor_reg11=N` | Força o `reg 0x11` do sensor (fps = 30/(N+1)); `-1` = usa a tabela de taxas |
| `stock_regs=1` | Bloco de registradores do bridge original (0 = Windows) |
| `stock_sensor=1` | Init do sensor original (0 = Windows) |
| `reg17=V` | Força o `reg 0x17` do bridge (clock do sensor + `LQ_SEL`); `-1` = não altera. Aceita hex: `0x60` |
| `reg19=V` | Força o `reg 0x19` do bridge (modo de sync / MCK_SIZE); `-1` = não altera |
| `top_alt=1` | Não pula o alt mais alto (1003 KB/s) mesmo com áudio USB; 0 = padrão do gspca (usa o alt 7, 900 KB/s). Valia a partir de 2026-10-11 20:44 |
| `lq_sel=0/1` | Só o bit 0 do `reg 0x17` (1 = compressão de baixa qualidade); `-1` = não altera. Vale depois do `reg17` |

### Acesso genérico a registradores (implementado em 2026-10-11 21:24)

Item: `TIPO:REG=VALOR`, com `b` = ponte SN9C103 e `s` = sensor OV7630 (decimal ou hex, ex.: `b:0x1a=0x1d`, `s:0x11=0x01`).

| Arquivo em `/sys/module/gspca_elgin_fps/parameters/` | Uso |
|---|---|
| `regw` (só escrita) | Aplica na hora, com a câmera aberta: `echo "b:0x1a=0x1d" \| sudo tee $P/regw` |
| `regr` | Lê um registrador: `echo "s:0x0a" \| sudo tee $P/regr` e depois `cat $P/regr`. A ponte devolve 0 em vários registradores; o sensor funciona (`s:0x0a` = 0x76) |
| `regs_start` | Lista separada por vírgula, reaplicada no **fim de todo início de captura** (depois do `apply_fps`): `echo "b:0x1a=0x1d,b:0x1b=0x10" \| sudo tee $P/regs_start`. Para limpar: `echo "" \| sudo tee $P/regs_start` |

Cada item aplicado sai no `dmesg` (`registrador: b:0x1a = 0x1d`). O avaliador usa o `regs_start`: `python3 avalia.py geom15` (9 casos) e `python3 avalia.py --varre b:0x1a 0x14 0x20` (varre qualquer registrador, por padrão S910 640x480 a 15 fps com alt 8).

Ciclo: `sudo rmmod gspca_elgin_fps && sudo insmod ./gspca_elgin_fps.ko`; conferir com `v4l2-ctl -d /dev/video2 --list-formats-ext` e `dmesg | grep apply_fps`. Antes, descarregar o `gspca_sonixb` (`sudo modprobe -r gspca_sonixb`) e, se ele voltar sozinho, usar blacklist temporária.

### 60 fps no driver: causa dos 29,9 fps e leitura I²C (2026-10-12 10:25)

- **Causa dos 29,9 fps:** `i2c_w_vector(buf, len)` conta **bytes** (`len -= 8` por linha) e o modo 60 fps passava `ARRAY_SIZE(hi_regs)` (3, linhas): só a 1ª escrita (`0x14 = 0xA0`) saía; `0x28` e `0x2D` ficavam nos valores do Windows. Corrigido com `sizeof(hi_regs)`. `[medido]`
- **Validação:** `testes/regs60.py` lendo o sensor durante a captura S910 160x120: em 60 fps `COMC = 0xA0`, `COMH = 0x80`, `CLKRC = 0x00`, `COMJ` lê 0x41 (gravado 0xC1; o bit 7 não volta na leitura); em 30 fps `COMC = 0x80`, `COMH = 0xA0`, `COMJ = 0x01`. `v4l2-ctl --set-parm=60`: **59,5 fps** medidos. `[medido]`
- **Leitura I²C do sensor (`i2c_r`, parâmetro `regr`) agora funciona** (leitura em captura). No SN9C103 o ciclo que só grava o endereço e o ciclo de leitura deixam o bit de erro (`0x08` do reg 0x08) ligado sem ser erro de verdade; o Risolia só confere depois de ler. `i2c_w_ign` ignora o bit nesses dois ciclos. O byte lido vem na posição 4 dos 5 bytes da porta (copiado para `usb_buf[0]`). `[medido]`
- **Armadilha dos testes:** `/dev/video0` é a câmera interna do laptop; a Elgin está em `/dev/video2` (varia). `regs60.py` agora acha a Elgin sozinho; `testa_60fps.py` ainda tem `/dev/video0` fixo.

### Hipóteses abertas (2026-10-12 12:44, a discutir depois de `testes/testa_bits.py`)

- **H1 (do autor):** `COMH` bit 7 ("uma linha" × "duas linhas") pode estar ligado a campo só ímpar × só par, ou ainda a VSYNC, HSYNC e HREF (`COMC` bit 4 e bit 3, `CLKRC` bits 7:6, `FSD`). `[hipótese]`
- **H2:** com `COMC` bit 4 = 1 (sync por quadro) a ponte recebeu 29,8 fps e só aproveitou as primeiras 240 linhas (um campo). Um quadro completo de cor exigiria janela de 320x480 (`V_SIZE` = 0x1E). `[hipótese, medido em parte]`
- **H3:** em QVGA cada campo já tem 240 linhas (não 120), por isso o 60 fps entrega só duas cores por quadro. `[dedução dos testes G/H e de cores]`

## 7. Pendências

1. **(Feito, ver "60 fps no driver")** **60 fps no S910 160x120:** `0x2D = 0xC1` (bit 6), `0x14` bit 5 (QVGA do sensor) e o modo do bridge. Banda: 160x120 comprimido cabe; cru 160x120 a 60 fps (1,15 MB/s) não.
2. **(Feito, ver "Modos CIF/QCIF")** Modo CIF 352x288 (e 176x144) como no Windows.
3. **Testar `LQ_SEL` (reg `0x17` bit 0 = 1, ou seja `0x69`):** compressão de baixa qualidade deve gerar quadros menores e permitir mais FPS em 640x480 e 320x240. O bit existe no 103 segundo o driver irmão, mas **a varredura de 2026-10-11 mostrou que ele não altera o tamanho do quadro** (640x480: 54.999 bytes com `lq_sel=0`, 55.074 com `lq_sel=1`); hipótese refutada neste ajuste. (A hipótese anterior, "nibble baixo do 0x18", estava errada.)
   - Também comparar o clock do sensor e o modo de sync: Windows (`0x17 = 0x68`, `0x19 = 0x20`: 24 MHz, sensor mestre) × original (`0x6C`, `0x2B`: 48 MHz, bridge gera o sync).
4. hstart/vstart (`0x12`/`0x13`) por modo, para eliminar artefatos de borda.
5. **(Feito)** Janela de AE `0x1C–0x1F`: restaurada (`02 03 0F 0C` e janelas por modo, menu "AE window"; o alvo do ganho automático acompanha a área).
6. Regra do alt 7 (áudio USB): **confirmada no código** (`gspca_init_transfer`: com interface de áudio, USB 1.1 e banda máxima ≥ 1.000.000, pula o alt mais alto). Parâmetro `top_alt=1` criado e testado: o alt 8 é aceito e o quadro de 640x480 a 15 fps chega completo, mas a imagem continua ruim (ver "Teste do alt 8").
7. Investigar o crash ao trocar de resolução em BA81 sem reiniciar o Multiview (rodar `dmesg | grep -iE 'bug|oops|warn|call trace'` depois de reproduzir).
8. Autogain mais inteligente: manter o FPS escolhido e deixar o ganho cobrir a luz, em vez de baixar o FPS.
9. Reduzir o log do `KDBG` (`DEBUG_103 = 1` registra cada acesso a registrador).
10. **(Implementado e testado em 2026-10-11)** Aplicar ganho, exposição e brilho ao iniciar a captura: as chamadas `setgain`/`setexposure`/`setbrightness` estão comentadas no `sd_start`, os controles só valem se mudados com a captura ativa, e a imagem sai escura (média ~15 em 255), o que prejudica o S910.
11. **(Implementado e testado em 2026-10-11)** 10 fps acrescentado à tabela do S910 640x480 (estável na varredura).
12. **(Implementado e testado em 2026-10-11)** Ajuste fino na tabela de taxas (S910 640x480: 14 e 13 fps) e `top_alt=1` por padrão; `avalia.py taxas` deu 8 de 8 boas.

## 8. Estado dos arquivos

- `../elgin/elgin_sn9c103.c`: último estado de 2022-12-22 (não commitado), preservado. O último commit do git é de 2022-11-10.
- `elgin_fps.c`: cópia de trabalho, ajustada ao kernel 7.0 (`min_queued_buffers`, sem `wait_prepare`/`wait_finish`); não compila mais no 5.4.
- Nada desta pasta foi commitado.
- Mudança de 2026-10-11 (20:17): o `sd_start` agora chama `setgain`, `setbrightness` e `setexposure` (valores padrão dos controles: ganho 31, exposição 66 ms, brilho 127). O `setexposure` passou a usar o `reg 0x11` da tabela de fps, e a trava `reg11 >= 4` em 640x480 foi desligada (`if (0 && ...)`). O `.ko` precisa ser recarregado para valer.

## Varredura de registradores do sensor e cor no modo 60 fps (2026-10-12 14:45)

Scripts: `testes/testa_bits.py` (bits do 60 fps) e `testes/varredura.py` (leitura dos 256 enderecos e escrita de 0x00/0xFF nos reservados). Resultados em `testes/bits_resultado.txt` e `testes/varredura_resultado.txt`.

**Cor no 60 fps**
- Nenhum bit testado (COMC 6/1/0, COMH 6/3/2, COMJ 5/3/1/0, FSD, SCAL) nem registrador reservado restaura a linha que falta.
- Param o stream: COMC bit 0 = 1 (0xA1), COMJ bit 1 = 1 (0xC3), FSD = 0x00. BW (COMH bit 6) gera padrao constante 116/0, nao e imagem.
- Em QVGA cada campo tem 240 linhas de um unico tipo (R/G ou B/G) e os campos alternam. Com COMC bit 4 = 1 e janela da ponte 320x480 (V_SIZE 0x1E, escala 1/2) o quadro 160x240 vem com os dois campos EMPILHADOS (cada metade e um campo); intercalar as linhas da imagem colorida. Da 30 fps coloridos, nao 60.
- Decisao: a recombinacao dos campos pertence a camada acima do driver (V4L2_FIELD_SEQ_TB / ALTERNATE), nao ao driver. O modo 160x240 de teste foi removido do elgin_fps.c.

**Leitura dos 256 enderecos (durante a captura)**: so existem registradores ate 0x7F. Enderecos sem registrador devolvem valor flutuante (0xFF, ou 0x28–0x2A). Acima de 0x7F ha apenas copias deslocadas (nao usar).

**Achados na escrita**
- 0x7E/0x7F: contador de 16 bits de LINHAS EXTRAS por quadro (hipotese; cada unidade soma ~62 us). 60 fps base 59,5: 0x7E=0x20 -> 53,4; 0x40 -> 48,2; 0x80 -> 40,4; 0xFF -> 30,5; 0x7F bit0 -> 30,4; bit1 -> 20,2; bit2 -> 11,9; bits 3-7 -> quadros de ~0,14 s a ~2 s (a medicao de fps estourou o tempo; a imagem chegou, cada vez mais clara). NAO e "stream para". sd_start nao reescreve esses registradores: apos a varredura o fps final ficou 58,0 ate restaurar (`regs_start` com `s:0x7e=0x00,s:0x7f=0x00`) ou reconectar a camera.
- 0x6B: deslocamento (hipotese: nivel de preto de um canal): bits 0..6 escurecem progressivamente (media 31 -> 0) e a diferenca entre colunas sobe de 0,10 a 1,9 (so uma das cores das colunas); bit 7 sozinho sem efeito; 0xFF clareia (media 75).
- 0x69 (ADRC, padrao 0x38): RANGE (bit 3) = 0 deixa a imagem ~2x mais clara; REF[2:0] muda o brilho sem ordem simples; bits 7:4 sem efeito.
- 0x4C bit 5 (MEDRGB): faixa escura de ~3 px na borda esquerda.
- 0x6C = 0xFF: quadro comprimido +16%, 56,8 fps (bits nao varridos). 0x4D, 0x6D, 0x6E: gravam e leem de volta, sem efeito visivel.
- Sem efeito: 07–09, 0E–0F, 1E, 34–4B, 4E–5F, 6A, 7B (leitura flutuante).
- Cuidado: a cena do primeiro teste ficou saturada (media 242, dp 0,5) e a metrica de linhas fica cega; o script agora recusa cena lisa.

**Pendente**: varrer por canal 0x6B e 0x6C com a cartela de cores; 0x69 com exposicao e ganho automaticos desligados; confirmar a formula do 0x7E/0x7F com valores intermediarios.

## Clock mestre do sensor e fps (2026-10-12 15:25)

Scripts: `testes/testa_clock.py` e `testes/testa_clock48.py` (resultados em `clock_resultado.txt` e `clock48_resultado.txt`).
- A ponte fornece o clock mestre (MCK) ao sensor: reg 0x17 bits 3:2 (11 = 48 MHz, 10 = 24 MHz, 01 = 12 MHz) e reg 0x19 (MCK_SIZE, PCK_2X). O driver usa 0x68 (24 MHz).
- O fps e' proporcional ao clock (modo S910 160x120, 60 fps): 12 MHz = 29,8; 24 MHz = 59,5; 48 MHz = 119,8 (x2,01).
- A 48 MHz a imagem e' RUIDO (correlacao entre vizinhos 0,2-0,3; USB ~800-990 KB/s). Nem a borda de captura (0x18 bit 0) nem a inversao do clock (0x17 bit 6) resolvem; a 24 MHz as duas bordas e as duas polaridades dao imagem boa (corr 0,85-0,90). Pedindo 30 fps (sensor em VGA) a 48 MHz nao vem imagem.
- Conclusao: o teto util e' 24 MHz. Hipotese: o ADC interno do sensor opera ate 12 MHz (datasheet) e o clock de entrada vai ate 40 MHz; 48 MHz esta fora dos dois limites. A ponte nao oferece clock intermediario (SEN_RATE=00 com MCK_SIZE >= 2 nao passa de 24 MHz). Caminho de mais de 60 fps por clock: fechado.

## Modo 80x60 (2026-10-12 15:40)
- Novo modo no driver, BA81 e S910: janela da ponte 320x240 (H_SIZE 0x14, V_SIZE 0x0F), SCAL 1/4 (reg 0x18 bits 5:4 = 10), sensor em QVGA (COMC = 0xA0; a 30 fps ou menos COMH = 0xA0 progressivo; a 60 fps reaproveita o bloco do 160x120 de 60 fps). Flag MODE_QVGA no priv. fps anunciados: 60, 30, 15, 10, 8.
- Medido (testes/testa_80x60.py): BA81 30 fps -> 27,3; BA81 60 -> 59,5; S910 30 -> 25,9; S910 60 -> 59,0. Imagem boa nos 4 casos (corr 0,83-0,85) e enquadramento igual ao do 160x120 (nao e recorte). O S910 decodifica apesar da altura 60 nao ser multiplo de 8. S910 80x60: ~1,4 KB por quadro.
- O fps do sensor nao muda com a resolucao (o QVGA e' lido inteiro); o ganho e' so de USB e tamanho do quadro.


---

## Modos CIF/QCIF, controles novos e balanço de branco (2026-10-12 16:00 a 18:10)

### CIF 352x288 e QCIF 176x144 (testes/testa_cif.py, 16:13)

- **Como funciona:** o sensor fica em VGA 640x480 e a ponte recorta uma janela de 352x288 centrada no quadro (`H_START` + 144, `V_START` + 96, `H_SIZE` = 22, `V_SIZE` = 18, janela de AE `02 02 09 07`). CIF usa escala 1/1 e QCIF escala 1/2 (SCAL, reg 0x18).
- **H_START e V_START estão em unidades de 1 pixel/1 linha:** com a cena não estourada, a janela caiu em x = 144, y = 96 (o esperado) em 6 dos 8 casos, com ncc 0,99–1,00. O de 30 fps no S910 CIF achou x = 140 (ncc 0,59): quadro de 20 KB, compressão forte; o fps estava certo.
- **S910:** CIF a 15 fps → 14,9 fps; QCIF a 30 fps → 30,5 fps; QCIF a 15 fps → 11,0 (trava de banda). Todos decodificam como imagem (corr 0,94–0,98).
- **BA81 e o alt do USB:** BA81 352x288 a 5 fps (507 KB/s, alt de 512) e BA81 176x144 a 15 fps (380 KB/s, alt de 384) chegaram CORTADOS (7 KB e 5 KB por quadro): ocupação de 99% do alt. Taxas anunciadas ajustadas para ficar em torno de 80%: `fps_cif_raw = {4, 3, 2}` (antes 5, 2) e `fps_qcif_raw = {12, 8, 6}` (antes 15, 10, 8); padrões 4 e 12. Regra empírica: o fluxo cru perde linhas perto de 96% do alt; o comprimido aguentou ~91%.
- **fps abaixo do pedido no escuro:** com a cena mais escura o BA81 mediu 2,8 (pedido 4), 1,5 (2), 7,5 (12) e 5,6 (8) fps; hipótese (não confirmada): a exposição automática alonga o quadro. Com a cena estourada o CIF a 2 fps deu 2,0.
- **Salto de 10 para 14 fps no S910 640x480 (registro):** veio de duas mudanças juntas. `top_alt = 1` libera o alt 8 (1003 B/ms, o teto da câmera; o gspca pulava o alt 8 pela regra do áudio USB) e o ajuste fino de taxa do sensor (`0x2A` bit 7 + `0x2B`, ~0,10% por passo) abaixa 15 fps para 13–14 sem trocar o divisor, ficando em 86–91% do canal, abaixo do ponto em que o quadro desliza (15 fps = 92%).
- **Modo customizado (janela, posição e escala escolhidas pelo usuário):** pensado e adiado. Desenho possível: `S_SELECTION`(CROP) + `S_FMT`, com janela múltipla de 16, x par e y ímpar (mosaico BGGR), escala = janela/saída, taxas calculadas pela banda.

### Controles novos do driver (testes/testa_controles.py, 16:59 e 17:08)

| Controle | Registrador | Medido (BA81 160x120, mesma cena) |
|---|---|---|
| `black_level` (0–255, padrão 10) | ponte 0x14 OFFSET | 0 / 10 / 30 / 60 → média 30,5 / 20,3 / 3,3 / 0,0 (maior = mais escuro) |
| `gamma` (0–8, padrão 2 = linear) | ponte 0x20 a 0x30 (17 pontos) | 0 / 2 / 5 / 8 → média 3,6 / 19,9 / 51,0 / 77,9 |
| `test_pattern` = 1 | ponte 0x17 bit 4 (TEST_IMG) | média 108,2, desvio 69,0 (imagem sintética) |
| `test_pattern` = 2 | sensor 0x12 (COMA) bit 1 | média 24,0, desvio 16,5 (barras de cor) |
| `horizontal_flip` | sensor 0x12 (COMA) bit 6 | ver abaixo |

- A curva de gama é `255 × (i/16)^(1/γ)` com γ = 0,6 + 0,2×k (k = 0 a 8); k = 2 é a linear exata de antes. A tabela é gravada de uma vez no início e ao vivo; a ponte a aplica também na saída crua.
- **Os valores dos controles ficam gravados no driver entre execuções:** o primeiro teste saiu inválido porque `black_level = 120` ficou ativo nas capturas seguintes (tudo preto). O script reinicia todos os controles antes de cada captura.
- **Espelho:** `HFLIP = 0` mantém o `0x78` do Windows no COMA; `HFLIP = 1` zera o bit 6. A imagem sai espelhada (correlação 0,93–0,97 com a original invertida). Primeira versão somava +1 a `H_START` para "manter o BGGR": errado. O sensor já inverte a paridade das colunas; com +1 a fase saía trocada (S910: pontos do mosaico 17,8/21,1/20,2/23,6 viraram 20,9/17,7/23,5/20,1; B/R 0,75 → 1,02) e o Multiview mostrava cinza esverdeado. Com `HFLIP_HSTART_ADJ = 0` a razão B/R fica 0,77 contra 0,76 e a cor se mantém. Se algum dia for preciso, a constante permite testar outro ajuste.
- Inversão vertical: o OV7630 TEM controle (COMN `0x75` bit 7, `VFLIP`; a frase anterior desta nota estava errada). Controle `vertical_flip` do driver grava `0x0E | v<<7` (a tabela original grava 0x0E, o Windows 0x8E). Testado ao vivo no Multiview: a imagem inverte (2026-10-12 18:55). Sem correção de fase Bayer (`VFLIP_VSTART_ADJ 0`).
- Novos controles (2026-10-12): `band_stop_filter` (COMJ `0x2D` bit 2, base 0x81, 0xC1 no modo 60 fps), `AWB Target Blue` e `AWB Target Red` (sensor `0x0C` e `0x0D`, 0–63, padrão 0x20). Implementados; efeito ainda não medido.
- Observações do uso ao vivo: o ganho automático (AGC/AEC) acompanha cenas mais claras e mais escuras, mas a qualidade do brilho não é boa; o balanço de branco automático do sensor (COMA bit 2) não balanceia adequadamente. Pendente de investigação (alvos 0x0C/0x0D, 0x6B/0x6C).
- **Ao vivo:** mudar gamma, black_level, hflip e test_pattern com o fluxo ligado não travou a captura (S910).

### Contraste e saturação do sensor não têm efeito (testes/testa_cs.py, 17:33)

- Controles `contrast` (reg 0x05, CNT) e `saturation` (reg 0x03, SAT) do sensor foram criados para teste. Varridos de 0 a 255 em BA81 e S910: o desvio da imagem ficou em 4,0 e a croma do mosaico entre 0,26 e 0,32, sem tendência; também sem efeito visível no Multiview. A cena era escura (média ~12), mas os valores parados descartam o efeito.
- **Explicação:** o sensor entrega Bayer cru nos dois formatos (COMH bit 7 = 1, que a ponte exige). BA81 é esse Bayer repassado; **S910 é o MESMO mosaico Bayer comprimido pela ponte** (CMP_MODE, reg 0x18 bit 7), descomprimido de volta a Bayer 8 bits; **não é YUV**. SAT, HUE, CNT e BRT só atuam na saída YUV do sensor, que o driver não usa.
- Alternativa para contraste: curva em S na tabela de gama da ponte (funciona). Saturação: só na aplicação (o Multiview converte em `convertToRgb`: `sonix_decompress` + `bayer2rgb24`, fase BGGR).
- Decisão pendente: manter ou retirar os dois controles.

### Balanço de branco: automático + manual (implementado 18:03, falta validar o automático)

- **Controles:** `White Balance, Automatic` (padrão ligado) é o mestre de um grupo com `Red Balance`, `Blue Balance` e `Green Balance`; os três ficam inativos com o automático ligado.
- **Automático:** liga o bit 2 (AWB) do COMA do sensor (0x12). **Não se sabe** se esse bit basta para o sensor ajustar os ganhos 0x01 e 0x02 sozinho: `python3 testes/testa_awb.py`.
- **Manual:** `Red Balance` → sensor 0x02 (0–255, padrão 0x3A); `Blue Balance` → sensor 0x01 (0–255, padrão 0x77); ambos validados antes (cada um age só na sua cor). `Green Balance` (controle próprio, 0–127, padrão 32) → ponte 0x05 e 0x06 juntos. Com o automático ligado o driver grava 0x20 em 0x05/0x06.
- **Por que a ponte só serve para o verde:** a ponte aplica os ganhos por POSIÇÃO do pixel, supondo a fase GBRG: `0x05` age só no pixel (1,0); `0x06` só em (0,1); `0x07` nos dois pixels da diagonal (0,0) e (1,1) ao mesmo tempo. No mosaico BGGR do driver, (1,0) e (0,1) são os dois verdes, e a diagonal é azul + vermelho juntos, com um único ganho: não dá para equilibrar R contra B. Por isso R e B usam o sensor e o `0x07` não ganhou controle (decisão do autor).
- **Histórico:** o balanço manual pelo sensor foi retirado e recolocado nesta sessão; os controles R/B do sensor tinham sido validados em 2026-10-12 00:10.

### Documentação
- `MAPA_REGISTRADORES.html` (0x03, 0x05, 0x05–0x07, 0x12, 0x14, 0x17 e a tabela de gama 0x20–0x30) e `BITS_ANALISE_60FPS.html` (COMA, TEST_IMG e a tabela de campos numéricos) atualizados em 2026-10-12 18:10.

## Formatos Bayer por H_START/V_START e ganhos da ponte por canal (2026-10-12 20:00 a 2026-10-13)

**Quatro formatos crus.** O sensor sempre envia o mesmo mosaico; a ponte descarta `H_START` colunas e `V_START` linhas antes da janela, então a paridade desses dois valores decide qual cor é o primeiro pixel. O driver (`sd_start`) soma +1 sobre a base BGGR (H_START par, V_START ímpar) e anuncia o formato correspondente. Todos passam direto pela ponte, sem conversão:

| Formato V4L2 | Fourcc | Mosaico (linha 0 / linha 1) | H_START | V_START |
|---|---|---|---|---|
| `V4L2_PIX_FMT_SBGGR8` | BA81 | B G / G R | base | base |
| `V4L2_PIX_FMT_SGBRG8` | GBRG | G B / R G | +1 | base |
| `V4L2_PIX_FMT_SGRBG8` | GRBG | G R / B G | base | +1 |
| `V4L2_PIX_FMT_SRGGB8` | RGGB | R G / G B | +1 | +1 |

O S910 (`V4L2_PIX_FMT_SN9C10X`, comprimido) continua só na fase BGGR. A descompressão devolve os pixels na mesma ordem em que a ponte os enviou, sem conhecer cores; a fase do mosaico decodificado é a da janela. Os formatos novos aparecem depois do S910 e do BA81 na enumeração. Modos: 6 resoluções por fase (80x60 a 640x480), com os mesmos limites de fps do BA81.

**Ganhos da ponte por canal (Red gain, Green gain, Blue gain).** Substituem os controles Red/Blue/Green Balance. Os registradores agem por POSIÇÃO (linha, coluna), não por cor: `0x05` → pixel (1,0), `0x06` → pixel (0,1), `0x07` → os dois pixels da diagonal (0,0) e (1,1). `setgains()` escolhe o registrador conforme o formato:

| Formato | 0x05 | 0x06 | 0x07 |
|---|---|---|---|
| GBRG | Red | Blue | Green |
| GRBG | Blue | Red | Green |
| BGGR e RGGB | Green | Green | Red (Blue sem efeito) |

Confirmado ao vivo no Multiview (2026-10-12): em GBRG e GRBG os três ganhos agem exatamente em seus canais, de forma independente. Em BGGR e RGGB o Green gain age no verde, o Red gain age em vermelho e azul juntos (a diagonal) e o Blue gain não tem efeito, como previsto. Decidido manter assim (alternativa não implementada: R e B pelos ganhos do sensor, 0x01/0x02, que o balanço automático do sensor sobrescreve). Intervalo 0–127, padrão 0x20 (1,0×).

**Quadro BGR do OpenCV.** O quadro de saída do Multiview é BGR: o primeiro byte gravado é o canal AZUL. Por isso a `sgbrg2rgb24()` herdada do OpenCV, que escreve "R" primeiro, decodifica de fato o mosaico GBRG (G B / R G); os papéis R/B nos comentários dela estão na ordem RGB, o que causou confusão. A função nova `bayer_phase2rgb24()` (`v4l2misc/camconv.cpp`, com `FASE_BGGR/GBRG/GRBG/RGGB`) grava na ordem B, G, R. A `bayer2rgb24()` (BA81 e S910) segue gravando R primeiro e pode estar com R e B trocados no quadro BGR: a verificar com um objeto vermelho vivo. Nada foi trocado nelas ainda.

**Controles e registradores novos (testes ainda por rodar).** `vertical_flip` (sensor `0x75` bit 7, funciona ao vivo), `band_stop_filter` (`0x2D` bit 2), `AWB Target Blue/Red` (sensor `0x0C`/`0x0D`, 0–63, padrão 0x20): implementados, efeito de cor ainda não medido (`testes/testa_novos.py`). `testes/testa_ganhos.py` mede os ganhos em números, mas ainda segue a tabela antiga de formatos. `testes/testa_awb.py` está desatualizado (usa os controles removidos).

**Pendente.** 0x7E/0x7F (fórmula com valores intermediários e restauração, teste `testes/testa_7e7f.py` escrito); 0x6B/0x6C por canal com a cartela de cores; 0x69 com exposição e ganho automáticos desligados; 0x19 = 0x50 para raw; qualidade do brilho do auto-ganho e do balanço automático do sensor; decisão sobre os controles de contraste e saturação (sem efeito); commit no git.

## Varredura dos registradores da ponte (2026-10-13)

Scripts: `testes/varredura_ponte.py` (leitura dos 256 endereços e escrita 0x00/0xFF com medida de imagem) e `testes/varredura_ponte_escrita.py` (escreve 00/FF/55/AA e lê de volta). Dica de partida: o mapa do SN9C102 (`CHIP_INFO/sn9c102_registers.pdf`). Parâmetros do módulo usados: `regr` e `regs_start` com `b:0xREG=0xVAL`.

**Mapa lido durante a captura (60 fps, S910 160x120)**
- 0x00 = 0x10 (ID) · 0x01 = 0x44 · 0x02 = 0x47 · 0x03 = 0x00 · 0x04 = 0x1A · 0x05–0x07 = 0x20 · 0x08 = 0xA4 · 0x09 = 0x21 · 0x10 = 0x03 · 0x12 = 2 · 0x13 = 1 · 0x14 = 0x0A · 0x15 = 0x14 · 0x16 = 0x0F · 0x17 = 0x68 · 0x18 = 0x9F · 0x19 = 0x20 · 0x1A = 0x14 · 0x1B = 0x0F · 0x1C–0x1F = 0x01, 0x02, 0x07, 0x06.
- 0x20–0x30: rampa linear 00, 10, 20 … F0, FF (tabela de gama).
- 0x80–0xFF repetem 0x00–0x7F: a ponte decodifica 7 bits de endereço (por isso `--ext` não serve).

**Escrita de 0x00/0xFF (leitura de volta = quantos bits o registrador guarda)**

| Reg | Guarda (máscara) | Efeito na imagem/fps |
|---|---|---|
| 0x02 | 0x77 (bits 6:4 gravados; 2:0 leem sempre 1) | nenhum |
| 0x03 | 0x3F | nenhum |
| 0x04 | 0x3F | nenhum comprovado (média 55 → 60 a 61, dentro da variação da base) |
| 0x0F, 0x11 | nada (lê 0x00) | nenhum |
| 0x10 | 0x77 | nenhum (não age como ganho no SN9C103) |
| 0x1A / 0x1B | 0x3F / 0x1F | nenhum (HO/VO_SIZE) |
| 0x1C / 0x1D / 0x1E / 0x1F | 0x07 / 0x07 / 0x1F / 0x0F | **janela da exposição automática: muda o brilho** |
| 0x31–0x3E | nada (lê 0x00) | não existem |
| 0x3F | 8 bits | nenhum visível |
| 0x40–0x7F | somente leitura | dados que mudam sozinhos |

**Janela de AE (0x1C–0x1F)**
- 0xFF em 0x1C ou 0x1D → média 174 (era 55); 0x00 em 0x1D ou 0x1E → média 42.
- Bits isolados: 0x1E bit 2 → média 123 e fps 50, bits 3 e 4 → 41 a 47; 0x1F bits 3, 4, 6, 7 → 49 a 52, bit 2 → 86, bit 5 → 78 com colunas desiguais.
- **Ressalva:** depois de 0x1C = 0xFF o brilho ficou preso em 174 nas medições seguintes. A explicação anotada aqui ("o script restaura só o registrador anterior") foi corrigida depois: a causa era o `ae_area` desatualizado no driver (mudar a janela sem atualizar a área invalida o alvo do ganho automático). As flags BRILHO em 0x31–0x3F desta rodada são deriva da medição, não efeito. Os valores acima de 0x1E/0x1F precisam ser refeitos um registrador por vez, restaurando todos.

**0x40–0x7F:** 32 valores de 10 bits (byte baixo nos endereços pares, bits 9:8 nos ímpares) que mudam sozinhos com a cena parada; gravar 00/FF/55/AA não altera nada. Provável estatística ou dado interno do chip. Os "GUARDA" que o segundo script marcou ali são falsos (critério frouxo).

**Pendências novas**
- Refazer a janela de AE (0x1C–0x1F) com restauração completa e várias janelas (tela cheia, centro, canto) para ver o efeito no brilho automático.
- Descobrir a função de 0x3F e dos dados de 0x40–0x7F (ver se acompanham brilho ou cor da cena).

## Versão e identificação do módulo (2026-10-13)

- **Versão:** `MODULE_VERSION` = `"elgin-AAAAMMDDHHMM"`. O número é a data e hora da compilação, gerado pelo `umilmod.sh` (`sudo make ELGIN_BUILD=$(date +%Y%m%d%H%M)`) e passado ao compilador pelo `Makefile` (`ccflags-y += -DELGIN_BUILD=\"$(ELGIN_BUILD)\"`, dentro de `ifdef ELGIN_BUILD`). Compilado sem o `umilmod.sh`, a versão fica `elgin-dev`. Antes era `2.14.0`, herdada do gspca.
- **Conferir qual compilação está carregada:** `modinfo gspca_elgin_fps | grep -i version` (ou `/sys/module/gspca_elgin_fps/version`). A versão do kernel aparece no `vermagic` do mesmo comando; por isso não entra no texto da versão.
- **Autores:** três linhas `MODULE_AUTHOR`: "Original: Jean-François Moine", "Modificador: Josemar Simão <josemars@ifes.edu.br>" e "Revisor: Claude (Anthropic)". O crédito do autor original permanece (código derivado do gspca, licença GPL). O `modinfo` pode listar as linhas na ordem inversa da do arquivo.
- **Descrição:** `MODULE_DESCRIPTION("GSPCA USB Elgin Camera Driver")`.
- **Atenção ao editar:** o `Makefile` não pode ser gravado por ferramentas remotas (protegido); alterações nele são feitas à mão (`Makefile.bak_versao` guarda a versão anterior). O editor com o `elgin_fps.c` aberto sobrescreveu gravações externas várias vezes: feche o arquivo antes de pedir alterações.

## Melhorias futuras (anotadas em 2026-10-13)

- **Controles XY da janela de AE fora de "Custom":** hoje o driver guarda o valor e o ignora (só vale com o menu em Custom), e o Multiview deixa ajustar as setas mesmo com o controle `[inativo]`, porque o V4L2 permite gravar controles inativos (comentário em `v4l2ctl_is_writable`). Opções avaliadas e **adiadas**: (a) o driver recusar a gravação nos XY quando o menu não é Custom, para o Multiview mostrar o erro; (b) o Multiview tratar `INACTIVE` como não gravável. A opção (b) não é recomendada: tiraria o ajuste do Gain com o ganho automático ligado, que hoje funciona (o valor é aplicado e o automático volta a sobrescrever).

## Varredura das fontes da pasta `sn9c103/` (2026-10-13)

Lidas todas as fontes da pasta para conhecer os registradores. **O que cada uma trouxe:**

| Fonte | Resultado |
|---|---|
| `log_sn9c103.txt` (captura USB do Windows, decodificada inteira: 53 operações) | Modo real da captura, sequência de parada, espera do I²C (abaixo) |
| `sn9c1xx/sn9c1xx-1.50.tar.gz` e `sn9c1xx/modified/` (Risolia 1.50) + `sn9c102.txt` | `0x19` por formato, cabeçalho de 18 bytes, escala do ganho da ponte, controles do OV7630 |
| `CHIP_INFO/OV7630-unlocked.pdf` (tabela completa) | Vários registradores do sensor que agem na saída RGB crua (já transcritos bit a bit no MAPA) |
| `gspca/*/sonixb.c` (kernel) | Ganho do OV7630: pular 32–47 da escala 0–63 (o driver já faz) |
| `regs.xlsx` | Mesma tabela "original × captura" da seção 2 (cabeçalho da planilha: 320x240) |
| `INFO*.TXT`, readmes, `desabilitar_modulos_boot.txt` | Só links e comandos; nada de registradores |
| `sn9c1xx/sn9c103`, `sn9c1xx/aio`, `gspca/aio`, `gspca/elgin` | Versões consolidadas e portadas para kernels novos; sem informação nova de registradores |
| `CHIP_INFO/sn9c102.tar.tar` (Ian Molton, 2003) | Programa de teste para outro sensor; sem relação |
| `CHIP_INFO/General_Electric_HO98067.pdf` | Manual do usuário da webcam GE MiniCam Pro 98067, que usa a mesma ponte ("USB PC Camera (SN9C103)" na escolha da fonte): instalação do driver de Windows e uso. Sem registradores |
| `uvc/` | Driver UVC do kernel (outra câmera); fora do escopo. **Removido em 2026-10-13** (recuperável pelo git: `git checkout -- uvc`) |

### Captura do Windows `[captura]`

- **O modo capturado é 320x240 comprimido, não CIF:** o bloco de 0x30 bytes traz `0x15/0x16 = 16 12` (CIF), mas depois da init do sensor o Windows grava `0x15/0x16 = 28 1E` (janela 640x480) e `0x18 = 0x9F` (compressão + escala 1/2). Bate com o cabeçalho "320x240" da `regs.xlsx`.
- **Ordem final:** `0x1C..1F = 02 03 0F 0C` → `0x15/0x16 = 28 1E` → `0x18 = 0x9F` → `0x01 = 0x44` → `0x17/0x18 = 68 9F` → sensor `0x10 = 0x83`, `0x76 = 0x00`, `0x11 = 0x00`, `0x00 = 0x10`, `0x2A/0x2B = 80 60`.
- **Parada:** `0x01 = 0x49` (S_PWR_DN + LED + SYS_SEL_24M), `0x17 = 0x48` (SEN_CLK_EN = 0: **desliga o clock do sensor**), `0x01/0x02 = 49 44`. O `sd_stopN` do driver grava só `0x01 = 0x09` (também passa o clock do sistema para 12 MHz).
- **Espera do I²C:** depois das escritas o Windows lê `0x08` até o bit 2 (I2C_RDY) acender: `B0, B0, B0, B0, B0, B4`, a cada ~10 ms.

### Driver do Risolia 1.50 `[driver irmão]`

- **`0x19` depende do formato** (`ov7630_set_pix_format`, SN9C101/102/103): Bayer cru → `0x50`; comprimido → `0x20`. Com `0x17 = 0x60` (SEN_RATE = 00, clock = 24 MHz / MCK_SIZE) o sensor roda a **4,8 MHz em cru e 12 MHz em comprimido**: o cru fica mais lento para caber na banda USB. Com o `0x17 = 0x68` do driver (24 MHz fixo) o MCK_SIZE não age. É a origem da pendência "`0x19 = 0x50` para raw"; o teste antigo do clock Risolia ("sem imagem") foi provavelmente com o formato comprimido. `[hipótese]`
- **Ganho da ponte no 103:** ganho = 1/2 + valor/64 (0x20 = 1,0×; 0 = 0,5×; 127 ≈ 2,5×). Nomes do Risolia para o 103: red → `0x07`, blue → `0x06`, green → `0x05`; as nossas medições mostram ação **por posição** e valem mais que os nomes.
- **Auto ganho/exposição do sensor:** controle "auto gain-exposure", padrão ligado: `0x13 = v | v<<1` (AEC e AGC internos juntos).
- **Brilho do sensor:** o Risolia 1.50 grava `0x61 = 0x4A` (BRTEN, bit 3, ligado; o 1.48 gravava 0x42 como o Windows). O datasheet só liga o brilho RGB com BRTEN = 1: **provável causa do `0x06` sem efeito na Bayer.** `[hipótese]`, testar com `s:0x61=0x4a`.
- **Outros controles do OV7630 no Risolia:** "rgb gamma" = `0x14` bit 2; "band filter" = `0x2D = 0x81 | v<<2` (igual ao driver); vertical flip = `0x75 = 0x0E | v<<7` (igual); ganho `0x00 & 0x3F` (padrão 0x20); exposição `0x10` (0–255, padrão 0x60); alvos do AWB `0x0C`/`0x0D` (0–63).
- **Recorte (`set_crop`, SN9C103):** `H_START = left + 1`, `V_START = top + 1`.
- **Sonda do 103:** `0x01 = 0x09`, `0x02 = 0x44`, `0x01 = 0x44`, `0x17 = 0x68`, `0x01 = 0x44` ("do not change anything!"); PID `0x0A = 0x76`, versão `0x0B = 0x31`.

### Datasheet OV7630: o que pode agir na Bayer crua `[datasheet]`

- **Nível de preto por canal:** `0x21` (G), `0x22` (B), `0x2E` (R) com COMF (`0x26`) bit 1 = 1 e COMG (`0x27`) bit 2 = 1 (manual); offset do ADC `0x78`/`0x79`/`0x7A` (G/B/R) com COMG bit 3.
- **Ganho analógico:** COMH (`0x28`) bit 1 = +3 dB; SPCA (`0x60`) bit 7 = pré-amplificador 1,5×, bits 3:2 = pré-ganho R e 1:0 = pré-ganho B (1,0×/1,2×/1,4×/1,6×; Windows `0x29` = R 1,4× e B 1,2×). **Os pré-ganhos separam R de B, o que a ponte não faz em BGGR/RGGB.**
- **Exposição de 10 bits:** `0x10` = bits 9:2, COMO (`0x76`) bits 1:0 = bits baixos.
- **AEC/AGC internos:** COMB (`0x13`) bits 0/1; alvos de pixels claros/escuros `0x24`/`0x25`; nível de referência ARL (`0x68`) bits 7:5; **janela central 1/4** COMI (`0x29`) bit 3; média interna FRARH (`0x2A`) bit 0; congelar COMH bit 4; ritmo COMJ (`0x2D`) bit 7; modo rápido `0x77`; SPCB (`0x61`) bit 7 deve ser 0 em RGB.
- **Medida de luz pelo sensor:** AVG (`0x7C`), com COME (`0x20`) bit 6 = 1 (o Windows grava `0x20 = 0x44`, ligado).
- **AWB:** COMA bit 2; passo COMD (`0x15`) bits 5:4; limiar COMC (`0x14`) bit 7; modos COME bits 1–3; faixa de luz AWBC (`0x66`); velocidade COMK (`0x70`) bit 1.
- **Gama:** COMC (`0x14`) bit 2 ("RGB gamma"). **Nitidez:** correção de abertura COME bit 4 + COMF bits 7:4 + COMK bits 5 e 2. **Matriz de cor:** COMG bit 4 a desliga.
- **Taxa:** COMI (`0x29`) bit 6 (clock 2×), FSD (`0x16`) pula campos, COMB bit 7 (24 MHz).
- Reservados no datasheet (só por medição): `0x6B`, `0x6C`, `0x7E`, `0x7F`.

### Cruzamento com as varreduras de registradores não documentados

As varreduras do sensor (2026-10-12, `testes/varredura.py`) e da ponte (2026-10-13, `testes/varredura_ponte*.py`) comparadas com todas as fontes da pasta:

- **Sem referência externa:** nenhuma fonte (Windows, Risolia 1.48/1.50, sonixb, datasheets) grava ou cita o sensor `0x6B`, `0x6C`, `0x7E`, `0x7F` (reservados no datasheet) nem a ponte `0x31`–`0x7F`. O que se sabe deles vem só das varreduras.
- **Concordam com o datasheet:** sensor `0x4C` (MEDC, filtro mediano: o bit 5 medido dá a faixa escura na borda), `0x4D` (ADDC, grava e lê, sem efeito visível) e `0x69` (ADRC: RANGE no bit 3 e REF nos bits 2:0, como medido).
- **Ponte `0x04` = 0x1A e `0x10` = 0x03:** o Windows e o Risolia (SN9C103) gravam esses valores sem explicação. A varredura mediu as máscaras (0x3F e 0x77) e nenhum efeito. No SN9C102 o `0x10` é ganho R/B; no 103 não age como ganho. `[captura]` `[driver irmão]` `[medido]`
- **Ponte `0x02`:** o Windows e o Risolia gravam 0x44; o datasheet do 102 só documenta os bits 1:0 (GPIO). A varredura leu a máscara 0x77, com os bits 2:0 sempre em 1 na leitura.
- **Ponte `0x3F`:** o Risolia só o grava (0x00) na init do SN9C105/120, não no 103. No 103 guarda 8 bits e não tem efeito visível.
- **Ponte `0x40`–`0x7F`:** 32 valores de 10 bits que mudam com a cena; nenhuma fonte os cita. `[hipótese]`: medidas de luz por região (estatística para exposição automática), o que seria útil ao ganho automático. Teste: ler os 32 valores com cenas escuras/claras em regiões diferentes (por exemplo, tampar metade da lente) e ver se acompanham.

### Grandezas controláveis pela ponte e pelo sensor (tabela atualizada)

| Grandeza | Sensor (OV7630) | Ponte (SN9C103) | Driver hoje |
|---|---|---|---|
| Ganho geral | `0x00` (6 bits); `0x28` bit 1 (+3 dB); `0x60` bit 7 (1,5×); `0x69` bit 3 (faixa do ADC) | `0x05`–`0x07` juntos (0,5×–2,5×) | Sensor `0x00` |
| Ganho por cor | `0x01` B, `0x02` R; `0x60` bits 3:0 (pré-ganho analógico R/B) | `0x05`/`0x06`/`0x07` por posição | Ponte (Red/Green/Blue gain) |
| Balanço automático | COMA bit 2, alvos `0x0C`/`0x0D` e ajustes acima | — (o cabeçalho devolve os ganhos) | COMA bit 2 + alvos |
| Exposição | `0x10` + `0x76` bits 1:0 | — | `0x10` (8 bits) |
| Exposição automática | AEC/AGC internos (`0x13`) e ajustes acima | Soma de Y na janela `0x1C`–`0x1F` (cabeçalho) | Algoritmo do driver com a soma da ponte |
| Medida de luz | `0x7C` AVG | Cabeçalho, dentro/fora da janela | Ponte |
| Nível de preto | `0x21`/`0x22`/`0x2E`, `0x78`–`0x7A` | `0x14` OFFSET | Ponte (`black_level`) |
| Brilho | `0x06` (com `0x61` bit 3) | `0x14` (indireto) | `0x06` sem efeito; vira alvo do automático |
| Contraste / curva | `0x05` (só YUV); gama `0x14` bit 2 | Gama `0x20`–`0x30`; `0x18` bit 6 | Ponte (`gamma`) |
| Nitidez | Correção de abertura (`0x20`, `0x26`, `0x70`) | — | — |
| Janela / recorte | `0x17`–`0x1A`, `0x1B` | `0x12`/`0x13`, `0x15`/`0x16`, `0x1A`/`0x1B` | Ponte |
| Escala | `0x14` bit 5, `0x2D` bit 6 | `0x18` bits 5:4 | As duas, por modo |
| Taxa | `0x11`, `0x2A`/`0x2B`, `0x29` bit 6, `0x16` | `0x17` bits 3:2, `0x19` | Sensor |
| Padrão de teste | COMA bit 1 | `0x17` bit 4 | As duas |

- **Modo YUV do sensor (avaliado e descartado em 2026-10-13):** COMA (`0x12`) bit 3 = 0 dá saída YUV 4:2:2 (bit 4 = ordem YUYV/UYVY), e com ela saturação, contraste e brilho do sensor passariam a agir. Na ponte só caberia QVGA (640 bytes por linha; a janela vai a 1008), sem compressão (~154 KB por quadro, **~6 fps** no USB), com ganhos, gama e offset da ponte neutros e provavelmente sem a escala da ponte. Não testado. Decisão: não vale a pena enquanto o fps cair tanto.

## 60 fps com cor completa: QVGA progressivo e opções de clock (2026-10-13 23:10)

Script `testes/testa_qvga60.py` (resultado `testes/qvga60_resultado.txt`, imagens em `testes/qvga60/`). S910 160x120, cena parada sob a lâmpada; registradores lidos de volta em cada caso (as gravações pegaram).

| Caso | Registradores | fps | Mosaico (dif × ref30) |
|---|---|---|---|
| ref30 | VGA progressivo (modo normal) | 29,8 | 0,00, cor completa |
| base60 | QVGA entrelaçado (`0x28 = 0x80`, modo 60 fps atual) | 59,5 | 0,16, **listras de cor erradas** |
| prog | QVGA progressivo (`0x28 = 0xA0`) | 29,8 | 0,07, cor completa |
| prog + `0x2D = 0x81` / `0x23 = 0xEC` (QCLK 0) / `0x23 = 0xEF` (EVEN2 1) / `0x29 = 0x70` (COMI clock 2×) e combinações | | **29,8 em todos** | 0,05–0,10, cor completa |
| entrelaçado + `0x23 = 0xEF` | | 59,5 | 0,14 |

- **Confirmado `[medido]`:** o QVGA de uma linha do sensor preserva o mosaico (blocos 2x2, como no datasheet). A falta de cor do 60 fps vem do **entrelaçado** (cada campo tem só uma paridade de linha).
- **Refutado:** nenhuma opção de clock (`0x23` bits 0/1, `0x29` bit 6) nem o `0x2D` bit 6 tira o progressivo de 30 fps. O teto é o clock interno de 12 MHz (24 MHz / 2 do CLKRC), já no limite do ADC.
- **Conclusão:** 60 fps com cor completa não é possível neste sensor. Os 60/s existem só como campos de meia cor. Opções: manter o 60 fps atual marcando os campos (`V4L2_FIELD_ALTERNATE`, para a aplicação combinar ou interpolar) ou usar o QVGA progressivo a 30 fps.
