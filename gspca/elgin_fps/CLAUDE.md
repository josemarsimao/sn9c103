# CLAUDE.md: projeto driver Elgin (SN9C103 + OV7630)

Leia primeiro: `NOTAS_REGISTRADORES.md` (a seção "Varredura das fontes", no fim, resume o que cada arquivo da pasta `sn9c103/` traz) (conhecimento acumulado, com legenda de confiança), depois `MAPA_REGISTRADORES.html` (registrador por registrador) e `BITS_ANALISE_60FPS.html` (bits do modo 60 fps). Este arquivo só resume o essencial e as combinações de trabalho.

## O projeto
- Driver gspca `gspca_elgin_fps` (`elgin_fps.c`) para a webcam Elgin: ponte Sonix SN9C103 + sensor OmniVision OV7630, USB `0c45:608f`. Kernel 7.0.0-38. Compilar/recarregar: `./umilmod.sh` (descarrega, compila com `make ELGIN_BUILD=<AAAAMMDDHHMM>`, instala, `depmod`, `modprobe`; termina com `echo "done"`).
- Versão do módulo: `elgin-AAAAMMDDHHMM` (`modinfo gspca_elgin_fps | grep -i "author\|description\|version"`). O `Makefile` repassa `ELGIN_BUILD` ao compilador.
- Consumidor: **Multiview**, em `/home/josemar/Downloads/projects/claude/multiview` (conversores de pixel em `/home/josemar/Downloads/projects/claude/v4l2misc/camconv.cpp`). **Nunca alterar** `/home/josemar/Downloads/projects/multiview` (cópia antiga).
- Ordem combinada: primeiro melhorar o driver; depois o **emulador de câmera** (driver que lê um vídeo e se comporta como uma câmera completa, com controles, modos e FPS).
- Critério: driver completo, com as funções padrão do V4L2 mesmo que o kernel não as habilite (ex.: `VIDIOC_DBG_*` sob `#ifdef`).

## Como trabalhar com o Josemar
- Responda em **português**, **curto**, **uma coisa de cada vez**, explicando comandos e termos de forma simples.
- Pergunta de teste ("seria possível...?", "como funciona...?"): dê a **resposta e o custo/impacto primeiro**; só implemente quando ele pedir.
- Não mude o que ele mandou deixar como está. Antes de editar um arquivo importante, faça cópia `.bak_<assunto>` na mesma pasta.
- Cabeçalho dos arquivos novos/alterados: Autor (Josemar Simão <josemars@ifes.edu.br>), Co-autor e Revisor (Claude), comentário de propósito em cada função e comentários no corpo.
- `MODULE_AUTHOR`: "Original: Jean-François Moine", "Modificador: Josemar Simão", "Revisor: Claude (Anthropic)". `MODULE_DESCRIPTION("GSPCA USB Elgin Camera Driver")`.
- Documentação: toda descoberta vai para NOTAS, MAPA e BITS (os scripts que geravam os dois HTML não existem mais; edite-os com `doc_html.py`: `nota()` acrescenta nota a um registrador do MAPA, `bloco()` acrescenta quadro no BITS, `troca()` troca trecho único, `salva(assunto)` confere a tabela e grava com cópia `.bak_<assunto>`; `python3 doc_html.py checa ARQ` só confere).
- Câmera: `/dev/video0` costuma ser a câmera do laptop; descubra a Elgin com `v4l2-ctl --list-devices` (os scripts de `testes/` usam `elgin_dev.acha_dev()`).
- Testes dependem de cena **parada e bem iluminada**; scripts pedem `sudo` uma vez.

## Fatos técnicos que já custaram caro
- **Registradores da ponte** (acesso: parâmetros do módulo `regr`, `regw`, `regs_start`; item `b:0xREG=0xVAL` = ponte, `s:0xREG=0xVAL` = sensor). Faixa 0x40–0x7F é só leitura (dados que mudam sozinhos); 0x80–0xFF espelha 0x00–0x7F; 0x31–0x3E não guardam valor.
- **Formatos Bayer crus** (paridade de H_START 0x12 / V_START 0x13 sobre a base BGGR): BA81=SBGGR8 (base) · GBRG (H_START+1) · GRBG (V_START+1) · RGGB (os dois +1). O S910 (comprimido) só existe em BGGR.
- **Ganhos da ponte 0x05/0x06/0x07 agem por posição (linha, coluna)**: 0x05→(1,0), 0x06→(0,1), 0x07→diagonal (0,0)+(1,1). O driver escolhe o registrador conforme o formato (GBRG: 0x05=R, 0x06=B, 0x07=G; GRBG: 0x05=B, 0x06=R, 0x07=G; BGGR/RGGB: 0x05 e 0x06 = verdes, 0x07 = R+B juntos, o Blue gain fica sem efeito, decisão mantida).
- O quadro do OpenCV no Multiview é **BGR** (o primeiro byte escrito é Blue): `sgbrg2rgb24` é GBRG de verdade.
- **Janela de exposição automática** (0x1C–0x1F, blocos de 32 px): o chip entrega a **soma** da luminosidade da janela; o driver escala o alvo pela área (`ae_area`, 117 = janela do Windows). Mudar a janela sem atualizar `ae_area` invalida a medida. Controles: menu "AE window" + 4 números (valem só em Custom).
- Sensor: AWB = COMA 0x12 bit 2; ganhos R/B do sensor 0x02/0x01 (padrões 0x3a/0x77); SAT/CNT/BRT só agem em YUV (sem efeito em Bayer); VFLIP = COMN 0x75 bit 7 (confirmado); BANDF = COMJ 0x2D bit 2.
- 60 fps: S910 160x120 (59,5 fps medidos); COMC 0xA0, COMH 0x80, COMJ 0xC1.
- USB: o kernel enumera o dispositivo (reset, endereço, descritores) antes do nosso driver; os registradores da ponte são acessados por `usb_control_msg()` na EP0; o gspca escolhe o alt setting (banda isócrona).

## Pendências (resumo; ver NOTAS, seção "Pendências" e "Melhorias futuras")
- **Feito (2026-10-17):** `sonixb.c` atual em `gspca/sonixb_2026/`: igual ao de 2022 (só um comentário). Achado: a init dele grava `0x61 = 0x4A` (BRTEN ligado); a nossa (Windows) grava 0x42 e desliga o brilho do sensor (NOTAS, seção "`sonixb.c` atual").
- **Ordem combinada (2026-10-17):** (1) `varre_bits.py reservados`; (2) testes de combinação dirigidos, sensor e depois ponte; (3) melhorias nos controles (com o lembrete acima).
- **Onde paramos (2026-10-16):** decisão do modo 60 fps ainda pendente (manter com `FIELD_ALTERNATE`, trocar por QVGA progressivo a 30 fps, ou os dois; 60 fps com cor completa foi refutado em 2026-10-13). Ferramentas novas em `testes/`, prontas mas ainda não rodadas com a câmera: `tela_padrao.py` (janela de padrões no notebook: +/- tamanho, "a" alinhar; comandada pelos scripts via `janela.py`), `testa_linha.py` (linha única × duas linhas: cor de cada amostra e comprimento da linha; usa a janela, `--celular` para o modo manual) e `varre_bits.py` (varredura bit a bit: fases `sensor` 511 casos ~32 min, `ponte` 188, `reservados` 472; `--ensaio`, `--continua`; gera CSV, resumo e HTML). O Josemar vai montar um suporte para a câmera a ~5 cm da tela (foco bom). Ordem combinada: `testa_linha.py` → `varre_bits.py sensor --ensaio` → `varre_bits.py sensor`. Depois: melhorias nos controles.
- **Próximo passo combinado (2026-10-13): registradores do sensor sem conclusão, nesta ordem:**
  1. 0x7E/0x7F: fórmula com valores intermediários e restauração para 0x00 (`testes/testa_7e7f.py`, pronto, ~5 min).
  2. Balanço de branco, num teste só com a cartela de cores: 0x0C/0x0D (alvos do AWB, controles "AWB Target Blue/Red", efeito não medido), 0x12 bit 2 (AWB do sensor, não equilibra bem; `testa_awb.py` desatualizado) e 0x6B/0x6C por canal.
  3. Testes rápidos "tem efeito ou não" (decidem se os controles ficam): 0x2D bit 2 (filtro de banda, `band_stop_filter`, efeito não medido), 0x06 (brilho do sensor; provável efeito só em YUV), 0x03/0x05 (saturação/contraste, sem efeito medido) e 0x69 com AE/AG desligados.
- Testar no Multiview o efeito de cada janela de AE no brilho automático (qualidade do brilho ainda ruim; balanço de branco do sensor não equilibra bem).
- Atualizar NOTAS/MAPA/BITS com: janelas padrão de AE por modo e controles novos (a correção do "brilho preso em 174", o cabeçalho de 18 bytes e a janela restaurada já foram registrados em 2026-10-13).
- Testes novos vindos da varredura das fontes (NOTAS, seção "Varredura das fontes"): `s:0x61=0x4a` (BRTEN) para o brilho `0x06` agir na Bayer; `0x17=0x60` + `0x19=0x50` em BA81 (clock do Risolia para cru); offsets por canal do sensor (`0x21`/`0x22`/`0x2E` com `0x26` bit 1 e `0x27` bit 2); pré-ganhos R/B `0x60`; gama do sensor `0x14` bit 2; média `0x7C`.
- Registradores sem documentação a estudar por cor: sensor 0x7E/0x7F (hipótese: contador de 16 bits de linhas extras), 0x6B/0x6C por canal com cartão de cor, 0x69 com AE/AG desligados, `0x19=0x50` para raw.
- Verificar se BA81/S910 saem com R/B trocados no `bayer2rgb24` (teste com objeto vermelho vivo).
- Melhoria futura: controles XY da janela de AE fora de "Custom" (hoje guardam o valor sem efeito).
- Contraste/saturação do sensor não atuam em Bayer: decidir manter ou remover.
- S_CROP / media controller (adiado).
- Git (raiz em `sn9c103/`): tudo commitado em 2026-10-13 na `main` (commits 7a5cfce e seguintes; o Josemar prefere trabalhar direto na `main`, sem branch). Push: o Josemar faz pelo VS Code (logado no GitHub; nesta máquina o git do terminal não tem credencial). Conferir com `git status -sb`. `.gitignore` exclui restos de compilação e `__pycache__`. As cópias `.bak_*` antigas foram removidas em 2026-10-13 e continuam no histórico (commit 7a5cfce): `git show 7a5cfce:gspca/elgin_fps/<arquivo>.bak_<assunto>`.
- Travamento ao trocar de resolução em BA81 sem reiniciar o Multiview (NOTAS, Pendências item 7): reproduzir e ver `dmesg | grep -iE 'bug|oops|warn|call trace'`.
- Scripts em `testes/` ainda não rodados: `testa_novos.py`, `testa_ganhos.py` (atualizar às tabelas de formato/posição), `testa_7e7f.py`; `testa_awb.py` está obsoleto.

## Cuidados
- Log do driver (`KDBG`, nível info, prefixo `Elgin_sn9c103:`): parâmetro `dbg_level` (0 = nada, 1 = eventos, padrão; 2 = tudo, cada registrador e controle; o ganho automático gera muitas mensagens). Muda na hora: `echo 2 | sudo tee /sys/module/gspca_elgin_fps/parameters/dbg_level`. `DEBUG_103 0` remove tudo na compilação. O `apply_fps` (chamado a cada ajuste do ganho automático) só aparece no nível 1 quando a taxa muda e uma vez por captura. Validado em 2026-10-13: uma sessão no Multiview = 19 mensagens.
- O `gspca_sonixb` (driver original do kernel, mesmo ID USB `0c45:608f`) não está na blacklist e volta a carregar ao reconectar a câmera; numa reconexão ou boot ele pode ficar com a câmera (aconteceu no boot de 2026-10-13 19:23: a câmera virou "USB camera"; o `./umilmod.sh` devolve ao nosso driver). Conferir com `v4l2-ctl --list-devices` (nome "Elgin camera") ou `lsmod | grep gspca`. Blacklist só com autorização do Josemar (`/etc/modprobe.d/blacklist.conf`).
- Compilar **sem sudo** (`umilmod.sh` faz `make` como usuário e para se a compilação falhar; só a instalação, o `depmod` e o `modprobe` usam sudo). Nunca rodar `sudo make` na pasta: deixa `.ko`/`.o` com dono root e o próximo `make` falha por permissão. 
- O `Makefile` e outros arquivos de build podem ser protegidos contra escrita por ferramentas remotas; nesse caso entregue como `Makefile.novo` e o Josemar faz `mv`.
- Editores com o arquivo aberto podem sobrescrever o que for gravado por fora; o mesmo vale para gravações atrasadas por ferramentas remotas. Depois de gravar, confira tamanho/conteúdo.
