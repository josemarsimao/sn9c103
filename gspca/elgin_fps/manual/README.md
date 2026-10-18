# Manual da câmera Elgin (SN9C103 + OV7630)

Autor: Josemar Simão <josemars@ifes.edu.br> · Co-autor e Revisor: Claude (Anthropic)

Manual em etapas (por tema ou por registradores), com texto, figuras e os resultados das medidas.

## Pastas

| Pasta | Conteúdo |
|---|---|
| `modelo/` | **Modelo de formatação (apostila)** — coloque aqui o arquivo modelo (`.odt`, `.docx`, `.ott`, `.dotx` ou PDF de exemplo). A montagem do PDF segue esse modelo. |
| `capitulos/` | Texto de cada capítulo, em Markdown (independente do formato final) |
| `figuras/` | Figuras geradas (PNG em resolução dupla, para impressão) |
| `scripts/` | Scripts que geram as figuras a partir dos dados dos testes (`../testes/`) |

## Capítulos

| Nº | Arquivo | Tema | Estado |
|---|---|---|---|
| 1 | `capitulos/01_automaticos.md` | Controles automáticos: AEC, AGC, AWB, brilho e preto automáticos, medida de luz da ponte, automáticos do driver | Texto e figuras prontos; falta aplicar o modelo |

## Como regenerar as figuras

```bash
python3 scripts/figuras_cap01.py
```

Cada afirmação traz a fonte: **[datasheet]**, **[Windows]** (captura USB do driver original), **[medido]** (teste nesta câmera) ou **[hipótese]**.
