# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## ⚠️ Contextos de produto — abra o Claude na PASTA CERTA

Este diretório é o guarda-chuva de vários produtos MaTel, mas cada produto
é um CONTEXTO PRÓPRIO do Claude Code (CLAUDE.md + memória + `.claude/qms.json`
próprios). Abra a sessão DENTRO do repo do produto, não aqui na raiz:

| Contexto | Pasta | Repo GitHub |
|---|---|---|
| LCD 10.1" Automação (iVS2008) | `matel-ivs-display-p4/` | Marine-Telematics/matel-ivs-display-p4 |
| MFD iVS-LCD7 7" StartStop | `matel-engine-panel/` | Marine-Telematics/matel-mfd-fw |
| App móvel (Flutter) | `matel-mobile/` | Marine-Telematics/matel-mobile |
| Gateway CM06 (CAN↔BLE↔4G) | `matel-ivs-gateway-fw/` | Marine-Telematics/matel-ivs-gateway-fw |
| CM01 leveler (CM200/300) | `CM01/` | Marine-Telematics/CM01 |
| Aviação (Cirrus SR22) | `matel-aviation-fw/` + `matel-aviation-app/` | local |
| PoC leitor de fonia ATC | `atc-radio-reader/` | local |
| Plataforma Web (contexto Nautica) | `~/MaTel-web_platform` (fora deste workspace) | — |

Todas essas pastas são repos git independentes e estão no `.gitignore` daqui
(assim como `refs/`, `matel-p4-docs/` e os spikes `p4-*`). O iVS-2008 (CM2008)
vive FORA deste workspace, em `~/CM2008`.

## O que ESTE repo versiona

Só três coisas — tudo o mais é ignorado:

- `handoffs/` — contratos entre produtos (ver abaixo).
- `simulador/` — simulador de atracação para boatshow: `propulsion_scene.html`
  (um arquivo só) + `can_adapter.py` (ponte CAN↔WebSocket, serve o HTML e emula
  as ECUs). Tem CLAUDE.md próprio com a arquitetura. Rodar: `./simulador.sh`
  (Linux) ou `simulador.command` (Mac) → `http://127.0.0.1:8765/`; sem hardware,
  abrir o `.html` direto. Único teste: `python3 can_adapter.py --selftest`.
- `tools/` — CLI e piloto do kanban MaTelQMS (ver abaixo).

`README.md` na raiz é o guia de uso dos logos da marca (cores, área de
proteção, snippets Flutter/web) — não descreve este repo.

## Handoffs (`handoffs/`)

Um handoff é a resposta escrita de um produto a outro: cabeçalho com
**Data · Origem (cartão #N, quadro N) · Para (produtos/cartões destino)**, depois
o contrato. Nomes seguem `HANDOFF-<tema>-<produto>[-<estado>].md`, e o estado
conta a história do tema: sem sufixo = pedido, `-pronto` = o lado de origem
implementou, `-alinhado`/`-resposta` = o outro lado respondeu. Não sobrescreva
um handoff anterior — crie o próximo estado.

`handoffs/mtcp/` é o protocolo CAN entre painel StartStop e MFD (MTCP).
`MTCP-planilha.md` é extração fiel da planilha do fornecedor: **não editar** —
edita-se a planilha e reextrai.

## Commits

Conventional commits em pt-BR com escopo: `docs(mtcp): …`, `feat: …`, `fix: …`,
`handoff: …`. A mensagem narra a decisão (o "porquê"), não o diff.

## Kanban MaTelQMS (`tools/`)

- `tools/qms.py` — CLI stdlib-only (urllib) para o kanban em
  `https://matel.ind.br/projetos/<id>`. Roda **de dentro do repo do produto**:
  lê `.claude/qms.json` (project_id) e credenciais em
  `~/.config/matelqms/claude.json` (fora de qualquer git). Subcomandos no
  docstring do arquivo: `demandas`, `proxima`, `tarefa`, `mover`, `resultado`,
  `perguntas`, `pilot-log`, `subtarefa`, `criar`; fora do docstring ainda há
  `projetos` (mapear ids), `orfaos` (cartões em quadros sem contexto) e
  `pilot-resposta` (fecha uma pergunta do piloto).
- `tools/qms-watch.sh` — piloto (launchd, 15 min): para cada contexto, pega o
  próximo cartão "A fazer" do usuário Claude e dispara `claude -p "/executar <id>"`
  headless, com trava por contexto em `.claude/.voo.lock` (trava >3h = voo
  morto). `tools/qms-responder.sh` faz o mesmo para perguntas sem resposta no
  card; só `qms.py pilot-resposta` fecha a pergunta.
- `tools/qms_narra.py` — traduz o stream-json do claude em linhas pt-BR
  postadas como `pilot_log` no card.
- Os scripts assumem o workspace em `~/MaTel` e logs em `tools/logs/`
  (ignorado). **Este checkout está em `~/dev/marine/boat-dock-simulator` e
  `~/MaTel` não existe** — os `.sh` não funcionam daqui sem ajustar os
  caminhos (ou um symlink `~/MaTel`) antes de instalar o launchd.
