# CLAUDE.md — Simulador de propulsão

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## O que é

Simulador de manobra de atracação da Marine Telematics para boatshow: um barco
com dois motores (e thrusters opcionais) numa marina, jogado com joystick na
tela, gamepad USB, manetes na tela, ou o **hardware real** (manetes CM300HD,
joystick CM04) via CAN. Cronômetro, ranking por categoria, game over por colisão.

## Arquivos

| Arquivo | Papel |
|---|---|
| `propulsion_scene.html` | **O simulador.** Um arquivo só (~4000 linhas): CSS, SVG dos 3 cascos e dos 3 cenários, e o JS. |
| `can_adapter.py` | Ponte CAN ↔ WebSocket (python-can + aiohttp). Serve o HTML em `/`, acha o USB-CAN sozinho, emula as ECUs CM03, persiste o ranking. |
| `simulador.command` / `simulador.sh` | Launcher (Mac / Linux): venv na 1ª vez, sobe o adapter, que abre o browser. |
| `config.json` | Sensação do motor emulado: rpm de lenta/máximo, rampa (subida/descida), curva, oscilação na lenta, tempo do atuador de marcha. Lido na subida do adapter; chaves omitidas usam o padrão do código. |
| `requirements.txt` | Deps do adapter. gs_usb/pyusb só no Mac. |
| `propulsion_validator.html` | Visualizador estático da lógica de zonas do joystick, sem física. |
| `iate_top_view.svg` | Ícone original do iate. Não editado. |

## Rodar

- **Boatshow:** duplo clique em `simulador.command` (Mac) ou `./simulador.sh` (Linux). Abre `http://127.0.0.1:8765/`. PEAK no Mac exige a libPCBUSB (mac-can) instalada.
- **Sem hardware:** abrir `propulsion_scene.html` direto no browser. A seção Hardware fica em "sem adapter, tentando…", inofensivo.
- **Teste do adapter:** `python3 can_adapter.py --selftest` (bus virtual: MTNet, engate, navigate, watchdog, auto_engage, master). Único teste automatizado do projeto; rode após mexer no adapter.
- **Sintaxe do JS** após editar o HTML: extrair o `<script>` e `node --check`.
- `/adapter` na porta 8765 é a página de escolha manual de canal (raramente necessária).

## Fluxo do `propulsion_scene.html` (na ordem em que o JS está)

1. **Perfis de embarcação** (`vesselProfiles`): dimensões em px, thrusters instalados, constantes físicas e `responseTau` (inércia). `applyVesselProfile()` copia as constantes para as variáveis globais `ACCEL`, `ANG_ACCEL`, … e troca o SVG ativo.
2. **Ganhos** (sliders 5 a 100%) e **Ambiente** (vento: direção DE onde vem; correnteza: direção PARA onde vai).
3. **Entradas** → `state = {surge, sway, yaw}` (joystick de tela ou gamepad) ou `manetePort`/`maneteStbd` (manetes de tela ou hardware). `controlMode` decide qual vale.
4. **`calcThrust()`** → `thrust = {port, stbd, bow, stern}`. Joystick: zonas (yaw anel → diferencial; |surge| ≥ 2|sway| → ambos; |sway| ≥ 2|surge| → só thrusters; diagonal → um motor). Manetes de tela: curva F-N-R (`maneteToThrust`: neutro < 0.10, engajado/idle < 0.25, aceleração acima). **Hardware não passa pela curva:** `hwApplyNav` põe gear+throttle direto em `hwThrust` (neutro 0, engatado sem throttle = `MANETE_IDLE`, senão throttle/100); a posição do handle na tela é só visual. Thrusters da tela e do hardware são somados; têm rampa (`THRUSTER_RAMP`).
5. **`updatePhysics()`**: forças no referencial do casco → mundo, massa relativa vem de `responseTau`, joystick passa por "DP" (60% de autoridade, 1.6× de inércia), prop walk, vento com weathervane, correnteza, limites da cena, e **colisão por pontos do casco** (elipse de 16 pontos contra AABBs de `OBSTACLES`; impacto > `GAMEOVER_IMPACT` = game over).
6. **Passo fixo:** o game loop acumula tempo e integra em fatias de 1/60 s; render é por frame. Não voltar a integrar por frame: em tela de 120 Hz o barco anda o dobro.
7. **Cenários** (`SCENARIOS`): spawn (`dock`), vaga-alvo (`target` com `align` ew/ns) e obstáculos. Cada um tem um grupo SVG `#scn-<id>` em `#world-g`. `applyScenario()` troca tudo.
8. **Cronômetro**: começa no 1º comando, termina com o casco inteiro dentro da vaga, parado, alinhado, por 1,5 s → modal de fim e ranking.
9. **Ranking**: categoria = cenário · dificuldade · tipo de controle (cm04/cm05/cm300hd) · gênero. Fonte de verdade é `ranking.json` via `/ranking` do adapter quando servido por http; `localStorage` é fallback e é migrado na 1ª carga. Nomes passam por `escHtml`.
10. **Hardware CAN**: WebSocket na mesma origem, conecta sozinho e reconecta para sempre (backoff 1→5 s). Eventos `sim` com `by` trocam o tipo de controle do ranking e forçam modo manetes (`hwSetSource`). Arrasto das manetes de tela é bloqueado só depois que um posto físico comandou (`hwActiveSource`).
11. **Resets**: `resetBoat()` (spawn, velocidades, rastro) e `resetControls()` (todas as entradas). Use-os; não copie a sequência.

## Contrato WebSocket (adapter → HTML)

`status` (searching / connected / reconnecting) · `state` ~1 Hz com `master` e as ECUs (throttle já efetivo) · `sim` (engage com `ack`, navigate com gear/throttle **aplicados**, navigate_ignored, watchdog_safe aos 200 ms, watchdog_disengage a 1 s, ctr_status com `commanding`; todos com `by` e `ctrl`) · `thruster` (J1939 0x18FF50xx/51xx do joystick: `name` Bow/Stern, `direction` 0/1 Stbd/2 Port, `power`, `active` já com gate de master, `fault`, `by`) · `frame` cru.

**Reiniciou o adapter com a manete ligada → re-engaje a manete** (neutro + botão de comando, ou desliga/liga): a ECU emulada sobe sem dono e, fiel ao firmware, ignora ECUN de quem não engajou. Nunca deixe dois adapters rodando: os dois publicam ECUStatus e o rpm na manete "pula".

A emulação da ECU segue o firmware V1 v2.5.0 (fonte: sessão do repo CM03): ECUN de sender não engajado é ignorado em silêncio, sem auto-engage; troca de marcha segura throttle 0 por `GEAR_TRAVEL_S`; mode do ECUN é ignorado (a manete já escala Dock a 20 % antes de mandar). A manete CM300HD 2.7.0 manda ECUN a 20 ms só para ECU que publicou ECUStatus nos últimos 500 ms: **sem ECUS emulado a manete fica muda**. O adapter nunca transmite com origem 0x21–0x24 (a manete detectaria colisão e pararia).

## Convenções

- **Port = BB (Bombordo)**, **Starboard = BE (Boreste)**. Nunca "Estibordo" ou "EB".
- Hélices contra-rotativas: BB gira CCW avante (`invertCW=true`), BE CW.
- Texto de UI em pt-BR. Tema navy escuro (`#09131f`), verde = ativo.
- Botões de thruster são momentâneos (mousedown/touchstart até soltar).
- Números de calibração física ficam nos perfis, não espalhados no código.
