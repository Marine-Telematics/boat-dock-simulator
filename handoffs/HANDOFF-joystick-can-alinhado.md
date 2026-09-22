# Handoff — joystick CM04: alinhamento com o firmware e o modo eixos novo

**Data:** 22/09/2026 · **Origem:** simulador de atracação (`simulador/`, branch `boatshow-sp`) ·
**Para:** firmware do joystick (CM04)

**Resposta a** `HANDOFF-joystick-can.md` (18/09/2026), a partir do que a sessão do firmware
confirmou em `~/dev/marine/joystick/esp32/main/mtnet.h` e `main.cpp`. Ainda **não testado
contra a placa real** — só contra o adapter (`can_adapter.py --selftest`) e bus virtual.

## 1. O que o firmware entregou diferente do handoff original

No modo "direto" (padrão), o fio é o mesmo do handoff (ECUEngage 0x11, ECUNavigate 0x12 a
20 ms, CTR Thruster 0x25 a 50 ms, CTRStatus 0x21 com `product_bitstr` 0b0010), com estes
ajustes:

- **Interpretação dos eixos alinhada com a `calcThrust()` do simulador** (pedido do Yuri em
  22/09/2026, implementado em `mtnet.h` to_nav/mix, testes de host passando, ainda não gravado):
  zona morta de 1,5 % no yaw e 2 % no módulo da translação, reescaladas; |Z| acima da zona morta
  = só diferencial, X/Y ignorados e thrusters off; senão |Y| ≥ 2|X| = os dois motores por Y;
  |X| ≥ 2|Y| = motores neutro e thrusters proa+popa iguais por X; diagonal = um motor só por Y
  (X>0 só BB, X<0 só BE), thrusters off. Throttle linear 0–100, sem rampa.
- **SEN (cmd 0x00) nunca é enviado** — o handoff pedia um SEN de identificação ao entrar no
  bus; o firmware não manda.
- **Sem escala Dock**: o throttle que vai no ECUNavigate é linear 0–100, sem limitar a 20 %
  antes de mandar.
- **Diferencial de giro invertido em relação ao que se esperaria**: Z>0 (horário) = **BB
  avante / BE ré** (não o contrário).
- **ECUNavigate do 0x13** (Middle/TrollingValve, quando existe) copia o mesmo gear/throttle
  do ECUNavigate mandado para o Port.
- **Pedir/ceder o comando** é por botão segurado ~1 s, não só pela lógica de neutro do
  handoff.

Nenhum desses pontos exige mudança no adapter — ele já segue o gear/throttle efetivamente
aplicado pela ECU, não reimplementa a lógica de zona morta ou escala do lado do joystick.

## 2. Comando novo: CTR Joystick Axes (cmd 0x26) — só no modo "eixos"

O joystick tem um segundo modo, "eixos", onde ele **não engaja, não manda ECUNavigate nem
CTR Thruster**; o CTRStatus fica `Idle`. Em vez disso manda os três eixos crus e quem
interpreta é o simulador (a tela, não o adapter).

| Item | Valor |
|---|---|
| ID | prio Normal `0x07`, receiver `0xFF` (broadcast), sender `0x24` (joystick), cmd `0x26` |
| DLC 8 | `[X lo][X hi][Y lo][Y hi][Z lo][Z hi][btn 0/1][0]` — `int16` little-endian, faixa -480..480 |
| Período | 50 ms |
| Convenção | X>0 = para BE (boreste) · Y>0 = avante · Z>0 = giro horário (proa para BE) |

## 3. Comando novo: CTR Joystick Config (cmd 0x27) — troca de modo

Endereçado ao joystick (`receiver 0x24`). DLC 0 = consulta; DLC 1 `[modo]` (0 = direto,
1 = eixos) = configura, e o joystick salva o modo em NVS. O joystick sempre responde em
broadcast, DLC 1, `[modo atual]`. O sender usado por quem pergunta/configura (o adapter, no
caso do simulador) é `0xFE` — nunca `0x21`-`0x24`, `0x11`-`0x13` nem `0x31`.

## 4. O que o simulador faz com cada um

- **0x26** (`simulador/can_adapter.py`, `_handle_rx`): normaliza os três eixos para -1..1
  (÷480, saturado) e emite `{"type":"joystick", x, y, z, btn, by, ctrl:"cm04"}` só quando
  algo muda. `ctrl` fica fixo em `"cm04"` — não passa pelo `CTR_PRODUCT` do CTRStatus, porque
  só o joystick manda 0x26. Sem frame por 200 ms (mesmo `THRUSTER_TIMEOUT_S` do CTR
  Thruster) → zera os eixos e emite de novo, o mesmo padrão do evento `thruster`.
  Na tela (`propulsion_scene.html`), o evento `joystick` força o modo joystick de tela
  (`switchToJoystick()`), marca a fonte de hardware (`hwSetSource`, que também entra a
  categoria `cm04` no ranking) e escreve `state.surge = y`, `state.sway = x`,
  `state.yaw = z` direto — a convenção do CTR Joystick Axes já bate com o sinal que
  `calcThrust()`/`applyXY()` usam para o joystick de tela, sem precisar inverter nada.
  Enquanto um posto físico está ativo (`hwActiveSource`), o arrasto do joystick de tela fica
  bloqueado, do mesmo jeito que já acontecia com as manetes de tela.
- **0x27**: o adapter expõe `GET`/`POST /joystick/mode` (HTTP, servido em
  `http://127.0.0.1:8765/`). `GET` manda a consulta (DLC 0); `POST {"mode":0|1}` manda a
  troca (DLC 1); os dois respondem 204 sem esperar resposta no CAN. A resposta broadcast do
  joystick vira o evento `{"type":"sim","event":"joystick_mode","mode":n,by,ctrl}`, que a
  tela mostra na seção Hardware (`joystick: direto/eixos`) com dois botões — "Direto" e
  "Eixos" — que chamam o `POST` acima.

## 5. Como testar contra o simulador sem a placa

`python3 simulador/can_adapter.py --selftest` cobre, com asserts, no mesmo estilo do bloco
do CTR Thruster: um 0x26 (X=240,Y=-480,Z=0,btn=1) virando evento `joystick` com x≈0.5, y=-1,
z=0, btn=1, ctrl `cm04`; 200 ms sem frame zerando e reemitindo; e um 0x27 DLC 1 `[1]`
virando `sim`/`joystick_mode` mode=1. Ainda falta validar isso com a placa CM04 real no bus.
