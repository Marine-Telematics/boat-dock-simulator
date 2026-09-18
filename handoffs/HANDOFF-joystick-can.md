# Handoff — joystick CM04: como se comportar no MTNet para comandar as ECUs

**Data:** 18/09/2026 · **Origem:** simulador de atracação (`simulador/`, branch `boatshow-sp`) ·
**Para:** firmware do joystick (CM04) · **Referências:** firmware da ECU CM03 v2.5.0 (`mtcp.cpp`,
`navigator.cpp`, `core/status_hub.h`), firmware da manete CM300HD 2.7.0 (repo CM01: `mtcp.cpp`,
`navigator.cpp`, `manet.cpp`, `doc/mtcp_protocol.md`), `handoffs/mtcp/MTCP-planilha.md`.

O joystick precisa se comportar **exatamente como uma estação de comando CM01** aos olhos da ECU e
das outras estações. A ECU não tem nenhum caminho especial para joystick; tudo o que está abaixo é
o que a ECU real (e a emulada no simulador) aceita, e o que a manete faz hoje. **Decisão do Gabriel (18/09/2026):** o joystick assume o papel de
manete no fio, sem identificação própria (seção 5); os thrusters usam o comando novo CTR Thruster
0x25 (seção 6).

## 1. Identificação no barramento

| Item | Valor |
|---|---|
| CAN | 250 kbps, ID estendido 29 bits: `[prio:5][receiver:8][sender:8][command:8]`, dados little-endian salvo onde indicado |
| Endereço próprio | **0x21–0x24** (grupo CTR). O joystick usa **0x24** por default (definido em 18/09/2026); as manetes ficam com 0x21–0x23 (default da CM300HD é 0x21, e dois nós com o mesmo endereço entram em colisão e param). A arbitragem entre estações só funciona nessa faixa. Nunca use 0x11/0x12/0x13 (ECUs) nem 0x31 (CM06). |
| ECUs alvo | Port `0x11`, Starboard `0x12` (Middle `0x13` se existir) |
| Prioridades | ECUNavigate = Normal `0x07` · ECUEngage / CTRTransfer = High `0x03` · CTRStatus = Low `0x1F` |
| SEN (0x00) | Mandar uma vez ao entrar no bus (identificação). A ECU não exige, mas as telas usam. |

## 2. Quando a ECU está "online"

Uma estação só manda comando para ECU que publicou **ECUStatus (cmd 0x02) nos últimos 500 ms**.
Sem ECUStatus, a estação fica muda para aquela ECU. Isso é regra da manete e o joystick deve seguir
igual, senão comanda uma ECU que não existe e confunde a arbitragem.

ECUStatus vem da ECU a 40 ms, broadcast, 6 bytes: `[mode][gear|0x10 se atuador andando][throttle 0–100][rpm>>8][rpm&0xFF][fail]`.
RPM é **big-endian** (exceção ao resto do protocolo). `fail`: `0x02` gearbox inoperante, `0x04` throttle
inoperante (só esses dois existem no firmware v2.5.0). Falha de atuador **não** é motivo para parar de
comandar; só avisar.

## 3. Assumir o comando (arbitragem entre estações)

1. Publicar **CTRStatus (cmd 0x21, prio Low, broadcast, 6 bytes, a cada 100 ms)** sempre:
   `[state][io][err_module][err_code][product_bitstr][ver_maj<<4|ver_min]`.
   `state`: `0x01` Idle, `0x02` Requesting, `0x04` Commanding, `0x10` self-test, `0x20` Fail.
2. Só tentar assumir com o joystick **na posição neutra** (equivalente a "as duas alavancas em neutro").
3. Se nenhuma estação publicou CTRStatus com `Commanding` nos últimos **1000 ms** → vira master direto.
4. Se há master: ir a `Requesting`, mandar **CTRTransfer (cmd 0x22, prio High, DLC 0)** ao master,
   repetir a cada 250 ms. Vira master ao receber CTRTransfer de resposta com `0x04 Transfered`, ou ao
   ver o master antigo anunciar `Idle`. Resposta `0x02 Unable` (master com ECU fora de neutro) ou
   timeout de 1000 ms → volta a Idle. O master só cede com as ECUs em neutro.
5. Ao **ver outra estação com `Commanding`** enquanto é master: virar slave na hora, mandar ECUEngage
   Out para as ECUs engajadas, parar de mandar ECUNavigate.

## 4. Engajar e comandar (o que a ECU aceita)

Ao virar master, duas coisas em paralelo (a manete não faz "engage e depois navigate"; o ECUN é o
keepalive do master):

- **ECUEngage In** (cmd 0x11, prio High, receiver = ECU, DLC 1, `[0x31]`) a cada 100 ms para cada
  ECU online-mas-não-engajada, até a ECU responder `[ack][engaged_addr]` com ack `0x01 Ok`. Outros
  acks: `0x02` parâmetro inválido, `0x04` já engajada por outro (engaged_addr diz quem), `0x05` Out de
  quem não é dono. **Engage Out** = `[0x34]`.
- **ECUNavigate** (cmd 0x12, prio Normal, receiver = ECU, **DLC exatamente 3**) a cada **20 ms**
  (limite da ECU: 200 ms) para cada ECU online, **inclusive em neutro** (`[0x00, 0x00, 0x00]`):
  `[mode][gear][throttle]` com `mode` **sempre 0x00** (a ECU só armazena, não usa), `gear` `0x00`
  Neutro / `0x01` Avante / `0x02` Ré (sem bit 0x10), `throttle` 0–100. Qualquer outro DLC, gear > 2 ou
  throttle > 100 é descartado em silêncio.

Regras da ECU que o joystick tem de respeitar:

- ECUN de quem **não está engajado é ignorado em silêncio**: sem resposta, sem auto-engage, sem
  alimentar o watchdog. Se o Engage ainda não foi aceito, o barco não anda; isso é normal por até
  ~100 ms.
- **200 ms** sem ECUN válido do dono → ECU vai a neutro/0% (continua engajada). **1000 ms** → desengaja
  sem publicar nada; para retomar, novo Engage In.
- **Ao entrar em Navigating a ECU exige ver gear = Neutro ao menos uma vez** antes de aceitar
  Avante/Ré. Sempre comece mandando neutro.
- Troca de marcha: a ECU força throttle 0 enquanto o atuador anda (ECUStatus mostra `gear|0x10`);
  mandar throttle junto com a troca é aceito, só entra depois. Modo Dock **não vai no fio**: se o
  joystick tem um modo de manobra com limite de aceleração, escale o throttle antes de mandar.
- Nunca transmitir com sender igual ao de outra estação: a manete que vê um frame com o próprio
  endereço entra em `ErrColision` e para tudo.

## 5. O que o simulador faz com isso

O adapter do simulador (`simulador/can_adapter.py`) emula as duas ECUs com as regras acima e move o
barco pelo gear/throttle **efetivamente aplicado**. A categoria do ranking (CM04 Joystick / CM05
EasyDock / CM300HD Manetes) vem do **byte 4 do CTRStatus** do sender, não do endereço. Hoje o adapter
mapeia `0b0000` CM200 e `0b0010` CM300 → manetes, `0b1000` CM06 → EasyDock, e qualquer outro valor cai
em "manetes".

O joystick usa o `product_bitstr` existente das manetes e entra no ranking na categoria
"CM300HD Manetes". Não há identificação própria de joystick no fio; o adapter fica como está.

## 6. Thrusters de proa e popa

Comando novo definido pelo Gabriel em 18/09/2026 (entra na planilha como **CTR Thruster, cmd
0x25**; 0x23/0x24 já são Position Sensor):

| Item | Valor |
|---|---|
| ID | prio Normal `0x07`, receiver `0xFF`, sender = estação (joystick `0x24`), cmd `0x25` |
| DLC 3 | `[thruster_id 1 proa / 2 popa][direção 0 off / 1 BE / 2 BB][potência 0–100]` |
| Período | 50 ms, um frame por id, **só enquanto a estação está Commanding** |
| Ceder o comando | último frame com direção 0 / potência 0 para os dois ids |

Mapeamento no joystick: X = proa e popa juntos na mesma direção (X > 0 = BE), zona morta 5 %.
O adapter traduz para o evento `thruster` do simulador e considera **off após 200 ms sem frame**
(`THRUSTER_TIMEOUT_S`). Só os thrusters que a embarcação escolhida na tela tem instalados respondem.

## 7. Como testar contra o simulador sem ECU real

1. `simulador/simulador.command` (Mac) ou `./simulador.sh` (Linux): sobe o adapter, acha o PEAK e
   abre o browser. O adapter publica ECUStatus das duas ECUs emuladas a 40 ms, responde Engage, e
   aplica ECUN com os watchdogs de 200 ms / 1 s.
2. Com o joystick no bus, o cabeçalho "Hardware CAN" da tela deve mostrar "comandando: ManetN"
   (N = endereço 0x21–0x24 escolhido) assim que o Engage for aceito.
3. `python3 simulador/can_adapter.py --selftest` documenta em código, com asserts, a sequência
   exata que a ECU emulada espera (engage → navigate keepalive → troca de marcha → watchdogs).
4. `/adapter` em `http://127.0.0.1:8765` mostra os frames crus e decodificados (MTNet) em tempo real.
