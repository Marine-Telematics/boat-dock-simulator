"""Adaptador CAN <-> WebSocket com seleção de canal pela UI.

Linux + Mac, adaptador USB-CAN via slcan (porta serial). Um arquivo só:
  - GET  /        -> HTML enxuto (lista canais + conectar)
  - GET  /ports   -> JSON dos adaptadores detectados
  - WS   /stream  -> push dos frames; aceita comandos {id,data} pra TX

Roda:   python can_adapter.py          (acha o USB-CAN sozinho e abre o simulador)
Testa:  python can_adapter.py --selftest   (usa CAN virtual, sem hardware)

Boatshow: sem passo manual. O adapter varre socketcan → PEAK → gs_usb → slcan
a cada 2 s até achar um, reconecta se cair, e serve propulsion_scene.html em /.
"""
import asyncio
import json
import os
import random
import sys
import time
import webbrowser

import can
from aiohttp import web
from serial.tools.list_ports import comports

HOST, PORT = "127.0.0.1", 8765
HTML_SIM = os.path.join(os.path.dirname(os.path.abspath(__file__)), "propulsion_scene.html")
RANKING_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ranking.json")
SCAN_S = 2.0                               # varredura de adaptadores enquanto sem bus
ECUS = {0x11: "Port", 0x12: "Starboard"}   # os 2 dispositivos emulados (ECUs CM03)
STATUS_S = 0.04                            # ECUStatus a 25 Hz (CM03 doc §7.2)
# Emulação fiel ao firmware V1 v2.5.0 da ECU (mtcp.cpp / navigator.cpp / status_hub.h):
NAV_DEADLINE_S  = 0.20   # sem ECUN do dono por 200 ms -> neutro/0%, segue engajado
ENGAGE_TIMEOUT_S = 1.0   # sem ECUN do dono por 1 s -> desengaja (nada é publicado)
GEAR_TRAVEL_S   = 0.40   # atuador de marcha andando: gear|0x10 e throttle 0. Ajuste p/ sensação real.
# rpm publicado no ECUStatus (a manete mostra). Padrões abaixo; config.json ao lado sobrescreve
# qualquer chave (ex.: {"RPM_MAX": 3500, "RPM_TAU_DOWN_S": 0.5}). Sem reiniciar não vale.
RPM_IDLE, RPM_MAX = 600, 3500   # marcha lenta e rpm a 100 % de throttle
RPM_TAU_S = 0.8          # rampa subindo (1ª ordem): ~63 % do degrau em TAU, ~95 % em 3·TAU. 0 = instantâneo.
RPM_TAU_DOWN_S = 0.8     # idem descendo. Motor real cai mais rápido do que sobe: baixe este se quiser acentuar.
RPM_CURVE = 1.6          # alvo = idle + (max-idle)·(thr/100)^CURVE. >1: ganha pouco no início do curso, muito no fim. 1 = linear.
RPM_IDLE_JITTER = 10     # ± rpm de oscilação na marcha lenta (passeio aleatório lento). Some até 30 % de throttle. 0 = desliga.
RPM_IDLE_JITTER_STEP = 0.05   # passo do passeio por frame (25 Hz), fração da amplitude. Maior = oscila mais rápido.
CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")
if os.path.exists(CONFIG_FILE):
    with open(CONFIG_FILE) as _f:
        _cfg = json.load(_f)
    for _k in ("RPM_IDLE", "RPM_MAX", "RPM_TAU_S", "RPM_TAU_DOWN_S", "RPM_CURVE",
               "RPM_IDLE_JITTER", "RPM_IDLE_JITTER_STEP", "GEAR_TRAVEL_S"):
        if _k in _cfg:
            globals()[_k] = float(_cfg[_k])


def rpm_target(thr):
    return RPM_IDLE + (RPM_MAX - RPM_IDLE) * (thr / 100) ** RPM_CURVE
# Byte 4 do CTRStatus (product_bitstr) -> categoria de controle do ranking do simulador
CTR_PRODUCT = {0b0000: "cm300hd", 0b0010: "cm300hd", 0b1000: "cm05"}   # CM200, CM300, CM06

# MTNet: CAN id estendido = [prio:5][receiver:8][sender:8][command:8] (dataFrame.h)
# Enums reais: docs/CM03_ECU_Behavior.md Apêndice A.
ADDR = {0x00: "Unknown", 0x11: "Port", 0x12: "Starboard", 0x13: "TrollingValve",
        0x21: "Manet1", 0x22: "Manet2", 0x23: "Manet3", 0x24: "Manet4",   # CM 0x21-0x24 (NetworkId configurável)
        0x31: "LoraInterface", 0xFF: "Bcast"}
# 0x21/0x22 também são CTRStatus/CTRTransfer entre manetes (depende do sender) — cm300hd.md §6.6
CMD = {0x02: "ECUStatus", 0x11: "ECUEngage", 0x12: "ECUNavigate", 0x13: "ValveControl",
       0x21: "ECUStarterState", 0x22: "ECUStarterRequest"}   # 0x21/0x22 vindos de CM = CTRStatus/CTRTransfer
CM_ADDRS = range(0x21, 0x25)
# Thrusters do joystick — DOIS contratos chegaram no mesmo dia (18/09/2026), o adapter aceita os dois:
# (a) J1939 PDU2 proprietário, ID = (6<<26)|(0xFF<<16)|(PS<<8)|SA, PS 0x50 = Bow, 0x51 = Stern;
#     data = [direction 0 Off/1 Stbd/2 Port, power 0-100, status b0 Active b1 Fault, 0xFF*5].
#     Ver handoffs/joystick/can_protocol-joystick.md §6 (repo do joystick).
# (b) MTNet CTR Thruster cmd 0x25, DLC 3 [id 1 proa / 2 popa][direção 0 off / 1 BE / 2 BB][potência],
#     broadcast a 50 ms enquanto a estação comanda; sem frame por THRUSTER_TIMEOUT_S = off.
#     Ver handoffs/HANDOFF-joystick-can.md §6. Quando o firmware fechar em um, apagar o outro.
THRUSTER_PS = {0x50: "Bow", 0x51: "Stern"}
THRUSTER_NAMES = {1: "Bow", 2: "Stern"}
THRUSTER_TIMEOUT_S = 0.2
PRIO_HIGH = 0b00011                          # Protocol::Priority.High (Apêndice A.3)


def mtnet_id(prio, receiver, sender, command):
    return (prio << 24) | (receiver << 16) | (sender << 8) | command


def decode_id(arb):
    return (arb >> 24) & 0x1F, (arb >> 16) & 0xFF, (arb >> 8) & 0xFF, arb & 0xFF


def socketcan_ifaces():
    # Linux: lê /sys p/ achar netdevs do tipo CAN (gs_usb, PEAK, Kvaser, vcan...).
    # type 280 = ARPHRD_CAN. Em Mac não existe /sys -> retorna [] (usa slcan serial).
    out = []
    try:
        names = os.listdir("/sys/class/net")
    except OSError:
        return out
    for n in sorted(names):
        try:
            with open(f"/sys/class/net/{n}/type") as f:
                if f.read().strip() == "280":
                    out.append(n)
        except OSError:
            pass
    return out


_libusb_primed = False


def _prime_libusb():
    # Mac: o find_library do pyusb costuma achar a libusb da ARQUITETURA ERRADA
    # (ex.: x86_64 em /usr/local rodando num Python arm64) -> NoBackendError e o
    # adaptador some da lista sem aviso. Aqui "primamos" o cache global do pyusb
    # com a 1a libusb que carrega de fato; serve p/ Apple Silicon e Intel. Depois
    # disso GsUsb.scan()/find() (inclusive os do python-can) usam essa backend.
    global _libusb_primed
    if _libusb_primed:
        return
    _libusb_primed = True
    try:
        import glob
        import platform
        import usb.backend.libusb1 as libusb1
        if platform.system() == "Darwin":
            # gs_usb.start() faz detach_kernel_driver() também no Mac, mas o CANable é
            # classe vendor (sem driver de kernel) e o macOS recusa o detach -> Errno 13.
            # libusb reporta is_kernel_driver_active=True por quirk; forçamos False -> pula.
            import usb.core
            usb.core.Device.is_kernel_driver_active = lambda self, intf: False
        cands = [None]   # padrão já basta em Linux e Mac Intel; só falha no arm64
        for pat in ("/opt/homebrew/lib/libusb-1.0*.dylib",
                    "/usr/local/lib/libusb-1.0*.dylib",
                    "/opt/homebrew/Cellar/libusb/*/lib/libusb-1.0.dylib",
                    "/usr/local/Cellar/libusb/*/lib/libusb-1.0.dylib"):
            cands += sorted(glob.glob(pat))
        for c in cands:
            be = (libusb1.get_backend() if c is None
                  else libusb1.get_backend(find_library=lambda x, p=c: p))
            if be is not None:                 # cacheia globalmente no pyusb -> some pra todos
                return
    except Exception:
        pass


def gs_usb_count():
    # candleLight/gs_usb via libusb (caminho do Mac, onde não há SocketCAN).
    # Import guardado: no Linux o pacote nem está instalado -> 0, sem listar.
    try:
        from gs_usb.gs_usb import GsUsb
    except Exception:
        return 0
    _prime_libusb()
    try:
        return len(GsUsb.scan())
    except Exception as e:                      # não engole mais: dá a dica do brew
        print(f"[gs_usb] libusb indisponível ({e}). Tente: brew install libusb",
              file=sys.stderr)
        return 0


def pcan_channels():
    # PEAK PCAN-USB (e clones): Linux via socketcan (peak_usb); Mac via libPCBUSB (mac-can).
    try:
        return [c["channel"] for c in can.detect_available_configs(interfaces=["pcan"])]
    except Exception:
        return []


def usb_serial_ports():
    # Só portas com VID (USB de verdade): Bluetooth/debug-console não são slcan.
    return [p for p in comports() if p.vid is not None]


def auto_candidates():
    """Canais reais, na ordem de preferência. Vazio = nada plugado."""
    c = [f"socketcan:{n}" for n in socketcan_ifaces()]
    c += [f"pcan:{ch}" for ch in pcan_channels()]
    c += [f"gs_usb:{i}" for i in range(gs_usb_count())]
    c += [p.device for p in usb_serial_ports()]
    return c


def list_channels():
    # ponytail: pyserial já é dep do python-can[slcan], não adiciona nada.
    # "virtual:*" deixa testar a UI inteira sem hardware plugado.
    ports = [{"path": "virtual:test", "label": "Virtual (teste sem hardware)"}]
    ports += [{"path": f"socketcan:{n}", "label": f"SocketCAN {n}"} for n in socketcan_ifaces()]
    ports += [{"path": f"pcan:{ch}", "label": f"PEAK {ch}"} for ch in pcan_channels()]
    ports += [{"path": f"gs_usb:{i}", "label": f"candleLight/gs_usb #{i}"} for i in range(gs_usb_count())]
    ports += [{"path": p.device, "label": p.description or p.device} for p in usb_serial_ports()]
    return ports


def open_bus(channel, bitrate):
    if channel.startswith("virtual:"):
        return can.Bus(interface="virtual", channel=channel.split(":", 1)[1])
    if channel.startswith("socketcan:"):
        # bitrate é definido no `ip link` (kernel), não aqui. A iface precisa estar UP.
        return can.Bus(interface="socketcan", channel=channel.split(":", 1)[1])
    if channel.startswith("pcan:"):
        return can.Bus(interface="pcan", channel=channel.split(":", 1)[1], bitrate=bitrate)
    if channel.startswith("gs_usb:"):
        # Mac: candleLight via libusb. Aqui o bitrate VALE (o backend seta o timing).
        _prime_libusb()                         # python-can chama GsUsb.scan() c/ backend padrão
        idx = int(channel.split(":", 1)[1])
        return can.Bus(interface="gs_usb", channel=f"gs_usb{idx}", index=idx, bitrate=bitrate)
    # slcan = serial pura (CANable/USBtin) -> mesmo código em Linux e Mac.
    return can.Bus(interface="slcan", channel=channel, bitrate=bitrate)


def frame_json(msg, direction):
    j = {
        "type": "frame", "dir": direction,
        "id": f"0x{msg.arbitration_id:X}", "ext": msg.is_extended_id,
        "dlc": msg.dlc, "data": msg.data.hex(),
        "ts": round(msg.timestamp, 3),
    }
    if msg.is_extended_id:                      # decodifica MTNet pra UI ler nome do comando
        p, rcv, snd, cmd = decode_id(msg.arbitration_id)
        j["mt"] = {"prio": p, "to": ADDR.get(rcv, hex(rcv)),
                   "from": ADDR.get(snd, hex(snd)), "cmd": CMD.get(cmd, hex(cmd))}
    return j


class CanManager:
    """Um barramento ativo por vez (é um cabo físico só). Trocar de canal
    reabre o bus pra todos os clientes. Reconecta sozinho em queda/bus-off."""

    def __init__(self):
        self.bus = None
        self.channel = None
        self.bitrate = 250000             # CAN fixo em 250 kbps (CM03 doc §12)
        self.state = "idle"               # idle | connected | reconnecting
        self.detail = ""
        self.subscribers = set()
        # override=False -> ECU ecoa o ECUNavigate; True -> valores vêm de fora (set_ecu)
        self.ecu_state = {a: {"mode": 1, "gear": 0, "throttle": 0, "rpm": 600, "fail": 0,
                              "moving": False, "move_until": 0.0,
                              "engaged_to": None, "last_cmd": 0.0,
                              "override": False} for a in ECUS}
        self.ctr_ctrl = {}                # sender CM -> categoria (byte 4 do CTRStatus)
        self.thrusters = {}               # id -> {"direction", "power", "by", "t"}
        self._task = None
        self._lock = asyncio.Lock()
        self.auto = True                  # False quando alguém escolheu canal pela UI/URL

    def master(self):
        # Quem comanda: o sender engajado na ECU Port (as duas andam juntas na prática).
        eng = self.ecu_state[0x11]["engaged_to"] or self.ecu_state[0x12]["engaged_to"]
        return ADDR.get(eng, hex(eng)) if eng else None

    def _by(self, sender):
        return {"by": ADDR.get(sender, hex(sender)),
                "ctrl": self.ctr_ctrl.get(sender, "cm300hd")}

    def _emit_thruster(self, tid, th):
        # Mesmo formato que o HTML já trata: name Bow/Stern, direction 0 off / 1 BE / 2 BB, power 0-100
        self.broadcast({"type": "thruster", "name": THRUSTER_NAMES[tid], "direction": th["direction"],
                        "power": th["power"], "active": th["power"] > 0, "fault": False,
                        **self._by(th["by"])})

    def _effective(self, st):
        # O que a ECU realmente aplica: throttle 0 enquanto o atuador de marcha anda
        return {"gear": st["gear"], "throttle": 0 if st["moving"] else st["throttle"]}

    def snapshot(self):
        return {"type": "state", "channel": self.channel, "bitrate": self.bitrate,
                "state": self.state, "detail": self.detail, "clients": len(self.subscribers),
                "master": self.master(),
                "ecus": {ADDR[a]: {**st, **self._effective(st)} for a, st in self.ecu_state.items()}}

    async def auto_loop(self):
        """Sem bus: varre adaptadores a cada SCAN_S e abre o primeiro. Com bus caído,
        re-varre (o USB pode ter voltado em outro nome). Para quando `auto` é False."""
        searching = False
        while True:
            if self.auto and self.state != "connected":
                cands = await asyncio.get_running_loop().run_in_executor(None, auto_candidates)
                if cands:
                    searching = False
                    if cands[0] != self.channel or self.bus is None:
                        await self.ensure(cands[0], self.bitrate, auto=True)
                elif not searching:
                    searching = True
                    self.state, self.detail = "searching", "nenhum adaptador USB-CAN"
                    self.broadcast({"type": "status", "state": "searching"})
            await asyncio.sleep(SCAN_S)

    async def snapshot_loop(self):
        while True:                       # ~1 Hz: o HTML sincroniza mesmo sem eventos
            if self.subscribers and self.state == "connected":
                self.broadcast(self.snapshot())
            await asyncio.sleep(1.0)

    def set_ecu(self, name, fields):
        addr = next((a for a in ECUS if ADDR[a].lower() == str(name).lower()), None)
        if addr is None:
            raise ValueError("ecu inválida (Port|Starboard)")
        st = self.ecu_state[addr]
        for k in ("mode", "gear", "throttle", "rpm", "fail", "moving", "override"):
            if k in fields:
                st[k] = bool(fields[k]) if k in ("moving", "override") else int(fields[k])
        return st

    def broadcast(self, payload):
        for q in self.subscribers:
            if not q.full():
                q.put_nowait(payload)            # ponytail: cliente lento dropa frame, não trava o CAN

    async def ensure(self, channel, bitrate, auto=False):
        self.auto = auto
        async with self._lock:                   # serializa reabertura -> nunca 2 bus na mesma serial
            if channel == self.channel and self.bus is not None:
                return
            await self._stop()
            self.channel, self.bitrate = channel, bitrate
            self._task = asyncio.create_task(self._run())

    def send_frame(self, arb_id, data, ext=False):
        if not self.bus:
            raise RuntimeError("sem barramento conectado")
        msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=ext)
        self.bus.send(msg)
        self.broadcast(frame_json(msg, "tx"))

    async def _stop(self):
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None
        self.state, self.detail = "idle", ""

    async def _run(self):
        loop = asyncio.get_running_loop()
        backoff = 1
        while True:
            notifier = tx = None
            try:
                self.bus = open_bus(self.channel, self.bitrate)
                reader = can.AsyncBufferedReader()
                notifier = can.Notifier(self.bus, [reader], loop=loop)
                tx = asyncio.create_task(self._ecu_status())
                backoff = 1
                self.state, self.detail = "connected", ""
                self.broadcast({"type": "status", "state": "connected", "channel": self.channel})
                while True:
                    rx = asyncio.ensure_future(reader.get_message())
                    done, _ = await asyncio.wait({rx, tx}, return_when=asyncio.FIRST_COMPLETED)
                    if tx in done:               # TX morreu = bus caiu → propaga e reconecta
                        rx.cancel()
                        raise tx.exception() or RuntimeError("ecu_status parou")
                    msg = rx.result()
                    self._handle_rx(msg)
                    self.broadcast(frame_json(msg, "rx"))
            except asyncio.CancelledError:
                raise
            except Exception as e:               # queda de USB / bus-off / canal inválido
                self.state, self.detail = "reconnecting", str(e)
                self.broadcast({"type": "status", "state": "reconnecting", "detail": str(e)})
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, 10)
            finally:
                if tx:
                    tx.cancel()
                if notifier:
                    notifier.stop()
                if self.bus:
                    self.bus.shutdown()
                    self.bus = None

    async def _ecu_status(self):
        """Emula as 2 ECUs: watchdog de engate (1 s) + ECUStatus@25Hz (CM03 §7.1-7.2)."""
        while True:
            now = time.monotonic()
            for tid, th in list(self.thrusters.items()):     # thruster sem frame -> off
                if th["power"] and now - th["t"] > THRUSTER_TIMEOUT_S:
                    th["direction"], th["power"] = 0, 0
                    self._emit_thruster(tid, th)
            for addr, st in self.ecu_state.items():
                if st["engaged_to"] is not None and not st["override"]:
                    idle = now - st["last_cmd"]
                    if idle > ENGAGE_TIMEOUT_S:                  # COMMANDING_TIMEOUT: desengaja, em silêncio no bus
                        st["engaged_to"] = None
                        self._safe(st)
                        self.broadcast({"type": "sim", "ecu": ADDR[addr], "event": "watchdog_disengage"})
                    elif idle > NAV_DEADLINE_S and (st["gear"] or st["throttle"]):
                        self._safe(st)                           # NAV_REQUEST_DEADLINE: neutro/0, segue engajado
                        self.broadcast({"type": "sim", "ecu": ADDR[addr], "event": "watchdog_safe"})
                if st["moving"] and now >= st["move_until"]:     # atuador chegou: throttle passa a valer
                    st["moving"] = False
                    self.broadcast({"type": "sim", "ecu": ADDR[addr], "event": "navigate",
                                    **self._by(st["engaged_to"] or 0), **self._effective(st)})
                g = (st["gear"] & 0x0F) | (0x10 if st["moving"] else 0)
                eff = 0 if st["moving"] else st["throttle"]
                if not st["override"]:           # rpm segue o throttle com rampa (a manete mostra isso)
                    target = rpm_target(eff)
                    tau = RPM_TAU_S if target > st["rpm"] else RPM_TAU_DOWN_S
                    k = 1.0 if tau <= 0 else min(1.0, STATUS_S / tau)
                    st["rpm"] += (target - st["rpm"]) * k     # float: int só no frame, senão trava a 1 passo do alvo
                    # lenta "respirando": passeio aleatório limitado a ±JITTER, pesa 1 na lenta e 0 a partir de 30 %
                    step = RPM_IDLE_JITTER * RPM_IDLE_JITTER_STEP
                    st["jit"] = max(-RPM_IDLE_JITTER, min(RPM_IDLE_JITTER, st.get("jit", 0.0) + random.uniform(-step, step)))
                shown = st["rpm"] + st.get("jit", 0.0) * max(0.0, 1 - eff / 30)
                payload = bytes([st["mode"] & 0xFF, g, min(eff, 100) & 0xFF,
                                 (int(shown) >> 8) & 0xFF, int(shown) & 0xFF, st["fail"] & 0xFF])
                # Sem try: se o TX falha (USB caiu, bus-off) a tarefa morre e o _run,
                # que observa esta tarefa, reconecta. O erro de RX do Notifier nunca
                # chega ao _run (o asyncio só loga), então o TX é o detector de queda.
                self.send_frame(mtnet_id(0, 0xFF, addr, 0x02), payload, ext=True)
            await asyncio.sleep(STATUS_S)

    # --- emulação das ECUs: reação aos comandos da manete (CM03 §7.1) ---

    @staticmethod
    def _safe(st):
        st["gear"], st["throttle"], st["moving"] = 0, 0, False   # Neutral / 0%; mode segue Navigating

    def _handle_rx(self, msg):
        if not msg.is_extended_id:
            return
        _, rcv, snd, cmd = decode_id(msg.arbitration_id)
        if rcv == 0xFF and snd in THRUSTER_PS and len(msg.data) >= 3:   # (a) J1939 thruster (PF 0xFF, PS 0x50/51, SA = cmd)
            self._thruster(snd, cmd, msg.data)
            return
        if snd in CM_ADDRS and cmd == 0x25 and len(msg.data) == 3:   # (b) CTR Thruster 0x25
            tid, direction, power = msg.data[0], msg.data[1], min(msg.data[2], 100)
            if tid in THRUSTER_NAMES and direction <= 2:
                th = self.thrusters.setdefault(tid, {"direction": 0, "power": 0, "by": snd, "t": 0.0})
                changed = (th["direction"], th["power"]) != (direction, power)
                th.update(direction=direction, power=power, by=snd, t=time.monotonic())
                if changed:
                    self._emit_thruster(tid, th)
            return
        if snd in CM_ADDRS and cmd == 0x21 and len(msg.data) >= 5:   # CTRStatus: quem é o produto e se comanda
            ctrl = CTR_PRODUCT.get(msg.data[4] & 0x0F, "cm300hd")
            if self.ctr_ctrl.get(snd) != ctrl:
                self.ctr_ctrl[snd] = ctrl
            self.broadcast({"type": "sim", "event": "ctr_status", "state": msg.data[0],
                            "commanding": bool(msg.data[0] & 0x04), **self._by(snd)})
            return
        for addr, st in self.ecu_state.items():
            if rcv not in (addr, 0xFF):
                continue
            if cmd == 0x11:                      # ECUEngage: posse exclusiva
                self._engage(addr, st, snd, msg.data)
            elif cmd == 0x12:                    # ECUNavigate: comando de propulsão
                self._navigate(addr, st, snd, msg.data)

    def _thruster(self, ps, sa, data):
        # Gate de master: se alguém comanda as ECUs e não é este SA, o thruster dele não vale.
        master = self.ecu_state[0x11]["engaged_to"] or self.ecu_state[0x12]["engaged_to"]
        active = bool(data[2] & 0x01) and data[0] in (1, 2) and data[1] > 0 and master in (None, sa)
        self.broadcast({"type": "thruster", "name": THRUSTER_PS[ps], "direction": data[0],
                        "power": min(data[1], 100), "active": active, "fault": bool(data[2] & 0x02),
                        **self._by(sa)})

    def _engage(self, addr, st, sender, data):
        in_out = data[0] if data else 0
        if in_out == 0x31:                       # In
            if st["engaged_to"] in (None, sender):
                st["engaged_to"] = sender
                st["last_cmd"] = time.monotonic()
                ack = 0x01                       # Ok
            else:
                ack = 0x04                       # Engaged (já tomado por outro sender)
        elif in_out == 0x34:                     # Out
            if st["engaged_to"] == sender:
                st["engaged_to"] = None
                self._safe(st)
                ack = 0x01                       # Ok
            else:
                ack = 0x05                       # NotEngaged
        else:
            ack = 0x02                           # InvalidParameter
        self.send_frame(mtnet_id(PRIO_HIGH, sender, addr, 0x11),
                        bytes([ack, st["engaged_to"] or 0]), ext=True)
        self.broadcast({"type": "sim", "ecu": ADDR[addr], "event": "engage",
                        "ack": ack, **self._by(sender)})

    def _navigate(self, addr, st, sender, data):
        # ECU real: DLC exatamente 3, gear 0/1/2, throttle <= 100; qualquer outra coisa
        # e qualquer sender que não seja o dono são ignorados EM SILÊNCIO (sem auto-engage,
        # sem resposta, sem alimentar o watchdog). O mode (byte 0) é só armazenado.
        if len(data) != 3 or data[1] > 2 or data[2] > 100:
            return
        if sender != st["engaged_to"]:
            self.broadcast({"type": "sim", "ecu": ADDR[addr], "event": "navigate_ignored", **self._by(sender)})
            return
        gear, thr = data[1], data[2]
        now = time.monotonic()
        st["last_cmd"] = now                     # alimenta os dois watchdogs
        if st["override"]:
            return
        changed = gear != st["gear"] or thr != st["throttle"]
        if gear != st["gear"]:                   # troca de marcha: atuador anda, throttle 0 até chegar
            st["gear"] = gear
            if gear:
                st["moving"], st["move_until"] = True, now + GEAR_TRAVEL_S
            else:
                st["moving"] = False
        st["throttle"] = thr
        if changed:
            self.broadcast({"type": "sim", "ecu": ADDR[addr], "event": "navigate",
                            **self._by(sender), **self._effective(st)})


manager = CanManager()


async def index(_):
    if os.path.exists(HTML_SIM):
        return web.FileResponse(HTML_SIM)
    return web.Response(text=HTML, content_type="text/html")


async def adapter_page(_):
    return web.Response(text=HTML, content_type="text/html")


async def ranking_get(_):
    """Ranking do boatshow, no disco: sobrevive a limpar o browser / trocar de origem."""
    try:
        with open(RANKING_FILE) as f:
            return web.json_response(json.load(f))
    except (OSError, ValueError):
        return web.json_response([])


async def ranking_post(request):
    data = await request.json()
    if not isinstance(data, list):
        return web.json_response({"error": "esperado lista"}, status=400)
    tmp = RANKING_FILE + ".tmp"                 # escrita atômica: nunca fica meio arquivo
    with open(tmp, "w") as f:
        json.dump(data, f, ensure_ascii=False)
    os.replace(tmp, RANKING_FILE)
    return web.json_response({"ok": True, "n": len(data)})


async def on_startup(app):
    app["auto"] = asyncio.create_task(manager.auto_loop())
    app["snap"] = asyncio.create_task(manager.snapshot_loop())
    if "--no-browser" not in sys.argv:
        webbrowser.open(f"http://{HOST}:{PORT}/")


async def ports(_):
    """Canais disponíveis (adaptadores detectados + virtual de teste)."""
    return web.json_response(list_channels())


async def connect(request):
    """Conecta/troca o canal ativo. ch + bitrate via query (?ch=...) ou JSON."""
    data = dict(request.query)
    if not data and request.can_read_body:
        data = await request.json()
    ch = data.get("ch")
    if not ch:
        return web.json_response({"error": "faltou 'ch'"}, status=400)
    await manager.ensure(ch, int(data.get("bitrate", 250000)))
    return web.json_response(manager.snapshot())


async def state(_):
    """Estado do canal atual: conexão, bitrate, nº de clientes e estado das 2 ECUs."""
    return web.json_response(manager.snapshot())


async def set_ecu(request):
    """Sobrescreve valores de uma ECU emulada. JSON: {ecu:"Port", rpm:1500, override:true, ...}."""
    data = await request.json()
    try:
        manager.set_ecu(data.get("ecu"), data)
    except (ValueError, TypeError) as e:
        return web.json_response({"error": str(e)}, status=400)
    return web.json_response(manager.snapshot())


async def stream(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    ch = request.query.get("ch", "")
    rate = int(request.query.get("bitrate", "250000"))
    if ch:
        await manager.ensure(ch, rate)

    q = asyncio.Queue(maxsize=2000)
    manager.subscribers.add(q)
    q.put_nowait({"type": "status", "state": manager.state, "channel": manager.channel,
                  "detail": manager.detail})

    async def pump():
        while True:
            await ws.send_json(await q.get())

    pumper = asyncio.create_task(pump())
    try:
        async for raw in ws:                     # também detecta o cliente fechando
            if raw.type == web.WSMsgType.TEXT:
                try:
                    cmd = json.loads(raw.data)
                    if "ecu" in cmd:                          # sobrescreve valores de uma ECU
                        manager.set_ecu(cmd["ecu"], cmd)
                    else:                                      # injeta um frame cru no barramento
                        arb = cmd["id"] if isinstance(cmd["id"], int) else int(cmd["id"], 16)
                        manager.send_frame(arb, bytes.fromhex(cmd["data"]),
                                           ext=cmd.get("ext", False))
                except Exception as e:
                    await ws.send_json({"type": "status", "state": "cmd_error", "detail": str(e)})
    finally:
        pumper.cancel()
        manager.subscribers.discard(q)
    return ws


def make_app():
    app = web.Application()
    app.add_routes([
        web.get("/", index),                            # o simulador
        web.get("/adapter", adapter_page),              # escolha manual de canal
        web.get("/ranking", ranking_get),               # ranking persistido (ranking.json)
        web.post("/ranking", ranking_post),
        web.get("/ports", ports),                       # canais disponíveis
        web.route("*", "/connect", connect),            # conectar/trocar canal (GET ou POST)
        web.get("/state", state),                       # estado do canal atual
        web.post("/ecu", set_ecu),                       # sobrescrever valores de uma ECU
        web.get("/stream", stream),                     # push WS + comandos
    ])
    app.on_startup.append(on_startup)
    return app


HTML = """<!doctype html><meta charset=utf-8><title>CAN adapter</title>
<style>body{font:14px system-ui;margin:2rem;max-width:760px}
select,button,input{font:inherit;padding:.4rem}#log{height:50vh;overflow:auto;
background:#111;color:#0f0;padding:.5rem;white-space:pre;border-radius:6px}
.tx{color:#6cf}.st{color:#fc6}</style>
<h3>Adaptador CAN</h3>
<select id=ch></select>
<select id=br>
  <option>125000<option selected>250000<option>500000<option>1000000
</select>
<button onclick=conectar()>Conectar</button>
<button onclick=carregar()>↻ canais</button>
<div id=log></div>
<script>
let ws;
function carregar(){fetch('/ports').then(r=>r.json()).then(ps=>
  ch.innerHTML=ps.map(p=>`<option value="${p.path}">${p.label}</option>`).join(''))}
function line(t,cls){const d=document.createElement('div');if(cls)d.className=cls;
  d.textContent=t;log.prepend(d)}
function conectar(){
  if(ws)ws.close();
  ws=new WebSocket(`ws://${location.host}/stream?ch=${encodeURIComponent(ch.value)}&bitrate=${br.value}`);
  ws.onmessage=e=>{const m=JSON.parse(e.data);
    if(m.type=='status')line(`[${m.state}] ${m.channel||m.detail||''}`,'st');
    else if(m.type=='frame')line(`${m.dir.toUpperCase()} ${m.mt?`${m.mt.from}→${m.mt.to} ${m.mt.cmd}`:m.id}  ${m.data}`,m.dir=='tx'?'tx':'')}
  ws.onclose=()=>line('[socket fechado]','st');
}
carregar();
</script>
"""


async def _selftest():
    m = CanManager()
    await m.ensure("virtual:t1", 250000)
    q = asyncio.Queue()
    m.subscribers.add(q)

    for _ in range(100):                         # espera o bus abrir (virtual só entrega pra quem já está no canal)
        if m.state == "connected":
            break
        await asyncio.sleep(0.02)
    assert m.state == "connected"

    # injeta um frame de outro bus virtual no mesmo canal -> manager deve receber como RX
    injector = can.Bus(interface="virtual", channel="t1")

    async def wait_rx():
        while True:
            p = await q.get()
            if p.get("type") == "frame" and p["dir"] == "rx" and p["id"] == "0x123":
                return p

    waiter = asyncio.ensure_future(wait_rx())
    for _ in range(20):                          # entrega virtual é one-shot; reenvia até pegar
        injector.send(can.Message(arbitration_id=0x123, data=b"\xAB", is_extended_id=False))
        if waiter.done():
            break
        await asyncio.sleep(0.05)
    got = await asyncio.wait_for(waiter, 3)
    assert got["data"] == "ab", got

    # codec MTNet ida e volta
    assert decode_id(mtnet_id(5, 0x11, 0x21, 0x12)) == (5, 0x11, 0x21, 0x12)
    snap = m.snapshot()
    assert snap["state"] == "connected" and set(snap["ecus"]) == {"Port", "Starboard"}, snap

    # state machine da ECU: engate -> navigate (eco) -> watchdog -> override
    head, port = 0x21, 0x11

    def head_send(cmd, payload):
        injector.send(can.Message(arbitration_id=mtnet_id(0, port, head, cmd),
                                  data=payload, is_extended_id=True))

    head_send(0x11, bytes([0x31]))               # ECUEngage In
    await asyncio.sleep(0.15)
    assert m.ecu_state[port]["engaged_to"] == head, m.ecu_state[port]

    ack = None                                   # ECU deve responder ECUEngage com Ok=0x01
    for _ in range(50):
        rx = injector.recv(timeout=0)
        if rx and rx.is_extended_id and decode_id(rx.arbitration_id)[3] == 0x11:
            ack = rx.data[0]
            break
        await asyncio.sleep(0.02)
    assert ack == 0x01, f"ack esperado Ok=0x01, veio {ack}"

    head_send(0x12, bytes([1, 1, 50]))           # ECUNavigate: mode=1, gear=Forward, throttle=50
    await asyncio.sleep(0.15)
    assert m.ecu_state[port]["gear"] == 1 and m.ecu_state[port]["throttle"] == 50, m.ecu_state[port]

    await asyncio.sleep(1.2)                      # sem novos comandos -> watchdog desengata
    assert m.ecu_state[port]["engaged_to"] is None and m.ecu_state[port]["throttle"] == 0

    # navigate sem engage -> ignorado em silêncio (ECU real não auto-engaja)
    head_send(0x12, bytes([0, 2, 30]))
    await asyncio.sleep(0.15)
    assert m.ecu_state[port]["engaged_to"] is None and m.ecu_state[port]["gear"] == 0

    # CTRStatus diz o produto (CM300 = 0b0010) -> eventos passam a trazer ctrl
    injector.send(can.Message(arbitration_id=mtnet_id(0x1F, 0xFF, head, 0x21),
                              data=bytes([0x04, 0, 0, 0, 0b0010, 0x27]), is_extended_id=True))
    await asyncio.sleep(0.1)
    assert m.ctr_ctrl.get(head) == "cm300hd", m.ctr_ctrl

    # thruster J1939 do joystick: bow 0x18FF50xx -> evento thruster; SA de outro posto com master = inativo
    def thr_send(ps, sa, direction, power):
        injector.send(can.Message(arbitration_id=(6 << 26) | (0xFF << 16) | (ps << 8) | sa,
                                  data=bytes([direction, power, 1 if power else 0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]),
                                  is_extended_id=True))
    while not q.empty():
        q.get_nowait()
    thr_send(0x50, head, 1, 80)
    await asyncio.sleep(0.1)
    ev = [q.get_nowait() for _ in range(q.qsize())]
    th = [e for e in ev if e.get("type") == "thruster"]
    assert th and th[-1]["name"] == "Bow" and th[-1]["direction"] == 1 and th[-1]["power"] == 80 \
        and th[-1]["active"] and th[-1]["by"] == "Manet1", th

    # engate + troca de marcha: throttle 0 enquanto o atuador anda, depois vale
    head_send(0x11, bytes([0x31]))
    await asyncio.sleep(0.1)
    while not q.empty():
        q.get_nowait()
    head_send(0x12, bytes([0, 2, 30]))
    await asyncio.sleep(0.05)
    st = m.ecu_state[port]
    assert st["gear"] == 2 and st["moving"] and m._effective(st)["throttle"] == 0, st
    assert m.snapshot()["master"] == "Manet1"
    for _ in range(int((GEAR_TRAVEL_S + 0.1) / 0.03)):   # keepalive como a manete real (20-30 ms)
        head_send(0x12, bytes([0, 2, 30]))
        await asyncio.sleep(0.03)
    assert not st["moving"] and m._effective(st)["throttle"] == 30, st
    target = rpm_target(30)
    assert RPM_IDLE < st["rpm"] < target, st["rpm"]            # subindo, ainda na rampa
    for _ in range(int(4 * RPM_TAU_S / 0.03) + 1):
        head_send(0x12, bytes([0, 2, 30]))
        await asyncio.sleep(0.03)
    assert abs(st["rpm"] - target) <= 25, (st["rpm"], target)    # assentou (≈98 % em 4·TAU)
    ev = [q.get_nowait() for _ in range(q.qsize())]
    navs = [e for e in ev if e.get("event") == "navigate"]
    assert navs and navs[-1]["throttle"] == 30 and navs[-1]["ctrl"] == "cm300hd", navs
    for _ in range(int(8 * RPM_TAU_DOWN_S / 0.03)):              # neutro com keepalive: rpm volta a 600, não trava em 610
        head_send(0x12, bytes([0, 0, 0]))
        await asyncio.sleep(0.03)
    assert int(st["rpm"]) == RPM_IDLE, st["rpm"]

    # CTR Thruster 0x25: proa BE 60 % -> evento; 200 ms sem frame -> off
    while not q.empty():
        q.get_nowait()
    injector.send(can.Message(arbitration_id=mtnet_id(0x07, 0xFF, head, 0x25),
                              data=bytes([1, 1, 60]), is_extended_id=True))
    await asyncio.sleep(0.1)
    ths = [e for e in [q.get_nowait() for _ in range(q.qsize())] if e.get("type") == "thruster"]
    assert ths and ths[-1]["name"] == "Bow" and ths[-1]["direction"] == 1 and ths[-1]["power"] == 60, ths
    await asyncio.sleep(THRUSTER_TIMEOUT_S + 0.1)
    ths = [e for e in [q.get_nowait() for _ in range(q.qsize())] if e.get("type") == "thruster"]
    assert ths and ths[-1]["power"] == 0 and ths[-1]["active"] is False, ths

    # 200 ms sem ECUN -> neutro/0 mas segue engajado; 1 s -> desengaja
    await asyncio.sleep(NAV_DEADLINE_S + 0.1)
    assert st["gear"] == 0 and st["throttle"] == 0 and st["engaged_to"] == head, st
    await asyncio.sleep(ENGAGE_TIMEOUT_S)
    assert st["engaged_to"] is None, st
    assert isinstance(auto_candidates(), list)

    # queda do bus por baixo (USB re-enumerado): TX falha -> reconnecting -> connected de novo
    m.bus.shutdown()
    for _ in range(100):
        if m.state == "reconnecting":
            break
        await asyncio.sleep(0.05)
    assert m.state == "reconnecting", m.state
    for _ in range(100):
        if m.state == "connected":
            break
        await asyncio.sleep(0.05)
    assert m.state == "connected", m.state

    m.set_ecu("Starboard", {"rpm": 1500, "override": True})   # valor vindo de fora
    assert m.ecu_state[0x12]["rpm"] == 1500 and m.ecu_state[0x12]["override"] is True

    # trocar de canal reabre o bus e mantém só um ativo
    await m.ensure("virtual:t2", 250000)
    assert m.channel == "virtual:t2"
    assert isinstance(list_channels(), list) and list_channels()[0]["path"] == "virtual:test"

    await m._stop()
    injector.shutdown()
    print("selftest ok")


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        asyncio.run(_selftest())
    else:
        print(f"http://{HOST}:{PORT}")
        web.run_app(make_app(), host=HOST, port=PORT)
