#!/usr/bin/env python3
"""
ELRS TLM RX - a tiny ground control station for the ELRS telemetry sniffer RX.

Receives CRSF telemetry from the T3-S3 ELRS TLM RX sniffer over EITHER:
  * serial / USB cable  - parses the firmware's "[CRSF] xx xx .." debug hex lines
  * BLE (Nordic UART)   - raw CRSF frames as GATT notifications

Transports are switchable AT RUNTIME from the web UI. On startup the GCS
auto-connects to a likely serial port if a cable is present; otherwise it waits
and you pick a source (Scan BLE / choose a serial port) in the dashboard.

  python3 gcs.py                       # auto: serial if present, else idle
  python3 gcs.py --serial /dev/cu.usbmodem2101
  python3 gcs.py --ble "ELRS TLM RX"
  python3 gcs.py --csv                 # also start CSV logging

Then open http://127.0.0.1:8760/
"""
import argparse
import asyncio
import csv
import datetime as dt
import json
import os
import queue
import re
import sys
import threading
import time

from aiohttp import web

import crsf

HERE = os.path.dirname(os.path.abspath(__file__))
STATIC = os.path.join(HERE, "static")

NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"     # notify (device -> us)
NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"     # write  (us -> device), e.g. "P:<phrase>"
NUS_SVC = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"

CSV_FIELDS = [
    "ts", "lat", "lon", "alt_gps_m", "sats", "groundspeed_kmh", "heading_deg",
    "batt_v", "batt_a", "batt_mah", "batt_pct",
    "pitch_deg", "roll_deg", "yaw_deg", "mode", "vario_ms", "baro_alt_m",
    "link_rssi1_dbm", "link_lq",
]

_HEX_LINE = re.compile(r"\[CRSF\]\s*([0-9a-fA-F ]+)")
_OPID_LINE = re.compile(r"\[RID\]\s*OPID=(.*)")   # Remote ID firmware: current Operator ID
_CLASS_LINE = re.compile(r"\[RID\]\s*CLASS=(C0|LEGACY)")   # Remote ID firmware: EU class (only these two)
_ADDR_RE = re.compile(r"^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$|^[0-9A-Fa-f\-]{36}$")


def list_serial_ports():
    try:
        from serial.tools import list_ports
    except Exception:
        return []
    out = []
    for p in list_ports.comports():
        out.append({"port": p.device, "desc": (p.description or "")})
    return out


def autopick_serial():
    keys = ("usbmodem", "usbserial", "wchusb", "slab", "ttyacm", "ttyusb", "cu.usb")
    for p in list_serial_ports():
        if any(k in p["port"].lower() for k in keys):
            return p["port"]
    return None


# --------------------------------------------------------------------------
class State:
    def __init__(self):
        self._d = {}
        self._lock = threading.Lock()
        self.frames = 0
        self.last_tlm = None   # epoch of last real telemetry frame (not LINK)

    def apply(self, fields, telemetry=True):
        with self._lock:
            self._d.update(fields)
            self._d["ts"] = time.time()
            # LINK_STATISTICS is injected by the sniffer at 5 Hz; it carries RSSI/LQ
            # but is NOT received drone telemetry, so it must not bump the frame
            # count or the telemetry-freshness clock.
            if telemetry:
                self.frames += 1
                self._d["frames"] = self.frames
                self.last_tlm = time.time()

    def snapshot(self):
        with self._lock:
            return dict(self._d)


def _settings_path():
    """A proper per-user config location, NOT relative to this script's own
    path - inside a packaged .app that resolves to Contents/Frameworks/,
    which is fragile (wiped on rebuild/re-signing) and not somewhere a user
    would think to look for their own recordings."""
    if sys.platform == "darwin":
        base = os.path.expanduser("~/Library/Application Support/ElrsTlmRx")
    elif sys.platform == "win32":
        base = os.path.join(os.environ.get("APPDATA", os.path.expanduser("~")), "ElrsTlmRx")
    else:
        base = os.path.join(os.environ.get("XDG_CONFIG_HOME", os.path.expanduser("~/.config")), "ElrsTlmRx")
    os.makedirs(base, exist_ok=True)
    return os.path.join(base, "settings.json")


def _default_log_dir():
    return os.path.expanduser("~/Documents/ElrsTlmRx Logs")


def _load_settings():
    try:
        with open(_settings_path()) as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError):
        return {}


def _save_settings(d):
    try:
        with open(_settings_path(), "w") as f:
            json.dump(d, f)
    except OSError as e:
        print(f"[gcs] could not save settings: {e}")


class CsvLogger:
    def __init__(self):
        self._f = None
        self._w = None
        self.path = None
        # Persisted across restarts (see _settings_path) - set once, remembered.
        self.log_dir = _load_settings().get("log_dir") or _default_log_dir()

    @property
    def active(self):
        return self._f is not None

    def set_log_dir(self, path):
        path = os.path.expanduser((path or "").strip())
        if not path:
            return
        self.log_dir = path
        settings = _load_settings()
        settings["log_dir"] = path
        _save_settings(settings)
        print(f"[gcs] log folder set to {path}")

    def start(self, path=None):
        if self.active:
            return self.path
        if path is None:
            os.makedirs(self.log_dir, exist_ok=True)
            stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
            path = os.path.join(self.log_dir, f"elrs_tlm_{stamp}.csv")
        self._f = open(path, "w", newline="")
        self._w = csv.DictWriter(self._f, fieldnames=CSV_FIELDS, extrasaction="ignore")
        self._w.writeheader()
        self.path = path
        return path

    def stop(self):
        if self._f:
            self._f.close()
        self._f = self._w = None

    def write(self, snap):
        if self._w:
            row = {k: snap.get(k, "") for k in CSV_FIELDS}
            row["ts"] = dt.datetime.fromtimestamp(snap.get("ts", time.time())).isoformat(timespec="milliseconds")
            self._w.writerow(row)
            self._f.flush()


# --------------------------------------------------------------------------
class TransportManager:
    """Owns exactly one active transport (serial thread OR BLE task), and can
    switch between them at runtime on request from the UI."""
    def __init__(self, hub, loop, baud):
        self.hub = hub
        self.loop = loop
        self.baud = baud
        self.kind = None       # 'serial' | 'ble' | None
        self.desc = None       # port or device label
        self.status = "idle"   # idle | connecting | connected | error
        self._ser_stop = None
        self._ser_thread = None
        self._ble_stop = None
        self._ble_task = None
        self._tx = queue.Queue()      # bytes to write out over the active serial port
        self._ble_client = None       # set while a BLE client is connected

    def info(self):
        return {"kind": self.kind, "desc": self.desc, "status": self.status}

    async def send_phrase(self, phrase):
        """Push a binding phrase to the sniffer so it recomputes the ELRS UID."""
        phrase = (phrase or "").strip()
        if not phrase:
            return
        payload = ("P:" + phrase + "\n").encode()
        if self.kind == "serial":
            self._tx.put(payload)
            print(f"[gcs] queued phrase over serial: {phrase!r}")
        elif self.kind == "ble" and self._ble_client is not None:
            try:
                await self._ble_client.write_gatt_char(NUS_RX, payload, response=False)
                print(f"[gcs] sent phrase over BLE: {phrase!r}")
            except Exception as e:
                print(f"[gcs] BLE phrase write failed: {e}")

    async def send_operator_id(self, opid):
        """Set the ODID Operator ID on the Remote ID firmware (mirrors "O:<id>")."""
        opid = (opid or "").strip()
        if not opid:
            return
        payload = ("O:" + opid + "\n").encode()
        if self.kind == "serial":
            self._tx.put(payload)
            print(f"[gcs] queued operator ID over serial: {opid!r}")
        elif self.kind == "ble" and self._ble_client is not None:
            try:
                await self._ble_client.write_gatt_char(NUS_RX, payload, response=False)
                print(f"[gcs] sent operator ID over BLE: {opid!r}")
            except Exception as e:
                print(f"[gcs] BLE operator ID write failed: {e}")

    async def request_operator_id(self):
        """Ask the device to echo its current Operator ID ("O?"). Serial only -
        the Remote ID BLE config characteristic is write-only (no read-back)."""
        if self.kind == "serial":
            self._tx.put(b"O?\n")
            print("[gcs] requested operator ID over serial")

    async def send_class(self, class_num):
        """Set the EU UA class on the Remote ID firmware. Only 0 (Legacy - no
        class marking) or 1 (C0) are valid - self-built aircraft can't
        legitimately claim the manufacturer-declared C1..C6 classes, so the
        firmware only accepts these two and the GCS only offers these two."""
        try:
            n = int(class_num)
        except (TypeError, ValueError):
            return
        if n not in (0, 1):
            return
        payload = ("C:" + str(n) + "\n").encode()
        label = "C0" if n == 1 else "Legacy"
        if self.kind == "serial":
            self._tx.put(payload)
            print(f"[gcs] queued EU class {label} over serial")
        elif self.kind == "ble" and self._ble_client is not None:
            try:
                await self._ble_client.write_gatt_char(NUS_RX, payload, response=False)
                print(f"[gcs] sent EU class {label} over BLE")
            except Exception as e:
                print(f"[gcs] BLE class write failed: {e}")

    async def request_class(self):
        """Ask the device to echo its current EU class ("C?"). Serial only."""
        if self.kind == "serial":
            self._tx.put(b"C?\n")
            print("[gcs] requested EU class over serial")

    async def send_time(self):
        """Push this machine's current UTC time to the Remote ID firmware
        ("T:<unix seconds>") so Location/System ODID timestamps aren't always
        "unknown". Uses the GCS host clock (not the firmware's, which has no
        RTC/GPS-time source of its own) - see RemoteID_SetTime() on the
        firmware side. Serial only: no BLE characteristic for this, matching
        the user's explicit "if present [on serial]" scope. Write-only, no
        request_time() - the firmware doesn't echo it back."""
        if self.kind == "serial":
            payload = ("T:" + str(int(time.time())) + "\n").encode()
            self._tx.put(payload)
            print("[gcs] queued UTC time sync over serial")

    def mark(self, status, desc=None):
        self.status = status
        if desc is not None:
            self.desc = desc
        try:
            self.loop.call_soon_threadsafe(
                lambda: asyncio.ensure_future(self.hub.broadcast_sources()))
        except RuntimeError:
            pass

    async def stop_current(self):
        if self._ser_stop:
            self._ser_stop.set()
        self._ser_stop = self._ser_thread = None
        if self._ble_stop:
            self._ble_stop.set()
        if self._ble_task:
            self._ble_task.cancel()
            try:
                await self._ble_task
            except (asyncio.CancelledError, Exception):
                pass
        self._ble_stop = self._ble_task = None
        self.kind = self.desc = None
        self.status = "idle"
        await self.hub.broadcast_sources()

    async def start_serial(self, port):
        await self.stop_current()
        self.kind = "serial"
        self.desc = port
        self.status = "connecting"
        self._ser_stop = threading.Event()
        self._ser_thread = threading.Thread(
            target=serial_reader, args=(self.hub, port, self.baud, self._ser_stop, self),
            daemon=True)
        self._ser_thread.start()
        await self.hub.broadcast_sources()

    async def start_ble(self, target, label=None):
        await self.stop_current()
        self.kind = "ble"
        self.desc = label or target
        self.status = "connecting"
        self._ble_stop = asyncio.Event()
        self._ble_task = asyncio.create_task(ble_reader(self.hub, target, self._ble_stop, self))
        await self.hub.broadcast_sources()


# --- transport workers -----------------------------------------------------
def serial_reader(hub, port, baud, stop, mgr):
    try:
        import serial
    except Exception:
        mgr.mark("error", f"{port} (pyserial not installed)")
        return
    while not stop.is_set():
        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                mgr.mark("connected", port)
                print(f"[gcs] serial connected {port} @ {baud}")
                buf = b""
                while not stop.is_set():
                    # flush any outbound commands (e.g. "P:<phrase>")
                    try:
                        while True:
                            ser.write(mgr._tx.get_nowait())
                    except queue.Empty:
                        pass
                    chunk = ser.read(256)
                    if not chunk:
                        continue
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        s_line = line.decode("utf-8", "replace")
                        mo = _OPID_LINE.search(s_line)
                        if mo:
                            hub.push_operator_id(mo.group(1).strip())
                            continue
                        mc = _CLASS_LINE.search(s_line)
                        if mc:
                            hub.push_class(1 if mc.group(1) == "C0" else 0)
                            continue
                        m = _HEX_LINE.search(s_line)
                        if not m:
                            continue
                        try:
                            frame = bytes(int(t, 16) for t in m.group(1).split() if t)
                        except ValueError:
                            continue
                        if len(frame) >= 4:
                            hub.submit(frame)
        except Exception as e:
            if stop.is_set():
                break
            mgr.mark("error", f"{port}: {e}")
            print(f"[gcs] serial error: {e}; retry in 2s")
            time.sleep(2)


async def ble_reader(hub, target, stop, mgr):
    try:
        from bleak import BleakClient, BleakScanner
    except Exception:
        mgr.mark("error", "bleak not installed")
        return
    streamer = crsf.FrameStreamer()

    def on_notify(_c, data):
        for fr in streamer.feed(bytes(data)):
            hub.submit(fr)

    while not stop.is_set():
        try:
            mgr.mark("connecting")
            dev = target
            if not _ADDR_RE.match(str(target)):
                dev = await BleakScanner.find_device_by_name(target, timeout=12)
            if not dev:
                mgr.mark("error", f"{target} not found")
                await asyncio.sleep(2)
                continue
            async with BleakClient(dev) as client:
                mgr._ble_client = client
                mgr.mark("connected")
                print(f"[gcs] BLE connected: {dev}")
                await client.start_notify(NUS_TX, on_notify)
                try:
                    while client.is_connected and not stop.is_set():
                        await asyncio.sleep(0.4)
                finally:
                    mgr._ble_client = None
        except asyncio.CancelledError:
            break
        except Exception as e:
            if stop.is_set():
                break
            mgr.mark("error", str(e))
            print(f"[gcs] BLE error: {e}; retry in 3s")
            await asyncio.sleep(3)


async def scan_ble(hub):
    hub.scanning = True
    await hub.broadcast_sources()
    devices = []
    try:
        from bleak import BleakScanner
        found = await BleakScanner.discover(timeout=6.0, return_adv=True)
        for addr, (d, adv) in found.items():
            name = (getattr(d, "name", None) or (adv.local_name if adv else "") or "")
            uuids = [u.lower() for u in (adv.service_uuids or [])] if adv else []
            match = (NUS_SVC in uuids) or ("tlm rx" in name.lower()) or ("elrs" in name.lower())
            devices.append({"addr": addr, "name": name,
                            "rssi": getattr(adv, "rssi", None), "match": match})
        devices.sort(key=lambda x: (not x["match"], -(x["rssi"] or -999)))
    except Exception as e:
        print(f"[gcs] BLE scan error: {e}")
        hub.scan_error = str(e)
    hub.ble_devices = devices
    hub.scanning = False
    await hub.broadcast_sources()


# --------------------------------------------------------------------------
class Hub:
    def __init__(self, loop, baud):
        self.loop = loop
        self.state = State()
        self.logger = CsvLogger()
        self.clients = set()
        self.queue = asyncio.Queue()
        self.mgr = TransportManager(self, loop, baud)
        self.ble_devices = []
        self.scanning = False
        self.scan_error = None

    def submit(self, frame):
        self.loop.call_soon_threadsafe(self.queue.put_nowait, frame)

    def push_operator_id(self, opid):
        """Thread-safe: broadcast the device's current Operator ID to WS clients."""
        msg = json.dumps({"type": "opid", "value": opid})
        self.loop.call_soon_threadsafe(lambda: asyncio.ensure_future(self._send_all(msg)))

    def push_class(self, class_num):
        """Thread-safe: broadcast the device's current EU class to WS clients."""
        msg = json.dumps({"type": "class", "value": class_num})
        self.loop.call_soon_threadsafe(lambda: asyncio.ensure_future(self._send_all(msg)))

    async def process_loop(self):
        while True:
            frame = await self.queue.get()
            r = crsf.parse_frame(frame)
            if not r:
                continue
            t, fields = r
            is_tlm = (t != crsf.LINK_STATISTICS)
            self.state.apply(fields, telemetry=is_tlm)
            snap = self.state.snapshot()
            if is_tlm and self.logger.active:
                self.logger.write(snap)
            await self.broadcast_telemetry(snap)

    def _sources_msg(self):
        return json.dumps({
            "type": "sources",
            "source": self.mgr.info(),
            "serial_ports": list_serial_ports(),
            "ble_devices": self.ble_devices,
            "scanning": self.scanning,
            "logging": self.logger.active,
            "log_path": self.logger.path,
            "log_dir": self.logger.log_dir,
        })

    def _telemetry_msg(self, snap):
        age = (time.time() - self.state.last_tlm) if self.state.last_tlm else None
        return json.dumps({"type": "telemetry", **snap,
                           "tlm_age": age,
                           "logging": self.logger.active,
                           "log_path": self.logger.path})

    async def _send_all(self, msg):
        for ws in list(self.clients):
            try:
                await ws.send_str(msg)
            except Exception:
                self.clients.discard(ws)

    async def broadcast_telemetry(self, snap):
        if self.clients:
            await self._send_all(self._telemetry_msg(snap))

    async def broadcast_sources(self):
        if self.clients:
            await self._send_all(self._sources_msg())


# --------------------------------------------------------------------------
async def ws_handler(request):
    hub: Hub = request.app["hub"]
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(request)
    hub.clients.add(ws)
    await ws.send_str(hub._sources_msg())
    await ws.send_str(hub._telemetry_msg(hub.state.snapshot()))
    try:
        async for msg in ws:
            if msg.type != web.WSMsgType.TEXT:
                continue
            try:
                cmd = json.loads(msg.data)
            except ValueError:
                continue
            c = cmd.get("cmd")
            if c == "scan_ble":
                asyncio.ensure_future(scan_ble(hub))
            elif c == "list_serial":
                await hub.broadcast_sources()
            elif c == "connect_serial" and cmd.get("port"):
                await hub.mgr.start_serial(cmd["port"])
            elif c == "connect_ble" and (cmd.get("addr") or cmd.get("name")):
                await hub.mgr.start_ble(cmd.get("addr") or cmd.get("name"), cmd.get("name"))
            elif c == "disconnect":
                await hub.mgr.stop_current()
            elif c == "set_phrase":
                await hub.mgr.send_phrase(cmd.get("phrase", ""))
            elif c == "set_operator_id":
                await hub.mgr.send_operator_id(cmd.get("operator_id", ""))
            elif c == "get_operator_id":
                await hub.mgr.request_operator_id()
            elif c == "set_class":
                await hub.mgr.send_class(cmd.get("class_num", 0))
            elif c == "get_class":
                await hub.mgr.request_class()
            elif c == "set_time":
                await hub.mgr.send_time()
            elif c == "log":
                if cmd.get("on"):
                    print(f"[gcs] logging -> {hub.logger.start()}")
                else:
                    hub.logger.stop()
                await hub.broadcast_sources()
            elif c == "set_log_dir":
                hub.logger.set_log_dir(cmd.get("dir", ""))
                await hub.broadcast_sources()
    finally:
        hub.clients.discard(ws)
    return ws


async def index_handler(request):
    return web.FileResponse(os.path.join(STATIC, "index.html"))


def build_app(hub):
    app = web.Application()
    app["hub"] = hub
    app.router.add_get("/", index_handler)
    app.router.add_get("/ws", ws_handler)
    app.router.add_static("/static/", STATIC)
    return app


async def main_async(args):
    loop = asyncio.get_running_loop()
    hub = Hub(loop, args.baud)
    if args.csv:
        print(f"[gcs] logging -> {hub.logger.start(getattr(args, '_csv_path', None))}")

    asyncio.create_task(hub.process_loop())

    app = build_app(hub)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, args.host, args.http_port)
    await site.start()
    print(f"[gcs] dashboard: http://{args.host}:{args.http_port}/")

    # startup transport selection: explicit flag wins, else auto serial-first
    if args.serial:
        await hub.mgr.start_serial(args.serial)
    elif args.ble:
        await hub.mgr.start_ble(args.ble, args.ble)
    else:
        port = autopick_serial()
        if port:
            print(f"[gcs] auto-connecting serial: {port}")
            await hub.mgr.start_serial(port)
        else:
            print("[gcs] no serial cable found - use the dashboard to Scan BLE or pick a port")

    try:
        while True:
            await asyncio.sleep(3600)
    finally:
        await runner.cleanup()


def main():
    ap = argparse.ArgumentParser(description="ELRS TLM RX ground control station")
    ap.add_argument("--serial", metavar="PORT", help="force a serial port")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--ble", metavar="NAME|ADDR", help="force a BLE device")
    ap.add_argument("--csv", nargs="?", const=True, default=False, help="log to CSV")
    ap.add_argument("--http-port", type=int, default=8760)
    ap.add_argument("--host", default="127.0.0.1")
    args = ap.parse_args()
    if isinstance(args.csv, str):
        args._csv_path = args.csv
        args.csv = True
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        print("\n[gcs] bye")


if __name__ == "__main__":
    main()
