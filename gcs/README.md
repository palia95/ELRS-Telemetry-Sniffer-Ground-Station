# ELRS TLM RX — ground control station for the ELRS telemetry sniffer

A small QGroundControl-style ground station that receives the CRSF telemetry
sniffed by the T3-S3 sniffer — over the **USB cable** or over **BLE** — decodes
it, optionally logs it to CSV, and shows it in a live web dashboard with a map
(position, heading, trail), an artificial-horizon attitude indicator, a compass,
and all decoded values.

```
gcs/
  gcs.py             backend: transports + CRSF parse + CSV log + web/websocket
  crsf.py            CRSF frame decoder (GPS/BATT/ATT/MODE/VARIO/BARO/LINK)
  static/index.html  dashboard (Leaflet map + attitude/HSI canvases + values)
  requirements.txt
  logs/              CSV logs land here (created on first record)
```

## Install

```bash
cd telemetry-sniffer/gcs
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

## Native macOS app

For a standalone app you double-click (native WKWebView window, no browser, its
own Dock icon), build the `.app` bundle:

```bash
cd telemetry-sniffer/gcs
./build_app.sh          # installs deps + PyInstaller, produces dist/ELRS TLM RX.app
open "dist/ELRS TLM RX.app"
```

It runs the exact same backend + dashboard as below, just in a native window.

- First launch of an unsigned app: right-click the `.app` → **Open** → **Open**.
- Approve the **Bluetooth** prompt on first BLE scan (the bundle already declares
  the usage string; macOS still asks the user once).
- `app.py` runs the app from source without packaging: `python3 app.py`.
- The build targets your Mac's architecture by default. For a true universal2
  (arm64 + x86_64) binary, set `TARGET_ARCH = 'universal2'` in `ElrsTlmRx.spec` —
  that needs a universal2 Python and universal2 wheels for aiohttp/pyobjc.

## Run (browser / headless)

Just start it and pick the source in the browser:

```bash
python3 gcs.py
```

On startup it **auto-connects to a serial cable** if one is plugged in
(it looks for a `usbmodem`/`usbserial`/`ttyACM`… port). Then open:

```
http://127.0.0.1:8760/
```

In the dashboard's **Source** panel you can switch transports live:
- pick a serial port from the dropdown and hit **Connect**, or
- hit **Scan BLE**, then click the discovered **ELRS TLM RX** (★ = it advertises
  the ELRS NUS service), or
- **Disconnect** the current source.

Serial parses the firmware's `[CRSF] ..` debug hex lines; BLE reads raw CRSF frames
from the Nordic UART Service. Only one source is active at a time.

You can also force a source from the CLI (skips auto-pick):

```bash
python3 gcs.py --serial /dev/cu.usbmodem2101 --baud 460800
python3 gcs.py --ble "ELRS TLM RX"      # name, MAC, or UUID
```

> Only one program can hold a given link: close `pio device monitor` before using
> serial, and disconnect nRF Connect before connecting the GCS over BLE.

### Binding phrase (set the ELRS UID)

The **Source** panel has a binding-phrase field. Type the target link's ELRS phrase
and hit **Set UID** (or Enter): the GCS sends `P:<phrase>` to the sniffer, which
re-derives the UID (`md5('-DMY_BINDING_PHRASE="…"')[:6]`), rebuilds its FHSS/CRC and
re-locks to that link — no reflash needed. Works over both serial and BLE.

The phrase is remembered in the browser and re-sent automatically whenever a
transport connects, so it effectively persists across reboots/reconnects. (The
firmware itself doesn't store it in flash; the compiled/`options.json` phrase is the
boot default until the GCS pushes one.)

### Logging

Click **● Record** in the header to start/stop a CSV (saved to `logs/`), or
auto-start at launch:

```bash
python3 gcs.py --serial /dev/cu.usbmodem2101 --csv                # logs/elrs_tlm_<timestamp>.csv
python3 gcs.py --serial /dev/cu.usbmodem2101 --csv myflight.csv   # explicit path
```

Each row is timestamped with every decoded field (lat/lon/alt/sats/speed/heading,
battery, attitude, mode, vario, link).

### Options

```
--serial PORT     serial port (e.g. /dev/cu.usbmodem2101)
--baud N          serial baud (default 460800)
--ble NAME|ADDR   BLE device name or address (e.g. "ELRS TLM RX")
--csv [PATH]      log to CSV (default logs/elrs_tlm_<ts>.csv)
--http-port N     dashboard port (default 8760)
--host ADDR       bind address (default 127.0.0.1)
```

## Notes / caveats

- **Map tiles need internet** (OpenStreetMap). The rest of the dashboard works
  offline; only the map background won't load without a connection.
- **RSSI / LQ** are the sniffer's *own* receive link quality (how well the T3-S3
  hears the target link), not the aircraft's. The aircraft doesn't send a CRSF
  `LINK_STATISTICS` frame, so the firmware injects one at 5 Hz built from
  `CRSF::LinkStatistics` — the same values shown on the OLED/heartbeat. Requires
  firmware built on/after 2026-07-30.
- Serial and BLE can both be given at once; frames from either are merged.
- The dashboard reconnects automatically if the backend restarts.
```
