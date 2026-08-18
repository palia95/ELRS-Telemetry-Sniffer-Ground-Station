# ELRS TLM RX — ground control station for the ELRS telemetry sniffer

A small QGroundControl-style ground station that receives the CRSF telemetry
sniffed by the T3-S3 sniffer — over **USB** or **BLE** — decodes it,
optionally logs it to CSV, and shows it in a live web dashboard with a map,
attitude indicator, compass, and all decoded values.

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

Standalone `.app` you double-click, no browser needed:

```bash
cd telemetry-sniffer/gcs
./build_app.sh          # installs deps + PyInstaller, produces dist/ELRS TLM RX.app
open "dist/ELRS TLM RX.app"
```

Same backend + dashboard as below, just in a native window. First launch of
an unsigned app: right-click → **Open** → **Open**, and approve the
Bluetooth prompt on first BLE scan. Run from source without packaging:
`python3 app.py`.

## Run (browser / headless)

```bash
python3 gcs.py
```

Auto-connects to a serial cable if one's plugged in, then open
`http://127.0.0.1:8760/`. In the **Source** panel you can switch transports
live: pick a serial port and **Connect**, or **Scan BLE** and click the
discovered device, or **Disconnect**.

Or force a source from the CLI:

```bash
python3 gcs.py --serial /dev/cu.usbmodem2101 --baud 460800
python3 gcs.py --ble "ELRS TLM RX"      # name, MAC, or UUID
```

> Only one program can hold a given link: close `pio device monitor` before
> using serial, and disconnect nRF Connect before connecting the GCS over BLE.

### Binding phrase (set the ELRS UID)

The **Source** panel has a binding-phrase field — type the target link's
phrase and hit **Set UID**: the GCS sends `P:<phrase>` to the sniffer, which
re-derives the UID and re-locks, no reflash needed. Remembered in the
browser and re-sent automatically on every connect.

### Logging

Click **● Record** to start/stop a CSV (saved to `logs/`), or auto-start:

```bash
python3 gcs.py --serial /dev/cu.usbmodem2101 --csv                # logs/elrs_tlm_<timestamp>.csv
python3 gcs.py --serial /dev/cu.usbmodem2101 --csv myflight.csv   # explicit path
```

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

- **Map tiles need internet** (OpenStreetMap); the rest of the dashboard
  works offline.
- **RSSI/LQ are the sniffer's own receive link quality**, not the
  aircraft's — the aircraft doesn't send a CRSF `LINK_STATISTICS` frame, so
  the firmware injects one at 5 Hz from its own link stats.
- Serial and BLE can both be given at once; frames from either are merged.
- The dashboard reconnects automatically if the backend restarts.
