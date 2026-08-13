# ELRS TLM RX — passive ELRS telemetry sniffer (LILYGO T3-S3)

A **receive-only** second receiver that overhears the telemetry your own
drone's ELRS receiver sends back to its handset — GPS, battery, attitude,
flight mode — decodes it to CRSF, and bridges it to a laptop/phone over BLE
and USB serial. Runs on a LILYGO T3-S3 (ESP32-S3 + SX1280). Requires knowing
the target link's **binding phrase** — this is a tool for monitoring **your
own aircraft** (a backup telemetry display, an antenna-tracker feed, a second
ground station), not for listening to links you don't operate.

Status: firmware, BLE bridge, OLED status display, and the GCS (macOS native
app + cross-platform Python) are built and validated on real hardware. The
Android app is a drafted skeleton, not yet functional. See **Roadmap** below.

## Credits — this project builds directly on:

- **[ExpressLRS](https://github.com/ExpressLRS/ExpressLRS)** (GPL-3.0). The
  firmware here is a small patch (6 edits to `rx_main.cpp`, see
  [`firmware/src/rx_main.patch.md`](firmware/src/rx_main.patch.md)) plus new
  files layered on top of stock ExpressLRS **3.5.6**. It reuses ExpressLRS's
  own FHSS/CRC/OTA/StubbornReceiver logic unmodified — the "sniffer" is
  mostly just *not transmitting* in the telemetry slot where a normal RX
  would. `firmware/` is a GPL-3.0 derivative work of ExpressLRS; this whole
  repo is licensed GPL-3.0 (see [`LICENSE`](LICENSE)) for consistency.
- **[HGSAFD8162/Expresslrs-Ghost-RX](https://github.com/HGSAFD8162/Expresslrs-Ghost-RX)**
  — the reference "ghost" passive-RX concept on ExpressLRS 3.5.0 / ESP8285,
  outputting over UART. This project independently reimplements that same
  core idea against ExpressLRS 3.5.6, ports it to the ESP32-S3-based T3-S3,
  and adds the OLED status display, BLE bridge, and GCS apps on top.

If you use or extend this, please keep both credits — none of the hard part
(the actual ELRS protocol/RF work) is ours. This repo is **not** a GitHub
fork of either project — it's a standalone rework/derivative that credits
and builds on both, since the actual change against ExpressLRS is a small
patch plus new files rather than a full in-tree modification.

## How it works

1. The binding phrase → 6-byte UID via MD5 (`md5('-DMY_BINDING_PHRASE="<phrase>"')[:6]`),
   exactly like ExpressLRS's own `binary_configurator.py`. That UID seeds the
   OTA CRC and the FHSS hop sequence — the same math a real second RX would
   use to lock onto the link.
2. The firmware locks to the transmitter's sync/hop sequence like a normal
   RX, but in the telemetry slot — where a real RX would *transmit* its
   telemetry — it stays in receive instead, and captures the aircraft RX's
   `PACKET_TYPE_TLM` uplink packets.
3. Telemetry chunks are reassembled into standard CRSF frames using the same
   `StubbornReceiver` logic the handset itself uses.
4. Completed CRSF frames fan out to whichever transports are enabled: USB
   serial (decoded + raw hex, for debugging), BLE (Nordic UART Service,
   notify), and the onboard SSD1306 OLED.

Full protocol write-up with source references: [`docs/01_protocol_reference.md`](docs/01_protocol_reference.md).

## Why a separate sniffer device, not the TX/handset module

We deliberately did **not** try to add this to the handset TX module instead
of building a separate receive-only device:

- **Not all TX/handset modules have BLE or WiFi.** Many are plain STM32-based
  ELRS TX modules with no wireless hardware beyond the SX1280 link itself —
  there's often nothing to bridge telemetry *out* over even if we wanted to.
- **Link robustness.** The TX module is the flight-critical side of the RC
  link. Adding a wireless broadcast task there — extra RF energy in-band on
  the same 2.4GHz hardware, extra CPU/interrupt load sharing the same
  time-critical chip — is a real risk to the control link for a
  nice-to-have telemetry feature, and that trade isn't worth it.
- A **separate, external, receive-only** device can't affect the RC link
  even in the worst case (firmware bug, crash, RF misbehavior) — it was
  never part of the transmit chain to begin with. It's also simpler: it
  only ever needs to be built once, independent of whatever TX hardware you
  own.

## Hardware

LILYGO T3-S3 (ESP32-S3 + SX1280). **Confirm your board revision before
flashing** — pin assignments differ between the non-PA (v1.0) and PA/FEM
(SX1280PA) variants, and using the wrong pins leaves the radio silently deaf
(inits over SPI fine, but the RxDone interrupt never fires).

Pins confirmed working on a PA/FEM revision board:

| Function | GPIO |
|---|---|
| SPI SCK / MISO / MOSI | 5 / 3 / 6 |
| Radio NSS / RST | 7 / 8 |
| Radio DIO1 / BUSY | **9 / 36** |
| RX/TX antenna switch (PA/FEM) | 21 / 10 |
| OLED (SSD1306) SDA / SCL | 18 / 17 |
| microSD/TF slot | onboard, unused by firmware today (see Roadmap) |
| Onboard charge/power IC | IP5306 (power-bank style; battery-capable) |

Full layout: [`firmware/hardware/lilygo_t3s3_sniffer_rx.min.json`](firmware/hardware/lilygo_t3s3_sniffer_rx.min.json).
Every hardware gotcha we hit (wrong pins, PA/FEM detection, OLED bring-up)
is logged in [`docs/05_working_config_and_gotchas.md`](docs/05_working_config_and_gotchas.md) — **read that first** if you're reproducing this on your own board.

## Compatibility

- **ExpressLRS 3.5.6** (pinned tag) — not tested against other versions.
- **LILYGO T3-S3, PA/FEM (SX1280PA) revision.** The non-PA v1.0 variant uses
  different antenna-switch pins; adjust `firmware/hardware/*.json` accordingly
  and re-verify.
- **GCS — macOS app:** built and tested on Apple Silicon. **GCS — Python:**
  cross-platform by design (aiohttp/bleak/pyserial), only exercised on macOS
  so far — Linux/Windows should work but aren't verified.
- **Android app:** not yet functional — skeleton only, see Roadmap.

## Binding phrase

The binding phrase is effectively a shared secret: anyone who has it can
derive your link's UID and decode its telemetry (and, per ExpressLRS's own
security model, its control link):

- `firmware/build.sh` takes the phrase as a **command-line argument** and
  writes it into `user_defines.txt` / `data/options.json` **inside the
  separate ExpressLRS checkout you point it at**.
- The UID can also be set **at runtime** without a reflash, over serial
  (`P:<phrase>`) or BLE — useful for switching target links without
  rebuilding firmware.

## Build & flash

```bash
# 1. Clone ExpressLRS separately, check out the pinned tag:
git clone https://github.com/ExpressLRS/ExpressLRS.git
cd ExpressLRS && git checkout 3.5.6 && cd ..

# 2. Build + integrate (patches rx_main.cpp, stages our sources, compiles):
./firmware/build.sh /path/to/ExpressLRS T3S3_Sniffer_2400_RX_Serial "your-binding-phrase"

# 3. Flash (see docs/04_flash_serial.md for full esptool/PlatformIO steps,
#    including the SPIFFS upload needed on first flash):
#    pio run -e T3S3_Sniffer_2400_RX_Serial -t upload -t uploadfs
```

For the BLE-enabled build, use env `T3S3_Sniffer_2400_RX_BLE` instead. Full
guides: [`docs/02_build_flash_guide.md`](docs/02_build_flash_guide.md),
[`docs/03_bringup_runbook.md`](docs/03_bringup_runbook.md) (phased bring-up
with success criteria), [`docs/04_flash_serial.md`](docs/04_flash_serial.md).

### Alternative firmware: broadcast EASA Direct Remote ID

Built to get ahead of the incoming sub-250g Remote ID rules with a working
(if unofficial and non-compliant — see `docs/06` §9) broadcast, and to give
other nearby pilots local situational awareness: that we're flying, and
roughly where.

Env `T3S3_Sniffer_2400_RX_RemoteID` re-purposes the BLE radio to **broadcast
EASA/ASTM Direct Remote ID** (the drone "license-plate" beacon) sourced from
the telemetry the sniffer overhears (GPS, plus VARIO/BARO/FLIGHT_MODE when the
aircraft sends them) — instead of streaming telemetry to a GCS. Both Legacy
(BT4) and Long-Range (BT5 Coded PHY) advertisements, built + confirmed on-air.
Derives vertical speed, height-above-takeoff, operator altitude, baro
altitude, and emergency status from that telemetry — no extra sensor needed.
Take-off/operator position latches at **arm**. Broadcast pauses entirely
whenever there's no GPS fix, rather than sending a Basic-ID-only "ghost" drone.

Your **Operator ID** (the EU UAS operator registration number — e.g. a 16-char
`SWEabcdefghijklm`-style value; **use your own**, not this placeholder) and
your **EU UA class** (`C0` or `Legacy`/no class marking — self-built aircraft
can't claim the manufacturer-declared C1–C6 classes, so those aren't offered;
default `C0`) are set once and stored in NVS, over BLE (write to
characteristic `6E400002`) or over USB serial / the GCS (`O:<id>`, `C:0`/`C:1`).
Full writeup, build/flash steps, Operator-ID format, and compliance status:
[`docs/06_remoteid_broadcast.md`](docs/06_remoteid_broadcast.md).

## Running the ground station (GCS)

**macOS native app:**
```bash
cd gcs && ./build_app.sh && open "dist/ELRS TLM RX.app"
```

**Cross-platform Python (browser dashboard):**
```bash
cd gcs && pip install -r requirements.txt
python3 gcs.py --serial /dev/cu.usbmodemXXXX   # or --ble "ELRS TLM RX"
# open http://127.0.0.1:8760/
```

Works fully offline (Leaflet map assets are vendored locally) except for the
OpenStreetMap basemap tiles themselves, which need internet. See
[`gcs/README.md`](gcs/README.md) for details.

**Android app:** not yet implemented — drafted skeleton in
[`android/`](android/) (CRSF parser, BLE client, Compose UI), no working
build yet.

## Roadmap / TODO

- [ ] **Android app** — finish the BLE client + live dashboard (parser and
      UI skeleton already drafted in `android/`).
- [ ] **microSD logging** — the T3-S3 has an onboard TF/SD slot, currently
      unused; log raw CRSF/telemetry to it for post-flight review without a
      phone/laptop connected.
- [ ] **GCS: flight statistics** — session summaries, distance/altitude/speed
      plots, beyond the current live-only dashboard.
- [ ] **GCS: network Remote ID** — relay decoded position as a *network*
      Remote ID feed (sent to a USS/service provider over the internet),
      distinct from *direct* (local broadcast) Remote ID, which is a
      separate, aircraft-side Direct Remote ID.
- [ ] **T3-S3 battery management** — the board's onboard IP5306 charge/
      power-bank IC already supports running from a LiPo; add charge-state
      reporting (voltage, charging/discharging) to the firmware heartbeat,
      OLED, and GCS, plus a low-battery warning.

## Responsible use

Receiving is passive, but monitoring a link you don't operate raises
radio-monitoring, privacy, and drone-surveillance questions that vary by
country. The unambiguous, intended use is **monitoring your own aircraft**.
Check local regulations before using this on anything else.

## Repository layout

```
telemetry-sniffer/
├─ README.md                 ← you are here
├─ LICENSE                    ← GPL-3.0 (inherited from ExpressLRS)
├─ docs/                      ← protocol reference, build/bringup guides
├─ firmware/                  ← patch + new sources layered on ExpressLRS 3.5.6
├─ gcs/                       ← Python ground station (macOS native + browser)
└─ android/                   ← Android app skeleton (not yet functional)
```
