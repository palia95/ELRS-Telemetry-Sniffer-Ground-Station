# ELRS TLM RX — passive ELRS telemetry sniffer (LILYGO T3-S3)

A **receive-only** second receiver that overhears the telemetry your own
drone's ELRS receiver sends back to its handset — GPS, battery, attitude,
flight mode — decodes it to CRSF, and bridges it to a laptop/phone over BLE
and USB serial. Runs on a LILYGO T3-S3 (ESP32-S3 + SX1280). Requires knowing
the target link's **binding phrase** — for monitoring **your own aircraft**
only, not for listening to links you don't operate.

Status: firmware, BLE bridge, OLED display, and the GCS (macOS native app +
cross-platform Python) are built and validated on real hardware. The Android
app is a drafted skeleton, not yet functional — see **Roadmap**.

## Gallery

<!--
  Images live in Img/ (repo root of telemetry-sniffer/), not yet added.
  Once a file lands there, drop it in below, e.g.:
    ![Sniffer board on the bench](Img/board_bench.jpg)
    ![GCS dashboard, live flight](Img/gcs_dashboard.jpg)
-->

*Photos coming soon — bare board, case/mount, and the GCS dashboard in use.*

## Credits

Builds directly on **[ExpressLRS](https://github.com/ExpressLRS/ExpressLRS)**
(GPL-3.0) — a small patch to `rx_main.cpp` plus new files layered on stock
3.5.6, reusing its FHSS/CRC/OTA/StubbornReceiver logic unmodified. The whole
repo is GPL-3.0 for consistency (see [`LICENSE`](LICENSE)). Also builds on
**[HGSAFD8162/Expresslrs-Ghost-RX](https://github.com/HGSAFD8162/Expresslrs-Ghost-RX)**,
the reference passive-RX concept this reimplements against 3.5.6 and ports to
the T3-S3. Not a fork of either — please keep both credits if you extend this.

## How it works

1. The binding phrase → 6-byte UID via MD5, exactly like ExpressLRS's own
   `binary_configurator.py`. That UID seeds the OTA CRC and FHSS hop
   sequence, same as a real second RX.
2. The firmware locks to the transmitter's hop sequence like a normal RX,
   but in the telemetry slot — where a real RX would *transmit* — it stays
   in receive and captures the aircraft RX's telemetry uplink instead.
3. Telemetry chunks are reassembled into CRSF frames via the same
   `StubbornReceiver` logic the handset uses.
4. Completed frames fan out to whichever transports are enabled: USB serial,
   BLE (Nordic UART Service), and the onboard OLED.

Full protocol write-up: [`docs/01_protocol_reference.md`](docs/01_protocol_reference.md).

## Hardware

LILYGO T3-S3 (ESP32-S3 + SX1280). **Confirm your board revision before
flashing** — pin assignments differ between non-PA (v1.0) and PA/FEM
(SX1280PA) variants; wrong pins leave the radio silently deaf.

| Function | GPIO |
|---|---|
| SPI SCK / MISO / MOSI | 5 / 3 / 6 |
| Radio NSS / RST | 7 / 8 |
| Radio DIO1 / BUSY | **9 / 36** |
| RX/TX antenna switch (PA/FEM) | 21 / 10 |
| OLED (SSD1306) SDA / SCL | 18 / 17 |

Full layout: [`firmware/hardware/lilygo_t3s3_sniffer_rx.min.json`](firmware/hardware/lilygo_t3s3_sniffer_rx.min.json).
Every hardware gotcha (wrong pins, PA/FEM detection, OLED bring-up) is in
[`docs/05_working_config_and_gotchas.md`](docs/05_working_config_and_gotchas.md) — **read that first**.

## Compatibility

- **ExpressLRS 3.5.6** (pinned, flashed/tested end-to-end). **3.5.x/3.6.x**
  should also work (patch anchors verified across the whole tag range, not
  runtime-tested beyond 3.5.6). **ExpressLRS 4.x will not build against this
  as-is** — it restructured the OTA packet-type scheme this patch hooks
  into; `integrate.py` fails loudly rather than mis-patching. Details in the
  code comments of `firmware/integrate.py`.
- **LILYGO T3-S3, PA/FEM revision.** Non-PA v1.0 uses different antenna
  pins — adjust `firmware/hardware/*.json` and re-verify.
- **GCS:** macOS app tested on Apple Silicon; the Python backend is
  cross-platform by design but only exercised on macOS so far.
- **Android app:** not yet functional — see Roadmap.

## Binding phrase

The binding phrase is effectively a shared secret — anyone who has it can
derive your link's UID and decode its telemetry. `firmware/build.sh` bakes
it into the firmware at build time, or set it at **runtime** without a
reflash over serial (`P:<phrase>`) or BLE.

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
[`docs/03_bringup_runbook.md`](docs/03_bringup_runbook.md),
[`docs/04_flash_serial.md`](docs/04_flash_serial.md).

### Alternative firmware: broadcast EASA Direct Remote ID

Env `T3S3_Sniffer_2400_RX_RemoteID` re-purposes the BLE radio to broadcast
EASA/ASTM Direct Remote ID sourced from the telemetry the sniffer overhears,
instead of streaming to a GCS. **Not a compliance story** — an unofficial
heads-up to nearby pilots that we're flying and roughly where, ahead of the
incoming sub-250g Remote ID rules. **Field-tested on real flights**
(2026-08-14/15) — position, altitude, arm state, Operator ID, and EU
classification all confirmed live in DroneTag and the OpenDroneID reference
app. Full write-up, setup, and compliance status:
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

See [`gcs/README.md`](gcs/README.md) for details. Android app: drafted
skeleton in [`android/`](android/), not yet functional.

## Roadmap / TODO

- [ ] **Android app** — finish the BLE client + live dashboard.
- [ ] **microSD logging** — the T3-S3's onboard TF/SD slot is unused today.
- [ ] **GCS: flight statistics** — session summaries beyond the live dashboard.
- [ ] **GCS: network Remote ID** — relay position to a USS over the internet,
      distinct from the aircraft-side Direct Remote ID broadcast above.
- [ ] **T3-S3 battery management** — charge-state reporting from the onboard
      IP5306 IC, plus a low-battery warning.

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
├─ Img/                       ← photos referenced from the Gallery section above
├─ docs/                      ← protocol reference, build/bringup guides
├─ firmware/                  ← patch + new sources layered on ExpressLRS 3.5.6
├─ gcs/                       ← Python ground station (macOS native + browser)
└─ android/                   ← Android app skeleton (not yet functional)
```
