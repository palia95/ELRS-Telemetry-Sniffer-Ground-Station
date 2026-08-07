# Build & flash guide

## 0. Prerequisites

- ExpressLRS source at **3.5.x** (this repo has a worktree pattern; use tag
  `3.5.6` or the `Expresslrs-Ghost-RX` fork as a base).
- PlatformIO (VS Code extension or `pip install platformio`).
- A LILYGO T3‑S3 with the **SX1280 2.4 GHz** module (confirm it's SX1280, not
  SX1262/SX1276).
- USB‑C cable.

## 1. Drop the sniffer files into the ELRS tree

From this folder, copy into the ELRS `src/` tree:

```
firmware/src/sniffer.h                  ->  src/src/sniffer.h
firmware/src/sniffer.cpp                ->  src/src/sniffer.cpp
firmware/src/devTransport_BLE.cpp       ->  src/src/devTransport_BLE.cpp
firmware/src/devTransport_WiFiUDP.cpp   ->  src/src/devTransport_WiFiUDP.cpp
firmware/hardware/lilygo_t3s3_sniffer_rx.json  ->  keep handy for step 5
firmware/target/T3S3_Sniffer_RX.h       ->  src/include/target/  (only if using the static-pin path)
```

Then apply the six edits in `firmware/src/rx_main.patch.md` to
`src/src/rx_main.cpp`, and add the PlatformIO env from
`firmware/targets/t3s3_sniffer.ini` to `src/targets/` (or append it to a custom
`platformio_sniffer.ini`).

In `devTransport_*` init: call `BLE_Transport_Init("T3S3 Ghost RX")` and
`WiFiUDP_Transport_Init()` once from `setup()` (near the other `devicesRegister`
calls) so the transports register their sinks.

## 2. Set the binding phrase

**Bench (compiled in):** the env sets
`-D MY_BINDING_PHRASE=\"my-test-link-phrase\"` — change it to your target's
phrase. ELRS fills `UID[]` at boot; also call `Sniffer_SetUidFromPhrase()` once
in `setup()` if you want runtime override to work.

**Field (runtime):** comment out `MY_BINDING_PHRASE`, flash, then send
`P:<phrase>\n` to the BLE RX characteristic from the Android app.

## 3. Build

```bash
cd src
pio run -e T3S3_Sniffer_2400_RX
# firmware at: src/.pio/build/T3S3_Sniffer_2400_RX/firmware.bin
```

## 4. Flash the firmware

First flash over USB (subsequent updates can go over WiFi):

```bash
pio run -e T3S3_Sniffer_2400_RX -t upload
# or: esptool.py --chip esp32s3 write_flash 0x0 firmware.bin  (see esptool for full offsets/merged bin)
```

If the S3 doesn't auto‑enter download mode, hold **BOOT**, tap **RST**, release
**BOOT**, then upload.

## 5. Flash the hardware layout (unified firmware path)

The recommended build uses the unified ESP32 RX target, which reads pins from a
layout stored in the config partition. Provide
`lilygo_t3s3_sniffer_rx.json` via the ELRS WiFi web UI:

1. Power the board; it exposes a WiFi AP (`ExpressLRS RX` or similar) or you can
   force WiFi mode.
2. Browse to `http://10.0.0.1` (or `elrs_rx.local`).
3. Upload/select the hardware layout JSON.
4. Reboot.

(If you used the **static‑pin** `T3S3_Sniffer_RX.h` path instead, skip step 5 —
pins are compiled in.)

## 6. Verify

- OLED (if wired) shows link state / rate / RSSI.
- Serial monitor at 460800 shows sync + LQ once locked to the target link.
- BLE advertises as `T3S3 Ghost RX`.

Proceed to `03_bringup_runbook.md` for the staged validation.

## Notes

- **PA safety:** on the SX1280PA board keep the SX1280's own output ≤ 2–5 dBm.
  RX‑only never keys TX, but don't re‑enable TX at high power.
- **BLE + WiFi coexist** on the S3 but share the 2.4 GHz PHY with each other
  (not with the SX1280). If throughput matters, prefer one at a time.
- NimBLE and the ESP32 WiFi stack are already ELRS dependencies (`common.ini`
  `env_common_esp32` → `h2zero/NimBLE-Arduino`), so no new libraries are needed.
