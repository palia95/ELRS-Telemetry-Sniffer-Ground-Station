# Quickstart — the validated working flow (T3‑S3 SX1280 PA)

Confirmed hardware: **LILYGO T3‑S3 rev 1.1**, ESP32‑S3FH4R2 (4 MB flash, native
USB‑CDC), **SX1280 PA** radio. This is the exact flow that produced a working,
telemetry‑decoding sniffer. For the *why* behind each choice and every gotcha,
see `docs/05_working_config_and_gotchas.md`.

## 1. One‑time setup

```bash
pip install -U platformio esptool
git clone --branch 3.5.6 --depth 1 https://github.com/ExpressLRS/ExpressLRS.git
# (this repo already has ExpressLRS/ checked out at 3.5.6)
```

## 2. Build (pass the TARGET link's binding phrase as the 3rd arg)

`build.sh` does everything: integrates the sniffer + patches `rx_main.cpp`,
writes the phrase to `user_defines.txt` (→ `-DMY_UID`), stages the SPIFFS files
`hardware.json` (pins) and `options.json` (`{uid, flash-discriminator}`), installs
the env, and compiles.

```bash
cd telemetry-sniffer/firmware
bash build.sh ../../ExpressLRS T3S3_Sniffer_2400_RX_Serial "your binding phrase"
```

The phrase is remembered, so later rebuilds can omit it. First build pulls the
ESP32‑S3 toolchain (a few minutes, once).

## 3. Flash BOTH app and filesystem

The pins/UID live in SPIFFS, so you must flash the **filesystem** too, not just
the app. Put the board in download mode first if upload can't sync (hold **BOOT**,
tap **RST**, release **BOOT**).

```bash
cd ../../ExpressLRS/src
pio run -e T3S3_Sniffer_2400_RX_Serial -t upload -t uploadfs --upload-port /dev/cu.usbmodem1101
```

- Rebuilt the **app** (code change)? → needs `-t upload`.
- Changed a pin / phrase (`hardware.json` / `options.json`)? → needs `-t uploadfs`.
- When unsure, do both (above).

## 4. Watch it

```bash
pio device monitor -e T3S3_Sniffer_2400_RX_Serial -p /dev/cu.usbmodem1101 -b 460800
```
(Must pass `-e …` or PlatformIO defaults to the `native` env and errors.)

Boot log should show `UID=(…)` = your phrase's UID, `Use RX pin: 21`,
`SX1280 Ready!`. Then a `[Ghost] …` heartbeat every second.

## 5. Validate

Power your **handset** (same phrase) and the **drone** (its ELRS RX + sensors):

1. Heartbeat cycles `SRCH rate=…` → settles to **`LOCK rate=150Hz`**, `raw`/`LQ` climb.
2. With the drone on, **`tlmPkts` climbs** and decoded lines appear:
   `GPS lat=… lon=… sats=…`, `BATT …V …%`, `ATT …`, `MODE …`, plus `[CRSF] …` hex.
3. Cross‑check against the handset's telemetry page — they should match.
   (GPS prints as raw deg×1e7; divide by 1e7.)

If it locks but `tlmPkts=0`, or never locks, the triage is in
`docs/05_working_config_and_gotchas.md` (§4, §5, §9 cover the usual suspects).

## Current build notes

- **OLED is disabled** (`GHOST_DISPLAY` off) — its blocking I²C init hung `setup()`.
  Re‑enabling it properly (non‑blocking, from `loop()`) is a TODO.
- Serial output uses ELRS `debugPrintf`, which supports only `%d %u %x %s %f`
  (no width specifiers) — keep that in mind if you add prints.
- BLE / WiFi transports are written but off; enable with the
  `T3S3_Sniffer_2400_RX_Wireless` env when you build the phone/laptop side.
