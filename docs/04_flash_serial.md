# Build & flash the serial-debug firmware (T3-S3)

Goal for this stage: a firmware that locks to your test link, captures
telemetry, and **prints it over the USB serial** — no BLE/WiFi yet.

> **Heads-up on toolchains.** ExpressLRS builds with **PlatformIO**, not
> `idf.py`. So you *build* with PlatformIO and *flash* with either PlatformIO
> or `esptool.py` (which ships with ESP-IDF). `idf.py flash` won't work on an
> Arduino/PlatformIO project, but `esptool.py` flashes the resulting `.bin`
> just fine — that's the "ESP-IDF CLI" path below.

---

## 1. Build the `.bin`

You need the ExpressLRS 3.5.x source (tag `3.5.6` recommended, or the
`Expresslrs-Ghost-RX` fork). From this `telemetry-sniffer/firmware/` folder:

```bash
# 1a. edit the phrase first:
#     targets/t3s3_sniffer.ini  ->  -D MY_BINDING_PHRASE=\"<your target's phrase>\"

# 1b. one command: integrate + patch + build
./build.sh /path/to/ExpressLRS T3S3_Sniffer_2400_RX_Serial
```

`build.sh` runs `integrate.py` (copies the sniffer sources and applies
`rx_main_ghost.patch`), installs the target env, and builds. Result:

```
/path/to/ExpressLRS/src/.pio/build/T3S3_Sniffer_2400_RX_Serial/firmware.bin
```

Alongside it PlatformIO also produces `bootloader.bin`, `partitions.bin`, and
copies `boot_app0.bin` — the four pieces of a full flash image.

*(Manual equivalent, if you don't want the script:)*
```bash
cd /path/to/ExpressLRS/telemetry-sniffer/firmware
python3 integrate.py /path/to/ExpressLRS/src
cp targets/t3s3_sniffer.ini /path/to/ExpressLRS/src/targets/
cd /path/to/ExpressLRS/src
pio run -e T3S3_Sniffer_2400_RX_Serial
```

---

## 2. Put the T3-S3 in download mode

The ESP32-S3 uses **native USB**. Plug the board's USB-C into your computer.
If flashing fails to start:

1. Hold **BOOT** (aka IO0).
2. Tap **RST**.
3. Release **BOOT**.

On Linux the port is typically `/dev/ttyACM0`; on macOS `/dev/cu.usbmodemXXXX`;
on Windows a `COMx`. (`ls /dev/ttyACM*` / check Device Manager.)

---

## 3. Flash — option A: PlatformIO (simplest)

```bash
cd /path/to/ExpressLRS/src
pio run -e T3S3_Sniffer_2400_RX_Serial -t upload
```

PlatformIO computes the correct offsets and flashes all parts.

## 3. Flash — option B: `esptool.py` (the ESP-IDF CLI)

Install once: `pip install esptool`. Then, from the build dir
`.pio/build/T3S3_Sniffer_2400_RX_Serial/`:

```bash
esptool.py --chip esp32s3 --port /dev/ttyACM0 --baud 921600 \
  --before default_reset --after hard_reset \
  write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect \
  0x0000  bootloader.bin \
  0x8000  partitions.bin \
  0xe000  boot_app0.bin \
  0x10000 firmware.bin
```

Notes:
- `boot_app0.bin` lives in the Arduino-ESP32 package; PlatformIO copies it into
  the build dir. If missing, find it under
  `~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin`.
- If you prefer a **single image**, merge first, then flash at `0x0`:
  ```bash
  esptool.py --chip esp32s3 merge_bin -o ghost_merged.bin \
    0x0 bootloader.bin 0x8000 partitions.bin 0xe000 boot_app0.bin 0x10000 firmware.bin
  esptool.py --chip esp32s3 --port /dev/ttyACM0 write_flash 0x0 ghost_merged.bin
  ```
- The `partitions.bin` offset is `0x8000` for the ELRS `min_spiffs` layout used
  by this env. If you change the partition scheme, confirm the offset with
  `pio run -v` (look at the `esptool ... write_flash` line PlatformIO prints).

---

## 4. Flash the pin layout (one time)

This env uses the ELRS **unified** ESP32 target, so pins come from a runtime
hardware layout, not the firmware. After the first flash:

1. The board boots into WiFi config after a few seconds with no link (or force
   it). Join its AP and browse to `http://10.0.0.1`.
2. Upload / paste `firmware/hardware/lilygo_t3s3_sniffer_rx.json`.
3. Reboot.

*(If you'd rather compile pins in and skip this step, switch the env's
`-include target/Unified_ESP_RX.h` to `-include target/T3S3_Sniffer_RX.h`
and rebuild — then there's no layout to upload. See `02_build_flash_guide.md`.)*

---

## 5. Watch the serial output

The decoded telemetry prints on the **native-USB CDC** serial at **460800**.

```bash
# Plain terminal (simplest, avoids PlatformIO env resolution):
python3 -m serial.tools.miniterm /dev/cu.usbmodem1101 460800   # exit: Ctrl-]
# or:  screen /dev/cu.usbmodem1101 460800                      # exit: Ctrl-A K

# PlatformIO monitor — MUST name the env, else it defaults to `native` and errors
# with "UnknownPlatform: 'native'":
pio device monitor -e T3S3_Sniffer_2400_RX_Serial -p /dev/cu.usbmodem1101 -b 460800
```

Once locked to your test link you should see lines like:

```
[Ghost] serial telemetry sink ready
BATT 16.4V 12.3A 850mAh 78%
GPS  lat=45.1234567 lon=9.7654321 alt=132m spd=4.2kmh sats=11
ATT  pitch=-120 roll=35 yaw=1785 (raw rad*1e4)
MODE ACRO
LINK rssi1=-42 rssi2=-88 lq=100 snr=9 rfmode=6 txpwr=3
[CRSF] EC 08 08 00 A4 00 7B 03 52 4E ...
```

`[CRSF] ...` lines are the raw frames (enabled by `-D GHOST_SERIAL_HEX`); a
laptop Python parser can consume these directly later. Drop `GHOST_SERIAL_HEX`
from the env for decoded-only output.

If you see the radio init but **no** telemetry, walk the triage table in
`03_bringup_runbook.md` — the usual first suspect on the PA board is the
RF-switch pins in the hardware layout.
