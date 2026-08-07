# Working configuration & bring‑up gotchas (validated 2026‑07‑30)

This is the definitive record of the **known‑good** configuration for the ELRS
telemetry sniffer on a **LILYGO T3‑S3 (rev 1.1, ESP32‑S3FH4R2 + SX1280 *PA*)**,
plus every non‑obvious problem hit during bring‑up and its fix. If you're
reproducing or extending this, read this first — it will save you a day.

## Validated result

With only the target link's **binding phrase**, the sniffer:
- locks to a live ELRS **2.4 GHz** link (tested at 150 Hz LoRa, TLM 1:32),
- passively overhears the aircraft's telemetry and decodes it over USB serial,
- matches the handset's own telemetry: **GPS** (65.6211 N / 22.1453 E, 9 sats),
  **battery** (3.8 V, 43 %), **attitude**, **flight mode**.

Example captured CRSF GPS frame:
`c8 11 02 27 1c fd cf 0d 33 1a 6e 00 0d 62 ac 03 fb 09 3b`.

## The known‑good pin layout (this board)

From LilyGo's own `examples/LoRa/T3S3/utilities.h` (board `T3-S3-SX1280PA`):

| Signal | GPIO | | Signal | GPIO |
|---|---|---|---|---|
| radio SCK | 5 | | radio DIO1 | **9** |
| radio MISO | 3 | | radio BUSY | **36** |
| radio MOSI | 6 | | RX switch (power_rxen) | 21 |
| radio NSS | 7 | | TX switch (power_txen) | 10 |
| radio RST | 8 | | OLED SDA / SCL | 18 / 17 |
| LED | 37 | | | |

**No `serial_rx/tx`, no `power_enable`.** (See gotchas #3 and #6.)

This is `firmware/hardware/lilygo_t3s3_sniffer_rx.min.json`.

## The known‑good build config

Env `T3S3_Sniffer_2400_RX_Serial` (`firmware/targets/t3s3_sniffer.ini`):
`GHOST_SNIFFER`, `GHOST_NO_TX`, `GHOST_TRANSPORT_SERIAL`, `GHOST_SERIAL_HEX`,
`DEBUG_LOG`, `CORE_DEBUG_LEVEL=0`, `Regulatory_Domain_ISM_2400`, flash pinned 4 MB.
**OLED disabled** (`GHOST_DISPLAY` commented — see gotcha #7). Binding phrase and
UID are set via SPIFFS files, not build flags (gotchas #1, #2).

## Reproduce (from `firmware/`)

```bash
# 1. set your target link's phrase (writes user_defines.txt + options.json UID)
bash build.sh /path/to/ExpressLRS T3S3_Sniffer_2400_RX_Serial "your phrase"
# 2. flash app + filesystem (SPIFFS carries hardware.json + options.json)
cd /path/to/ExpressLRS/src
pio run -e T3S3_Sniffer_2400_RX_Serial -t upload -t uploadfs --upload-port /dev/cu.usbmodem1101
# 3. watch
pio device monitor -e T3S3_Sniffer_2400_RX_Serial -p /dev/cu.usbmodem1101 -b 460800
```

---

# Gotchas (the whole journey)

### 1. Binding phrase can't go in an ini build flag
PlatformIO's flag tokenizer mangles `-DMY_BINDING_PHRASE=\"…\"` ("No closing
quotation"). **Fix:** put it in `src/user_defines.txt`; ELRS's `build_flags.py`
MD5s it into `-DMY_UID=…`. `build.sh` writes this for you.

### 2. Unified firmware loads the UID at runtime, not from `MY_UID` — and needs a discriminator
The UID comes from `options.json` in SPIFFS, not the compiled `MY_UID`. And ELRS
only copies it into the stored config when `flash-discriminator` **differs** from
what's saved (`RxConfig::CheckUpdateFlashedUid`). With no discriminator (0 == 0)
the UID stayed `0,0,0,0,0,0`. **Fix:** `options.json = {"uid":[…],
"flash-discriminator":<non‑zero>}` (build.sh derives it from the phrase).

### 3. A sniffer has no flight controller — remove the serial pins
Leaving `serial_rx/tx` in the layout makes ELRS open a CRSF FC serial. Unrelated
but adjacent: an ESP‑IDF UART autobaud call spams
`uartDetectBaudrate not supported` at ~9 Hz. **Fixes:** omit `serial_rx/tx` (ELRS
uses `SerialNOOP`), and set `-D CORE_DEBUG_LEVEL=0` to silence ESP‑IDF `[E]` logs
(our ELRS `DBGLN` uses a different path and is unaffected).

### 4. DIO1/BUSY were wrong → radio "inits fine" but is stone deaf (`raw=0`)
Our first pinout research had DIO1=33/BUSY=34. The real board is **DIO1=9,
BUSY=36**. SPI init succeeded anyway (BUSY on the wrong floating pin reads low),
so the SX1280 reported its version — but with DIO1 wrong the **RxDone interrupt
never fires**, so `ProcessRFPacket` is never called and nothing is ever received.
**Fix:** DIO1=9, BUSY=36 (LilyGo's authoritative pins). This was the single
biggest blocker; the diagnostic that found it was a raw‑packet counter added at
the very top of `ProcessRFPacket` (before any CRC/UID check).

### 5. PA board needs the RX/TX switch pins
The T3‑S3 **PA** front‑end must be switched into the RX path. `RFAMP_hal` drives
`GPIO_PIN_RX_ENABLE` high during receive — but only if defined. **Fix:**
`power_rxen=21`, `power_txen=10`. (A separate `power_enable=35` "LDO" guess was
wrong; LilyGo's working example uses only the two switch pins.)

### 6. Band matters: SX1280 is 2.4 GHz only
If the link were 900 MHz (SX127x), this board physically can't hear it. Confirm
the target link is 2.4 GHz.

### 7. A blocking OLED init hung the entire firmware
`GhostDisplay_Init()` called `u8g2.begin()` synchronously at the end of `setup()`.
On the un‑ACKing/mis‑wired I²C it **wedged**, so `setup()` never returned → `loop()`
never ran → no scanning, no lock, no heartbeat, blank OLED. Symptom looked like
"dead board." **Fix for now:** `GHOST_DISPLAY` disabled. **Proper fix (TODO):**
probe 0x3C with an I²C timeout and init from `loop()`, never blocking.

### 8. `DEBUG_RX_SCOREBOARD` crashes under load
It prints a char **per packet from the RxDone ISR**; at 150 Hz that floods
USB‑CDC from interrupt context and trips the interrupt watchdog (Guru Meditation).
**Fix:** only enable it briefly for bring‑up; off in normal use.

### 9. Telemetry slot: the sniffer wasn't listening (`tlmPkts=0` despite lock)
Two bugs, both mine:
- In the telemetry slot I set `alreadyTLMresp = true`. The tick does
  `if (!alreadyTLMresp) LQCalc.inc();`, so LQ never advanced,
  `LQCalc.currentIsSet()` stayed true, and `RXdoneISR`'s guard
  `if (LQCalc.currentIsSet() && connected) return false;` **dropped the incoming
  telemetry packet**. **Fix:** in the tlm slot just `Radio.RXnb(); return false;`
  — do **not** set `alreadyTLMresp`.
- `Sniffer_ProcessTLM` gated on `s_uidValid`, which is only set by
  `Sniffer_SetUidFromPhrase` — never called when the UID comes from `MY_UID`.
  **Fix:** removed that guard.

### 10. On‑device telemetry looked wrong — but it was the printf, not the data
ELRS `debugPrintf` (logging.cpp) supports **only** `%s %d %u %x %f` — **no width
specifiers**. My `%02X` and `%07d` produced garbage (`2X`, `65.7d`) while `%u`/`%d`
lines (battery, attitude) were fine. The captured CRSF bytes were always correct.
**Fix / rule:** in `DBG`/`DBGLN` use only `%d %u %x %s %f`; for 2‑char hex print
two single‑nibble `%x`; print GPS as raw scaled ints.

---

## Notes for the wireless bridge (next step)
The raw CRSF frames handed to the transports are **correct** (validated on serial),
so BLE (`devTransport_BLE.cpp`) and WiFi‑UDP (`devTransport_WiFiUDP.cpp`) just need
enabling (`T3S3_Sniffer_2400_RX_Wireless` env) — the `CrsfParser.kt` / a laptop
Python parser decodes the same bytes shown in the `[CRSF] …` lines.

## Passive‑sniffing caveat
The sniffer sends no telemetry confirmations, so for multi‑chunk frames it relies
on overhearing the same chunks the real RX receives. At high LQ this is reliable;
at the link edge, long frames (GPS) may occasionally drop a chunk.
