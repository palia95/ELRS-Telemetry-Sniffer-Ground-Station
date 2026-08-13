# 06 — EASA/ASTM Direct Remote ID broadcast (alternative firmware)

An **alternative build** of the T3‑S3 sniffer that, instead of streaming the
overheard telemetry to a ground station, re‑purposes the board's BLE radio to
**broadcast EASA Direct Remote ID** (ASTM F3411 / ASD‑STAN EN 4709‑002) — the
"license‑plate for drones" beacon — sourced from the GPS the sniffer already
hears on the ELRS link. Same passive sniffer core, opposite job on the BLE side:
connectionless advertisement, not a GATT stream.

> **Status (2026‑08‑13): built, flashed, and confirmed on‑air on real hardware.**
> Both the Legacy (BT4) and the Long‑Range (BT5 Coded PHY) advertisements decode
> correctly in nRF Connect. Not yet validated end‑to‑end against a Remote ID
> scanner app with a live GPS lock, and **not a legal compliance claim** — see §9.

Companion design doc with the wider rationale (standalone‑module vs this
in‑sniffer path, dual‑PHY receiver‑compatibility analysis, PCB notes):
[`../../remote-id/PLAN.md`](../../remote-id/PLAN.md).

---

## 1. What it does

- Rides the ELRS link passively exactly like the normal sniffer (§ `00_README.md`).
- Pulls `CRSF_FRAMETYPE_GPS` out of the reassembled telemetry it already decodes.
- Encodes ASTM F3411 / EN 4709‑002 messages (Basic ID, Location/Vector, System,
  Operator ID) with Intel/opendroneid's `opendroneid-core-c`.
- Broadcasts them as connectionless BLE advertisements — **no phone pairing, no
  GATT stream, no separate GPS tap** on the airframe.

This is a *different transport*, selected at build time. It is **mutually
exclusive** with the GCS BLE transport (`devTransport_BLE.cpp`) — both own the
one BLE radio identity; the firmware `#error`s if both are enabled.

---

## 2. On‑air structure — three advertising instances

| Inst | PHY | Type | Contents |
|---|---|---|---|
| **0** | 1M | Legacy (BT4) | One ODID message/second, biased to Location; rotates Basic ID / System in every 3rd slot. 31‑byte cap = one 25‑byte message. |
| **1** | **LE Coded (S=8)** | BT5 Extended Adv (Long Range) | The full ODID **message pack** (Basic ID + Location + System [+ Operator ID]) in one advert. |
| **2** | 1M | Legacy, **connectable** | A tiny Nordic‑UART‑Service GATT server for one‑time Operator‑ID entry. Retires itself once airborne. |

Why both 0 and 1: Legacy is what nearly every scanner (all iOS, older Android)
can receive; Coded PHY adds range for the subset of Android phones that support
it. iOS has **no** API to scan Coded PHY at all. See `../../remote-id/PLAN.md`
§9 for the full receiver‑compatibility analysis. Broadcasting both is the ASTM
F3411 Annex A4 pattern.

### AD frame byte layout (verified byte‑exact vs opendroneid reference)

NimBLE's `setServiceData(0xFFFA, payload)` emits `[Len][0x16][FA][FF][payload]`.
With `payload = [0x0D][msg_counter][ODID data]`:

```
Legacy (inst 0):  1E 16 FA FF | 0D <ctr> | <25-byte ODID message>          = 31 bytes (legacy max)
Coded  (inst 1):  <Len> 16 FA FF | 0D <ctr> | F2 19 <N> <N×25-byte msgs>   (pack; F2 = PACKED,v2)
                                              └ ODID_MessagePack: [type|ver][singlesize=25][count]
```

- `0x0D` = AD Application Code = Open Drone ID.
- `0xFFFA` = ASTM International (Remote ID) 16‑bit service UUID.
- This matches `opendroneid/transmitter-linux/bluetooth.c` exactly; NimBLE does
  **not** auto‑inject a Flags AD (which would overflow the 31‑byte legacy cap).

---

## 3. Files

```
firmware/src/devTransport_RemoteID.cpp   ← the transport (this feature)
firmware/src/opendroneid.h / .c          ← vendored UNMODIFIED from opendroneid-core-c (Apache-2.0)
firmware/third_party/LICENSE-opendroneid-core-c
firmware/test/test_remoteid.c            ← host unit test (GPS parse + ODID round-trip)
firmware/targets/t3s3_sniffer.ini        ← env: T3S3_Sniffer_2400_RX_RemoteID
```

Wired into the shared sniffer via `sniffer.h`/`sniffer.cpp` (transport init +
per‑loop `RemoteID_Tick`) and copied by `integrate.py`.

---

## 4. Build

```bash
# from telemetry-sniffer/firmware
python3 integrate.py /path/to/ExpressLRS/src     # copies sources incl. opendroneid + ini
cp targets/t3s3_sniffer.ini /path/to/ExpressLRS/src/targets/

# from ExpressLRS/src
pio run -e T3S3_Sniffer_2400_RX_RemoteID
```

Build flags that matter (all in the env):

- `-D GHOST_TRANSPORT_REMOTEID=1` — selects this transport.
- `-D CONFIG_BT_NIMBLE_EXT_ADV=1` — enables NimBLE extended advertising
  (needed for Coded PHY). **Global** macro: it removes NimBLE's legacy
  `NimBLEAdvertising` class, which is why `lib_ignore = ESP32-BLE-Gamepad`
  (that lib still uses the old class; it's a TX‑only feature, irrelevant here).
- `-D CONFIG_BT_NIMBLE_MAX_EXT_ADV_INSTANCES=2` — we use 3 instances (0,1,2);
  the macro value is "+1" so 2 ⇒ instances 0–2.
- `-D ODID_DISABLE_PRINTF=1` — we only encode, never decode/print; drops the
  library's printf helpers.

Footprint: RAM ~22.8 %, Flash ~64.4 % of the 4 MB T3‑S3.

---

## 5. Flash (two ESP32‑S3 native‑USB gotchas — read this)

Two things bite on the T3‑S3's native USB‑CDC. Both are solved below.

**Gotcha 1 — the flasher stub drops at ~32 %.** `Could not configure port:
Device not configured`, always around the same offset. The stub's high‑speed
USB is what this board's native CDC can't sustain. **Fix: flash with the ROM
loader (`--no-stub`) at 115200 using a modern esptool (pip `esptool` 4.x).**
esptool's `--before default_reset` *does* enter the bootloader over native USB
here — no BOOT/RST button dance needed.

**Gotcha 2 — fresh flash boot‑loops (`Guru Meditation … LoadProhibited @ 0x0`).**
The Unified firmware reads its pin layout from SPIFFS `/hardware.json`. A bare
`-t upload` (firmware only) leaves SPIFFS empty ⇒ `options_init()` returns
false ⇒ ELRS takes the *unconfigured* boot path (registers only WiFi, never
sets up the serial device) ⇒ `loop()`'s `handleSerialIO()` dereferences a null
`serial0.io` and panics **before WiFi config is even usable**. **Fix: also
flash a SPIFFS image containing the hardware layout.**

```bash
# from ExpressLRS/src. ENV = T3S3_Sniffer_2400_RX_RemoteID, B=.pio/build/$ENV
# (a) put the layout where SPIFFS build picks it up, then build the fs image:
mkdir -p data && cp /path/to/telemetry-sniffer/firmware/hardware/lilygo_t3s3_sniffer_rx.json data/hardware.json
pio run -e $ENV -t buildfs        # -> $B/spiffs.bin (bundles data/hardware.json + data/options.json)

# (b) flash firmware (ROM loader, slow, reliable):
python3 -m esptool --chip esp32s3 --port /dev/cu.usbmodemXXXX --baud 115200 \
  --no-stub --before default_reset --after hard_reset write_flash -z \
  --flash_mode dio --flash_freq 80m --flash_size 4MB \
  0x0 $B/bootloader.bin 0x8000 $B/partitions.bin \
  0xe000 $B/boot_app0.bin 0x10000 $B/firmware.bin

# (c) flash the filesystem (SPIFFS at 0x3D0000 for min_spiffs.csv):
python3 -m esptool --chip esp32s3 --port /dev/cu.usbmodemXXXX --baud 115200 \
  --no-stub write_flash -z 0x3D0000 $B/spiffs.bin
```

`data/options.json` (already present from earlier bring‑up) carries the target
UID + flash‑discriminator, so the sniffer locks to the right link.

---

## 6. Bring‑up — what a good boot looks like

Serial debug is USB‑CDC @ **460800** (`-D DEBUG_LOG`). A healthy boot:

```
UID=(18, 52, 86, 120, 154, 188) ModelId=255     ← your UID from options.json (example, redacted)
Setting ISM2G4 Mode / Number of FHSS frequencies = 80
SX1280 Ready! / Read Vers sx1280 #1: 43447       ← radio SPI init OK
[RID] init: BLE Direct Remote ID broadcast (ASTM F3411 / EN 4709-002)
[RID] no operator ID in NVS yet - write "O:<id>\n" to the config characteristic before flying
[RID] config instance (connectable, 'ELRS-RID-CFG') up
[RID] instance 0 ADVERTISING ok (Legacy 1M, N data bytes)      ← diagnostic (if built with it)
[RID] instance 1 ADVERTISING ok (Coded PHY/LongRange, N data bytes)
[TLM RX] SRCH rate=1000Hz ...                    ← searching (no target link nearby)
```

No `Guru Meditation`, no boot loop, no `Advertising config error`.

`SRCH` with `rssi/lq=0` is correct when no target drone is transmitting. With a
matching‑phrase link nearby it goes `LOCK`, decodes GPS, and the broadcast
Location becomes valid.

---

## 7. Verification — how we know each PHY works

**Legacy (instance 0)** — visible to any BLE scanner. In nRF Connect it shows as
an unnamed device with Service Data `UUID 0xFFFA`.

**Coded PHY / Long Range (instance 1)** — only Coded‑PHY‑capable phones can scan
it. In nRF Connect on such a phone it appears as:

```
Advertising type: Bluetooth 5 Advertising Extension
Primary PHY: LE Coded   Secondary PHY: LE Coded
Advertising Set ID: 1
Service Data: UUID 0xFFFA  Data: 0x0D 0F F2 19 01 02 42 45 4C 52 53 2D 31 32 33 34 35 36 37 38 39 41 42 43 …
```

Decoded (example UID redacted): `0D`=app code, `0F`=counter, `F2`=PACKED/v2,
`19`=25, `01`=1 message, then `02 42` = BasicID/v2 IDType‑4/UAType‑2, ASCII
`ELRS-123456789ABC` (= `ELRS-` + the hex of your target UID). **Confirmed
on‑air 2026‑08‑13.**

If your scanner can't do Coded PHY, the firmware's one‑shot
`[RID] instance 1 ADVERTISING ok (Coded PHY/LongRange)` serial line is the
firmware‑side confirmation the controller accepted and is emitting it.

Definitive receiver validation: the **OpenDroneID Android receiver app** (reads
Android feature flags, listens on Long Range where supported) decodes the
`0xFFFA` frames as an actual drone. macOS blocks terminal BLE scanning, so
verify from a phone, not the build host.

---

## 8. Operator ID (the mandatory‑for‑EU field), set over BLE

### 8.1 What value to enter (get / format your Operator ID)

The Operator ID broadcast by Remote ID is your **UAS operator registration
number** — the number you get when you register as an *operator* with your
national aviation authority (d‑flight.it, LBA, Transportstyrelsen, …). It is
**not** the pilot competency certificate / "diploma" from the A1‑A3 or A2 online
exam; that's a different credential and is not broadcast.

Format (EU / EN 4709‑002, based on ANSI/CTA‑2063‑A) — 16 characters:

```
SWE abcdefghijkl m   ← illustrative placeholder, not a real registration
└┬┘ └────┬─────┘ ┬
 │       │       └ 1 checksum character (assigned, not computed by you)
 │       └ 12-character operator portion
 └ 3-letter country code (SWE, ITA, DEU, GBR, …)
```

- Your registration document may show the number with a trailing **`-xyz`
  secret group** (3 characters after a dash). Those are **private** (used to
  prove you're the real operator) and must **NOT** be broadcast — drop the dash
  and everything after it.
- Broadcast only the **16‑character public part** (country + 12 + checksum). It
  fits the 20‑char ODID Operator‑ID field.

### 8.2 Set it over BLE

No display/keypad on the module, so the Operator ID is entered from a phone and
persisted to NVS. On boot the module opens a **config window** (default 60 s,
`REMOTEID_CONFIG_WINDOW_MS`) during which **only** the connectable config
instance advertises — the ODID broadcast is held off so the phone can connect
without Coded‑PHY / multi‑instance radio contention (running all instances at
once starves the connection handshake and the connect fails):

1. Power on / reset. Within the config window, connect to **`ELRS-RID-CFG`**
   (instance 2).
2. Nordic UART Service `6E400001‑…`.
3. Write to characteristic **`6E400002`** (the write one), as **Text**:
   `O:SWEabcdefghijklm` (your 16‑char operator ID; no dash/secret, no newline).
4. Serial prints `[RID] operator ID received … saving in loop task` then
   `[RID] operator ID saved to NVS: '…'`. (The NVS flash write is done in the
   loop task, not the BLE callback — a flash write inside a BLE callback
   crashes the chip.)
5. Disconnect. The window closes early (`config window closed (operator ID set)
   -> ODID broadcast`) and instances 0 + 1 start broadcasting, now carrying the
   Operator ID message.

Persistence: the value is stored in NVS and reloaded on every boot
(`[RID] loaded operator ID from NVS`), surviving power cycles and firmware
reflashes (only a full chip erase clears it). The config window **reopens on
every boot** even when an ID is already saved, so you can always reconnect to
change it; if you don't connect, it closes on timeout and broadcasts the saved
ID.

### 8.3 Set / read over USB serial (and from the GCS)

The Operator ID can also be set over the debug serial (USB‑CDC @ 460800),
mirroring the existing `P:<phrase>` command — handy for bench config with no
phone:

- **Set:** send `O:<operator id>\n` → `[RID] operator ID set via serial, saved
  to NVS`.
- **Read:** send `O?\n` → `[RID] OPID=<operator id>` (empty if none set).
- The firmware also emits `[RID] OPID=…` at boot and after any change (BLE or
  serial), so a host can track the current value passively.

The **GCS** (`gcs/`) exposes this in the Source panel: an *Operator ID* field
with **Set** and **Read** buttons. It sends `O:`/`O?` over the active serial
link and displays the device's current value (parsed from the `[RID] OPID=`
line). Works over the serial transport; the Remote ID BLE config
characteristic is write‑only, so read‑back is serial‑only. Note the GCS shows
the *Operator ID*, not drone telemetry from it — the ODID broadcast itself is a
separate BLE advertisement only a Remote ID scanner app decodes.

> Implementation notes for the two bugs this flow fixes: (a) the config‑only
> window exists because a connectable instance running alongside the Coded‑PHY
> broadcast can't complete the connection handshake; (b) `s_adv->setCallbacks()`
> **must** be called — NimBLE's `NimBLEExtAdvertising` leaves its callback
> pointer uninitialized and dereferences it on the ADV_COMPLETE event that
> fires when the connectable instance turns into a connection, crashing the
> instant a phone connects.

---

## 9. Compliance status — what's done, what's missing

Broadcasting valid frames ≠ EASA conformance. Current state:

| Field / message | Status |
|---|---|
| Basic ID | ✅ sent — but a **synthesized** `ELRS-<UID>` identifier, **not** a real CAA registration / ANSI‑CTA‑2063 serial (we only overhear the aircraft). |
| Location/Vector | ✅ sent when a GPS fix exists; withheld (status undeclared) otherwise. |
| System (operator/takeoff position) | ✅ sent once a take‑off fix is latched. |
| Operator ID | ✅ sent when set (§8). **Mandatory for EU/EASA**, optional in base ASTM. |
| Location/System **timestamp** | ⚠️ left "unknown" — **no UTC time source** on this data path (CRSF GPS carries no time). Not faked from `millis()`. |
| EU UA **classification** (C0–C4) | ❌ `UNDECLARED`. |
| Self‑ID | ❌ not sent (optional in spec). |
| Physical placement | ⚠️ must be **onboard the aircraft**, not ground‑side with the handset. |

**A working broadcast is a technical prototype, not a legal compliance claim.**
Operating under EASA Direct Remote ID requires a conformity assessment against
the harmonized standard — a separate, non‑code track. See
`../../remote-id/PLAN.md` §9–§11.

---

## 10. Host unit test

`firmware/test/test_remoteid.c` runs on any dev machine (no ESP32) and guards
the two things that compile fine but broadcast garbage — the CRSF‑GPS byte
parse/scaling and the ODID encode↔decode round‑trip:

```bash
cd firmware/test
cc -std=c11 test_remoteid.c ../src/opendroneid.c -I../src -lm -o /tmp/t && /tmp/t
# -> ALL CHECKS PASSED  (Location/BasicID/System/OperatorID round-trip, pack layout)
```

---

## 11. Known discrepancy to resolve

Boot log prints `Use RX pin: 21 / TX pin: 10` — the flashed
`hardware/lilygo_t3s3_sniffer_rx.json` still carries `power_rxen:21 /
power_txen:10`, although this project earlier recorded the board as **V1
non‑PA** with those removed. Not fatal (radio inits fine), but confirm the
board revision; if truly non‑PA, drop those two keys from the layout JSON.
