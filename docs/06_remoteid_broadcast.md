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

> **ELRS version disclaimer:** this broadcast only has data to send once the
> underlying sniffer locks onto and decodes the target aircraft's telemetry —
> so it inherits the base sniffer's ELRS‑version dependency (top‑level
> [`../README.md`](../README.md) § Compatibility). Anchor‑verified against the
> full **ELRS 3.5.x/3.6.x** tag range. **ELRS 4.x is not a "just recompile"
> situation** — 4.x restructured the OTA packet‑type scheme itself
> (`PACKET_TYPE_TLM` no longer exists in `rx_main.cpp`), so the build fails
> outright against a 4.x checkout, and a real protocol‑level port would be
> needed before this works against a drone/handset running ELRS 4.x.

Companion design doc with the wider rationale (standalone‑module vs this
in‑sniffer path, dual‑PHY receiver‑compatibility analysis, PCB notes):
[`../../remote-id/PLAN.md`](../../remote-id/PLAN.md).

---

## 0. Why this exists

Two motivations, neither of them "get certified":

- **Get ahead of the incoming sub‑250g rules.** Remote ID requirements are
  expanding to cover lighter aircraft than they used to, including builds
  that have so far flown under that radar. This is a practical, working
  broadcast built *before* that requirement lands on a sub‑250g self‑build —
  an effective solution, but an **unofficial and non‑compliant** one (see §9
  and §11 of the companion plan). It is not a substitute for the actual
  conformity assessment once that becomes mandatory for this airframe class.
- **Local situational awareness for other pilots.** Independent of any
  regulation, broadcasting "there's a drone here, at this position" is
  useful on its own: it lets other people flying nearby — at the same field,
  in the same airspace — know we're up and roughly where, the same way a
  transponder helps full‑scale aircraft see each other. That's a safety
  benefit regardless of whether it satisfies a legal requirement.

Both goals are served by getting the broadcast *working and useful*, not by
it being airtight against every edge case a certified product would have to
handle — hence the pragmatic, telemetry‑derived approach throughout this
doc, with every simplification flagged rather than hidden (§9).

**To be blunt about what this is not: this is not Remote ID for your FPV
drone.** Don't treat a working broadcast as "my drone now has Remote ID" —
it doesn't, not in the legal sense, and §9's gap list (no real registration,
no UTC time source, self‑declared class, etc.) is exactly why. Think of it
instead as a **side‑channel heads‑up to nearby drones and pilots** — closer
in spirit to shouting "dropping in" at a shared flying site than to a
certified transponder. Useful, worth having, not a compliance story.

---

## 1. What it does

- Rides the ELRS link passively exactly like the normal sniffer (§ `00_README.md`).
- Pulls GPS, and — if the aircraft sends them — VARIO / BARO_ALTITUDE /
  FLIGHT_MODE, out of the reassembled telemetry it already decodes.
- Encodes ASTM F3411 / EN 4709‑002 messages (Basic ID, Location/Vector, System,
  Operator ID) with Intel/opendroneid's `opendroneid-core-c`.
- Broadcasts them as connectionless BLE advertisements — **no phone pairing, no
  GATT stream, no separate GPS tap** on the airframe.
- **Derives what it can from that telemetry alone** (no extra sensor, no extra
  wiring):
  - **Vertical speed** — from VARIO/BARO telemetry when the aircraft sends
    it, else differentiated from successive GPS altitude samples.
  - **Height above take‑off** — current geodetic altitude minus the altitude
    latched at take‑off.
  - **Operator/take‑off altitude** — the same latched take‑off altitude.
  - **Baro altitude** — passed through when `BARO_ALTITUDE` telemetry is
    present; left "unknown" otherwise (optional field).
  - **Status** — decoded from CRSF `FLIGHT_MODE`, verified against Betaflight's
    actual source (`src/main/telemetry/crsf.c`), not just assumed:
    - `!FS!` (failsafe) or `RTH` (GPS Rescue — itself part of Betaflight's own
      failsafe escalation on RX loss) → `ODID_STATUS_EMERGENCY`.
    - Disarmed + a GPS fix → `ODID_STATUS_GROUND` (a disarmed, GPS‑locked
      drone sitting on the bench is not "airborne" — this was a real gap in
      an earlier version, which read any valid fix as airborne regardless of
      arm state).
    - Armed + a GPS fix, **or** no `FLIGHT_MODE` telemetry at all → `AIRBORNE`
      (best‑effort fallback for FCs that don't send flight‑mode telemetry).
    - Disarmed/armed itself is read from the mode string's trailing
      character: no suffix = armed; `*`/`!`/`?` = disarmed (ready‑to‑arm /
      arming‑disabled / GPS‑rescue‑disabled respectively — all three, not
      just `*`, an earlier version missed `!`/`?` and would misclassify
      e.g. `"ACRO!"` as armed). The suffix rule does **not** apply to `!FS!`
      or `RTH` — Betaflight never appends a suffix during failsafe, and GPS
      Rescue only ever runs while armed, so both are treated as armed
      outright rather than suffix‑parsed.
  - **Take‑off/operator position** — latched at the moment `FLIGHT_MODE`
    reports a **clean armed transition** (falls back to "first GPS fix" if no
    flight‑mode telemetry ever arrives, and self‑heals if we join mid‑flight
    already‑armed before a fix exists). Deliberately does **not** re‑latch
    across a mid‑flight `RTH`/failsafe excursion — once the emergency clears
    and the mode string returns to a normal armed state, no new arm‑edge
    fires (the emergency period is treated as "still armed" throughout), so
    the reference point stays anchored to the original arm location instead
    of jumping to wherever the excursion happened to end.
    
    **This is a one‑time snapshot, not the pilot's live position — read this
    before trusting any "distance from pilot" a receiver computes from it.**
    We have no visibility into the *pilot's* own GNSS from a drone‑side
    sniffer, so it's reported as `OperatorLocationType = TAKEOFF`, not
    `LIVE_GNSS`. It goes stale the moment the pilot moves after arming
    (walking a track between race gates, repositioning after launch, handing
    off the controller) or the flight goes long‑range. There's no field in
    the spec to flag "this is stale" (unlike `Location`, `System` has no
    operator‑location accuracy field) — so a receiver has no way to know how
    old this position is. Treat it as approximate, accurate only near the
    moment of arming.

    **Not persisted, by deliberate choice — not fixed** (2026‑08‑13): unlike
    the Operator ID and EU class (both NVS‑backed), the take‑off snapshot is
    plain RAM state. A restart of **this module** (not the aircraft's — a
    brownout, watchdog reset, reflash) mid‑flight doesn't just lose it, it
    silently **relatches to a wrong position**: the armed‑state tracking also
    resets, so the next `FLIGHT_MODE` frame showing "armed" looks like a
    fresh arm edge and captures wherever the aircraft happens to be at that
    moment — not the real launch point — with nothing to signal this
    happened. Accepted trade‑off: NVS persistence would need a reliable way
    to tell "sniffer hiccup mid‑flight" from "genuinely new flight after a
    full power‑down" (reusing a stale take‑off point from the *previous*
    flight would be worse than losing it), which isn't free.
- **No telemetry, no broadcast.** Without a GPS fix there is nothing real to
  report, so instances 0/1 stay off the air rather than broadcast a
  Basic‑ID‑only "ghost" drone — see §2.

This is a *different transport*, selected at build time. It is **mutually
exclusive** with the GCS BLE transport (`devTransport_BLE.cpp`) — both own the
one BLE radio identity; the firmware `#error`s if both are enabled.

---

## 2. On‑air structure — three advertising instances

| Inst | PHY | Type | Contents |
|---|---|---|---|
| **0** | 1M | Legacy (BT4) | One ODID message/second, biased to Location; rotates Basic ID / System in every 3rd slot. 31‑byte cap = one 25‑byte message. |
| **1** | **LE Coded (S=8)** | BT5 Extended Adv (Long Range) | The full ODID **message pack** (Basic ID + Location + System [+ Operator ID]) in one advert. |
| **2** | 1M | Legacy, **connectable** | A tiny Nordic‑UART‑Service GATT server for Operator ID / EU class entry. Up only during the boot **config window** (§8.2), not "until airborne" — see below. |

Instances 0/1 only broadcast while a GPS fix exists (or is fresh — stale
`>REMOTEID_FIX_STALE_MS` = treated as lost, **12 s by default**). **No fix
means the module goes quiet** rather than broadcasting a Basic‑ID‑only
"ghost" drone with an undeclared position — that isn't useful Remote ID.
Broadcasting resumes automatically the moment a fix (re)appears. This wait
state is scoped so it can never delay the config window or touch the debug
serial link: it only runs once `RemoteID_Tick` has already left the config
phase, and it only ever calls `stop()`/`start()` on instances 0/1 (see the
`devTransport_RemoteID.cpp` comment marked `INVARIANT` for the enforcement).

> **This threshold matters more than it looks — tune it to your Telem
> Ratio.** GPS is one of several sensor types sharing ELRS's rate‑limited
> uplink telemetry pipe (see `TLMBurstMaxForRateRatio()` in
> `common.cpp`), so its real update interval can be several seconds even at
> a moderate Telem Ratio — not the ~1Hz an idealized "GPS is fast" assumption
> would suggest. Field‑measured (2026‑08‑14, 150Hz / 1:32 (STD) Telem Ratio):
> genuine GPS updates every **~4.6–4.9s**. The threshold used to be a
> hardcoded `5000ms`, sitting right on top of that real cadence — normal
> jitter (one delayed/dropped telemetry chunk) was enough to push an
> interval over it, flapping the broadcast off. A receiver that treats
> identity/operator fields as "learned once, sticky" but treats *live
> aircraft position* as something that must be fresh will show exactly that
> as a symptom: pilot/operator info populates, aircraft position doesn't (or
> flickers). **Later same‑day retests at 1:32 and 1:16 showed zero staleness
> events yet the position‑not‑shown symptom persisted — this threshold fix
> was real and worth keeping, but it was not the actual cause of that
> symptom. See §11 below.** If your Telem Ratio is more conservative than 1:32, raise
> `REMOTEID_FIX_STALE_MS` further — it should be a solid multiple of your
> actual observed GPS cadence, not just barely above it.

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

### 8.4 EU UA classification: C0 or Legacy only (default C0)

The System message's EU classification field is configurable — but
**deliberately limited to two options**, not the full C0–C6 range the ODID
spec defines:

- **`C0`** — `ClassificationType = EU`, `ClassEU = CLASS_0`. Only pick this if
  the aircraft genuinely meets the C0 criteria (EU 2019/945 Part 1: <250g,
  <19 m/s max speed, etc.) — it's a declared‑conformity claim, not a free
  default for "small drone".
- **`Legacy`** (default’s sibling option) — `ClassificationType = UNDECLARED`,
  no class marking claimed at all.

**Why not C1–C6:** those are *manufacturer‑declared* classes tied to a CE
marking on a commercially placed‑on‑market product. A self‑built airframe —
which is what this sniffer/RemoteID module is designed around — cannot
legitimately claim them. Offering the full dropdown would let someone
broadcast a certified class their aircraft doesn't have, which is worse than
not broadcasting a class at all. **Default is C0**; switch to `Legacy` if
that doesn't apply to your build.

Same three control paths as the Operator ID:

- **BLE** (instance 2, config window): write `C:1` (C0) or `C:0` (Legacy) to
  characteristic `6E400002`.
- **Serial:** `C:1\n` / `C:0\n` to set, `C?\n` to read →
  `[RID] CLASS=C0` or `[RID] CLASS=LEGACY`.
- **GCS:** a *class* dropdown (only "C0" / "Legacy (no class marking)") with
  **Set class** / **Read** buttons next to the Operator ID field.

Any other value (`C:2`..`C:6`, garbage) is rejected by the firmware and
ignored — the stored value never changes. Persisted to NVS like the Operator
ID, survives reboots, defaults to C0 on first boot / if NVS is empty.

---

## 9. Compliance status — what's done, what's missing

Broadcasting valid frames ≠ EASA conformance. Current state:

| Field / message | Status |
|---|---|
| Basic ID | ✅ sent — but a **synthesized** `ELRS-<UID>` identifier, **not** a real CAA registration / ANSI‑CTA‑2063 serial (we only overhear the aircraft). |
| Location/Vector (lat/lon/alt/speed/heading) | ✅ sent when a GPS fix exists; withheld (status undeclared) otherwise — and instances 0/1 go off the air entirely with no fix (§2), not just "undeclared". |
| Location **vertical speed** | ✅ derived — from VARIO/BARO telemetry when present, else from successive GPS altitude samples. |
| Location **height above take‑off** | ✅ derived — current altitude minus the altitude latched at take‑off. |
| Location **baro altitude** | ✅ pass‑through when the aircraft sends `BARO_ALTITUDE` telemetry; "unknown" otherwise (optional field either way). |
| Location **status** (GROUND/AIRBORNE/EMERGENCY) | ✅ derived, best‑effort, verified against Betaflight's actual source — from CRSF `FLIGHT_MODE` (`!FS!`/`RTH` → EMERGENCY; disarmed+fix → GROUND; armed+fix → AIRBORNE). Not every FC/mode string follows this convention. |
| System (operator/take‑off position) | ✅ sent once take‑off is latched (at a **clean armed transition**, via `FLIGHT_MODE`, not just first fix, and not re‑latched across an RTH/failsafe excursion — §1). **One‑time snapshot, not live — see §1 for the distance‑from‑pilot caveat.** |
| System **operator altitude** | ✅ derived — the same latched take‑off altitude. |
| System **EU classification** | ✅ user‑configurable, **C0 or Legacy only** (§8.4), default C0. Not the full C1–C6 range — see §8.4 for why. |
| Operator ID | ✅ sent when set (§8). **Mandatory for EU/EASA**, optional in base ASTM. |
| Location/System **timestamp** | ✅ set from the **GCS's system clock**, pushed once per serial connect (`T:<unix seconds>`, §1). CRSF GPS itself carries no time field, so this only works while a GCS has connected over USB serial at least once this boot — pure BLE-only sessions (no cable ever attached) still report "unknown", honestly, not faked from `millis()`. |
| Self‑ID | ❌ not sent (optional in spec). |
| Physical placement | ⚠️ must be **onboard the aircraft**, not ground‑side with the handset. |
| C0 classification accuracy | ⚠️ **the module reports whatever class you configure — it does not verify the aircraft actually meets C0's weight/speed criteria.** That check is on you. |

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
# -> ALL CHECKS PASSED  (GPS parse, ODID round-trip, pack layout, FLIGHT_MODE
#    armed/emergency classification, Location.Status mapping, serial
#    time-sync epoch/hour-wrap math)
```

The FLIGHT_MODE and Status test blocks mirror the exact expressions in
`devTransport_RemoteID.cpp`'s `RemoteID_sink()`/`fillUasData()` line for line
— they exist specifically because those expressions have already had two real
bugs caught this way: the disarmed‑suffix check originally missed `!`/`?`
(only checked `*`), and `Location.Status` originally ignored arm state
entirely (any valid fix read as AIRBORNE, even disarmed‑on‑the‑ground).

**GCS parity:** `gcs/crsf.py`'s own CRSF decoder had a matching gap — it
didn't extract vertical speed from the combined 4‑byte `BARO_ALTITUDE` frame
(altitude + vspeed together) the way the firmware does, silently dropping
data it had already parsed most of. Fixed to match; verified with an inline
round‑trip (build a frame, decode it, check `vario_ms` appears iff the
payload is the 4‑byte combined variant).

---

## 11. Known discrepancy to resolve

Boot log prints `Use RX pin: 21 / TX pin: 10` — the flashed
`hardware/lilygo_t3s3_sniffer_rx.json` still carries `power_rxen:21 /
power_txen:10`, although this project earlier recorded the board as **V1
non‑PA** with those removed. Not fatal (radio inits fine), but confirm the
board revision; if truly non‑PA, drop those two keys from the layout JSON.

---

## 12. Next: real-drone field testing

Everything above is bench‑verified only (synthetic test cases, no real
GPS/link). [`07_remoteid_field_test_checklist.md`](07_remoteid_field_test_checklist.md)
is the phased checklist for the first real end‑to‑end test.

---

## 13. Field-test finding: DroneTag showed identity but not live position

Three real flights (2026‑08‑14, at 1:32/STD then 1:16 Telem Ratio) all showed
the same symptom in the DroneTag receiver app: pilot/Operator ID fields
displayed correctly, but the aircraft's live position never appeared.

**Ruled out, in order, each with real evidence — not assumption:**
1. **Stale/flapping fix** — `REMOTEID_FIX_STALE_MS` was genuinely too tight
   (§2) and was fixed, but a 1:16‑ratio retest with **zero** staleness events
   still showed the same symptom. Real bug, not the cause of this one.
2. **Vertical‑speed out‑of‑range** (ODID's ±62 m/s `SpeedVertical` bound
   silently failing `encodeLocationMessage()`) — computed precisely from the
   captured CSVs; worst case was 9.8 m/s. Ruled out.
3. **Encoding correctness** — a real BLE capture (`nRF Connect`) was decoded
   byte‑exact against the actual `opendroneid-core-c` encoder source. Every
   field (BasicID, Location lat/lon/status/speed, System operator
   location/class, OperatorID) matched real telemetry exactly. Ruled out.

**Leading suspect, addressed this session:** `Location.TimeStamp` was always
`INV_TIMESTAMP` (0xFFFF) and — more notably — `ODID_System_data.Timestamp`
was always the **literal integer 0**, which decodes as
"2019‑01‑01T00:00:00Z" with **no "unknown" sentinel defined for that field**
in the spec/library. A safety‑conscious receiver plausibly treats that as
"this position claims to be 7+ years stale" and withholds it, while
timestamp‑independent identity fields still display fine — which matches the
observed symptom exactly.

**Fix implemented:** the GCS pushes its own system clock over the existing
serial connection once per connect (`T:<unix seconds>`, mirroring the
existing phrase/Operator‑ID/EU‑class auto‑push pattern — see §8's config
flow). The firmware derives both ODID timestamp fields from it:
`Location.TimeStamp = unixNow % 3600` (seconds after the full hour, per
spec) and `System.Timestamp = unixNow - 1546300800` (seconds since the ODID
2019 epoch). Session‑only/RAM‑only by design (no NVS write) — same rationale
as the take‑off‑position snapshot (§1): `millis()` resets on reboot anyway,
and the GCS re‑syncs it on every reconnect for free, including post‑reboot.
If no GCS ever connects over serial this session (BLE‑only flight, no
cable), both fields still honestly report "unknown" rather than a fake time.

**Not yet confirmed against a real flight** — this is the mechanism most
consistent with the evidence gathered, not a proven fix. Re‑test with
DroneTag (and ideally the OpenDroneID Android reference app as a second,
independent receiver) is the next step once this build is flashed and
verified on the bench.
