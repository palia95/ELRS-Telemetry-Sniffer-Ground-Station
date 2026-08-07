# Protocol reference (code‑anchored, ELRS 3.5.x / SX1280 2.4 GHz)

Everything here is what the sniffer must replicate to lock onto and decode a
link. Citations are file paths at tag `3.5.6`.

## 1. Passphrase → UID

`python/binary_configurator.py`:
```python
uid = hashlib.md5(('-DMY_BINDING_PHRASE="' + phrase + '"').encode()).digest()[0:6]
```
The firmware reproduces this in `Sniffer_SetUidFromPhrase()` (MD5 over the exact
string `-DMY_BINDING_PHRASE="<phrase>"`, first 6 bytes → `UID[0..5]`).

## 2. What the UID drives

| Derived value | Formula | Source |
|---|---|---|
| CRC initializer | `((UID[4]<<8)\|UID[5]) ^ OTA_VERSION_ID` | `OTA.cpp:OtaUpdateCrcInitFromUid` |
| FHSS seed | `(UID[2]<<24)+(UID[3]<<16)+(UID[4]<<8)+(UID[5]^OTA_VERSION_ID)` | `common.cpp:uidMacSeedGet` |
| LoRa InvertIQ | `UID[5] & 0x01` | `rx_main.cpp:SetRFLinkRate` |
| FLRC sync word | `uidMacSeedGet()` (FLRC rates only) | `rx_main.cpp` → `SX1280.cpp:SetPacketParamsFLRC` |
| Sync‑packet check | TX embeds `UID[3],UID[4],UID[5]` in every sync | `rx_main.cpp:ProcessRfPacket_SYNC` |

`OTA_VERSION_ID == 3` (`include/common.h`). Apply order after setting `UID[]`:
`OtaUpdateCrcInitFromUid()` then `FHSSrandomiseFHSSsequence(uidMacSeedGet())` —
exactly what `sniffer.cpp:applyUid()` does.

The CRC is **not** cryptographic — it's a 13/14/16‑bit CRC keyed by 16 bits of
UID. Combined with the per‑UID hop sequence it makes a link private in practice,
but knowing the passphrase reproduces all of it deterministically.

## 3. FHSS (2.4 GHz ISM)

`FHSS.cpp`: band 2400.4–2479.4 MHz, **80 channels**, 1 MHz spacing, sync channel
`80/2+1 = 41`, 256‑entry sequence shuffled from the UID‑seeded PRNG
(`rngSeed`/`rngN`). The link hops every `FHSShopInterval` packets; `nonce` and a
periodic sync packet keep both ends aligned. The sniffer's `HandleFHSS()` (in the
timer tock) advances in lockstep — no change needed.

## 4. OTA packet types

Low 2 bits of byte 0 = type (`OTA.h`):

| Type | Value | Direction | Purpose |
|---|---|---|---|
| RCDATA | `0b00` | TX→RX | channels |
| MSPDATA | `0b01` | either | MSP/config |
| SYNC | `0b10` | TX→RX | rate, tlm ratio, fhssIndex, nonce, UID3‑5 |
| **TLM** | `0b11` | **RX→TX** | **telemetry — captured by the sniffer** |

Two on‑air sizes: 8‑byte std (`OTA_Packet4_s`) and 13‑byte full‑res
(`OTA_Packet8_s`). CRC is validated by `OtaValidatePacketCrc()` before any packet
is accepted — this silently rejects everything not matching our UID.

## 5. Sync acquisition

`ProcessRfPacket_SYNC` (the model the sniffer relies on): verifies `UID[3]`,
`UID[4]`, and the non‑modelmatch bits of `UID[5]`; then adopts the TX's
`rateIndex`, `newTlmRatio`, `switchEncMode`, `fhssIndex`, and `nonce`. With a
known passphrase but unknown rate, the stock `RFmodeCycle` scan sweeps the rate
table until a sync validates, then locks. Nothing to add for the sniffer.

## 6. Telemetry stream

The aircraft's `StubbornSender` splits a CRSF frame into TLM‑packet‑sized chunks;
the handset's `StubbornReceiver` reassembles them. The sniffer instantiates its
own `StubbornReceiver` and feeds it captured TLM payloads.

Per‑packet telemetry payload sizes (`telemetry_protocol.h`):
- std: `ELRS4_TELEMETRY_BYTES_PER_CALL = 5`, `ELRS4_TELEMETRY_MAX_PACKAGES = 63`
- full‑res: `ELRS8_TELEMETRY_BYTES_PER_CALL = 10`, `ELRS8_TELEMETRY_MAX_PACKAGES = 31`

TLM sub‑types: `ELRS_TELEMETRY_TYPE_LINK (0x01)` (link stats only) and
`ELRS_TELEMETRY_TYPE_DATA (0x02)` (a reassembly chunk).

The unpack logic in `sniffer.cpp:Sniffer_ProcessTLM()` is lifted verbatim from
the handset's own `tx_main.cpp:ProcessTLMpacket()` — handling both std and
full‑res, and the `containsLinkStats` variant.

## 7. Reassembled CRSF frames

A completed frame in the buffer is `[dest][len][type][payload…][crc8]`, where
`len` counts `type..crc8`. CRC8 is DVB‑S2 (poly `0xD5`) over `type..payload`.
Relevant types (`crsf_protocol.h`): GPS `0x02`, Vario `0x07`, Battery `0x08`,
Baro `0x09`, Link stats `0x14`, Attitude `0x1E`, Flight mode `0x21`, Device info
`0x29`. The Android `CrsfParser.kt` decodes these (all fields big‑endian).

## 8. SX1280 2.4 GHz air‑rate table (`common.cpp`)

| idx | Rate | Modem | SF/BR · BW | CR | TLM | Hop | Size |
|---|---|---|---|---|---|---|---|
| 0 | FLRC 1000 | FLRC | 0.65 Mb/s · 0.6 | 1/2 | 1:128 | 2 | 8 |
| 1 | FLRC 500 | FLRC | 0.65 Mb/s · 0.6 | 1/2 | 1:128 | 2 | 8 |
| 2 | DVDA 500 | FLRC | 0.65 Mb/s | 1/2 | 1:128 | 2 | 8 |
| 3 | DVDA 250 | FLRC | 0.65 Mb/s | 1/2 | 1:128 | 2 | 8 |
| 4 | LoRa 500 | LoRa | SF5 · BW800 | 4/6 | 1:128 | 4 | 8 |
| 5 | LoRa 333/8ch | LoRa | SF5 · BW800 | 4/8 | 1:128 | 4 | 13 |
| 6 | LoRa 250 | LoRa | SF6 · BW800 | 4/8 | 1:64 | 4 | 8 |
| 7 | LoRa 150 | LoRa | SF7 · BW800 | 4/8 | 1:32 | 4 | 8 |
| 8 | LoRa 100/8ch | LoRa | SF7 · BW800 | 4/8 | 1:32 | 4 | 13 |
| 9 | LoRa 50 | LoRa | SF8 · BW800 | 4/8 | 1:16 | 2 | 8 |

LoRa mode uses fixed/implicit header, CRC off, fixed payload length — so the
receiver must already know the exact rate (obtained from sync).

## 9. Why the sniffer stays in RX during the telemetry slot

On SX1280, ELRS runs continuous RX (`SX1280.cpp:RXnb` with a 0xFFFF period), so
after each packet it keeps listening. Declining to transmit in the telemetry
slot leaves the radio listening on the correctly‑hopped frequency, where it hears
the aircraft RX's telemetry uplink. This is why the firmware change is tiny (see
`../firmware/src/rx_main.patch.md`).
