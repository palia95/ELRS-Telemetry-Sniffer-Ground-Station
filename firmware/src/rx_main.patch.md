# `rx_main.cpp` integration patch (ExpressLRS 3.5.x)

These are the **only** edits needed in `src/rx_main.cpp` to turn a stock RX
into the passive Ghost sniffer. Line numbers are from tag `3.5.6`; match on
the surrounding code, not the exact line. Everything else lives in
`sniffer.cpp` / `devTransport_*.cpp`.

Why this is so small: on the SX1280, ELRS runs the radio in **continuous RX**
(`RXnb()` with a 0xFFFF period). After each received packet it keeps
listening. So if we simply *decline to transmit* in the telemetry slot, the
radio is already listening and will hear the aircraft RX's telemetry uplink on
the correctly-hopped frequency. No timer or FHSS changes required — `HandleFHSS()`
in `HWtimerCallbackTock()` still advances the hop pointer whether or not we TX.

---

### 1. Include the module (top of file, with the other includes)

```cpp
#include "sniffer.h"
```

### 2. Never transmit — arm RX in the telemetry slot

In `HandleSendTelemetryResponse()` (≈ line 476), insert the block **after** the
existing "don't bother sending tlm" guard (not at the top):

```cpp
    if ((connectionState == disconnected) || (ExpressLRS_currTlmDenom == 1) || (alreadyTLMresp == true) || (modresult != 0) || !teamraceHasModelMatch)
    {
        return false; // don't bother sending tlm if disconnected or TLM is off
    }
#if defined(GHOST_NO_TX)
    // Ghost sniffer: this IS a telemetry slot. Rather than transmit our own
    // telemetry, arm the radio in RX so we overhear the aircraft telemetry.
    // IMPORTANT: do NOT set alreadyTLMresp here. ...
    Radio.RXnb();
    return false;
#endif
```

Why: the guard filters out non-telemetry slots, so reaching the new block means
the slot is a real telemetry slot; we call `Radio.RXnb()` to listen in it and
return `false` so nothing is transmitted. `alreadyTLMresp` must **not** be set:
that flag makes the tick skip `LQCalc.inc()`, leaving `LQCalc.currentIsSet()`
true, which makes `RXdoneISR` drop the incoming telemetry packet before
`ProcessRFPacket` ever runs.

### 3. Capture telemetry packets

In `ProcessRFPacket()`, the `switch (otaPktPtr->std.type)` block (≈ line 1146)
has a `case PACKET_TYPE_TLM:` that today only handles Airport. Extend it:

```cpp
    case PACKET_TYPE_TLM:
#if defined(GHOST_SNIFFER)
        // Passive capture of the aircraft RX -> handset telemetry uplink.
        Sniffer_ProcessTLM(otaPktPtr);
#endif
        if (firmwareOptions.is_airport)
        {
            OtaUnpackAirportData(otaPktPtr, &apOutputBuffer);
        }
        break;
```

> Note: a normal RX never *receives* TLM packets, so this path is dormant in
> stock firmware. The CRC in `OtaValidatePacketCrc()` (called earlier in
> `ProcessRFPacket`) already guarantees the packet belongs to our UID before
> we get here.

### 4. Keep the reassembler in sync with the air-rate

In `SetRFLinkRate()` (≈ line 347), right after `OtaUpdateSerializers(...)`
(≈ line 393):

```cpp
    OtaUpdateSerializers(smWideOr8ch, ModParams->PayloadLength);
#if defined(GHOST_SNIFFER)
    Sniffer_OnRateChanged();   // std vs full-res max package index + buffer reset
#endif
```

### 5. Init transports and emit completed CRSF frames

After `devicesStart();` in `setup()`:

```cpp
    devicesStart();
#if defined(GHOST_SNIFFER)
    Ghost_SetupTransports();
#endif
```

In `loop()`, right after `devicesUpdate(now);`:

```cpp
#if defined(GHOST_SNIFFER)
    Ghost_Loop(now);           // forward reassembled CRSF frames
#endif
```

`Ghost_Loop()` calls `Sniffer_Poll()` (fan reassembled frames to the transports)
and `GhostDisplay_Tick()` (a no-op kept for the call site; the OLED runs in its
own FreeRTOS task when `-D GHOST_DISPLAY` is set).

### 6. Set the UID

Two options:

* **Compiled phrase (bench):** `MY_BINDING_PHRASE` is set in the PlatformIO
  env, so ELRS already fills `UID[]` at boot and calls the CRC/FHSS setup. The
  sniffer works with no extra call — but for a runtime-changeable build, still
  call `Sniffer_SetUidFromPhrase()` once in `setup()` after config load.
* **Runtime phrase (field):** leave `MY_BINDING_PHRASE` unset and call
  `Sniffer_SetUidFromPhrase(userPhrase)` when the phrase arrives over
  BLE/WiFi/UART. It recomputes `UID[]`, CRC seed, and the hop table live.

### 7. (Optional, from Ghost RX) disable auto-WiFi after long disconnect

In `lib/WIFI/devWIFI.cpp`, in the `TARGET_RX` auto-WiFi branch, add an early
`return DURATION_NEVER;` so a monitored link going quiet doesn't kick the
sniffer into WiFi-config mode. Remove to restore stock behaviour.

### 8. Diagnostic: count every demodulated packet

At the very top of `ProcessRFPacket()` (before any CRC/UID check):

```cpp
bool ICACHE_RAM_ATTR ProcessRFPacket(SX12xxDriverCommon::rx_status const status)
{
#if defined(GHOST_SNIFFER)
    Sniffer_OnRawPacket(status == SX12xxDriverCommon::SX12XX_RX_OK);
#endif
```

### 9. Per-packet RSSI/SNR of the drone's telemetry

In `ProcessRFPacket()`, right after `Radio.GetLastPacketStats(); getRFlinkInfo();`
(so the stats belong to the packet just received):

```cpp
#if defined(GHOST_SNIFFER)
    Sniffer_OnPacketStats(otaPktPtr->std.type, Radio.LastPacketRSSI, Radio.LastPacketSNRRaw);
#endif
```

This lets the sniffer measure RSSI/LQ of the aircraft's telemetry packets
(`PACKET_TYPE_TLM`) separately from the RC packets that dominate
`CRSF::LinkStatistics` (which reflect the handset TX link).

---

## Behavioural notes / things to verify on the bench

* **Two RXs, one slot.** The real aircraft RX transmits its telemetry in the
  slot; our sniffer listens. As long as the sniffer is at least as well-placed
  as a handset would be, it hears the same uplink. If the sniffer sits right
  next to the TX, the aircraft's uplink may be comparatively weak — antenna and
  placement matter.
* **Defensive re-arm.** If bench testing shows the SX1280 occasionally not
  listening during the telemetry slot, add a `Radio.RXnb();` at the end of
  `HWtimerCallbackTock()` guarded by `#if defined(GHOST_NO_TX)`.
* **DVDA rates (idx 2/3)** send each packet `numOfSends` times — the
  StubbornReceiver tolerates duplicate `packageIndex` values, so no change.
* **Rate acquisition.** With a known phrase but unknown rate, the stock
  `RFmodeCycle` scan already sweeps the rate table until a valid sync arrives,
  then locks. Nothing to add.
