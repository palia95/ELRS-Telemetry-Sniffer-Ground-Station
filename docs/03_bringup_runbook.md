# Bench bring‑up runbook

Staged validation. Each phase has a **success criterion** — don't move on until
it's met. You control both ends of a **test link** (your own ELRS TX + RX with a
passphrase you set), so you can compare the sniffer's output against ground
truth on the real handset.

## Phase 0 — Test rig

- ELRS TX (handset/module) + a real ELRS 2.4 GHz RX, bound with a known
  passphrase, telemetry ON, sending real sensors (GPS/battery/attitude) if
  possible — otherwise link‑stats telemetry is enough to prove the path.
- The T3‑S3 flashed per `02_build_flash_guide.md` with the same passphrase.
- Serial monitor open at 460800.

## Phase 1 — Radio bring‑up  `VERIFY hardware`

Goal: the SX1280 initializes and receives energy.

- On boot, serial should show the radio init succeed (no `radioFailed`).
- **RF‑switch check:** the single most likely failure. If the board is the PA
  ("H658") variant, confirm `power_rxen`/`power_txen` in the layout put the FEM
  in RX. If you see the radio init OK but **never** hear packets in Phase 2,
  suspect these pins first. On the non‑PA v1.0 board, remove those keys.
- **Success:** radio init OK; with the test TX running nearby you see occasional
  CRC‑failing packets (energy is reaching the demod) even before lock.

## Phase 2 — Lock to the link (passive)  `VERIFY timer`

Goal: the sniffer follows the target link without transmitting.

- Serial should show sync packets validating and LQ climbing, exactly like a
  normal RX connecting.
- Put a spectrum analyzer / second SDR on the band, or watch the real link: the
  sniffer must **not** add any transmissions. (`GHOST_NO_TX` guarantees this in
  code; confirm the guard is present.)
- If lock is flaky or LQ collapses at the telemetry slot, add the defensive
  `Radio.RXnb();` at the end of `HWtimerCallbackTock()` (see the patch doc) and
  retest.
- **Success:** stable LQ on the sniffer while the real TX↔RX link runs normally,
  with zero sniffer transmissions.

## Phase 3 — Telemetry decode (UART first)

Goal: reassemble real telemetry and confirm it matches the handset.

- Temporarily route completed CRSF frames to USB serial (register a UART sink,
  or `Serial.write(frame,len)` inside `Sniffer_Poll`).
- Feed the serial stream to any CRSF parser on a PC (or eyeball the hex).
- Compare decoded battery voltage / GPS / attitude against what the real handset
  shows for the same aircraft.
- **Success:** values match ground truth within rounding, updating at the
  telemetry rate.

## Phase 4 — Wireless bridge (BLE)

Goal: phone shows live telemetry.

- Build/run the Android app (`android/`). Tap **Scan & Connect** → it finds
  `T3S3 Ghost RX`.
- Telemetry cards populate (battery/GPS/attitude/mode/link).
- Test runtime passphrase: change the phrase in the app → the sniffer re‑locks
  to a link using that phrase.
- **Success:** phone telemetry tracks the handset in real time.

## Phase 5 — WiFi‑UDP (optional) & polish

- Bring up WiFi (AP or join the phone's hotspot); confirm CRSF frames arrive as
  UDP broadcasts on `:14555` (test with `nc -ul 14555` or a small script).
- Optional: add MAVLink conversion to stream to `:14550` for off‑the‑shelf GCS
  apps (ELRS ships `lib/CRSF2MSP` and `lib/MAVLink` to build on).
- OLED status page, runtime phrase entry, rate auto‑scan, multi‑target handling.

## Quick failure triage

| Symptom | Most likely cause |
|---|---|
| Radio init OK, never any packets | RF‑switch / PA pins wrong (Phase 1) |
| Energy seen, never locks | wrong passphrase → wrong UID/CRC/FHSS; or wrong rate scan |
| Locks then drops each TLM slot | radio not re‑armed in RX — add defensive `RXnb()` |
| Locks, but no telemetry frames | aircraft telemetry OFF, or reassembly max‑index not set (`Sniffer_OnRateChanged`) |
| Frames arrive, garbage values | endian/offset mismatch in `CrsfParser.kt`, or CRC not checked |
| BLE connects, no data | notifications not enabled (CCCD write), or MTU too small |
