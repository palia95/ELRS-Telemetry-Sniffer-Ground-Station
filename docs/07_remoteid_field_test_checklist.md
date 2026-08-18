# 07 — Field test checklist (real drone, real link)

**Status: completed (2026‑08‑14/15, multiple real flights).** Kept as
reference for what was checked and how — see
[`06_remoteid_broadcast.md`](06_remoteid_broadcast.md) §12–13 for the actual
findings and fixes that came out of running it. Useful again after any
future change to the RemoteID data path.

This checklist covers the first time real GPS/VARIO/BARO/FLIGHT_MODE
telemetry flowed through this code end‑to‑end, beyond the synthetic host
unit tests (`firmware/test/test_remoteid.c`) and no‑link bench sanity checks.
Expect to find things — that's the point of testing.

Work top to bottom; each phase assumes the previous one passed. Note the
exact serial log lines next to each item — that's what to grep for while
watching the debug console (USB‑CDC @ 460800).

---

## Phase 0 — Bench sanity (confirm before leaving the bench)

- [ ] Board boots clean, no `Guru Meditation` / boot loop.
- [ ] `O?` (or GCS Read) → Operator ID reads back correctly.
- [ ] `C?` (or GCS Read) → EU class reads back as intended (**C0 vs Legacy —
      decide this before flying**, not after; see `docs/06` §8.4).
- [ ] Config window opens (`config instance ... up - open 60s`) and closes on
      timeout if untouched (`config window closed (timeout)`).
- [ ] `hardware.json` / `options.json` on SPIFFS match this aircraft's real
      binding phrase — `UID=(...)` in the boot log should match what you
      expect, not a leftover test value.

---

## Phase 1 — Link lock (no GPS needed yet)

- [ ] Power the real TX/handset with the matching binding phrase.
- [ ] Serial log transitions `SRCH` → `TENT` → `LOCK`.
- [ ] `tlmPkts` counter increments (telemetry actually being captured, not
      just RC lock).
- [ ] **The real RC link is unaffected** — LQ/RSSI on the handset/OSD/goggles
      stay normal with the sniffer powered and locked. This is the whole
      "passive, non‑interfering" claim from `../ELRS_Telemetry_Sniffer_Plan.md`
      — never bench‑verified against a real link before now, worth actually
      confirming, not assuming.

---

## Phase 2 — GPS / Location fields

- [ ] Once the aircraft has a real GPS fix, broadcast comes alive:
      `[RID] GPS fix acquired - resuming ODID broadcast`, then
      `[RID] instance 0/1 ADVERTISING ok`.
- [ ] Lat/lon reported match reality (compare against a phone's GPS or a
      known reference point).
- [ ] Altitude (`AltitudeGeo`) is sane (not wildly offset — geodetic vs your
      phone's barometric altitude will differ, that's expected).
- [ ] Horizontal speed tracks reality while moving/walking the aircraft.
- [ ] Heading changes correctly as the aircraft is rotated.

---

## Phase 3 — Derived fields (§1 of `docs/06`)

- [ ] **Vertical speed, telemetry source:** if the FC sends VARIO/BARO
      telemetry, raising/lowering the aircraft by hand shows a real
      `SpeedVertical` response.
- [ ] **Vertical speed, GPS fallback:** if the FC sends *no* VARIO/BARO,
      confirm the GPS‑differentiated fallback still produces a plausible
      (if noisier) value after telemetry has been "stale" for >3 s.
- [ ] **Baro altitude:** if `BARO_ALTITUDE` frames are present,
      `AltitudeBaro` populates and is roughly sane; otherwise confirm it
      correctly stays "unknown" rather than showing garbage.
- [ ] **Height above take‑off:** starts near 0 right after the take‑off latch,
      increases as the aircraft climbs.
- [ ] **Operator/take‑off altitude:** matches the aircraft's altitude at the
      moment it was armed.

---

## Phase 4 — Arm / disarm / emergency state machine (the most bug‑prone part — two real bugs already found and fixed here from bench testing alone)

- [ ] `[RID] latched take-off (arm) ...` fires **exactly once per arm**, not
      once per `FLIGHT_MODE` frame.
- [ ] Disarmed + GPS fix → `Location.Status = GROUND` (confirm via a scanner
      app or by checking the broadcast doesn't read AIRBORNE while sitting
      disarmed on the ground).
- [ ] Arm → `Location.Status` flips to AIRBORNE.
- [ ] Disarm, then re‑arm at a **different physical spot** → confirm the
      take‑off/operator position re‑latches to the **new** arm location, not
      stuck at the old one (`[RID] latched take-off (arm) ...` fires again
      with different coordinates).
- [ ] If GPS Rescue/RTH is configured: trigger it (switch or simulated RX
      loss) → confirm `Location.Status = EMERGENCY`, and confirm the
      operator/take‑off position does **not** jump to wherever the RTH
      excursion ends once it clears (§1 "distance from pilot" fix).
- [ ] If safely testable, trigger failsafe (e.g., briefly cut RC at a safe
      moment) → confirm `!FS!` → EMERGENCY, same non‑relatch behavior as RTH.
- [ ] After landing + disarm → status correctly returns to GROUND, not stuck
      on AIRBORNE.
- [ ] Watch for any FC/mode string this project hasn't seen — the classifier
      only inspects the arm‑state suffix (`*`/`!`/`?`) and the `!FS`/`RTH`
      substrings, so an unrecognized mode name should still classify
      correctly on arm state alone (`docs/06` §1). If it doesn't, that's a
      new bug to report, not expected behavior.

---

## Phase 5 — Operator ID / EU class persistence & reconfiguration

- [ ] Operator ID survives a normal power‑cycle reboot.
- [ ] EU class survives a normal power‑cycle reboot.
- [ ] Reconfigure both via the BLE config window on a fresh boot (nRF Connect
      or similar).
- [ ] Reconfigure both via USB serial (`O:`/`C:` + `O?`/`C?`).
- [ ] Reconfigure both via the GCS UI (Set/Read buttons).
- [ ] Confirm an out‑of‑range class write (`C:2`..`C:6`) is rejected and the
      stored value is unchanged.

---

## Phase 6 — BLE broadcast verification (the actual Remote ID payload)

- [ ] **Legacy (BT4)** instance visible in nRF Connect, Service Data
      `0xFFFA` decodes to a valid ODID message.
- [ ] **Long Range (BT5 Coded PHY)** instance visible on a Coded‑PHY‑capable
      phone (`Advertising type: Bluetooth 5 Advertising Extension`,
      `Primary/Secondary PHY: LE Coded`), decodes to a valid message pack.
- [ ] **OpenDroneID Android receiver app** picks up the aircraft as an actual
      drone with correct Basic ID / Location / System / Operator ID — this is
      the real end‑to‑end proof, not just a byte‑level nRF Connect decode.
- [ ] Broadcast **stops** (no more `ADVERTISING ok`, invisible to scanners)
      when GPS fix is lost or goes stale (>5 s) — the wait‑state (§2 of
      `docs/06`).
- [ ] Broadcast **resumes automatically** once the fix returns.
- [ ] Compare position reported over BLE against the aircraft's actual
      position/telemetry at the same moment (sanity cross‑check against
      whatever GCS/OSD you're already trusting).

---

## Phase 7 — Restart / robustness scenarios

- [ ] Confirm the config window reopens on **every** boot, even with
      Operator ID/class already saved.
- [ ] Confirm reconnecting to the config BLE instance works after a
      disconnect (not just the very first connection of the session).
- [ ] If you can safely simulate it: power‑cycle the **sniffer module only**
      while the aircraft is already armed and flying. Per `docs/06` §1 this
      is expected to **silently relatch to the current position**, not
      recover the original take‑off point — confirm this matches the
      documented (accepted, not fixed) behavior rather than something worse.

---

## Phase 8 — RF coexistence / physical

- [ ] Real RC link (LQ, RSSI, packet loss) unaffected by the BLE radio being
      active, in **both** the config phase and the broadcast phase — flagged
      as a residual risk in `../remote-id/PLAN.md`, never verified against a
      real link before this session.
- [ ] Module is physically **onboard the aircraft**, not ground‑side with the
      handset (`docs/06` §9) — required for the take‑off/pilot‑position
      framing to mean anything at all.
- [ ] Weight/CG impact acceptable if this is a weight‑sensitive airframe.

---

## Phase 9 — Base sniffer builds (Serial / BLE‑to‑GCS), for regression

Not RemoteID‑specific, but worth a quick pass since `crsf.py`/firmware
changes this session touched shared code:

- [ ] `T3S3_Sniffer_2400_RX_Serial`: telemetry displays/logs correctly on the
      debug console against the real link.
- [ ] `T3S3_Sniffer_2400_RX_BLE`: GCS connects over BLE, live dashboard
      updates.
- [ ] GCS CSV log captures `vario_ms` correctly from a combined
      `BARO_ALTITUDE` frame if this aircraft sends that variant (the
      `crsf.py` fix from this session) — check the CSV, not just the live UI.

---

## When something's wrong

Capture the serial log around the failure (see `docs/06` §6 for what a good
boot looks like, as a baseline to diff against) and the exact aircraft
state (armed/disarmed, GPS fix quality, FC/mode string if visible) at the
moment it happened — that's what turned "what flight modes exist" into two
real, fixed bugs during bench testing. Real telemetry will surface things
synthetic test cases can't.
