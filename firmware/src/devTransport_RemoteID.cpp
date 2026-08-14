// =====================================================================
// Ghost RX transport: EASA/ASD-STAN Direct Remote ID over BLE
// ---------------------------------------------------------------------
// Alternative to devTransport_BLE.cpp: instead of streaming reassembled
// CRSF telemetry to a GCS phone app over a GATT connection, this transport
// broadcasts ASTM F3411 / ASD-STAN EN 4709-002 "Direct Remote ID" messages
// as connectionless BLE advertisements, sourced from the telemetry the
// sniffer already overhears on the ELRS link - GPS (0x02), and if the
// aircraft sends them, VARIO (0x07) / BARO_ALTITUDE (0x09) / FLIGHT_MODE
// (0x21). No GATT server, no phone pairing, no separate GPS tap on the
// airframe - this module rides passively on the RC link exactly like the
// other Ghost transports and repurposes the same BLE radio to advertise
// instead of notify.
//
// Derived fields (no separate sensor needed):
//   - Vertical speed: from VARIO/BARO telemetry when the aircraft sends it,
//     else differentiated from successive GPS altitude samples.
//   - Height above take-off: current geodetic altitude minus the altitude
//     latched at take-off (see below).
//   - Operator/take-off altitude: the same latched take-off altitude.
//   - Baro altitude: passed through when BARO_ALTITUDE telemetry is present;
//     "unknown" otherwise (Location.AltitudeBaro is optional in the spec).
//   - Status: CRSF FLIGHT_MODE decoded Betaflight-style - "!FS!" (failsafe)
//     or "RTH" (GPS Rescue, itself part of Betaflight's failsafe escalation
//     on RX loss) -> ODID_STATUS_EMERGENCY; disarmed with a fix -> GROUND;
//     armed with a fix -> AIRBORNE. Best-effort - not every FC/mode string
//     follows this convention (see PASS/CHIR and other FC's mode strings,
//     which aren't specifically handled - only the arm-state suffix and the
//     "!FS"/"RTH" substrings are inspected, so unrecognized mode names still
//     classify correctly on arm state alone).
// WAIT state: instances 0/1 only broadcast while a GPS fix exists (or is
// fresh - stale >5s = treated as lost). No fix = nothing real to report, so
// rather than broadcast a Basic-ID-only "ghost" drone with an undeclared
// position, the ODID broadcast is paused entirely until a fix (re)appears.
//
// Take-off/operator position is latched at the moment FLIGHT_MODE reports a
// clean ARMED transition (falls back to "first GPS fix" if no flight-mode
// telemetry ever arrives; does NOT re-latch across a mid-flight RTH/failsafe
// excursion - see the FLIGHT_MODE case below for why). We have no visibility
// into the pilot's own GNSS from a drone-side sniffer, so this is reported as
// OperatorLocationType TAKEOFF, not LIVE_GNSS - spec-legal, and a reasonable
// snapshot when the pilot launches from where they're standing.
//
// IMPORTANT, read before trusting any "distance from pilot" computed from
// this broadcast: OperatorLatitude/Longitude/AltitudeGeo are a ONE-TIME
// SNAPSHOT taken at arm, not the pilot's live position. They go stale the
// moment the pilot moves after arming (walking a track between race gates,
// repositioning after launch, being handed the controller, etc.) or the
// moment the flight goes long-range - there is no live pilot GNSS source on
// this data path, and no field in the spec to flag "this operator location is
// stale" (System has no operator-location accuracy field, unlike Location's
// HorizAccuracy). Treat any distance/proximity a receiver derives from this
// System message as approximate at best, accurate only near the moment of
// arming, and not something to rely on for real separation/BVLOS decisions.
//
// NOT PERSISTED, BY DESIGN CHOICE (documented, not fixed - see chat
// 2026-08-13): s_haveTakeoff/s_takeoffLat/s_takeoffLon/s_takeoffAltM are
// plain RAM state, unlike the Operator ID and EU class (both NVS-backed).
// A sniffer restart - not the aircraft's, THIS MODULE'S - clears them. If
// that restart happens mid-flight (brownout, watchdog reset, reflash) while
// the aircraft is already armed, the take-off reference is not just lost,
// it is silently WRONG afterward: s_armed also resets to false, so the very
// next FLIGHT_MODE frame showing "armed" looks like a fresh disarmed->armed
// edge and re-latches to wherever the aircraft happens to be at that
// moment - not the real launch point - with no signal to anyone that this
// happened. Accepted trade-off, not a bug to silently work around: NVS
// persistence would need a reliable way to distinguish "sniffer hiccup,
// mid-flight" from "genuinely new flight after a full power-down" (a stale
// take-off point from the previous flight would be worse than none), which
// isn't free - revisit if this restart scenario turns out to matter in
// practice.
//
// Two lifecycle phases so the phone can actually connect:
//
//   CONFIG window (first REMOTEID_CONFIG_WINDOW_MS after boot, default 60 s):
//     ONLY instance 2 advertises. The ODID broadcast is held off because
//     running the Coded-PHY / multi-instance broadcast alongside a connectable
//     instance starves the CONNECT_IND handshake and connections fail (this is
//     the bug this phase split fixes). The window closes early once the
//     Operator ID is written and the phone disconnects.
//   BROADCAST phase (after the window): instance 2 is retired, instances 0+1
//     broadcast the ODID messages. Pure connectionless broadcaster, matching
//     ../../remote-id/PLAN.md §4's "not active during flight".
//
// Three advertising instances, per ASTM F3411 Annex A4 + one config-only:
//   Instance 0: Bluetooth 4 Legacy Advertising (1M PHY). 31-byte legacy
//               payload only fits ONE 25-byte ODID message per update, so
//               this instance round-robins Basic ID / Location / System,
//               biased toward Location (the only message ASTM requires at
//               a minimum 1 Hz).
//   Instance 1: Bluetooth 5 Long Range (LE Coded PHY, extended advertising).
//               Bigger payload budget (251B) fits the whole message pack
//               (Basic ID + Location + System) in one update.
//   Instance 2: Legacy, CONNECTABLE. A tiny GATT server (Nordic UART
//               Service, same UUIDs/command style as devTransport_BLE.cpp's
//               "P:<phrase>") for Operator ID entry from a phone, since this
//               module has no display/keypad. Persisted to NVS. Only up during
//               the CONFIG window (see above).
// Legacy-only scanners (most consumer phones, all of iOS) see instance 0;
// BT5-capable scanners (a subset of Android) can additionally pick up
// instance 1's longer range. See ../../remote-id/PLAN.md for the
// receiver-compatibility caveat this is working around.
//
// Requires NimBLE-Arduino built with -D CONFIG_BT_NIMBLE_EXT_ADV=1 and
// -D CONFIG_BT_NIMBLE_MAX_EXT_ADV_INSTANCES=2 (three instances: 0,1,2 - set
// in the T3S3_Sniffer_2400_RX_RemoteID env). Mutually exclusive with
// GHOST_TRANSPORT_BLE - both claim the one BLE radio identity.
//
// UTC TIME SOURCE: the CRSF GPS frame this sniffer decodes carries no time
// field, so there is no time source on the telemetry data path itself.
// Instead, the GCS (which has an accurate system clock and connects over
// USB serial most sessions) pushes its current UTC time once per connect via
// "T:<unix seconds>" - see RemoteID_SetTime(). This is RAM-only / session-only
// (millis()-derived, no NVS): every reconnect - including after a sniffer
// reboot - re-syncs it, same rationale as the take-off-position snapshot
// below. CAVEAT: if the sniffer flies a full session without a GCS ever
// connecting over serial (BLE broadcast only, no cable), both timestamp
// fields report their spec "unknown" value for that whole session - this is
// the accepted, honest fallback, not a bug.
// =====================================================================
#if defined(GHOST_TRANSPORT_REMOTEID) && defined(PLATFORM_ESP32)

#include <NimBLEDevice.h>
#include <Preferences.h>
#if !CONFIG_BT_NIMBLE_EXT_ADV
#error GHOST_TRANSPORT_REMOTEID needs -D CONFIG_BT_NIMBLE_EXT_ADV=1 (extended advertising)
#endif
#if CONFIG_BT_NIMBLE_MAX_EXT_ADV_INSTANCES < 2
#error GHOST_TRANSPORT_REMOTEID needs -D CONFIG_BT_NIMBLE_MAX_EXT_ADV_INSTANCES=2 (uses instances 0,1,2)
#endif

#undef htobe16
#undef be16toh
#undef htobe32
#undef be32toh

#include "sniffer.h"
#include "logging.h"
#include "common.h"   // extern UID[UID_LEN] - the ELRS binding UID

extern "C" {
#include "opendroneid.h"
}

// ---- Configurable at build time (mirrors the MY_BINDING_PHRASE pattern) ----
#ifndef REMOTEID_UA_TYPE
#define REMOTEID_UA_TYPE ODID_UATYPE_HELICOPTER_OR_MULTIROTOR
#endif
#ifndef REMOTEID_BROADCAST_PERIOD_MS
#define REMOTEID_BROADCAST_PERIOD_MS 1000   // ASTM F3411: Location >= 1 Hz while airborne
#endif
#ifndef REMOTEID_FIX_STALE_MS
// How long without a new GPS frame before s_haveFix is cleared and the WAIT
// state pauses the broadcast entirely (see RemoteID_Tick). MUST be
// comfortably larger than the real inter-GPS-frame interval this ELRS link
// can achieve, not an idealized "GPS is ~1Hz" assumption - that interval is
// entirely a function of the TX's configured Telem Ratio (shared with every
// other sensor type over the same limited uplink slots) and can easily be
// several seconds at a conservative ratio. Measured field data (2026-08-14,
// 150Hz air rate / 1:32 (STD) Telem Ratio): genuine GPS updates every
// ~4.6-4.9s. The previous hardcoded 5000ms sat right on top of that cadence,
// so any normal jitter (a single dropped/delayed telemetry chunk) pushed an
// interval over the cutoff and flapped the broadcast off - which a real
// scanner app can reasonably read as "no live aircraft position available"
// even while identity/operator fields (learned once, not re-validated every
// tick) keep showing. If your Telem Ratio is even more conservative than
// 1:32, raise this further; it should be a solid multiple of your actual
// observed GPS cadence, not just barely above it.
#define REMOTEID_FIX_STALE_MS 12000
#endif
#ifndef REMOTEID_CONFIG_WINDOW_MS
// After boot, advertise ONLY the connectable config instance for this long so a
// phone can reliably connect and set the Operator ID. The ODID broadcast
// (Legacy + Coded PHY) does not start until this window closes - running the
// Coded-PHY / multi-instance broadcast alongside a connectable instance starves
// the CONNECT_IND handshake and connections fail. The window also ends early
// once the Operator ID has been written and the phone disconnects. 60 s is well
// inside the ground time before a flight (arming + GPS lock take longer), and
// the sniffer has no GPS fix to broadcast this early anyway.
#define REMOTEID_CONFIG_WINDOW_MS 60000
#endif

static const uint16_t ODID_ASTM_UUID16 = 0xFFFA;      // ASTM International
static const uint8_t  ODID_AD_APP_CODE = 0x0D;         // AD Application Code = Open Drone ID

// Same NUS UUIDs + "<letter>:<value>\n" command style as devTransport_BLE.cpp,
// so the same generic "BLE UART" phone app works for both variants.
#define NUS_SERVICE "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_CHAR "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // phone writes here

static NimBLEExtAdvertising *s_adv = nullptr;
static uint8_t s_msgCounter = 0;

// Two-phase lifecycle:
//   RID_CONFIG    - boot: ONLY the connectable config instance (2) advertises,
//                   so a phone can connect and set the Operator ID cleanly.
//   RID_BROADCAST - config window closed: instance 2 down, ODID broadcast
//                   (Legacy inst 0 + Coded PHY inst 1) runs.
enum RIDPhase { RID_CONFIG, RID_BROADCAST };
static RIDPhase       s_phase = RID_CONFIG;
static uint32_t       s_bootMs = 0;
static volatile bool  s_clientConnected = false;  // a phone is connected right now
static volatile bool  s_configWriteDone = false;  // Operator ID written+saved this session

// BLE host-task callbacks must NOT do flash I/O (NVS) or drive advertising -
// a flash write while the BLE controller's ISRs run from flash crashes the
// chip. So the callbacks only set these flags/buffer; RemoteID_Tick (loop
// task) performs the actual NVS write and advertising restart.
static volatile bool  s_pendingSave = false;              // onWrite -> Tick: persist operator ID
static char           s_pendingOpId[ODID_ID_SIZE + 1] = {0};
static volatile bool  s_pendingReadvertise = false;       // onDisconnect -> Tick: restart config adv
static volatile int   s_pendingClass = -1;                // onWrite -> Tick: persist EU class (-1 = none)

// Operator ID: entered once over BLE from a phone (no display/keypad on this
// module), persisted to NVS. Mandatory for real EU/EASA conformance (unlike
// base ASTM F3411, where the Operator ID message is optional).
static Preferences s_prefs;
static char s_operatorId[ODID_ID_SIZE + 1] = {0};
static bool s_haveOperatorId = false;

// EU UA classification (System message), user-configurable, persisted to NVS.
// Deliberately only two options, not the full C0..C6 range: this firmware
// targets self-built aircraft, which are not CE-class-marked products - only
// a manufacturer can legitimately declare C1..C6. Offering those would let
// the module claim a certified class the airframe doesn't have. The two
// honest choices are:
//   REMOTEID_CLASS_LEGACY (0): no class marking declared. ClassificationType
//     = UNDECLARED. The accurate choice for almost every self-built craft.
//   REMOTEID_CLASS_C0     (1): ClassificationType = EU, ClassEU = CLASS_0.
//     Only select this if the aircraft genuinely meets the C0 criteria
//     (<250g, <19m/s Vmax, etc per EU 2019/945 Part 1) - "C0" is not a
//     free pass, it's a declared conformity claim like any other class.
#define REMOTEID_CLASS_LEGACY 0
#define REMOTEID_CLASS_C0     1
#ifndef REMOTEID_DEFAULT_CLASS
#define REMOTEID_DEFAULT_CLASS REMOTEID_CLASS_C0
#endif
static uint8_t s_classNum = REMOTEID_DEFAULT_CLASS;

// UTC time source for Location.TimeStamp / System.Timestamp: pushed once per
// serial (re)connect by the GCS (which has an accurate system clock) via the
// "T:<unix seconds>" command - see RemoteID_SetTime(). Session-only, RAM-only,
// same design as the take-off-position snapshot: no NVS persistence, because
// millis()-since-boot resets on every reboot anyway, and the GCS re-sends it
// on every connect (including post-reboot) for free. If no GCS ever connects
// over serial this session, s_haveTime stays false and both timestamp fields
// report their spec "unknown" value, same as before this feature existed.
static bool     s_haveTime     = false;
static uint32_t s_timeBaseUnix = 0;   // unix seconds at the moment of last sync
static uint32_t s_timeBaseMs   = 0;   // millis() at that same moment

// ODID's System.Timestamp epoch is 00:00:00 01/01/2019 UTC, not the Unix
// epoch - see opendroneid.h's ODID_System_data.Timestamp comment. Verified via
// date computation (1970-01-01T00:00:00Z -> 2019-01-01T00:00:00Z = 1546300800s).
#define REMOTEID_ODID_EPOCH_2019_UNIX 1546300800UL

static uint32_t nowUnix(void)
{
    return s_haveTime ? s_timeBaseUnix + (millis() - s_timeBaseMs) / 1000 : 0;
}

// ---- Location state, fed by CRSF telemetry frames sniffed off-air ----
// Written by the sink (loop task), read by fillUasData (loop task) - same
// thread, no contention. s_haveTakeoff also latches the operator/take-off
// position for the ASTM System message.
static volatile bool s_haveFix = false;
static double  s_lat = 0, s_lon = 0;          // degrees
static float   s_altM = -1000;                // m (ODID "invalid" sentinel)
static float   s_speedMs = 0;
static float   s_headingDeg = 0;
static uint8_t s_sats = 0;
static uint32_t s_lastFixMs = 0;

// Vertical speed (m/s): from VARIO/BARO telemetry when present, else derived
// from successive GPS geodetic-altitude samples.
static float    s_vspeedMs = 0;
static uint32_t s_vspeedTeleMs = 0;           // last time vspeed came from telemetry (not GPS-derived)
static float    s_lastGpsAltM = -1000;        // for GPS-derived vspeed
static uint32_t s_lastGpsAltMs = 0;

// Barometric altitude (m), from CRSF BARO_ALTITUDE telemetry if the aircraft sends it.
static float    s_baroAltM = -1000;
static bool     s_haveBaro = false;
static uint32_t s_lastBaroMs = 0;

// Flight state, decoded from CRSF FLIGHT_MODE string (Betaflight convention,
// verified against src/main/telemetry/crsf.c: "!FS!" = failsafe; otherwise a
// disarmed-only suffix of '*'/'!'/'?' means disarmed, no suffix means armed).
static bool s_haveMode = false;
static bool s_armed = false;
// EMERGENCY-worthy state, NOT just literal RC failsafe: also true for "RTH"
// (GPS Rescue). In Betaflight, GPS Rescue is entered as part of the failsafe
// escalation on RX loss (and can also be switch-triggered), so an aircraft
// autonomously returning home is exactly the situation Remote ID's EMERGENCY
// status exists for, even on the tick where the FLIGHT_MODE string reads
// "RTH" rather than "!FS!".
static bool s_emergency = false;

// Take-off/operator position: latched at ARM (or first fix as fallback).
static bool   s_haveTakeoff = false;
static double s_takeoffLat = 0, s_takeoffLon = 0;
static float  s_takeoffAltM = -1000;

class RIDConfigCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *c) override {
        std::string v = c->getValue();
        // All heavy work (NVS flash write) is deferred to RemoteID_Tick via
        // these flags/buffers - a flash write inside a BLE callback crashes the chip.
        if (v.rfind("O:", 0) == 0) {              // "O:<operator id>"
            std::string opId = v.substr(2);
            while (!opId.empty() && (opId.back() == '\n' || opId.back() == '\r')) opId.pop_back();
            if (opId.length() > ODID_ID_SIZE) opId.resize(ODID_ID_SIZE);   // bound untrusted input
            strncpy(s_pendingOpId, opId.c_str(), ODID_ID_SIZE);            // fill buffer BEFORE flag
            s_pendingOpId[ODID_ID_SIZE] = 0;
            s_pendingSave = true;
            DBGLN("[RID] operator ID received (%u chars), saving in loop task", (unsigned)opId.length());
        }
        else if (v.rfind("C:", 0) == 0 && v.length() >= 3) {   // "C:0"=Legacy, "C:1"=C0 only
            int cls = v[2] - '0';
            if (cls == REMOTEID_CLASS_LEGACY || cls == REMOTEID_CLASS_C0) {
                s_pendingClass = cls;
                DBGLN("[RID] EU class %s received, saving in loop task", cls == REMOTEID_CLASS_C0 ? "C0" : "Legacy");
            }
        }
    }
};

// Server callbacks: track connection state and, during the config window,
// re-advertise the connectable config instance after a disconnect. With
// CONFIG_BT_NIMBLE_EXT_ADV enabled, NimBLEServer's built-in
// advertise-on-disconnect is compiled out (`#if !CONFIG_BT_NIMBLE_EXT_ADV` in
// NimBLEServer.cpp), so without this the phone would get exactly ONE connection
// per boot to set the ID.
class RIDServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *) override {
        s_clientConnected = true;
        DBGLN("[RID] config client connected");
    }
    void onDisconnect(NimBLEServer *) override {
        s_clientConnected = false;
        // Defer the re-advertise to the loop task (don't drive advertising from
        // the host-task callback). RemoteID_Tick restarts instance 2 only if
        // we're still in the config window and the ID wasn't already saved.
        s_pendingReadvertise = true;
        DBGLN("[RID] config client disconnected");
    }
};

static inline int32_t be32s(const uint8_t *p) {
    return (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]);
}
static inline uint16_t be16u(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static inline int16_t  be16s(const uint8_t *p) { return (int16_t)((p[0] << 8) | p[1]); }

// Latch the current position+altitude as the take-off / operator reference.
static void latchTakeoff(const char *why)
{
    if (!s_haveFix) return;
    s_takeoffLat  = s_lat;
    s_takeoffLon  = s_lon;
    s_takeoffAltM = s_altM;
    s_haveTakeoff = true;
    DBGLN("[RID] latched take-off (%s) %d,%d alt=%dm",
          why, (int)(s_lat * 1e7), (int)(s_lon * 1e7), (int)s_altM);
}

// ---- Sink: pull GPS / vario / baro / flight-mode out of the sniffed CRSF ----
static void RemoteID_sink(const uint8_t *frame, uint8_t len)
{
    if (len < 4) return;
    const uint8_t type = frame[2];
    const uint8_t *p   = frame + 3;
    const uint8_t plen = (len >= 4) ? (uint8_t)(len - 4) : 0;   // payload bytes (excl dest/len/type/crc)

    switch (type)
    {
    case CRSF_FRAMETYPE_GPS: {
        if (plen < 15) return;
        s_lat        = be32s(p)      * 1e-7;               // deg
        s_lon        = be32s(p + 4)  * 1e-7;                // deg
        s_speedMs    = be16u(p + 8)  * 0.1f * (1000.0f / 3600.0f); // 0.1km/h -> m/s
        s_headingDeg = be16u(p + 10) * 0.01f;               // deg
        s_altM       = (float)((int32_t)be16u(p + 12) - 1000); // m
        s_sats       = p[14];
        s_lastFixMs  = millis();
        s_haveFix    = (s_lat != 0.0 || s_lon != 0.0);

        // GPS-derived vertical speed (fallback when no vario/baro telemetry).
        // Upper dt bound shares REMOTEID_FIX_STALE_MS's reasoning (see its
        // comment) rather than an idealized "GPS is ~1Hz" assumption - a
        // hardcoded 5.0f here sat right on top of the real ~4.6-4.9s cadence
        // measured at 1:64 Telem Ratio and would intermittently skip valid
        // updates for no reason.
        uint32_t now = millis();
        if (s_lastGpsAltMs != 0) {
            float dt = (now - s_lastGpsAltMs) / 1000.0f;
            if (dt >= 0.15f && dt < (REMOTEID_FIX_STALE_MS / 1000.0f) && (now - s_vspeedTeleMs) > 3000) {
                s_vspeedMs = (s_altM - s_lastGpsAltM) / dt;
            }
        }
        s_lastGpsAltM = s_altM;
        s_lastGpsAltMs = now;

        // Take-off latch, GPS side: normal case is "first fix" when no flight-mode
        // telemetry ever arrives. Also covers joining mid-flight already-armed: the
        // FLIGHT_MODE handler's rising-edge latch below no-ops without a fix yet
        // (armed state has already been observed, so no *later* edge will fire) -
        // catch that here as soon as a fix does arrive.
        if (s_haveFix && !s_haveTakeoff && (!s_haveMode || s_armed))
            latchTakeoff(s_armed ? "arm (deferred, fix arrived after mode)" : "first fix");
        break;
    }
    case CRSF_FRAMETYPE_VARIO: {
        if (plen < 2) return;
        s_vspeedMs = be16s(p) / 100.0f;   // cm/s -> m/s
        s_vspeedTeleMs = millis();
        break;
    }
    case CRSF_FRAMETYPE_BARO_ALTITUDE: {
        if (plen < 2) return;
        uint16_t raw = be16u(p);
        if (raw & 0x8000) s_baroAltM = (float)(raw & 0x7FFF);        // high bit set -> meters
        else              s_baroAltM = ((int32_t)raw - 10000) / 10.0f;  // decimeters + 10000dm
        s_haveBaro = true;
        s_lastBaroMs = millis();
        if (plen >= 4) {                   // combined baro+vario frame also carries vertical speed
            s_vspeedMs = be16s(p + 2) / 100.0f;
            s_vspeedTeleMs = millis();
        }
        break;
    }
    case CRSF_FRAMETYPE_FLIGHT_MODE: {
        char mode[17]; uint8_t i = 0;
        while (i < 16 && i < plen && p[i]) { mode[i] = (char)p[i]; i++; }
        mode[i] = 0;
        s_haveMode = true;
        // Betaflight (src/main/telemetry/crsf.c, crsfFrameFlightMode()):
        // "!FS!" alone = failsafe, "RTH" = GPS Rescue (also entered as part of
        // Betaflight's own failsafe escalation on RX loss) - both EMERGENCY.
        // Otherwise a disarmed-only suffix is appended to the mode string -
        // '*' ready to arm, '!' arming disabled, '?' GPS rescue disabled -
        // armed craft carry no suffix at all.
        s_emergency = (strstr(mode, "!FS") != nullptr) || (strstr(mode, "RTH") != nullptr);

        bool armedNow;
        if (s_emergency) {
            // The generic disarmed-suffix check does NOT apply to these two
            // strings: Betaflight never appends a suffix while FAILSAFE_MODE
            // is set, so "!FS!"'s trailing '!' is just the last character of
            // that fixed 4-char string, not an "arming disabled" marker - and
            // GPS Rescue only ever runs while armed. Treat both as armed
            // (matches reality almost always) without re-deriving from the
            // suffix, which would misread "!FS!" as disarmed.
            armedNow = true;
        } else {
            bool disarmed = (i > 0) && (mode[i - 1] == '*' || mode[i - 1] == '!' || mode[i - 1] == '?');
            armedNow = (i > 0) && !disarmed;
        }
        // Only latch on a clean, non-emergency arm edge. Excluding emergency
        // here (on top of forcing armedNow=true above) matters once the
        // emergency ends: s_armed is already true by then, so the edge
        // (armedNow && !s_armed) does not re-fire and re-anchor the take-off/
        // operator position to wherever the RTH/failsafe excursion happened
        // to end - it stays at the original arm point, which is the whole
        // point of latching at arm in the first place.
        if (armedNow && !s_armed && !s_emergency) latchTakeoff("arm");
        s_armed = armedNow;
        break;
    }
    default:
        break;
    }
}

// ---- Mirrors opendroneid-core-c's wifi.c odid_message_build_pack() logic
// (same header, same encoders - just re-linked here so we don't pull the
// Linux NAN/beacon-frame compilation unit into an ESP32 build). ----
static int buildMessagePack(const ODID_UAS_Data *uas, uint8_t *out, size_t outLen)
{
    ODID_MessagePack_data pack;
    pack.SingleMessageSize = ODID_MESSAGE_SIZE;
    pack.MsgPackSize = 0;

    for (int i = 0; i < ODID_BASIC_ID_MAX_MESSAGES; i++)
        if (uas->BasicIDValid[i] && pack.MsgPackSize < ODID_PACK_MAX_MESSAGES &&
            encodeBasicIDMessage(&pack.Messages[pack.MsgPackSize].basicId, &uas->BasicID[i]) == ODID_SUCCESS)
            pack.MsgPackSize++;

    if (uas->LocationValid && pack.MsgPackSize < ODID_PACK_MAX_MESSAGES &&
        encodeLocationMessage(&pack.Messages[pack.MsgPackSize].location, &uas->Location) == ODID_SUCCESS)
        pack.MsgPackSize++;

    if (uas->SystemValid && pack.MsgPackSize < ODID_PACK_MAX_MESSAGES &&
        encodeSystemMessage(&pack.Messages[pack.MsgPackSize].system, &uas->System) == ODID_SUCCESS)
        pack.MsgPackSize++;

    if (uas->OperatorIDValid && pack.MsgPackSize < ODID_PACK_MAX_MESSAGES &&
        encodeOperatorIDMessage(&pack.Messages[pack.MsgPackSize].operatorId, &uas->OperatorID) == ODID_SUCCESS)
        pack.MsgPackSize++;

    if (pack.MsgPackSize == 0) return -1;

    const size_t len = 3 + (size_t)pack.MsgPackSize * ODID_MESSAGE_SIZE; // header(3) + N*25
    if (len > outLen) return -1;

    out[0] = (uint8_t)((ODID_MESSAGETYPE_PACKED << 4) | (ODID_PROTOCOL_VERSION & 0x0F));
    out[1] = pack.SingleMessageSize;
    out[2] = pack.MsgPackSize;
    memcpy(out + 3, pack.Messages, (size_t)pack.MsgPackSize * ODID_MESSAGE_SIZE);
    return (int)len;
}

// ---- Fill the shared ODID_UAS_Data snapshot from current sniffer state ----
static void fillUasData(ODID_UAS_Data *uas)
{
    odid_initUasData(uas);

    // Basic ID: no real registered serial number on this data path (we only
    // overhear the aircraft, we don't own it) - synthesize an identifier
    // from the ELRS binding UID so the same aircraft is at least consistently
    // identifiable. NOT a substitute for a real CAA-issued ID in production.
    char uasId[ODID_ID_SIZE + 1];
    snprintf(uasId, sizeof(uasId), "ELRS-%02X%02X%02X%02X%02X%02X",
             UID[0], UID[1], UID[2], UID[3], UID[4], UID[5]);
    uas->BasicID[0].UAType = REMOTEID_UA_TYPE;
    uas->BasicID[0].IDType = ODID_IDTYPE_SPECIFIC_SESSION_ID; // synthesized, not a CAA registration
    strncpy(uas->BasicID[0].UASID, uasId, ODID_ID_SIZE);
    uas->BasicIDValid[0] = 1;

    // Status, from CRSF FLIGHT_MODE's arm state (priority order matches
    // Betaflight's own: failsafe/RTH override everything else):
    //   failsafe or RTH (GPS Rescue)   -> EMERGENCY
    //   no GPS fix                     -> UNDECLARED (nothing to report)
    //   have fix, mode telemetry seen, disarmed -> GROUND (was wrongly AIRBORNE
    //     before this fix - a disarmed, GPS-locked drone sitting on the bench
    //     or the field is not "airborne")
    //   have fix, armed OR no mode telemetry ever seen -> AIRBORNE (fallback
    //     for aircraft that don't send FLIGHT_MODE at all - best-effort, matches
    //     the pre-existing default when ground/air can't be determined)
    if (s_emergency)              uas->Location.Status = ODID_STATUS_EMERGENCY;
    else if (!s_haveFix)          uas->Location.Status = ODID_STATUS_UNDECLARED;
    else if (s_haveMode && !s_armed) uas->Location.Status = ODID_STATUS_GROUND;
    else                          uas->Location.Status = ODID_STATUS_AIRBORNE;
    uas->Location.Direction       = s_headingDeg;
    uas->Location.SpeedHorizontal = s_speedMs;
    uas->Location.SpeedVertical   = s_vspeedMs;         // from vario/baro telemetry, else GPS-derived
    uas->Location.Latitude        = s_lat;
    uas->Location.Longitude       = s_lon;
    uas->Location.AltitudeGeo     = s_altM;
    uas->Location.AltitudeBaro    = s_haveBaro ? s_baroAltM : -1000;   // if the aircraft sends baro
    uas->Location.HeightType      = ODID_HEIGHT_REF_OVER_TAKEOFF;
    // Height above take-off = current geodetic altitude - latched take-off altitude.
    uas->Location.Height          = (s_haveTakeoff && s_takeoffAltM > -1000) ? (s_altM - s_takeoffAltM) : -1000;
    uas->Location.HorizAccuracy   = createEnumHorizontalAccuracy(s_sats >= 6 ? 10.0f : 92.6f);
    uas->Location.VertAccuracy    = ODID_VER_ACC_UNKNOWN;
    uas->Location.BaroAccuracy    = ODID_VER_ACC_UNKNOWN;
    uas->Location.SpeedAccuracy   = ODID_SPEED_ACC_UNKNOWN;
    uas->Location.TSAccuracy      = ODID_TIME_ACC_UNKNOWN;   // synced once per serial connect, not a live GNSS PPS - don't overclaim precision
    uas->Location.TimeStamp       = s_haveTime ? (float)(nowUnix() % 3600) : INV_TIMESTAMP;  // seconds after the full hour (spec units), UNKNOWN until a GCS has synced us over serial - see RemoteID_SetTime()
    uas->LocationValid            = s_haveFix ? 1 : 0;

    // Operator/take-off location: a STATIC SNAPSHOT latched once at the clean
    // arm edge (see file header "IMPORTANT" note above and the FLIGHT_MODE
    // case for exactly when it (re)latches and when it deliberately doesn't -
    // e.g. never mid-RTH/failsafe). Not the pilot's live position; treat any
    // distance-from-pilot derived from it as approximate, accurate only near
    // the moment of arming.
    uas->System.OperatorLocationType = ODID_OPERATOR_LOCATION_TYPE_TAKEOFF;
    uas->System.OperatorLatitude     = s_haveTakeoff ? s_takeoffLat : 0;
    uas->System.OperatorLongitude    = s_haveTakeoff ? s_takeoffLon : 0;
    uas->System.OperatorAltitudeGeo  = (s_haveTakeoff && s_takeoffAltM > -1000) ? s_takeoffAltM : -1000;
    uas->System.AreaCount            = 1;
    uas->System.AreaRadius           = 0;
    uas->System.AreaCeiling          = -1000;
    uas->System.AreaFloor            = -1000;
    // EU classification (user-configured, default C0; see REMOTEID_CLASS_*
    // above for why only C0/Legacy are offered). Legacy = no class claimed.
    if (s_classNum == REMOTEID_CLASS_C0) {
        uas->System.ClassificationType = ODID_CLASSIFICATION_TYPE_EU;
        uas->System.CategoryEU         = ODID_CATEGORY_EU_OPEN;
        uas->System.ClassEU            = ODID_CLASS_EU_CLASS_0;
    } else {
        uas->System.ClassificationType = ODID_CLASSIFICATION_TYPE_UNDECLARED;   // "Legacy" - no class marking
    }
    // Seconds since the ODID 2019 epoch, UNKNOWN as literal 0 (no sentinel is
    // defined for this field - see file header) until a GCS has synced us.
    uas->System.Timestamp            = (s_haveTime && nowUnix() >= REMOTEID_ODID_EPOCH_2019_UNIX)
                                            ? (nowUnix() - REMOTEID_ODID_EPOCH_2019_UNIX) : 0;
    uas->SystemValid                 = s_haveTakeoff ? 1 : 0;

    if (s_haveOperatorId) {
        uas->OperatorID.OperatorIdType = ODID_OPERATOR_ID;
        strncpy(uas->OperatorID.OperatorId, s_operatorId, ODID_ID_SIZE);
        uas->OperatorIDValid = 1;
    }
    // No Self-ID input path on this variant - SelfID is optional in the spec
    // (unlike Operator ID, which is mandatory for EU/EASA conformance and so
    // gets the BLE config path above). Add one the same way if needed.
}

// ---- Push one AD "Service Data - 0xFFFA" payload into an adv instance,
// restarting that instance so the new data actually goes out.
// ponytail: stop/set/start each tick rather than relying on a live
// in-place data update - simplest thing that is definitely correct with
// this NimBLE-Arduino version; costs a sub-second gap in the advertising
// train once a second. Revisit only if bench testing shows a scanner
// missing broadcasts because of it. ----
static void setServiceDataInstance(uint8_t instance, const uint8_t *appPayload, size_t appLen, bool legacy, uint8_t phy)
{
    NimBLEExtAdvertisement adv(phy, phy);
    adv.setLegacyAdvertising(legacy);
    adv.setConnectable(false);
    adv.setScannable(false);
    adv.setServiceData(NimBLEUUID(ODID_ASTM_UUID16), std::string((const char *)appPayload, appLen));

    s_adv->stop(instance);
    bool dataOk = s_adv->setInstanceData(instance, adv);
    bool startOk = s_adv->start(instance);

    // One-shot diagnostic per instance: the ONLY firmware-side signal that the
    // controller accepted this advertisement - especially instance 1, whose
    // Coded PHY / extended advert most phones can't scan to confirm externally.
    // Logs the first success (with the on-air size) and every failure.
    static bool loggedOk[3] = {false, false, false};
    if (instance < 3) {
        if (startOk && dataOk) {
            if (!loggedOk[instance]) {
                loggedOk[instance] = true;
                DBGLN("[RID] instance %u ADVERTISING ok (%s, %u data bytes)",
                      instance,
                      phy == BLE_HCI_LE_PHY_CODED ? "Coded PHY/LongRange" :
                          (legacy ? "Legacy 1M" : "Ext 1M"),
                      (unsigned)appLen);
            }
        } else {
            loggedOk[instance] = false;   // re-log if it recovers
            DBGLN("[RID] instance %u start FAILED (setData=%d start=%d)",
                  instance, (int)dataOk, (int)startOk);
        }
    }
}

// Machine-parseable Operator ID line for a host/GCS reading the debug serial.
void RemoteID_ReportOperatorId(void)
{
    DBGLN("[RID] OPID=%s", s_haveOperatorId ? s_operatorId : "");
}

// Set + persist the Operator ID (serial "O:" path). Loop-task context, so the
// NVS flash write is safe to do directly here (unlike the BLE callback path).
void RemoteID_SetOperatorId(const char *id)
{
    if (!id) return;
    strncpy(s_operatorId, id, ODID_ID_SIZE);
    s_operatorId[ODID_ID_SIZE] = 0;
    s_haveOperatorId = (s_operatorId[0] != 0);
    s_prefs.putString("opid", s_operatorId);
    s_configWriteDone = true;   // configured -> let the config window close
    DBGLN("[RID] operator ID set via serial, saved to NVS");
    RemoteID_ReportOperatorId();
}

// Machine-parseable EU class line for a host/GCS reading the debug serial.
// Prints the human label ("LEGACY"/"C0"), not the raw stored number, since
// that's what a host needs to display/round-trip unambiguously.
void RemoteID_ReportClass(void)
{
    DBGLN("[RID] CLASS=%s", s_classNum == REMOTEID_CLASS_C0 ? "C0" : "LEGACY");
}

// Set + persist the EU class. Only REMOTEID_CLASS_LEGACY (0) or
// REMOTEID_CLASS_C0 (1) are accepted - see the REMOTEID_CLASS_* comment
// above for why the rest of the C1..C6 range isn't offered. Loop-task
// context (NVS write).
void RemoteID_SetClass(uint8_t classNum)
{
    if (classNum != REMOTEID_CLASS_LEGACY && classNum != REMOTEID_CLASS_C0) return;
    s_classNum = classNum;
    s_prefs.putUChar("class", s_classNum);
    DBGLN("[RID] EU class saved to NVS");
    RemoteID_ReportClass();
}

// Sync the ODID clock from the GCS's system clock (serial "T:" path only -
// there's no other UTC source on this data path, see file header). No NVS
// write: deliberately session-only, re-sent by the GCS on every connect
// (including after a sniffer reboot), same rationale as the take-off-position
// snapshot. Loop-task context, called directly from Sniffer_PollSerialCmd().
void RemoteID_SetTime(uint32_t unixSeconds)
{
    s_timeBaseUnix = unixSeconds;
    s_timeBaseMs   = millis();
    s_haveTime     = true;
    // debugPrintf() is a minimal hand-rolled formatter (see logging.cpp) -
    // only single-char %s/%d/%u/%x/%f, no %l length modifier. %lu here would
    // silently print garbage ("u" with the value dropped), not a real number.
    DBGLN("[RID] UTC time synced via serial (unix=%u)", (unsigned)unixSeconds);
}

void RemoteID_Tick(uint32_t nowMs)
{
    if (!s_adv) return;

    // ---- Service work the BLE callbacks deferred to us (loop task) ----
    // NVS flash write for the Operator ID: doing this here, not in onWrite,
    // avoids a flash-write-in-BLE-callback crash.
    if (s_pendingSave) {
        s_pendingSave = false;
        strncpy(s_operatorId, s_pendingOpId, ODID_ID_SIZE);
        s_operatorId[ODID_ID_SIZE] = 0;
        s_haveOperatorId = (s_operatorId[0] != 0);
        s_prefs.putString("opid", s_operatorId);
        s_configWriteDone = true;   // lets the config window close once the phone disconnects
        DBGLN("[RID] operator ID saved to NVS (via BLE)");
        RemoteID_ReportOperatorId();   // parseable line for a GCS on the serial
    }
    if (s_pendingClass >= 0) {
        s_classNum = (uint8_t)s_pendingClass;
        s_pendingClass = -1;
        s_prefs.putUChar("class", s_classNum);
        DBGLN("[RID] EU class saved to NVS (via BLE)");
        RemoteID_ReportClass();
    }
    if (s_pendingReadvertise) {
        s_pendingReadvertise = false;
        // Only bother if still configuring and the ID wasn't already saved
        // (if it was, we're about to transition to broadcast anyway).
        if (s_phase == RID_CONFIG && !s_configWriteDone && !s_clientConnected) {
            bool ok = s_adv->start(2);
            DBGLN("[RID] re-advertising config after disconnect: %s", ok ? "ok" : "FAIL");
        }
    }

    // ---- CONFIG phase: ONLY instance 2 (connectable config) is advertising.
    // Do not broadcast ODID yet - a phone can't reliably connect while the
    // Coded-PHY / multi-instance broadcast is contending for the radio. Close
    // the window once it expires OR the Operator ID was written, but never
    // while a phone is mid-session (don't cut off an in-flight write). ----
    if (s_phase == RID_CONFIG) {
        bool windowExpired = (nowMs - s_bootMs) >= REMOTEID_CONFIG_WINDOW_MS;
        if ((windowExpired || s_configWriteDone) && !s_clientConnected) {
            s_adv->stop(2);
            s_phase = RID_BROADCAST;
            DBGLN("[RID] config window closed (%s) -> ODID broadcast (Legacy + Coded PHY)",
                  s_configWriteDone ? "operator ID set" : "timeout");
        }
        return;   // no ODID broadcast during config
    }

    // ---- BROADCAST phase ----
    static uint32_t last = 0;
    if (nowMs - last < REMOTEID_BROADCAST_PERIOD_MS) return;
    last = nowMs;

    // GPS telemetry is only as fresh as the last sniffed CRSF GPS frame;
    // if the sniffer lost lock on the RC link, stop claiming a fix. See
    // REMOTEID_FIX_STALE_MS's comment - this must stay comfortably above the
    // real achievable GPS cadence for this link, not a hardcoded guess.
    if (s_haveFix && (nowMs - s_lastFixMs) > REMOTEID_FIX_STALE_MS) s_haveFix = false;

    // WAIT state: no drone telemetry (no GPS fix, ever or currently) means we
    // have nothing real to report. Broadcasting a Basic-ID-only "drone" with an
    // undeclared position isn't useful Remote ID - it's a ghost. Pause
    // instances 0/1 entirely and wait for a real fix instead of burning
    // airtime on it; resume the moment one arrives.
    //
    // INVARIANT (deliberate, do not "simplify" this away in a future edit):
    // this only ever touches instances 0/1. It can only run once we're past
    // the `s_phase == RID_CONFIG` block's unconditional `return` above, so the
    // BLE config window (instance 2) is untouched by it - and it never touches
    // Serial/SerialLogger, so the GCS's serial link is untouched too. Waiting
    // for telemetry must never delay or interrupt either of those.
    static bool s_broadcastActive = false;
    if (!s_haveFix) {
        if (s_broadcastActive) {
            s_adv->stop(0);
            s_adv->stop(1);
            s_broadcastActive = false;
            DBGLN("[RID] no drone telemetry - broadcast paused, waiting for a GPS fix");
        }
        return;
    }
    if (!s_broadcastActive) {
        s_broadcastActive = true;
        DBGLN("[RID] GPS fix acquired - resuming ODID broadcast");
    }

    ODID_UAS_Data uas;
    fillUasData(&uas);
    s_msgCounter++;

    // Instance 0 (Legacy, 1M PHY): 31-byte cap -> one message per tick.
    // Bias toward Location (the only ASTM-mandated >=1Hz field): 2 of every
    // 3 ticks send Location, the 3rd alternates Basic ID / System so
    // Legacy-only scanners still see identity, not just position.
    ODID_Message_encoded single;
    static uint8_t rr = 0;
    static bool sendBasicIdNext = true;
    rr = (rr + 1) % 3;
    int ok = -1;
    if (rr == 2) {
        if (sendBasicIdNext && uas.BasicIDValid[0])
            ok = encodeBasicIDMessage(&single.basicId, &uas.BasicID[0]);
        else if (uas.SystemValid)
            ok = encodeSystemMessage(&single.system, &uas.System);
        sendBasicIdNext = !sendBasicIdNext;
    }
    if (ok != ODID_SUCCESS && uas.LocationValid)
        ok = encodeLocationMessage(&single.location, &uas.Location);

    if (ok == ODID_SUCCESS) {
        uint8_t legacyPayload[2 + ODID_MESSAGE_SIZE];
        legacyPayload[0] = ODID_AD_APP_CODE;
        legacyPayload[1] = s_msgCounter;
        memcpy(legacyPayload + 2, single.rawData, ODID_MESSAGE_SIZE);
        setServiceDataInstance(0, legacyPayload, sizeof(legacyPayload), /*legacy=*/true, BLE_HCI_LE_PHY_1M);
    }

    // Instance 1 (Long Range, Coded PHY): full pack in one extended advert.
    uint8_t packPayload[2 + 3 + ODID_PACK_MAX_MESSAGES * ODID_MESSAGE_SIZE];
    packPayload[0] = ODID_AD_APP_CODE;
    packPayload[1] = s_msgCounter;
    int packLen = buildMessagePack(&uas, packPayload + 2, sizeof(packPayload) - 2);
    if (packLen > 0) {
        setServiceDataInstance(1, packPayload, 2 + (size_t)packLen, /*legacy=*/false, BLE_HCI_LE_PHY_CODED);
    }
}

void RemoteID_Transport_Init(void)
{
    DBGLN("[RID] init: BLE Direct Remote ID broadcast (ASTM F3411 / EN 4709-002)");
    NimBLEDevice::init("");   // no GAP name AD - legacy instance has no byte budget left for one

    s_prefs.begin("remoteid", /*readOnly=*/false);
    String savedOpId = s_prefs.getString("opid", "");
    if (savedOpId.length() > 0) {
        strncpy(s_operatorId, savedOpId.c_str(), ODID_ID_SIZE);
        s_operatorId[ODID_ID_SIZE] = 0;
        s_haveOperatorId = true;
        DBGLN("[RID] loaded operator ID from NVS (%u chars)", savedOpId.length());
    } else {
        DBGLN("[RID] no operator ID in NVS yet - set via BLE 'O:' char or serial 'O:<id>'");
    }
    RemoteID_ReportOperatorId();   // emit parseable OPID= line at boot for a GCS

    s_classNum = s_prefs.getUChar("class", REMOTEID_DEFAULT_CLASS);
    if (s_classNum != REMOTEID_CLASS_LEGACY && s_classNum != REMOTEID_CLASS_C0)
        s_classNum = REMOTEID_DEFAULT_CLASS;   // guard against garbage/stale NVS
    RemoteID_ReportClass();        // emit parseable CLASS= line at boot for a GCS

    s_adv = NimBLEDevice::getAdvertising();
    // MUST set ext-adv callbacks: NimBLEExtAdvertising's constructor leaves
    // m_pCallbacks uninitialized, and NimBLE dereferences it on the
    // ADV_COMPLETE event that fires when the connectable config instance
    // terminates on a connection -> crash (LoadProhibited @ garbage ptr) the
    // instant a phone connects. nullptr installs the library's default no-op
    // handler (we don't need adv-stop/scan-req events).
    s_adv->setCallbacks(nullptr);
    s_bootMs = millis();
    s_phase = RID_CONFIG;   // config window opens now; RemoteID_Tick closes it

    // Instance 2: connectable config GATT server (Operator ID entry). This is
    // the ONLY thing advertising during the config window - the ODID broadcast
    // (instances 0/1) does not start until RemoteID_Tick transitions to
    // RID_BROADCAST, so the phone can connect without radio contention.
    NimBLEServer *server = NimBLEDevice::createServer();
    server->setCallbacks(new RIDServerCallbacks());   // re-advertise config on disconnect
    NimBLEService *svc = server->createService(NUS_SERVICE);
    NimBLECharacteristic *rx =
        svc->createCharacteristic(NUS_RX_CHAR, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    rx->setCallbacks(new RIDConfigCallbacks());
    svc->start();

    NimBLEExtAdvertisement cfgAdv(BLE_HCI_LE_PHY_1M, BLE_HCI_LE_PHY_1M);
    cfgAdv.setLegacyAdvertising(true);
    cfgAdv.setConnectable(true);
    cfgAdv.setName("ELRS-RID-CFG");
    NimBLEExtAdvertisement cfgScanResp(BLE_HCI_LE_PHY_1M, BLE_HCI_LE_PHY_1M);
    cfgScanResp.setCompleteServices(NimBLEUUID(NUS_SERVICE));
    s_adv->setInstanceData(2, cfgAdv);
    s_adv->setScanResponseData(2, cfgScanResp);
    bool cfgUp = s_adv->start(2);
    DBGLN("[RID] config instance (connectable, 'ELRS-RID-CFG') %s - open %us for Operator ID entry",
          cfgUp ? "up" : "FAILED TO START", (unsigned)(REMOTEID_CONFIG_WINDOW_MS / 1000));

    Ghost_Transport_Register(&RemoteID_sink);
}

#endif // GHOST_TRANSPORT_REMOTEID
