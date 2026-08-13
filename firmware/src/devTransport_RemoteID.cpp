// =====================================================================
// Ghost RX transport: EASA/ASD-STAN Direct Remote ID over BLE
// ---------------------------------------------------------------------
// Alternative to devTransport_BLE.cpp: instead of streaming reassembled
// CRSF telemetry to a GCS phone app over a GATT connection, this transport
// broadcasts ASTM F3411 / ASD-STAN EN 4709-002 "Direct Remote ID" messages
// as connectionless BLE advertisements, sourced from the GPS telemetry the
// sniffer already overhears on the ELRS link (CRSF_FRAMETYPE_GPS). No GATT
// server, no phone pairing, no separate GPS tap on the airframe - this
// module rides passively on the RC link exactly like the other Ghost
// transports and repurposes the same BLE radio to advertise instead of
// notify.
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
//               "P:<phrase>") for one-time Operator ID entry from a phone,
//               since this module has no display/keypad. Persisted to NVS
//               so it only needs setting once. Stops itself the first time
//               the aircraft is seen airborne (first GPS fix) - once
//               flying, this is a pure connectionless broadcaster, matching
//               ../../remote-id/PLAN.md §4's "not active during flight".
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
// KNOWN GAP (flagged, not hidden): ODID_System_data.Timestamp wants real
// UTC seconds since 2019-01-01. The CRSF GPS frame this sniffer decodes
// carries no time field, so there is no time source on this data path.
// Using millis()-since-boot here would be actively wrong (decoders may
// treat it as a real UTC offset), so the timestamp is left at 0
// (unknown/not provided) until a real time source (GPS NMEA ZDA/RMC via a
// direct tap, or SNTP over the WiFi-UDP transport's AP) is wired in.
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

static const uint16_t ODID_ASTM_UUID16 = 0xFFFA;      // ASTM International
static const uint8_t  ODID_AD_APP_CODE = 0x0D;         // AD Application Code = Open Drone ID

// Same NUS UUIDs + "<letter>:<value>\n" command style as devTransport_BLE.cpp,
// so the same generic "BLE UART" phone app works for both variants.
#define NUS_SERVICE "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_CHAR "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // phone writes here

static NimBLEExtAdvertising *s_adv = nullptr;
static uint8_t s_msgCounter = 0;
static bool    s_cfgInstanceUp = false;   // instance 2 (connectable config) still advertising?

// Operator ID: entered once over BLE from a phone (no display/keypad on this
// module), persisted to NVS. Mandatory for real EU/EASA conformance (unlike
// base ASTM F3411, where the Operator ID message is optional).
static Preferences s_prefs;
static char s_operatorId[ODID_ID_SIZE + 1] = {0};
static bool s_haveOperatorId = false;

// ---- Location state, fed by CRSF_FRAMETYPE_GPS frames sniffed off-air ----
// (Declared here, above the callbacks, because RIDServerCallbacks reads
// s_haveTakeoff. Written by the sink in the loop task; s_haveTakeoff is also
// read from the BLE host task in onDisconnect - a plain aligned bool read,
// atomic on this target.)
static volatile bool s_haveFix = false;
static double  s_lat = 0, s_lon = 0;          // degrees
static float   s_altM = -1000;                // m (ODID "invalid" sentinel)
static float   s_speedMs = 0;
static float   s_headingDeg = 0;
static uint8_t s_sats = 0;
static uint32_t s_lastFixMs = 0;

// Take-off/operator position: latched from the first fix (ASTM System message).
static bool   s_haveTakeoff = false;
static double s_takeoffLat = 0, s_takeoffLon = 0;

class RIDConfigCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *c) override {
        std::string v = c->getValue();
        if (v.rfind("O:", 0) != 0) return;   // only "O:<operator id>" understood here
        std::string opId = v.substr(2);
        while (!opId.empty() && (opId.back() == '\n' || opId.back() == '\r')) opId.pop_back();
        if (opId.length() > ODID_ID_SIZE) opId.resize(ODID_ID_SIZE);   // bound untrusted input
        // Write the string fully BEFORE setting the "have it" flag: fillUasData()
        // reads these from the loop task while this runs in the BLE host task, so
        // the flag must never go true while the buffer is still half-written.
        strncpy(s_operatorId, opId.c_str(), ODID_ID_SIZE);
        s_operatorId[ODID_ID_SIZE] = 0;
        s_haveOperatorId = !opId.empty();
        s_prefs.putString("opid", s_operatorId);
        DBGLN("[RID] operator ID set via BLE (%u chars), saved to NVS", (unsigned)opId.length());
    }
};

// Server callbacks: the ONLY reason these exist is to re-advertise the
// connectable config instance after a disconnect. With CONFIG_BT_NIMBLE_EXT_ADV
// enabled, NimBLEServer's built-in advertise-on-disconnect is compiled out
// (it's `#if !CONFIG_BT_NIMBLE_EXT_ADV` in NimBLEServer.cpp), so without this
// the operator would get exactly ONE connection per boot to set the ID.
class RIDServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *) override { DBGLN("[RID] config client connected"); }
    void onDisconnect(NimBLEServer *) override {
        if (s_haveTakeoff) {   // airborne: config is closed for the flight
            DBGLN("[RID] config client disconnected (airborne - staying broadcast-only)");
            return;
        }
        bool ok = s_adv->start(2);   // adv data persists in the controller; just re-enable
        s_cfgInstanceUp = ok;
        DBGLN("[RID] config client disconnected, re-advertising config: %s", ok ? "ok" : "FAIL");
    }
};

static inline int32_t be32s(const uint8_t *p) {
    return (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]);
}
static inline uint16_t be16u(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }

// ---- Sink: pull GPS out of every CRSF frame the sniffer reassembles ----
static void RemoteID_sink(const uint8_t *frame, uint8_t len)
{
    if (len < 4 || frame[2] != CRSF_FRAMETYPE_GPS) return;
    const uint8_t *p = frame + 3;

    s_lat        = be32s(p)      * 1e-7;               // deg
    s_lon        = be32s(p + 4)  * 1e-7;                // deg
    s_speedMs    = be16u(p + 8)  * 0.1f * (1000.0f / 3600.0f); // 0.1km/h -> m/s
    s_headingDeg = be16u(p + 10) * 0.01f;               // deg
    s_altM       = (float)((int32_t)be16u(p + 12) - 1000); // m
    s_sats       = p[14];
    s_lastFixMs  = millis();
    s_haveFix    = (s_lat != 0.0 || s_lon != 0.0);

    if (s_haveFix && !s_haveTakeoff) {
        s_takeoffLat = s_lat;
        s_takeoffLon = s_lon;
        s_haveTakeoff = true;
        DBGLN("[RID] latched takeoff/operator fix %d,%d (x1e-7)", (int)(s_lat * 1e7), (int)(s_lon * 1e7));
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

    uas->Location.Status          = s_haveFix ? ODID_STATUS_AIRBORNE : ODID_STATUS_UNDECLARED;
    uas->Location.Direction       = s_headingDeg;
    uas->Location.SpeedHorizontal = s_speedMs;
    uas->Location.SpeedVertical   = 0;                 // not available from this GPS frame
    uas->Location.Latitude        = s_lat;
    uas->Location.Longitude       = s_lon;
    uas->Location.AltitudeGeo     = s_altM;
    uas->Location.AltitudeBaro    = -1000;              // unknown on this data path
    uas->Location.HeightType      = ODID_HEIGHT_REF_OVER_TAKEOFF;
    uas->Location.Height          = -1000; // no independent AGL/height-over-takeoff source on this data path
    uas->Location.HorizAccuracy   = createEnumHorizontalAccuracy(s_sats >= 6 ? 10.0f : 92.6f);
    uas->Location.VertAccuracy    = ODID_VER_ACC_UNKNOWN;
    uas->Location.BaroAccuracy    = ODID_VER_ACC_UNKNOWN;
    uas->Location.SpeedAccuracy   = ODID_SPEED_ACC_UNKNOWN;
    uas->Location.TSAccuracy      = ODID_TIME_ACC_UNKNOWN;
    uas->Location.TimeStamp       = INV_TIMESTAMP;      // no UTC time source on this data path - see file header
    uas->LocationValid            = s_haveFix ? 1 : 0;

    uas->System.OperatorLocationType = ODID_OPERATOR_LOCATION_TYPE_TAKEOFF;
    uas->System.ClassificationType   = ODID_CLASSIFICATION_TYPE_UNDECLARED;
    uas->System.OperatorLatitude     = s_haveTakeoff ? s_takeoffLat : 0;
    uas->System.OperatorLongitude    = s_haveTakeoff ? s_takeoffLon : 0;
    uas->System.AreaCount            = 1;
    uas->System.AreaRadius           = 0;
    uas->System.AreaCeiling          = -1000;
    uas->System.AreaFloor            = -1000;
    uas->System.OperatorAltitudeGeo  = -1000;
    uas->System.Timestamp            = 0;               // no UTC time source - see file header
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

void RemoteID_Tick(uint32_t nowMs)
{
    if (!s_adv) return;
    static uint32_t last = 0;
    if (nowMs - last < REMOTEID_BROADCAST_PERIOD_MS) return;
    last = nowMs;

    // GPS telemetry is only as fresh as the last sniffed CRSF GPS frame;
    // if the sniffer lost lock on the RC link, stop claiming a fix.
    if (s_haveFix && (nowMs - s_lastFixMs) > 5000) s_haveFix = false;

    // First fix seen == airborne (proxy, no arm signal on this data path -
    // see sniffer.h). Retire the connectable config instance right then:
    // once flying, this should be a pure broadcaster, not a connectable one.
    if (s_haveTakeoff && s_cfgInstanceUp) {
        s_adv->stop(2);
        s_cfgInstanceUp = false;
        DBGLN("[RID] airborne - config instance stopped, broadcast-only from here");
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
        DBGLN("[RID] no operator ID in NVS yet - write \"O:<id>\\n\" to the config characteristic before flying");
    }

    s_adv = NimBLEDevice::getAdvertising();

    // Instance 2: one-time connectable config GATT server (Operator ID entry).
    // Instances 0/1 (the actual ODID broadcast) are (re)configured with real
    // data on the first RemoteID_Tick().
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
    s_cfgInstanceUp = s_adv->start(2);
    DBGLN("[RID] config instance (connectable, 'ELRS-RID-CFG') %s", s_cfgInstanceUp ? "up" : "FAILED TO START");

    Ghost_Transport_Register(&RemoteID_sink);
}

#endif // GHOST_TRANSPORT_REMOTEID
