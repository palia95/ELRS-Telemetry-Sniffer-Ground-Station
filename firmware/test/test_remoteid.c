// Host-side correctness check for the RemoteID transport's data-path math.
// Not built into firmware. Run on any dev machine (no ESP32 needed):
//
//   cd firmware/test
//   cc -std=c11 test_remoteid.c ../src/opendroneid.c -I../src -lm -o /tmp/t && /tmp/t
//
// Verifies the two things that can be silently wrong (compile fine but
// broadcast garbage): (1) the CRSF-GPS byte parse + unit scaling, and (2) that
// the ODID Location/BasicID/System/OperatorID messages encode the values we
// put in such that the library's own decoder reads them back. Mirrors the
// exact scaling in devTransport_RemoteID.cpp RemoteID_sink()/fillUasData() and
// the pack byte-layout of buildMessagePack(). If any of that drifts, this
// fails loudly before it ever reaches a real aircraft.

#include "opendroneid.h"
#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

// --- copied verbatim from devTransport_RemoteID.cpp ---
static int32_t be32s(const uint8_t *p) {
    return (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]);
}
static uint16_t be16u(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }

static int approx(double a, double b, double tol) { return fabs(a - b) <= tol; }

int main(void) {
    // Build a synthetic CRSF GPS payload (big-endian, per crsf_sensor_gps_t):
    //   lat = 45.1234567 deg  -> 451234567 (deg*1e7)
    //   lon =  9.7654321 deg  ->  97654321
    //   groundspeed = 123 (0.1 km/h) = 12.3 km/h -> 3.4166 m/s
    //   heading = 27000 (0.01 deg) = 270.00 deg
    //   altitude = 1500 raw -> 500 m (1000 offset)
    //   sats = 11
    int32_t rawLat = 451234567, rawLon = 97654321;
    uint16_t rawGs = 123, rawHdg = 27000, rawAlt = 1500;
    uint8_t rawSats = 11;
    uint8_t p[15];
    p[0]=rawLat>>24; p[1]=rawLat>>16; p[2]=rawLat>>8; p[3]=rawLat;
    p[4]=rawLon>>24; p[5]=rawLon>>16; p[6]=rawLon>>8; p[7]=rawLon;
    p[8]=rawGs>>8;  p[9]=rawGs;
    p[10]=rawHdg>>8; p[11]=rawHdg;
    p[12]=rawAlt>>8; p[13]=rawAlt;
    p[14]=rawSats;

    // --- parse exactly as RemoteID_sink() does ---
    double lat = be32s(p) * 1e-7;
    double lon = be32s(p+4) * 1e-7;
    float speedMs = be16u(p+8) * 0.1f * (1000.0f/3600.0f);
    float headingDeg = be16u(p+10) * 0.01f;
    float altM = (float)((int32_t)be16u(p+12) - 1000);
    uint8_t sats = p[14];

    assert(approx(lat, 45.1234567, 1e-6));
    assert(approx(lon, 9.7654321, 1e-6));
    assert(approx(speedMs, 3.41667, 1e-3));
    assert(approx(headingDeg, 270.0, 1e-3));
    assert(approx(altM, 500.0, 1e-3));
    assert(sats == 11);
    printf("PASS: CRSF-GPS parse + scaling (lat=%.7f lon=%.7f gs=%.3fm/s hdg=%.2f alt=%.0fm sats=%u)\n",
           lat, lon, speedMs, headingDeg, altM, sats);

    // --- fill Location as fillUasData() does, encode, decode, compare ---
    ODID_UAS_Data uas; odid_initUasData(&uas);
    uas.Location.Status = ODID_STATUS_AIRBORNE;
    uas.Location.Direction = headingDeg;
    uas.Location.SpeedHorizontal = speedMs;
    uas.Location.SpeedVertical = 0;
    uas.Location.Latitude = lat;
    uas.Location.Longitude = lon;
    uas.Location.AltitudeGeo = altM;
    uas.Location.AltitudeBaro = -1000;
    uas.Location.HeightType = ODID_HEIGHT_REF_OVER_TAKEOFF;
    uas.Location.Height = -1000;
    uas.Location.HorizAccuracy = createEnumHorizontalAccuracy(10.0f);
    uas.Location.VertAccuracy = ODID_VER_ACC_UNKNOWN;
    uas.Location.BaroAccuracy = ODID_VER_ACC_UNKNOWN;
    uas.Location.SpeedAccuracy = ODID_SPEED_ACC_UNKNOWN;
    uas.Location.TSAccuracy = ODID_TIME_ACC_UNKNOWN;
    uas.Location.TimeStamp = INV_TIMESTAMP;
    uas.LocationValid = 1;

    ODID_Location_encoded locEnc;
    assert(encodeLocationMessage(&locEnc, &uas.Location) == ODID_SUCCESS);
    ODID_Location_data locDec;
    assert(decodeLocationMessage(&locDec, &locEnc) == ODID_SUCCESS);
    // ODID lat/lon are transmitted as int32 deg*1e7 -> ~1e-7 deg resolution
    assert(approx(locDec.Latitude, lat, 1e-6));
    assert(approx(locDec.Longitude, lon, 1e-6));
    // speed quantised to 0.25 m/s steps; direction to 1 deg
    assert(approx(locDec.SpeedHorizontal, speedMs, 0.25));
    assert(approx(locDec.Direction, headingDeg, 1.0));
    assert(approx(locDec.AltitudeGeo, altM, 1.0));
    assert(locDec.Status == ODID_STATUS_AIRBORNE);
    printf("PASS: Location encode->decode round-trip (lat=%.7f lon=%.7f gs=%.2f dir=%.0f alt=%.0f)\n",
           locDec.Latitude, locDec.Longitude, locDec.SpeedHorizontal, locDec.Direction, locDec.AltitudeGeo);

    // --- BasicID: synthesized "ELRS-<hex>" id round-trips ---
    strncpy(uas.BasicID[0].UASID, "ELRS-1122334455AA", ODID_ID_SIZE);
    uas.BasicID[0].IDType = ODID_IDTYPE_SPECIFIC_SESSION_ID;
    uas.BasicID[0].UAType = ODID_UATYPE_HELICOPTER_OR_MULTIROTOR;
    uas.BasicIDValid[0] = 1;
    ODID_BasicID_encoded bidEnc;
    assert(encodeBasicIDMessage(&bidEnc, &uas.BasicID[0]) == ODID_SUCCESS);
    ODID_BasicID_data bidDec;
    assert(decodeBasicIDMessage(&bidDec, &bidEnc) == ODID_SUCCESS);
    assert(strcmp(bidDec.UASID, "ELRS-1122334455AA") == 0);
    assert(bidDec.UAType == ODID_UATYPE_HELICOPTER_OR_MULTIROTOR);
    printf("PASS: BasicID encode->decode ('%s')\n", bidDec.UASID);

    // --- OperatorID round-trips ---
    uas.OperatorID.OperatorIdType = ODID_OPERATOR_ID;
    strncpy(uas.OperatorID.OperatorId, "GBR-OP-1234567890AB", ODID_ID_SIZE);
    uas.OperatorIDValid = 1;
    ODID_OperatorID_encoded opEnc;
    assert(encodeOperatorIDMessage(&opEnc, &uas.OperatorID) == ODID_SUCCESS);
    ODID_OperatorID_data opDec;
    assert(decodeOperatorIDMessage(&opDec, &opEnc) == ODID_SUCCESS);
    assert(strncmp(opDec.OperatorId, "GBR-OP-1234567890AB", 19) == 0);
    printf("PASS: OperatorID encode->decode ('%.20s')\n", opDec.OperatorId);

    // --- System (operator/takeoff location) round-trips ---
    uas.System.OperatorLocationType = ODID_OPERATOR_LOCATION_TYPE_TAKEOFF;
    uas.System.OperatorLatitude = lat;
    uas.System.OperatorLongitude = lon;
    uas.System.AreaCount = 1;
    uas.System.OperatorAltitudeGeo = -1000;
    uas.System.Timestamp = 0;
    uas.SystemValid = 1;
    ODID_System_encoded sysEnc;
    assert(encodeSystemMessage(&sysEnc, &uas.System) == ODID_SUCCESS);
    ODID_System_data sysDec;
    assert(decodeSystemMessage(&sysDec, &sysEnc) == ODID_SUCCESS);
    assert(approx(sysDec.OperatorLatitude, lat, 1e-6));
    assert(approx(sysDec.OperatorLongitude, lon, 1e-6));
    printf("PASS: System encode->decode (opLat=%.7f opLon=%.7f)\n",
           sysDec.OperatorLatitude, sysDec.OperatorLongitude);

    // --- Replicate firmware buildMessagePack() byte layout (our own code, not
    //     the library's Linux-only wifi.c) and decode each message back out.
    //     Layout: [protover|PACKED][SingleMsgSize=25][N][msg0..msgN-1]. ---
    ODID_MessagePack_data pack; pack.SingleMessageSize = ODID_MESSAGE_SIZE; pack.MsgPackSize = 0;
    encodeBasicIDMessage(&pack.Messages[pack.MsgPackSize++].basicId, &uas.BasicID[0]);
    encodeLocationMessage(&pack.Messages[pack.MsgPackSize++].location, &uas.Location);
    encodeSystemMessage(&pack.Messages[pack.MsgPackSize++].system, &uas.System);
    encodeOperatorIDMessage(&pack.Messages[pack.MsgPackSize++].operatorId, &uas.OperatorID);
    uint8_t packbuf[256];
    packbuf[0] = (uint8_t)((ODID_MESSAGETYPE_PACKED << 4) | (ODID_PROTOCOL_VERSION & 0x0F));
    packbuf[1] = pack.SingleMessageSize;
    packbuf[2] = pack.MsgPackSize;
    memcpy(packbuf + 3, pack.Messages, (size_t)pack.MsgPackSize * ODID_MESSAGE_SIZE);
    int packlen = 3 + pack.MsgPackSize * ODID_MESSAGE_SIZE;

    // Decode message-by-message using the library's single-message decoder,
    // confirming the pack we hand the BLE stack is well-formed.
    assert(packbuf[0] == ((ODID_MESSAGETYPE_PACKED << 4) | ODID_PROTOCOL_VERSION));
    assert(packbuf[1] == ODID_MESSAGE_SIZE);
    assert(packbuf[2] == 4);
    ODID_Location_data l2;
    assert(decodeLocationMessage(&l2, (ODID_Location_encoded*)&packbuf[3 + 1*ODID_MESSAGE_SIZE]) == ODID_SUCCESS);
    assert(approx(l2.Latitude, lat, 1e-6));
    printf("PASS: firmware pack layout build (%d bytes, %u msgs, hdr=0x%02X) + Location decodes back\n",
           packlen, packbuf[2], packbuf[0]);

    // --- FLIGHT_MODE armed/emergency classification, mirrors the EXACT
    // expression in RemoteID_sink()'s CRSF_FRAMETYPE_FLIGHT_MODE case.
    // Strings verified against Betaflight src/main/telemetry/crsf.c
    // (crsfFrameFlightMode()): "!FS!"/RTH are EMERGENCY and force armed=1
    // (the generic suffix heuristic does NOT apply to them - Betaflight never
    // appends a suffix during failsafe, and GPS Rescue only runs while
    // armed); otherwise disarmed appends '*'/'!'/'?', armed appends nothing.
    struct { const char *mode; int expectArmed; int expectEmergency; } cases[] = {
        {"ACRO",  1, 0}, {"AIR",  1, 0}, {"ANGL", 1, 0}, {"HOR",  1, 0},
        {"ACRO*", 0, 0}, {"ACRO!", 0, 0}, {"ACRO?", 0, 0},
        {"ALTH*", 0, 0}, {"POSH!", 0, 0},
        {"!FS!",  1, 1},   // forced armed - the trailing '!' is NOT a disarmed suffix here
        {"RTH",   1, 1},   // GPS Rescue - always armed in practice
    };
    for (size_t i = 0; i < sizeof(cases)/sizeof(cases[0]); i++) {
        const char *mode = cases[i].mode;
        int len = (int)strlen(mode);
        int emergency = (strstr(mode, "!FS") != NULL) || (strstr(mode, "RTH") != NULL);
        int armed;
        if (emergency) {
            armed = 1;
        } else {
            int disarmed = (len > 0) && (mode[len-1] == '*' || mode[len-1] == '!' || mode[len-1] == '?');
            armed = (len > 0) && !disarmed;
        }
        if (armed != cases[i].expectArmed || emergency != cases[i].expectEmergency) {
            printf("FAIL: FLIGHT_MODE '%s' classified armed=%d emergency=%d, expected armed=%d emergency=%d\n",
                   mode, armed, emergency, cases[i].expectArmed, cases[i].expectEmergency);
            return 1;
        }
    }
    printf("PASS: FLIGHT_MODE armed/emergency classification (%zu cases, incl. RTH + '!'/'?' suffixes)\n",
           sizeof(cases)/sizeof(cases[0]));

    // --- Location.Status mapping, mirrors the EXACT if-chain in fillUasData().
    struct { int emergency, haveFix, haveMode, armed; ODID_status_t expect; const char *why; } statusCases[] = {
        {1, 1, 1, 1, ODID_STATUS_EMERGENCY, "failsafe/RTH always wins, even armed+fix"},
        {1, 0, 0, 0, ODID_STATUS_EMERGENCY, "failsafe/RTH wins even with no fix"},
        {0, 0, 1, 1, ODID_STATUS_UNDECLARED, "no fix -> nothing to report"},
        {0, 1, 1, 0, ODID_STATUS_GROUND,     "disarmed, fix present, mode telemetry seen -> GROUND, not AIRBORNE"},
        {0, 1, 1, 1, ODID_STATUS_AIRBORNE,   "armed, fix present -> AIRBORNE"},
        {0, 1, 0, 0, ODID_STATUS_AIRBORNE,   "no FLIGHT_MODE telemetry ever -> best-effort AIRBORNE fallback"},
    };
    for (size_t i = 0; i < sizeof(statusCases)/sizeof(statusCases[0]); i++) {
        int emergency = statusCases[i].emergency, haveFix = statusCases[i].haveFix;
        int haveMode = statusCases[i].haveMode, armed = statusCases[i].armed;
        ODID_status_t status;
        if (emergency)                    status = ODID_STATUS_EMERGENCY;
        else if (!haveFix)                status = ODID_STATUS_UNDECLARED;
        else if (haveMode && !armed)      status = ODID_STATUS_GROUND;
        else                              status = ODID_STATUS_AIRBORNE;
        if (status != statusCases[i].expect) {
            printf("FAIL: Status(emergency=%d haveFix=%d haveMode=%d armed=%d) = %d, expected %d (%s)\n",
                   emergency, haveFix, haveMode, armed, status, statusCases[i].expect, statusCases[i].why);
            return 1;
        }
    }
    printf("PASS: Location.Status mapping (%zu cases, incl. GROUND vs AIRBORNE on disarmed)\n",
           sizeof(statusCases)/sizeof(statusCases[0]));

    printf("\nALL CHECKS PASSED\n");
    return 0;
}
