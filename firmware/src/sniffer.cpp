// =====================================================================
// ELRS Telemetry Sniffer (Ghost RX) - core implementation
// Targets ExpressLRS 3.5.x, SX1280 2.4GHz, ESP32-S3.
// =====================================================================
#include "sniffer.h"

#include "common.h"                 // UID[], OtaUpdateCrcInitFromUid, uidMacSeedGet, OtaIsFullRes
#include "FHSS.h"                   // FHSSrandomiseFHSSsequence
#include "OTA.h"
#include "telemetry_protocol.h"     // ELRS_TELEMETRY_TYPE_*, ELRS*_TELEMETRY_MAX_PACKAGES
#include "stubborn_receiver.h"
#include "crsf_protocol.h"

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>                 // strtoul (RemoteID "T:" serial time-sync command)
#include <MD5Builder.h>             // ESP32 Arduino core (stable across mbedtls versions)
#include "logging.h"                // DBGLN heartbeat
#include "CRSF.h"                   // CRSF::LinkStatistics

// External symbols from the ELRS core (already linked in an RX build)
extern uint8_t UID[UID_LEN];
extern bool OtaIsFullRes;
extern uint8_t ExpressLRS_currTlmDenom;   // 1 = tlm off; else 1:N telemetry ratio
void OtaUpdateCrcInitFromUid();
uint32_t uidMacSeedGet();

// Diagnostics: count captured TLM packets, and RAW demodulated packets
static volatile uint32_t s_tlmPkts = 0;
static volatile uint32_t s_rawPkts = 0;   // every SX1280 demod (any CRC result)
static volatile uint32_t s_rawOk   = 0;   // subset with HW status RX_OK

// Sniffer's own reception quality of the DRONE telemetry link. RSSI/SNR are
// captured per telemetry packet; LQ is received-vs-expected tlm slots per second.
// These are OUR measurements of the aircraft->handset uplink we overhear, NOT the
// handset RC downlink that dominates CRSF::LinkStatistics.
static volatile int      s_tlmRssiDbm = 0;   // negative dBm (IIR), 0 if none
static volatile int      s_tlmSnrRaw  = 0;   // RADIO_SNR_SCALE (=4) units
static volatile bool     s_tlmRssiValid = false;
static volatile uint8_t  s_tlmLq = 0;        // 0..100
static uint32_t          s_tlmPktsPrev = 0;

void Sniffer_OnRawPacket(bool hwOk)
{
    s_rawPkts++;
    if (hwOk) s_rawOk++;
}

// PACKET_TYPE_TLM == 0b11 (see OTA.h). Only telemetry packets carry the drone's
// signal; ignore RC/SYNC/MSP so RSSI reflects the aircraft, not the handset.
void Sniffer_OnPacketStats(uint8_t type, int rssiDbm, int snrRaw)
{
    if (type != PACKET_TYPE_TLM) return;
    if (rssiDbm > 0) rssiDbm = 0;
    // light IIR so a single weak packet doesn't jump the reading
    s_tlmRssiDbm = s_tlmRssiValid ? ((s_tlmRssiDbm * 3 + rssiDbm) / 4) : rssiDbm;
    s_tlmSnrRaw  = snrRaw;
    s_tlmRssiValid = true;
}

int Sniffer_TlmRssiDbm(void) { return s_tlmRssiValid ? s_tlmRssiDbm : 0; }
int Sniffer_TlmLq(void)      { return s_tlmLq; }
int Sniffer_TlmSnrDb(void)   { return s_tlmSnrRaw / 4; }   // RADIO_SNR_SCALE = 4

// ---------------------------------------------------------------------
// Reassembly state
// ---------------------------------------------------------------------
static StubbornReceiver TelemetrySniffer;
// CRSF frames are small; 72 bytes covers the largest ELRS telemetry frame.
static uint8_t CRSFinBuffer[72];
static volatile bool s_uidValid = false;

// ---------------------------------------------------------------------
// UID / passphrase
// ---------------------------------------------------------------------
static void applyUid()
{
    // Mirror exactly what rx_main does after a config/bind change:
    OtaUpdateCrcInitFromUid();                       // CRC seed from UID[4..5]
    FHSSrandomiseFHSSsequence(uidMacSeedGet());      // hop table from UID[2..5]
    s_uidValid = true;
    // Drop any existing lock so the scanner re-acquires with the NEW hop table /
    // CRC seed. Harmless at boot (already disconnected); on a live phrase change
    // this is what retargets the sniffer to the new link.
    connectionState = disconnected;
}

void Sniffer_SetUid(const uint8_t uid[6])
{
    memcpy(UID, uid, 6);
    applyUid();
}

void Sniffer_SetUidFromPhrase(const char *phrase)
{
    // Build the exact string the ELRS toolchain hashes.
    // md5('-DMY_BINDING_PHRASE="' + phrase + '"')  ->  first 6 bytes = UID
    char buf[160];
    int n = snprintf(buf, sizeof(buf), "-DMY_BINDING_PHRASE=\"%s\"", phrase ? phrase : "");
    if (n <= 0) return;

    uint8_t digest[16];
    MD5Builder md5;
    md5.begin();
    md5.add((uint8_t *)buf, (uint16_t)n);
    md5.calculate();
    md5.getBytes(digest);

    memcpy(UID, digest, 6);
    applyUid();
}

// ---------------------------------------------------------------------
// Rate change: tell the reassembler the max package index in use
// ---------------------------------------------------------------------
void Sniffer_OnRateChanged(void)
{
    TelemetrySniffer.setMaxPackageIndex(
        OtaIsFullRes ? ELRS8_TELEMETRY_MAX_PACKAGES : ELRS4_TELEMETRY_MAX_PACKAGES);
    TelemetrySniffer.SetDataToReceive(CRSFinBuffer, sizeof(CRSFinBuffer));
    TelemetrySniffer.ResetState();
}

// ---------------------------------------------------------------------
// Hot path: unpack a validated TLM packet into the reassembler.
// Lifted from tx_main.cpp ProcessTLMpacket() (the handset's own logic).
// ---------------------------------------------------------------------
void Sniffer_ProcessTLM(const OTA_Packet_s *otaPkt)
{
    // NB: UID is set by ELRS at boot (MY_UID / options.json), not necessarily via
    // Sniffer_SetUidFromPhrase, so we do NOT gate on s_uidValid here.
    s_tlmPkts++;

    if (OtaIsFullRes)
    {
        const OTA_Packet8_s *ota8 = &otaPkt->full;
        const uint8_t *telemPtr;
        uint8_t dataLen;
        if (ota8->tlm_dl.containsLinkStats)
        {
            // ul_link_stats.stats are the aircraft's link stats; we ignore
            // them here (we report our OWN RX RSSI/LQ instead). The trailing
            // payload bytes still carry reassembly data.
            telemPtr = ota8->tlm_dl.ul_link_stats.payload;
            dataLen  = sizeof(ota8->tlm_dl.ul_link_stats.payload);
        }
        else
        {
            telemPtr = ota8->tlm_dl.payload;
            dataLen  = sizeof(ota8->tlm_dl.payload);
        }
        TelemetrySniffer.ReceiveData(
            ota8->tlm_dl.packageIndex & ELRS8_TELEMETRY_MAX_PACKAGES, telemPtr, dataLen);
    }
    else
    {
        switch (otaPkt->std.tlm_dl.type)
        {
        case ELRS_TELEMETRY_TYPE_LINK:
            // link-stats-only packet, no reassembly payload
            break;
        case ELRS_TELEMETRY_TYPE_DATA:
            TelemetrySniffer.ReceiveData(
                otaPkt->std.tlm_dl.packageIndex & ELRS4_TELEMETRY_MAX_PACKAGES,
                otaPkt->std.tlm_dl.payload,
                sizeof(otaPkt->std.tlm_dl.payload));
            break;
        }
    }
}

// ---------------------------------------------------------------------
// Main loop: emit completed CRSF frames.
// ---------------------------------------------------------------------
void Sniffer_Poll(void)
{
    if (TelemetrySniffer.HasFinishedData())
    {
        // CRSFinBuffer[0] = dest addr, [1] = length of (type..crc)
        const uint8_t frameLen = CRSFinBuffer[1] + 2; // + dest + len bytes
        Ghost_Transport_SendCRSF(CRSFinBuffer, frameLen);
        TelemetrySniffer.Unlock();
    }
}

// ---------------------------------------------------------------------
// Transport sink registry (BLE / WiFi-UDP / UART register here)
// ---------------------------------------------------------------------
#define GHOST_MAX_SINKS 4
static Ghost_Sink_t s_sinks[GHOST_MAX_SINKS] = {0};

bool Ghost_Transport_Register(Ghost_Sink_t sink)
{
    for (int i = 0; i < GHOST_MAX_SINKS; i++) {
        if (s_sinks[i] == nullptr) { s_sinks[i] = sink; return true; }
    }
    return false;
}

void Ghost_Transport_SendCRSF(const uint8_t *frame, uint8_t len)
{
    for (int i = 0; i < GHOST_MAX_SINKS; i++) {
        if (s_sinks[i]) s_sinks[i](frame, len);
    }
}

// CRC8 / DVB-S2 (poly 0xD5) over type+payload, matching the CRSF wire CRC.
static uint8_t ghostCrc8(const uint8_t *p, uint8_t n)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
    return crc;
}

// The sniffed link does NOT send a CRSF LINK_STATISTICS frame, so RSSI/LQ never
// reach a host over serial/BLE. Synthesize one from OUR receiver's measured
// CRSF::LinkStatistics (same values as the OLED/heartbeat) and fan it out to the
// transports. crsfLinkStatistics_t is packed in exact CRSF wire order (10 bytes),
// so it copies straight into the payload.
static void Sniffer_EmitLinkStats()
{
    uint8_t f[14];
    f[0] = CRSF_ADDRESS_RADIO_TRANSMITTER;        // dest (host parser ignores)
    f[1] = 12;                                    // len = type(1)+payload(10)+crc(1)
    f[2] = CRSF_FRAMETYPE_LINK_STATISTICS;        // 0x14
    memset(&f[3], 0, 10);
    // OUR reception of the DRONE telemetry link (not CRSF::LinkStatistics, which
    // tracks the handset RC link). Payload order: [0]RSSI1 [1]RSSI2 [2]LQ [3]SNR...
    f[3] = (uint8_t)(-Sniffer_TlmRssiDbm());      // uplink_RSSI_1 (positive magnitude)
    f[5] = (uint8_t)Sniffer_TlmLq();              // uplink_Link_quality
    f[6] = (int8_t)Sniffer_TlmSnrDb();            // uplink_SNR (dB)
    f[13] = ghostCrc8(&f[2], 11);                 // crc over type+payload
    Ghost_Transport_SendCRSF(f, 14);
}

// Set once Ghost_SetupTransports() has actually initialized (transports/OLED).
// Until then, Ghost_Loop() must not touch any of that state.
static volatile bool s_ghostReady = false;

// Accept "P:<phrase>\n" (set binding phrase) and, on the Remote ID build,
// "O:<operator id>\n" / "O?" (Operator ID) and "C:<0..6>\n" / "C?" (EU class)
// on the debug serial (USBSerial) so the host can configure over the cable,
// exactly like the BLE characteristics. Non-blocking line accumulator;
// ignores anything that isn't a known command.
static void Sniffer_PollSerialCmd()
{
    if (!SerialLogger) return;
    static char line[80];
    static uint8_t idx = 0;
    while (SerialLogger->available())
    {
        char ch = (char)SerialLogger->read();
        if (ch == '\n' || ch == '\r')
        {
            if (idx >= 2 && line[0] == 'P' && line[1] == ':')
            {
                line[idx] = 0;
                DBGLN("[TLM RX] set phrase via serial");
                Sniffer_SetUidFromPhrase(line + 2);
            }
#if defined(GHOST_TRANSPORT_REMOTEID)
            else if (idx >= 2 && line[0] == 'O' && line[1] == ':')
            {
                line[idx] = 0;
                RemoteID_SetOperatorId(line + 2);   // runs in loop task -> NVS write is safe here
            }
            else if (idx >= 2 && line[0] == 'O' && line[1] == '?')
            {
                RemoteID_ReportOperatorId();        // host/GCS reads back "[RID] OPID=<id>"
            }
            else if (idx >= 3 && line[0] == 'C' && line[1] == ':' && (line[2] == '0' || line[2] == '1'))
            {
                // "C:0"=Legacy (no class marking), "C:1"=C0 only - see REMOTEID_CLASS_*
                // in devTransport_RemoteID.cpp for why the rest of C1..C6 isn't offered.
                RemoteID_SetClass((uint8_t)(line[2] - '0'));   // runs in loop task -> NVS write is safe here
            }
            else if (idx >= 2 && line[0] == 'C' && line[1] == '?')
            {
                RemoteID_ReportClass();             // host/GCS reads back "[RID] CLASS=Cn"
            }
            else if (idx >= 3 && line[0] == 'T' && line[1] == ':')
            {
                // "T:<unix seconds>" - GCS pushes its system clock over serial so
                // Location/System timestamps aren't always "unknown". See
                // RemoteID_SetTime() for why this can only come from serial (no
                // other UTC source exists on this data path).
                line[idx] = 0;
                uint32_t unixSecs = (uint32_t)strtoul(line + 2, nullptr, 10);
                if (unixSecs > 0) RemoteID_SetTime(unixSecs);
            }
#endif
            idx = 0;
        }
        else if (idx < sizeof(line) - 1) { line[idx++] = ch; }
        else { idx = 0; }   // overflow -> resync
    }
}

void Ghost_Loop(uint32_t nowMs)
{
    if (!s_ghostReady) return;
    Sniffer_Poll();
    Sniffer_PollSerialCmd();
#if defined(GHOST_DISPLAY)
    GhostDisplay_Tick(nowMs);
#endif
#if defined(GHOST_TRANSPORT_REMOTEID)
    RemoteID_Tick(nowMs);
#endif

    // 5 Hz: inject our own link stats (RSSI/LQ) as a CRSF frame into the stream.
    static uint32_t lastLink = 0;
    if (nowMs - lastLink >= 200) { lastLink = nowMs; Sniffer_EmitLinkStats(); }

    // 1 Hz heartbeat so we can see link state even without telemetry / OLED.
    static uint32_t lastHb = 0;
    if (nowMs - lastHb >= 1000)
    {
        lastHb = nowMs;
        uint16_t hz = ExpressLRS_currAirRate_Modparams
                    ? (uint16_t)(1000000UL / ExpressLRS_currAirRate_Modparams->interval) : 0;
        // telemetry LQ = drone tlm packets received this second / expected slots.
        uint32_t hits = s_tlmPkts - s_tlmPktsPrev;
        s_tlmPktsPrev = s_tlmPkts;
        uint8_t denom = ExpressLRS_currTlmDenom;
        uint32_t expected = (denom > 1 && hz > 0) ? (hz / denom) : 0;
        if (connectionState == connected && expected > 0) {
            uint32_t lq = (100u * hits) / expected;
            s_tlmLq = (uint8_t)(lq > 100 ? 100 : lq);
        } else {
            s_tlmLq = 0;
        }
        if (hits == 0) s_tlmRssiValid = false;   // no telemetry -> RSSI goes stale

        const char *st = connectionState == connected    ? "LOCK" :
                         connectionState == tentative     ? "TENT" :
                         connectionState == disconnected  ? "SRCH" : "----";
        DBGLN("[TLM RX] %s rate=%uHz raw=%u tlmPkts=%u | DRONE rssi=%d lq=%u snr=%d | tx rssi=-%u lq=%u",
              st, hz, (unsigned)s_rawPkts, (unsigned)s_tlmPkts,
              Sniffer_TlmRssiDbm(), (unsigned)s_tlmLq, Sniffer_TlmSnrDb(),
              CRSF::LinkStatistics.uplink_RSSI_1,
              CRSF::LinkStatistics.uplink_Link_quality);
    }
}

void Ghost_SetupTransports(void)
{
    // In the unconfigured / WiFi-update boot path, ELRS skips setupSerial()
    // (SerialLogger is null) and the radio/pins are not set up. Do NOT init the
    // sniffer there - it would dereference null and panic. Wait for a real,
    // hardware-configured boot (connectionState is a normal state, < MODE_STATES).
    if (connectionState > MODE_STATES) return;

#if defined(MY_BINDING_PHRASE)
    // ELRS options already set UID[] from the compiled phrase at boot; this
    // re-applies CRC seed + FHSS table from the same phrase to be explicit and
    // to allow a fully runtime build to reuse the same path.
    Sniffer_SetUidFromPhrase(MY_BINDING_PHRASE);
#endif
#if defined(GHOST_TRANSPORT_SERIAL)
    GhostSerial_Init();
#endif
#if defined(GHOST_TRANSPORT_BLE) && defined(GHOST_TRANSPORT_REMOTEID)
#error GHOST_TRANSPORT_BLE and GHOST_TRANSPORT_REMOTEID both claim the one BLE radio identity - pick one env.
#endif
#if defined(GHOST_TRANSPORT_BLE)
    BLE_Transport_Init("ELRS TLM RX");
#endif
#if defined(GHOST_TRANSPORT_REMOTEID)
    RemoteID_Transport_Init();
#endif
#if defined(GHOST_TRANSPORT_WIFI_UDP)
    WiFiUDP_Transport_Init();
#endif
#if defined(GHOST_DISPLAY)
    GhostDisplay_Init();
#endif
    s_ghostReady = true;
}
