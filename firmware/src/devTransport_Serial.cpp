// =====================================================================
// Ghost RX transport: SERIAL debug output
// ---------------------------------------------------------------------
// The first-stage transport for bench bring-up. Registers a sink that
// prints every reassembled CRSF telemetry frame over the ELRS debug
// serial, both as a decoded human-readable line and (optionally) as raw
// hex. Uses the ELRS logging macros (DBG/DBGLN) so it shares the debug
// UART/USB-CDC the rest of the firmware already uses -- no serial-port
// contention. Requires -D DEBUG_LOG (set in the serial env).
//
// Enable with -D GHOST_TRANSPORT_SERIAL (and optionally
// -D GHOST_SERIAL_HEX to also dump raw frame bytes).
// =====================================================================
#if defined(GHOST_TRANSPORT_SERIAL)

#include <Arduino.h>
#include "logging.h"
#include "crsf_protocol.h"
#include "sniffer.h"

// --- big-endian helpers over the CRSF payload ---
static inline int16_t  be16s(const uint8_t *p) { return (int16_t)((p[0] << 8) | p[1]); }
static inline uint16_t be16u(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static inline int32_t  be32s(const uint8_t *p) {
    return (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]);
}
static inline uint32_t be24u(const uint8_t *p) {
    return ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];
}

static void Serial_sink(const uint8_t *frame, uint8_t len)
{
    // frame = [dest][len][type][payload...][crc]
    if (len < 4) return;
    const uint8_t type = frame[2];
    const uint8_t *p   = frame + 3;   // payload

#if defined(GHOST_SERIAL_HEX)
    DBG("[CRSF] ");
    // NB: ELRS debugPrintf supports only %s %d %u %x %f (NO width like %02X),
    // so print each byte as two single-nibble %x (each nibble is 0..15).
    for (uint8_t i = 0; i < len; i++) DBG("%x%x ", (frame[i] >> 4) & 0xF, frame[i] & 0xF);
    DBGCR;
#endif

    switch (type)
    {
    case CRSF_FRAMETYPE_GPS: {
        int32_t lat = be32s(p);
        int32_t lon = be32s(p + 4);
        uint16_t spd = be16u(p + 8);       // km/h * 10
        uint16_t hdg = be16u(p + 10);      // deg * 100
        int32_t  alt = (int32_t)be16u(p + 12) - 1000; // m
        uint8_t  sats = p[14];
        // Print raw scaled integers (no width specifiers). lat/lon are deg*1e7,
        // gs is 0.1km/h, hdg is 0.01deg. A host parser converts; this is debug.
        DBGLN("GPS  lat=%d lon=%d (x1e-7deg) altGPS=%dm gs=%u(0.1kmh) hdg=%u(0.01deg) sats=%u",
              lat, lon, alt, spd, hdg, sats);
        break; }
    case CRSF_FRAMETYPE_BATTERY_SENSOR: {
        uint16_t v = be16u(p);             // 0.1V
        uint16_t c = be16u(p + 2);         // 0.1A
        uint32_t mah = be24u(p + 4);
        uint8_t  pct = p[7];
        DBGLN("BATT %u.%uV %u.%uA %umAh %u%%",
              v / 10, v % 10, c / 10, c % 10, mah, pct);
        break; }
    case CRSF_FRAMETYPE_ATTITUDE: {
        // radians * 10000 -> centi-degrees (×5729.58/10000 ≈ ×0.5730)
        int16_t pitch = be16s(p), roll = be16s(p + 2), yaw = be16s(p + 4);
        DBGLN("ATT  pitch=%d roll=%d yaw=%d (raw rad*1e4)", pitch, roll, yaw);
        break; }
    case CRSF_FRAMETYPE_VARIO: {
        int16_t vs = be16s(p);             // cm/s
        DBGLN("VARIO %d cm/s", vs);
        break; }
    case CRSF_FRAMETYPE_BARO_ALTITUDE: {
        uint16_t a = be16u(p);
        DBGLN("BARO alt raw=%u", a);
        break; }
    case CRSF_FRAMETYPE_FLIGHT_MODE: {
        char mode[20]; uint8_t i = 0;
        while (i < sizeof(mode) - 1 && p[i] && (3 + i) < len - 1) { mode[i] = (char)p[i]; i++; }
        mode[i] = 0;
        DBGLN("MODE %s", mode);
        break; }
    case CRSF_FRAMETYPE_LINK_STATISTICS: {
        DBGLN("LINK rssi1=-%u rssi2=-%u lq=%u snr=%d rfmode=%u txpwr=%u",
              p[0], p[1], p[2], (int8_t)p[3], p[5], p[6]);
        break; }
    default:
        DBGLN("CRSF type=0x%x len=%u", type, len);
        break;
    }
}

void GhostSerial_Init()
{
    Ghost_Transport_Register(&Serial_sink);
    DBGLN("[TLM RX] serial telemetry sink ready");
}

#endif // GHOST_TRANSPORT_SERIAL
