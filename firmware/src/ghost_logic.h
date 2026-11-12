#pragma once
// =====================================================================
// Ghost sniffer - pure logic helpers (no Arduino / ELRS includes).
// Header-only static inlines that compile as both C and C++, so the host
// tests (test/test_frame.cpp) exercise the exact code the firmware runs.
//   * CRSF CRC8 + frame validator
//   * reassembly sequence tracker (mirrors StubbornReceiver)
//   * Remote ID legacy (instance 0) slot schedule
//   * unix-time-at-millis math
// =====================================================================
#include <stdint.h>

// ---- CRSF CRC8 / DVB-S2 (poly 0xD5) over type+payload ----
static inline uint8_t ghostCrc8(const uint8_t *p, uint8_t n)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
    return crc;
}

// ---- Reassembly sequence tracker ----
// The sniffer is passive: the sender moves on when the HANDSET acks, so we can
// miss a packet that StubbornReceiver then splices over (it accepts the final
// packageIndex 0 even after a gap). This mirrors the receiver's package index
// progression so a gap is visible; ProcessTLM feeds it (ISR-safe, no blocking)
// only while the receiver is not holding a finished frame, as the receiver
// ignores packets in that state.
typedef struct {
    uint8_t  seq;     // last in-order data index accepted (0 = none yet)
    uint8_t  ok;      // 1 = every packet since idx 1 arrived in order
    uint16_t bytes;   // payload bytes the receiver has collected (incl. final)
} GhostSeq_t;

static inline void ghostSeqReset(GhostSeq_t *s)
{
    s->seq = 0;
    s->ok = 0;
    s->bytes = 0;
}

static inline void ghostSeqFeed(GhostSeq_t *s, uint8_t idx, uint8_t maxIdx, uint8_t dataLen)
{
    if (idx == maxIdx) { ghostSeqReset(s); return; }   // resync: receiver restarts, we wait for idx 1
    if (idx == 0) {                                    // final packet: no sequence info, but it carries data
        if (s->seq >= 1) s->bytes = (uint16_t)(s->bytes + dataLen);
        return;
    }
    if (idx == 1) { s->seq = 1; s->ok = 1; s->bytes = dataLen; return; }   // (re)start of a frame
    if (idx == s->seq) return;                                              // duplicate (re-send), receiver ignores it
    if (idx == (uint8_t)(s->seq + 1)) { s->seq = idx; s->bytes = (uint16_t)(s->bytes + dataLen); return; }
    s->ok = 0;                                                              // gap
}

// ---- Frame validator ----
// buf = [dest][len][type][payload...][crc], len counts type..crc. avail = payload
// bytes the receiver collected (a frame needing more than that was spliced).
// Returns the total frame length to emit, or 0 if the frame must be dropped.
static inline uint8_t ghostFrameLen(const uint8_t *buf, uint8_t bufSize, uint16_t avail)
{
    if (bufSize < 4) return 0;
    uint8_t len = buf[1];
    if (len < 2) return 0;                          // needs at least type + crc
    if ((unsigned)len + 2 > bufSize) return 0;      // would run off the buffer
    uint8_t total = (uint8_t)(len + 2);
    if (total > avail) return 0;                    // more bytes claimed than were received
    if (ghostCrc8(&buf[2], (uint8_t)(len - 1)) != buf[len + 1]) return 0;
    return total;
}

// ---- Telemetry LQ ----
// hits = tlm packets received in elapsedMs; expected = hz * elapsedMs / (denom * 1000).
// Returns 0..100, rounded. Caller guarantees hz > 0 and denom > 1.
static inline uint8_t ghostTlmLq(uint32_t hits, uint16_t hz, uint8_t denom, uint32_t elapsedMs)
{
    const uint64_t den = (uint64_t)hz * elapsedMs;
    if (den == 0) return 0;
    const uint64_t lq = ((uint64_t)hits * denom * 100000u + den / 2) / den;
    return (uint8_t)(lq > 100 ? 100 : lq);
}

// ---- Remote ID legacy (instance 0) slot schedule ----
// One 25-byte message per advert. 6-slot cycle at REMOTEID_LEGACY_PERIOD_MS:
// Location, Basic ID, Location, System, Location, Operator ID.
#define GHOST_LEG_SLOTS    6
#define GHOST_LEG_LOCATION 0
#define GHOST_LEG_BASICID  1
#define GHOST_LEG_SYSTEM   2
#define GHOST_LEG_OPERATOR 3

// Message kind for a slot; falls back to Location if that message isn't valid.
static inline uint8_t ghostLegacyKind(uint8_t slot, uint8_t basicOk, uint8_t systemOk, uint8_t operatorOk)
{
    switch (slot % GHOST_LEG_SLOTS) {
    case 1: return basicOk    ? GHOST_LEG_BASICID  : GHOST_LEG_LOCATION;
    case 3: return systemOk   ? GHOST_LEG_SYSTEM   : GHOST_LEG_LOCATION;
    case 5: return operatorOk ? GHOST_LEG_OPERATOR : GHOST_LEG_LOCATION;
    default: return GHOST_LEG_LOCATION;
    }
}

// ---- Unix time at an earlier millis() ----
// baseUnix/baseMs = last time sync. atMs may be BEFORE the sync (fix older than
// the sync), so the delta is signed and floored (not truncated toward zero).
static inline uint32_t ghostUnixAtMs(uint32_t baseUnix, uint32_t baseMs, uint32_t atMs)
{
    int32_t d = (int32_t)(atMs - baseMs);
    int32_t s = (d >= 0) ? d / 1000 : -(int32_t)((-(int64_t)d + 999) / 1000);
    int64_t t = (int64_t)baseUnix + s;
    return t < 0 ? 0 : (uint32_t)t;
}
