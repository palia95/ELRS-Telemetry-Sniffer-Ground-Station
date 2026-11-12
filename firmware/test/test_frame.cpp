// Host-side check of the sniffer's frame-integrity and Remote ID schedule
// logic (src/ghost_logic.h), driven through the REAL StubbornReceiver so the
// splice cases are the ones seen on air. Not built into firmware.
//
//   cd firmware/test
//   c++ -std=c++14 test_frame.cpp ../../../ExpressLRS/src/lib/StubbornReceiver/stubborn_receiver.cpp \
//       -I../src -I../../../ExpressLRS/src/lib/StubbornReceiver -o /tmp/tf && /tmp/tf
//
// Covers: clean frame accepted; lost middle / lost final packet dropped (even
// with a forged-matching CRC); bad CRSF length bytes; duplicates; the Legacy
// Remote ID 6-slot schedule timing; unix-at-ms with a fix before/after the
// time sync. Mirrors Sniffer_ProcessTLM()/Sniffer_Poll() in sniffer.cpp.

#include "ghost_logic.h"
#include "stubborn_receiver.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#define PAYLOAD  5      // ELRS4_TELEMETRY_BYTES_PER_CALL
#define MAXIDX   63     // ELRS4_TELEMETRY_MAX_PACKAGES (255 >> 2)
#define BUFSZ    72

// --- mirror of the sniffer's reassembly path ---
static StubbornReceiver rx;
static uint8_t buf[BUFSZ];
static GhostSeq_t seq;
static int dropped, emitted;
static uint8_t lastFrame[BUFSZ];
static uint8_t lastLen;

static void resetAll(void) {
    memset(buf, 0, sizeof(buf));
    rx.setMaxPackageIndex(MAXIDX);
    rx.SetDataToReceive(buf, sizeof(buf));
    rx.ResetState();
    ghostSeqReset(&seq);
    dropped = emitted = 0;
}

static void feed(uint8_t idx, const uint8_t *p) {           // Sniffer_ProcessTLM
    if (!rx.HasFinishedData()) ghostSeqFeed(&seq, idx, MAXIDX, PAYLOAD);
    rx.ReceiveData(idx, p, PAYLOAD);
}

static void poll(bool forceCrc = false) {                    // Sniffer_Poll
    if (!rx.HasFinishedData()) return;
    uint8_t f[BUFSZ];
    memcpy(f, buf, sizeof(f));
    GhostSeq_t s = seq;
    ghostSeqReset(&seq);
    rx.Unlock();
    if (forceCrc && f[1] >= 2 && f[1] + 2 <= BUFSZ) f[f[1] + 1] = ghostCrc8(&f[2], f[1] - 1);
    uint8_t n = s.ok ? ghostFrameLen(f, sizeof(f), s.bytes) : 0;
    if (n) { memcpy(lastFrame, f, n); lastLen = n; emitted++; } else dropped++;
}

// Build a CRSF frame [dest][len][type][payload...][crc] with `pl` payload bytes.
static int makeFrame(uint8_t *f, uint8_t type, uint8_t pl, uint8_t fill) {
    f[0] = 0xEA; f[1] = pl + 2; f[2] = type;
    for (int i = 0; i < pl; i++) f[3 + i] = (uint8_t)(fill + i);
    f[3 + pl] = ghostCrc8(&f[2], pl + 1);
    return pl + 4;
}

// Feed packet i of a frame; the last packet of a multi-packet frame is idx 0.
static void feedPacket(const uint8_t *f, int len, int i, int pkts) {
    uint8_t chunk[PAYLOAD] = {0};
    int n = len - i * PAYLOAD; if (n > PAYLOAD) n = PAYLOAD;
    memcpy(chunk, f + i * PAYLOAD, n);
    feed((i == pkts - 1 && pkts > 1) ? 0 : (uint8_t)(i + 1), chunk);
}

// Send a frame the way StubbornSender does. `lose` bit i set -> packet i
// (0-based, in send order) never reaches us. forceCrc forges a matching CRC
// over whatever the receiver reassembled.
static void sendFrame(const uint8_t *f, int len, unsigned lose = 0, bool dupMiddle = false,
                      bool forceCrc = false) {
    int pkts = (len + PAYLOAD - 1) / PAYLOAD;
    for (int i = 0; i < pkts; i++) {
        if (lose & (1u << i)) continue;
        feedPacket(f, len, i, pkts);
        if (dupMiddle && i == 1 && pkts > 2) feedPacket(f, len, i, pkts);   // re-send (handset ack missed)
    }
    if (pkts == 1) { uint8_t z[PAYLOAD] = {0}; feed(0, z); }   // blank closing packet
    poll(forceCrc);
}

static void testClean(void) {
    resetAll();
    uint8_t f[BUFSZ]; int len = makeFrame(f, 0x02, 15, 0x10);   // 19 bytes, 4 packets
    sendFrame(f, len);
    assert(emitted == 1 && dropped == 0);
    assert(lastLen == len && memcmp(lastFrame, f, len) == 0);
    // single-packet frame (<= PAYLOAD bytes)
    len = makeFrame(f, 0x21, 1, 0x40);
    sendFrame(f, len);
    assert(emitted == 2 && lastLen == len && memcmp(lastFrame, f, len) == 0);
    // longest frame that fits: len byte 70 -> 72 total
    len = makeFrame(f, 0x14, 68, 0x01);
    sendFrame(f, len);
    assert(emitted == 3 && lastLen == 72);
    puts("PASS clean frames accepted");
}

static void testLostMiddle(void) {
    uint8_t f[BUFSZ];
    // 4-packet frame (idx 1,2,3,0): lose packet 1 (idx 2), tracker sees the gap
    int len = makeFrame(f, 0x02, 14, 0x50);                      // 18 bytes
    resetAll();
    sendFrame(f, len, 1u << 1);
    assert(emitted == 0 && dropped == 1);
    // ... even if the CRC is forged to match the spliced bytes
    resetAll();
    sendFrame(f, len, 1u << 1, false, /*forceCrc=*/true);
    assert(emitted == 0 && dropped == 1);
    // 3-packet frame (idx 1,2,0): lose idx 2. Receiver splices idx 0 after idx 1 and
    // the index sequence looks fine; only the received byte count exposes it.
    len = makeFrame(f, 0x02, 8, 0x30);                           // 12 bytes
    resetAll();
    sendFrame(f, len, 1u << 1);
    assert(emitted == 0 && dropped == 1);
    resetAll();
    sendFrame(f, len, 1u << 1, false, true);
    assert(emitted == 0 && dropped == 1);
    // lost first packet: receiver never starts, nothing to emit
    resetAll();
    sendFrame(f, len, 1u << 0);
    assert(emitted == 0);
    puts("PASS lost middle packet dropped (also with forged CRC)");
}

static void testLostFinal(void) {
    uint8_t a[BUFSZ], b[BUFSZ];
    int la = makeFrame(a, 0x02, 13, 0x10);                       // 4 packets
    int lb = makeFrame(b, 0x07, 9, 0x70);                        // 3 packets
    resetAll();
    sendFrame(a, la, 1u << 3);                                   // final packet never heard
    assert(emitted == 0 && dropped == 0);                        // receiver never finished -> nothing to poll
    sendFrame(b, lb);                                            // next frame restarts at idx 1
    assert(emitted == 1 && dropped == 0);
    assert(lastLen == lb && memcmp(lastFrame, b, lb) == 0);
    // next frame's idx 1 is ALSO lost: stale bytes of the old frame must not pass
    resetAll();
    sendFrame(a, la, 1u << 3);
    sendFrame(b, lb, 1u << 0);
    assert(emitted == 0);
    sendFrame(b, lb);
    assert(emitted == 1 && memcmp(lastFrame, b, lb) == 0);
    puts("PASS lost final packet, next frame clean");
}

static void testBadLen(void) {
    const uint8_t lens[] = {0, 1, 71, 200, 254};
    for (unsigned i = 0; i < sizeof(lens); i++) {
        uint8_t f[BUFSZ];
        memset(f, 0, sizeof(f));
        f[1] = lens[i];
        // forge a correct CRC wherever it is in bounds, so only the length check can reject
        if (lens[i] >= 2 && (unsigned)lens[i] + 2 <= sizeof(f)) f[lens[i] + 1] = ghostCrc8(&f[2], lens[i] - 1);
        assert(ghostFrameLen(f, sizeof(f), 0xFFFF) == 0);
    }
    // len 70 (largest legal) with a good CRC is accepted; len 2 (type + crc only) too
    uint8_t f[BUFSZ]; int n = makeFrame(f, 0x14, 68, 0);
    assert(ghostFrameLen(f, sizeof(f), 0xFFFF) == n);
    n = makeFrame(f, 0x14, 0, 0);
    assert(ghostFrameLen(f, sizeof(f), 0xFFFF) == n);
    // corrupted CRC rejected
    f[n - 1] ^= 1;
    assert(ghostFrameLen(f, sizeof(f), 0xFFFF) == 0);
    // claims more bytes than the receiver collected
    n = makeFrame(f, 0x02, 15, 0);
    assert(ghostFrameLen(f, sizeof(f), 10) == 0 && ghostFrameLen(f, sizeof(f), 20) == n);
    puts("PASS bad length bytes rejected");
}

static void testDuplicates(void) {
    uint8_t f[BUFSZ]; int len = makeFrame(f, 0x02, 17, 0x22);   // 21 bytes -> 5 packets
    resetAll();
    sendFrame(f, len, 0, /*dupMiddle=*/true);
    assert(emitted == 1 && dropped == 0 && memcmp(lastFrame, f, len) == 0);
    // duplicated idx 1 (sender restart mid-receive) is just a restart
    resetAll();
    feedPacket(f, len, 0, 5); feedPacket(f, len, 0, 5);
    for (int i = 1; i < 5; i++) feedPacket(f, len, i, 5);
    poll();
    assert(emitted == 1 && dropped == 0 && memcmp(lastFrame, f, len) == 0);
    // resync packets (idx == max) between frames don't break the next frame
    resetAll();
    uint8_t z[PAYLOAD] = {0};
    feed(MAXIDX, z);
    sendFrame(f, len);
    assert(emitted == 1 && dropped == 0);
    puts("PASS duplicates tolerated");
}

// --- Legacy Remote ID schedule: simulate the 333 ms slot ticker ---
static void testSchedule(void) {
    const unsigned period = 333;                 // REMOTEID_LEGACY_PERIOD_MS
    unsigned last[4] = {0, 0, 0, 0}, mx[4] = {0, 0, 0, 0};
    int seen[4] = {0, 0, 0, 0};
    for (unsigned i = 0; i < 60; i++) {
        unsigned t = 1000 + i * period;
        uint8_t k = ghostLegacyKind((uint8_t)(i % GHOST_LEG_SLOTS), 1, 1, 1);
        if (seen[k] && t - last[k] > mx[k]) mx[k] = t - last[k];
        seen[k]++;
        last[k] = t;
    }
    assert(seen[GHOST_LEG_OPERATOR] > 0);
    assert(mx[GHOST_LEG_LOCATION] <= 1000);      // ASTM: Location >= 1 Hz
    assert(mx[GHOST_LEG_BASICID] <= 3000 && mx[GHOST_LEG_SYSTEM] <= 3000 && mx[GHOST_LEG_OPERATOR] <= 3000);
    // fall back to Location when a slot's message is invalid
    assert(ghostLegacyKind(1, 0, 1, 1) == GHOST_LEG_LOCATION);
    assert(ghostLegacyKind(3, 1, 0, 1) == GHOST_LEG_LOCATION);
    assert(ghostLegacyKind(5, 1, 1, 0) == GHOST_LEG_LOCATION);
    assert(ghostLegacyKind(5, 0, 0, 1) == GHOST_LEG_OPERATOR);
    // slot order
    const uint8_t want[6] = {GHOST_LEG_LOCATION, GHOST_LEG_BASICID, GHOST_LEG_LOCATION,
                             GHOST_LEG_SYSTEM, GHOST_LEG_LOCATION, GHOST_LEG_OPERATOR};
    for (int i = 0; i < 6; i++) assert(ghostLegacyKind((uint8_t)i, 1, 1, 1) == want[i]);
    puts("PASS legacy schedule timing");
}

static void testUnix(void) {
    // time synced at unix 1000000 when millis() = 50000
    assert(ghostUnixAtMs(1000000, 50000, 50000) == 1000000);
    assert(ghostUnixAtMs(1000000, 50000, 52999) == 1000002);       // fix after sync
    assert(ghostUnixAtMs(1000000, 50000, 62000) == 1000012);
    assert(ghostUnixAtMs(1000000, 50000, 40000) == 999990);        // fix 10 s BEFORE the sync
    assert(ghostUnixAtMs(1000000, 50000, 49999) == 999999);        // floor, not truncate toward zero
    assert(ghostUnixAtMs(1000000, 50000, 48500) == 999998);        // -1.5 s -> -2
    assert(ghostUnixAtMs(5, 100000, 0) == 0);                      // clamps instead of wrapping
    // millis() wrap between fix and sync: 0xFFFFFC18 is 2 s before millis 1000 after the wrap
    assert(ghostUnixAtMs(2000000, 1000, 0xFFFFFC18u) == 2000000 - 2);
    puts("PASS unix-at-ms (fix before/after sync)");
}

// Telemetry LQ: fractional expected slots (150 Hz 1:32 = 4.69/s, not 4).
static void testTlmLq(void) {
    assert(ghostTlmLq(4, 150, 32, 1000) == 85);    // was 100 with integer hz/denom
    assert(ghostTlmLq(5, 150, 32, 1000) == 100);   // clamped
    assert(ghostTlmLq(0, 150, 32, 1000) == 0);
    assert(ghostTlmLq(1, 50, 64, 1000) == 100);    // <1 slot/s: was forced to 0
    assert(ghostTlmLq(125, 500, 2, 1000) == 50);
    assert(ghostTlmLq(5, 150, 32, 2000) == 53);    // late heartbeat scales expected
    assert(ghostTlmLq(3, 150, 32, 0) == 0);
}

int main(void) {
    testTlmLq();
    testClean();
    testLostMiddle();
    testLostFinal();
    testBadLen();
    testDuplicates();
    testSchedule();
    testUnix();
    puts("ALL OK");
    return 0;
}
