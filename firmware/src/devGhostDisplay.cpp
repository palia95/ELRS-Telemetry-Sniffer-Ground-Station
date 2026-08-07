// =====================================================================
// Ghost sniffer OLED display (LILYGO T3-S3, 0.96" SSD1306, I2C)
// ---------------------------------------------------------------------
// Shows link status (SEARCH/LOCK, rate, packet count) and decoded
// telemetry (battery, GPS, flight mode, link LQ). Driven directly with
// U8g2 on the onboard OLED (SDA=18, SCL=17, addr 0x3C) - independent of
// the ELRS screen system. Enable with -D GHOST_DISPLAY.
//
// FULLY ISOLATED FROM THE RF CORE (this is the important property):
//   The onboard I2C wedges on its FIRST transaction on this board even with
//   Wire.setTimeOut() set, and any I2C done from loop() takes the whole RF
//   loop down with it (no heartbeat, no lock). So ALL I2C now runs in a
//   dedicated FreeRTOS task pinned to CORE 0. The ELRS RF loop runs on core 1;
//   if I2C ever blocks it blocks only this task, never the sniffer.
//   GhostDisplay_Init() just registers the sink + spawns the task (no I2C in
//   setup()); GhostDisplay_Tick() is now a no-op kept for the call site.
//
// SOFTWARE (bit-banged) I2C: on this board a plain Wire probe ACKs 0x3C, but
// u8g2's HARDWARE-I2C begin() hangs - the ESP32 IDF I2C peripheral wedges when
// U8g2 re-inits Wire after our own Wire.begin(). Software I2C drives the pins
// with plain GPIO toggles (no IDF peripheral, no double-init) and cannot hang
// on a hardware wait, so it sidesteps the problem entirely.
//
// Before init the task pulses SCL 9x (bus recovery) to free a slave that may be
// holding SDA low from a prior wedged transfer - the usual first-transaction
// hang - then goes straight to u8g2 software-I2C begin().
//
// Confirmed board facts (LilyGo T3-S3-SX1280 hw reference):
//   SSD1306 @ 0x3C, shared I2C bus SDA=18 / SCL=17, no reset line,
//   explicit Wire.begin(sda,scl) required (pins not the ESP32 default).
// =====================================================================
#if defined(GHOST_DISPLAY) && defined(PLATFORM_ESP32)

#include <U8g2lib.h>
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sniffer.h"
#include "crsf_protocol.h"
#include "common.h"          // connectionState, ExpressLRS_currAirRate_Modparams
#include "logging.h"         // DBGLN
#include "CRSF.h"            // CRSF::LinkStatistics (our sniff RSSI/LQ)

#ifndef GHOST_OLED_SDA
#define GHOST_OLED_SDA 18
#endif
#ifndef GHOST_OLED_SCL
#define GHOST_OLED_SCL 17
#endif
#ifndef GHOST_OLED_RST
#define GHOST_OLED_RST U8X8_PIN_NONE   // T3-S3 OLED has no dedicated reset line
#endif
#ifndef GHOST_OLED_ADDR
#define GHOST_OLED_ADDR 0x3C           // 7-bit
#endif

#define GHOST_OLED_REFRESH       250    // ms between redraws (~4 Hz)

// Full-buffer SSD1306 128x64 over SOFTWARE (bit-banged) I2C.
// SW_I2C arg order: (rotation, clock=SCL, data=SDA, reset).
static U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, GHOST_OLED_SCL, GHOST_OLED_SDA, GHOST_OLED_RST);

static TaskHandle_t s_dispTask = nullptr;

// ---- shared decoded state (written by sink in loop ctx, read by task) ----
static struct {
    float    battV = 0, battA = 0; int battPct = 0;
    double   lat = 0, lon = 0; int alt = 0, sats = 0;
    int      spd = 0;   // groundspeed, 0.1 km/h
    int      hdg = 0;   // heading, 0.01 deg
    char     mode[16] = "";
    int      rssi1 = 0, lq = 0;
    uint32_t pkts = 0;
    uint32_t lastFrameMs = 0;
} T;

// ---- big-endian helpers ----
static inline uint16_t be16u(const uint8_t *p){ return (p[0]<<8)|p[1]; }
static inline int16_t  be16s(const uint8_t *p){ return (int16_t)be16u(p); }
static inline int32_t  be32s(const uint8_t *p){ return (int32_t)(((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3]); }

// ---- CRSF sink: decode the fields we display (runs in loop context) ----
static void Display_sink(const uint8_t *f, uint8_t len)
{
    if (len < 4) return;
    const uint8_t type = f[2];
    const uint8_t *p = f + 3;
    // The LINK_STATISTICS frame is injected locally at 5 Hz (our RSSI/LQ), not
    // received drone telemetry - don't count it, or PKT climbs and the "age since
    // last packet" timer never goes stale when the drone is off.
    if (type == CRSF_FRAMETYPE_LINK_STATISTICS) return;
    T.pkts++;
    T.lastFrameMs = millis();
    switch (type) {
    case CRSF_FRAMETYPE_BATTERY_SENSOR:
        T.battV = be16u(p) / 10.0f;
        T.battA = be16u(p + 2) / 10.0f;
        T.battPct = p[7];
        break;
    case CRSF_FRAMETYPE_GPS:
        T.lat = be32s(p) / 1e7;
        T.lon = be32s(p + 4) / 1e7;
        T.spd = be16u(p + 8);       // 0.1 km/h
        T.hdg = be16u(p + 10);      // 0.01 deg
        T.alt = (int)be16u(p + 12) - 1000;
        T.sats = p[14];
        break;
    case CRSF_FRAMETYPE_FLIGHT_MODE: {
        uint8_t i = 0;
        while (i < sizeof(T.mode) - 1 && p[i] && (3 + i) < len - 1) { T.mode[i] = (char)p[i]; i++; }
        T.mode[i] = 0;
        break; }
    default: break;
    }
}

static const char *stateStr()
{
    switch (connectionState) {
        case connected:    return "LOCK";
        case tentative:    return "TENT";
        case disconnected: return "SRCH";
        default:           return "----";
    }
}

// Manually clock the bus free: some SSD1306s hold SDA low after a wedged
// transfer, which hangs the very first Wire transaction. Pulse SCL 9x with SDA
// released, then issue a STOP. Runs on the task's core; touches GPIO only.
static void i2cBusRecover()
{
    pinMode(GHOST_OLED_SCL, OUTPUT_OPEN_DRAIN);
    pinMode(GHOST_OLED_SDA, INPUT_PULLUP);
    for (int i = 0; i < 9; i++) {
        digitalWrite(GHOST_OLED_SCL, HIGH); delayMicroseconds(6);
        digitalWrite(GHOST_OLED_SCL, LOW);  delayMicroseconds(6);
    }
    // STOP condition: SDA low->high while SCL high
    pinMode(GHOST_OLED_SDA, OUTPUT_OPEN_DRAIN);
    digitalWrite(GHOST_OLED_SDA, LOW);  delayMicroseconds(6);
    digitalWrite(GHOST_OLED_SCL, HIGH); delayMicroseconds(6);
    digitalWrite(GHOST_OLED_SDA, HIGH); delayMicroseconds(6);
}

static void drawFrame(uint32_t nowMs)
{
    uint8_t rateIdx = ExpressLRS_currAirRate_Modparams ? ExpressLRS_currAirRate_Modparams->index : 0;
    uint32_t age = (T.lastFrameMs) ? (nowMs - T.lastFrameMs) / 1000 : 999;
    if (age > 999) age = 999;
    // OUR reception quality of the DRONE telemetry link (measured on the tlm
    // packets we overhear), NOT CRSF::LinkStatistics which tracks the handset RC
    // link. rssi is negative dBm (0 when no telemetry).
    int rssi = Sniffer_TlmRssiDbm();
    unsigned lq = (unsigned)Sniffer_TlmLq();

    char l1[24], l2[24], l3[24], l4[32], l5[32];
    snprintf(l1, sizeof l1, "TLM RX %-4s r%u LQ%u", stateStr(), rateIdx, lq);
    snprintf(l2, sizeof l2, "PKT%lu %lus %.1fV %.0fA",
             (unsigned long)T.pkts, (unsigned long)age, T.battV, T.battA);
    snprintf(l3, sizeof l3, "SAT%d ALT%dm RSSI%ddB", T.sats, T.alt, rssi);
    // mode + groundspeed (0.1km/h -> km/h, no unit) + heading (0.01deg -> deg).
    // Degree sign as UTF-8 (0xC2 0xB0); drawn with drawUTF8 (6x12_tf has it).
    snprintf(l4, sizeof l4, "%s %.1f %d\xC2\xB0",
             T.mode[0] ? T.mode : "-", T.spd / 10.0f, T.hdg / 100);
    // 6 decimals (~0.1 m) + automatic sign, no unit letters. Worst case (both
    // negative, 3-digit lon) ~22 chars, so this row uses a narrow 5x8 font.
    snprintf(l5, sizeof l5, "%.6f %.6f", T.lat, T.lon);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 10, l1);
    u8g2.drawHLine(0, 12, 128);
    u8g2.drawStr(0, 25, l2);
    u8g2.drawStr(0, 38, l3);
    u8g2.drawUTF8(0, 51, l4);
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(0, 63, l5);
    u8g2.sendBuffer();
}

// Runs on core 0. Any I2C stall here can NEVER block the RF loop on core 1.
static void displayTask(void *)
{
    // Let the RF core finish boot + start scanning before we poke the bus.
    vTaskDelay(pdMS_TO_TICKS(500));

    DBGLN("[TLM RX] OLED task: bus recover");
    i2cBusRecover();

    // Software I2C: bit-banged, no IDF peripheral, cannot hang on a hw wait.
    DBGLN("[TLM RX] OLED task: u8g2.begin (sw i2c)");
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 12, "ELRS TLM RX");
    u8g2.drawStr(0, 28, "searching...");
    u8g2.sendBuffer();
    DBGLN("[TLM RX] OLED task: display up");

    for (;;) {
        drawFrame(millis());
        vTaskDelay(pdMS_TO_TICKS(GHOST_OLED_REFRESH));
    }
}

// Register the sink + spawn the display task. NO I2C here -> setup() can't hang.
void GhostDisplay_Init(void)
{
    Ghost_Transport_Register(&Display_sink);
    if (!s_dispTask) {
        // Core 0 (RF loop runs on core 1); low priority; 4 KB stack for U8g2.
        xTaskCreatePinnedToCore(displayTask, "ghostoled", 4096, nullptr, 1, &s_dispTask, 0);
    }
    DBGLN("[TLM RX] OLED: display task spawned on core 0");
}

// Task-driven now; nothing to do per-loop.
void GhostDisplay_Tick(uint32_t)
{
}

#endif // GHOST_DISPLAY
