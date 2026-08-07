#pragma once
// OLED status + telemetry display for the Ghost sniffer (T3-S3 SSD1306).
#include <cstdint>

void GhostDisplay_Init(void);          // set up U8g2 + register a CRSF sink
void GhostDisplay_Tick(uint32_t nowMs); // called each loop (rate-limited inside)
