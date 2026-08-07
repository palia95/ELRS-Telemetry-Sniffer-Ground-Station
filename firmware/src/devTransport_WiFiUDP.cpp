// =====================================================================
// Ghost RX transport: WiFi UDP
// Streams reassembled CRSF telemetry frames over UDP so a phone/PC ground
// station on the same network can ingest them. Two modes:
//   * RAW CRSF   -> UDP :14555 (custom app parses CRSF)
//   * MAVLink    -> UDP :14550 (standard GCS apps) IF you enable conversion
//
// This module assumes ELRS WiFi (devWIFI) has brought up SoftAP or STA.
// It broadcasts on the active interface's subnet so any client can listen
// without prior registration.
// =====================================================================
#if defined(GHOST_TRANSPORT_WIFI_UDP) && defined(PLATFORM_ESP32)

#include <WiFi.h>
#include <WiFiUdp.h>
#include "sniffer.h"

#define GHOST_UDP_PORT_CRSF 14555

static WiFiUDP s_udp;
static bool s_up = false;

static void WiFiUDP_sink(const uint8_t *frame, uint8_t len)
{
    if (!s_up) {
        // lazily confirm an interface is available
        if (WiFi.getMode() == WIFI_OFF) return;
        s_up = true;
    }

    // Broadcast address of the active interface.
    IPAddress bcast;
    if (WiFi.getMode() & WIFI_AP) {
        IPAddress ip = WiFi.softAPIP();
        bcast = IPAddress(ip[0], ip[1], ip[2], 255);
    } else {
        IPAddress ip = WiFi.localIP();
        IPAddress mask = WiFi.subnetMask();
        for (int i = 0; i < 4; i++) bcast[i] = ip[i] | ~mask[i];
    }

    s_udp.beginPacket(bcast, GHOST_UDP_PORT_CRSF);
    s_udp.write(frame, len);
    s_udp.endPacket();
}

void WiFiUDP_Transport_Init()
{
    Ghost_Transport_Register(&WiFiUDP_sink);
    // Note: actual TX only happens once WiFi is up (handled in the sink).
}

#endif // GHOST_TRANSPORT_WIFI_UDP
