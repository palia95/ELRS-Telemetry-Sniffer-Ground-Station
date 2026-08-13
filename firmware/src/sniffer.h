#pragma once
// =====================================================================
// ELRS Telemetry Sniffer (Ghost RX) - core module
// ---------------------------------------------------------------------
// Turns a standard ELRS 3.5.x receiver into a passive telemetry sniffer:
//   * Locks to a target link using the UID derived from its passphrase.
//   * Never transmits (strictly receive-only, invisible to the link).
//   * Captures PACKET_TYPE_TLM uplink packets the aircraft RX sends to
//     the handset, reassembles them into CRSF frames, and hands them to
//     a transport (BLE / WiFi-UDP / UART).
//
// Integration: see docs/rx_main.patch.md for the small set of call-outs
// to add to src/rx_main.cpp. This module is self-contained otherwise.
// =====================================================================

#include <cstdint>
#include "OTA.h"   // OTA_Packet_s, packet types

#ifdef __cplusplus
extern "C" {
#endif

// --- Lifecycle ---------------------------------------------------------

// Compute UID[0..5] from an ELRS binding phrase (identical to the ELRS
// Python: md5('-DMY_BINDING_PHRASE="' + phrase + '"')[0:6]), then apply
// it: OtaUpdateCrcInitFromUid() + FHSSrandomiseFHSSsequence(...).
// Call at boot (if phrase known) or at runtime when the user sets one.
void Sniffer_SetUidFromPhrase(const char *phrase);

// Set UID directly from 6 raw bytes (e.g. captured via bind) and apply.
void Sniffer_SetUid(const uint8_t uid[6]);

// Must be called whenever the air-rate changes (from SetRFLinkRate), so
// the reassembler knows the max package index for std vs full-res.
void Sniffer_OnRateChanged(void);

// --- Hot path (call from ProcessRFPacket) ------------------------------

// Diagnostic: called at the very top of ProcessRFPacket for EVERY demodulated
// packet, before any CRC/UID check. `hwOk` = SX1280 HW status was RX_OK.
// Lets us tell "radio deaf" (count stays 0) from "RF present but rejected".
void Sniffer_OnRawPacket(bool hwOk);

// Called after Radio.GetLastPacketStats() with THIS packet's stats. Used to
// measure RSSI/SNR of the DRONE's telemetry packets (type == PACKET_TYPE_TLM),
// as opposed to the handset RC packets that dominate CRSF::LinkStatistics.
void Sniffer_OnPacketStats(uint8_t type, int rssiDbm, int snrRaw);

// Sniffer's own reception quality of the DRONE telemetry link (not the TX link).
int Sniffer_TlmRssiDbm(void);  // negative dBm, 0 if no telemetry seen
int Sniffer_TlmLq(void);       // 0..100 (received tlm / expected tlm slots)
int Sniffer_TlmSnrDb(void);    // dB

// Feed a CRC-validated PACKET_TYPE_TLM packet. Reassembles telemetry.
// Safe to call from ISR context (does no blocking I/O).
void Sniffer_ProcessTLM(const OTA_Packet_s *otaPkt);

// --- Main loop ---------------------------------------------------------

// Call from loop(): if a full CRSF frame is ready, forward it to the
// active transport(s) and unlock the reassembler.
void Sniffer_Poll(void);

// Convenience loop entry (the integrate.py patch calls this from loop()):
// runs Sniffer_Poll() plus the optional OLED display tick.
void Ghost_Loop(uint32_t nowMs);

// --- Transport registry (implemented by devTransport_*.cpp) ------------
// A sink receives every completed CRSF frame: [dest][len][type][payload][crc].
// Multiple transports (BLE, WiFi-UDP, UART) can register independently.
typedef void (*Ghost_Sink_t)(const uint8_t *frame, uint8_t len);

// Register a transport sink. Returns false if the (small) table is full.
bool Ghost_Transport_Register(Ghost_Sink_t sink);

// Fan a frame out to all registered sinks (called by Sniffer_Poll).
void Ghost_Transport_SendCRSF(const uint8_t *frame, uint8_t len);

// Initialize whichever transports are compiled in (SERIAL / BLE / WiFi-UDP)
// and, if MY_BINDING_PHRASE is compiled in, apply the UID from it.
// Call once from setup() (the integrate.py patch does this).
void Ghost_SetupTransports(void);

// Transport init entry points (each defined only when its macro is set).
void GhostSerial_Init(void);
void BLE_Transport_Init(const char *deviceName);
void WiFiUDP_Transport_Init(void);
void GhostDisplay_Init(void);
void GhostDisplay_Tick(uint32_t nowMs);

// EASA/ASD-STAN Direct Remote ID BLE broadcaster (devTransport_RemoteID.cpp).
// Alternative to BLE_Transport_Init: broadcasts advertisements instead of
// running a GATT server to a GCS. Ticked once per loop; internally rate-limits
// to REMOTEID_BROADCAST_PERIOD_MS.
void RemoteID_Transport_Init(void);
void RemoteID_Tick(uint32_t nowMs);

// Set the ODID Operator ID and persist to NVS (mirrors the BLE "O:" write).
// Safe to call from the loop task (does an NVS flash write).
void RemoteID_SetOperatorId(const char *id);
// Print the current Operator ID as a machine-parseable line ("[RID] OPID=<id>")
// for a host/GCS reading the debug serial. Empty value prints "[RID] OPID=".
void RemoteID_ReportOperatorId(void);

#ifdef __cplusplus
}
#endif
