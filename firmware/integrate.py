#!/usr/bin/env python3
"""
Integrate the Ghost sniffer into an ExpressLRS 3.5.x source tree.

Usage:
    python3 integrate.py /path/to/ExpressLRS/src

- Copies the sniffer + transport sources into <ELRS>/src/src/
- Applies the (idempotent, anchor-based) edits to src/rx_main.cpp
- Safe to run multiple times; each edit checks whether it is already applied.

All edits are guarded by GHOST_SNIFFER / GHOST_NO_TX so a normal build is
unaffected unless the sniffer env defines those macros.
"""
import os, shutil, sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC_FILES = [
    "sniffer.h", "sniffer.cpp",
    "devTransport_Serial.cpp",
    "devTransport_BLE.cpp", "devTransport_WiFiUDP.cpp",
    "devGhostDisplay.h", "devGhostDisplay.cpp",
]

# (anchor, replacement) — anchor must appear verbatim & exactly once.
EDITS = [
    # 1. include
    ('#include "stubborn_receiver.h"',
     '#include "stubborn_receiver.h"\n#include "sniffer.h"'),

    # 2. never transmit; in the telemetry slot, arm RX to OVERHEAR the aircraft's
    #    telemetry instead of transmitting our own.
    ('    if ((connectionState == disconnected) || (ExpressLRS_currTlmDenom == 1) || (alreadyTLMresp == true) || (modresult != 0) || !teamraceHasModelMatch)\n'
     '    {\n'
     '        return false; // don\'t bother sending tlm if disconnected or TLM is off\n'
     '    }',
     '    if ((connectionState == disconnected) || (ExpressLRS_currTlmDenom == 1) || (alreadyTLMresp == true) || (modresult != 0) || !teamraceHasModelMatch)\n'
     '    {\n'
     '        return false; // don\'t bother sending tlm if disconnected or TLM is off\n'
     '    }\n'
     '#if defined(GHOST_NO_TX)\n'
     '    // Ghost sniffer: this IS a telemetry slot. Rather than transmit our own\n'
     '    // telemetry, arm the radio in RX so we overhear the aircraft telemetry.\n'
     '    // IMPORTANT: do NOT set alreadyTLMresp here. That flag makes the tick skip\n'
     '    // LQCalc.inc(), leaving LQCalc.currentIsSet() == true, which makes RXdoneISR\n'
     '    // DROP the incoming telemetry packet before ProcessRFPacket ever runs.\n'
     '    Radio.RXnb();\n'
     '    return false;\n'
     '#endif'),

    # 3. capture TLM packets
    ('    case PACKET_TYPE_TLM:\n        if (firmwareOptions.is_airport)',
     '    case PACKET_TYPE_TLM:\n'
     '#if defined(GHOST_SNIFFER)\n'
     '        Sniffer_ProcessTLM(otaPktPtr);\n'
     '#endif\n'
     '        if (firmwareOptions.is_airport)'),

    # 4. keep reassembler in sync with the air-rate
    ('    OtaUpdateSerializers(smWideOr8ch, ModParams->PayloadLength);',
     '    OtaUpdateSerializers(smWideOr8ch, ModParams->PayloadLength);\n'
     '#if defined(GHOST_SNIFFER)\n'
     '    Sniffer_OnRateChanged();\n'
     '#endif'),

    # 5. init transports at boot
    ('    devicesStart();',
     '    devicesStart();\n'
     '#if defined(GHOST_SNIFFER)\n'
     '    Ghost_SetupTransports();\n'
     '#endif'),

    # 6. poll for completed CRSF frames + update OLED each loop
    ('    devicesUpdate(now);',
     '    devicesUpdate(now);\n'
     '#if defined(GHOST_SNIFFER)\n'
     '    Ghost_Loop(now);\n'
     '#endif'),

    # 7. diagnostic: count every demodulated packet before CRC/UID check
    ('bool ICACHE_RAM_ATTR ProcessRFPacket(SX12xxDriverCommon::rx_status const status)\n{\n',
     'bool ICACHE_RAM_ATTR ProcessRFPacket(SX12xxDriverCommon::rx_status const status)\n{\n'
     '#if defined(GHOST_SNIFFER)\n'
     '    Sniffer_OnRawPacket(status == SX12xxDriverCommon::SX12XX_RX_OK);\n'
     '#endif\n'),

    # 8. per-packet RSSI/SNR - AFTER GetLastPacketStats() so the stats belong to
    #    the packet we just received. Lets the sniffer measure RSSI/LQ of the
    #    DRONE's telemetry packets (PACKET_TYPE_TLM) separately from the RC packets
    #    that dominate CRSF::LinkStatistics (which reflect the handset TX link).
    ('    // Store the LQ/RSSI/Antenna\n'
     '    Radio.GetLastPacketStats();\n'
     '    getRFlinkInfo();',
     '    // Store the LQ/RSSI/Antenna\n'
     '    Radio.GetLastPacketStats();\n'
     '    getRFlinkInfo();\n'
     '#if defined(GHOST_SNIFFER)\n'
     '    Sniffer_OnPacketStats(otaPktPtr->std.type, Radio.LastPacketRSSI, Radio.LastPacketSNRRaw);\n'
     '#endif'),
]

def main():
    if len(sys.argv) != 2:
        print(__doc__); sys.exit(1)
    elrs_src = os.path.abspath(sys.argv[1])            # .../ExpressLRS/src
    dst_src = os.path.join(elrs_src, "src")            # .../ExpressLRS/src/src
    rx_main = os.path.join(dst_src, "rx_main.cpp")
    if not os.path.isfile(rx_main):
        print(f"ERROR: {rx_main} not found. Point me at ExpressLRS/src"); sys.exit(2)

    # copy sources
    for f in SRC_FILES:
        shutil.copy2(os.path.join(HERE, "src", f), os.path.join(dst_src, f))
        print(f"copied  src/{f}")

    # apply edits
    text = open(rx_main, encoding="utf-8").read()
    for i, (anchor, repl) in enumerate(EDITS, 1):
        if repl in text:
            print(f"edit {i}: already applied"); continue
        n = text.count(anchor)
        if n != 1:
            print(f"edit {i}: ABORT - anchor found {n} times (expected 1). "
                  f"ELRS version mismatch?"); sys.exit(3)
        text = text.replace(anchor, repl)
        print(f"edit {i}: applied")
    open(rx_main, "w", encoding="utf-8").write(text)
    print("\nDone. Now add targets/t3s3_sniffer.ini to src/targets/ and build:")
    print("  pio run -e T3S3_Sniffer_2400_RX_Serial")

if __name__ == "__main__":
    main()
