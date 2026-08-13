"""
CRSF telemetry frame parser for the ELRS TLM RX GCS.

A frame is:  [dest][len][type][payload...][crc8]
  dest : sync/destination byte (0xC8 for FC->handset telemetry)
  len  : number of bytes that follow the len field, i.e. type + payload + crc
  type : CRSF frame type (see constants below)
  crc8 : CRC8/DVB-S2 (poly 0xD5) over  type + payload

Only the frame types the aircraft actually emits are decoded in detail
(GPS / BATTERY / ATTITUDE / FLIGHT_MODE); VARIO / BARO / LINK_STATISTICS are
decoded best-effort in case a different craft sends them.

parse_frame() returns a dict of decoded fields (keys documented per type) or
None if the frame is too short / fails CRC.
"""
import struct

# --- CRSF frame types ---
GPS            = 0x02
VARIO          = 0x07
BATTERY_SENSOR = 0x08
BARO_ALTITUDE  = 0x09
LINK_STATISTICS = 0x14
ATTITUDE       = 0x1E
FLIGHT_MODE    = 0x21

RAD_TO_DEG = 57.29577951308232


def crc8_dvb_s2(data: bytes, poly: int = 0xD5) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def _u16(p, i):  return struct.unpack_from('>H', p, i)[0]
def _s16(p, i):  return struct.unpack_from('>h', p, i)[0]
def _s32(p, i):  return struct.unpack_from('>i', p, i)[0]
def _u24(p, i):  return (p[i] << 16) | (p[i + 1] << 8) | p[i + 2]


def frame_type_name(t: int) -> str:
    return {
        GPS: "GPS", VARIO: "VARIO", BATTERY_SENSOR: "BATTERY",
        BARO_ALTITUDE: "BARO", LINK_STATISTICS: "LINK", ATTITUDE: "ATTITUDE",
        FLIGHT_MODE: "MODE",
    }.get(t, "0x%02X" % t)


def parse_frame(frame: bytes, check_crc: bool = True):
    """Decode one CRSF frame. Returns (type, dict) or None."""
    if len(frame) < 4:
        return None
    length = frame[1]
    total = 2 + length
    if len(frame) < total:
        return None
    ftype = frame[2]
    payload = frame[3:total - 1]
    crc = frame[total - 1]
    if check_crc and crc8_dvb_s2(frame[2:total - 1]) != crc:
        return None

    p = payload
    out = {}
    try:
        if ftype == GPS and len(p) >= 15:
            out = {
                "lat": _s32(p, 0) / 1e7,          # deg
                "lon": _s32(p, 4) / 1e7,          # deg
                "groundspeed_kmh": _u16(p, 8) / 10.0,
                "heading_deg": _u16(p, 10) / 100.0,
                "alt_gps_m": _u16(p, 12) - 1000,
                "sats": p[14],
            }
        elif ftype == BATTERY_SENSOR and len(p) >= 8:
            out = {
                "batt_v": _u16(p, 0) / 10.0,
                "batt_a": _u16(p, 2) / 10.0,
                "batt_mah": _u24(p, 4),
                "batt_pct": p[7],
            }
        elif ftype == ATTITUDE and len(p) >= 6:
            out = {
                "pitch_deg": _s16(p, 0) / 1e4 * RAD_TO_DEG,
                "roll_deg":  _s16(p, 2) / 1e4 * RAD_TO_DEG,
                "yaw_deg":   _s16(p, 4) / 1e4 * RAD_TO_DEG,
            }
        elif ftype == FLIGHT_MODE:
            s = bytes(p)
            nul = s.find(0)
            if nul >= 0:
                s = s[:nul]
            out = {"mode": s.decode("ascii", "replace")}
        elif ftype == VARIO and len(p) >= 2:
            out = {"vario_ms": _s16(p, 0) / 100.0}
        elif ftype == BARO_ALTITUDE and len(p) >= 2:
            v = _u16(p, 0)
            # best-effort: high bit => meters, else decimeters with -10000 offset
            out = {"baro_alt_m": (v & 0x7FFF) if (v & 0x8000) else (v / 10.0 - 1000)}
            # Some aircraft send the combined 4-byte baro+vario variant (altitude
            # + vertical speed in the same frame) instead of a separate VARIO
            # frame - extract it here too, or it's silently dropped even though
            # it's already in the payload we successfully parsed.
            if len(p) >= 4:
                out["vario_ms"] = _s16(p, 2) / 100.0
        elif ftype == LINK_STATISTICS and len(p) >= 10:
            out = {
                "link_rssi1_dbm": -p[0],
                "link_rssi2_dbm": -p[1],
                "link_lq": p[2],
                "link_snr": struct.unpack_from('b', p, 3)[0],
                "link_tx_power": p[6],
            }
        else:
            return None
    except (struct.error, IndexError):
        return None

    return ftype, out


class FrameStreamer:
    """Accumulate raw bytes (e.g. from BLE) and yield complete CRSF frames.

    Each BLE notification from the sniffer is already one frame, but this makes
    the path robust to fragmentation / concatenation.
    """
    def __init__(self, max_len=64):
        self.buf = bytearray()
        self.max_len = max_len

    def feed(self, data: bytes):
        self.buf.extend(data)
        frames = []
        while len(self.buf) >= 2:
            length = self.buf[1]
            total = 2 + length
            if length < 2 or length > self.max_len:
                # not a plausible frame start; drop one byte and resync
                self.buf.pop(0)
                continue
            if len(self.buf) < total:
                break
            frames.append(bytes(self.buf[:total]))
            del self.buf[:total]
        return frames
