package com.elrs.ghost

/**
 * Minimal CRSF telemetry parser for the Ghost RX stream.
 *
 * A CRSF frame is: [addr][len][type][payload...][crc8]
 *   - len  = number of bytes from `type` through `crc8` inclusive
 *   - crc8 = DVB-S2 (poly 0xD5) over [type..payload]
 *
 * The Ghost RX forwards whole frames, so we can feed the byte stream in and
 * emit decoded telemetry structs. All multi-byte fields are big-endian.
 */

data class GpsData(
    val latDeg: Double, val lonDeg: Double,
    val groundSpeedKmh: Double, val headingDeg: Double,
    val altitudeM: Int, val satellites: Int
)

data class BatteryData(
    val voltage: Double, val current: Double,
    val capacityMah: Int, val remainingPct: Int
)

data class AttitudeData(val pitchDeg: Double, val rollDeg: Double, val yawDeg: Double)

data class LinkStats(
    val upRssiAnt1: Int, val upRssiAnt2: Int, val upLinkQuality: Int, val upSnr: Int,
    val rfMode: Int, val txPower: Int
)

sealed interface Telemetry {
    data class Gps(val d: GpsData) : Telemetry
    data class Battery(val d: BatteryData) : Telemetry
    data class Attitude(val d: AttitudeData) : Telemetry
    data class Vario(val verticalSpeedMs: Double) : Telemetry
    data class BaroAlt(val altitudeM: Double) : Telemetry
    data class FlightMode(val mode: String) : Telemetry
    data class Link(val d: LinkStats) : Telemetry
    data class Unknown(val type: Int) : Telemetry
}

object CrsfTypes {
    const val GPS = 0x02
    const val VARIO = 0x07
    const val BATTERY = 0x08
    const val BARO_ALT = 0x09
    const val LINK_STATISTICS = 0x14
    const val ATTITUDE = 0x1E
    const val FLIGHT_MODE = 0x21
}

class CrsfParser(private val onFrame: (Telemetry) -> Unit) {

    private val buf = ArrayDeque<Int>()

    /** Feed raw bytes from BLE notifications / UDP packets. */
    fun feed(data: ByteArray) {
        for (b in data) buf.addLast(b.toInt() and 0xFF)
        drain()
    }

    private fun drain() {
        while (buf.size >= 4) {
            // buf[0] = addr, buf[1] = len
            val len = buf.elementAt(1)
            if (len < 2 || len > 62) { buf.removeFirst(); continue } // resync
            val total = len + 2
            if (buf.size < total) return

            val frame = IntArray(total) { buf.elementAt(it) }
            repeat(total) { buf.removeFirst() }

            if (crc8(frame, 2, len - 1) == frame[total - 1]) {
                decode(frame)
            }
            // bad CRC -> drop frame, continue
        }
    }

    private fun decode(f: IntArray) {
        val type = f[2]
        val p = 3 // payload start
        when (type) {
            CrsfTypes.GPS -> onFrame(Telemetry.Gps(GpsData(
                latDeg = i32(f, p) / 1e7,
                lonDeg = i32(f, p + 4) / 1e7,
                groundSpeedKmh = u16(f, p + 8) / 10.0,
                headingDeg = u16(f, p + 10) / 100.0,
                altitudeM = u16(f, p + 12) - 1000,
                satellites = f[p + 14]
            )))
            CrsfTypes.BATTERY -> onFrame(Telemetry.Battery(BatteryData(
                voltage = u16(f, p) / 10.0,
                current = u16(f, p + 2) / 10.0,
                capacityMah = u24(f, p + 4),
                remainingPct = f[p + 7]
            )))
            CrsfTypes.ATTITUDE -> onFrame(Telemetry.Attitude(AttitudeData(
                pitchDeg = s16(f, p) / 10000.0 * 57.2958,
                rollDeg = s16(f, p + 2) / 10000.0 * 57.2958,
                yawDeg = s16(f, p + 4) / 10000.0 * 57.2958
            )))
            CrsfTypes.VARIO -> onFrame(Telemetry.Vario(s16(f, p) / 100.0))
            CrsfTypes.BARO_ALT -> onFrame(Telemetry.BaroAlt((u16(f, p) - 10000) / 10.0))
            CrsfTypes.FLIGHT_MODE -> {
                val sb = StringBuilder()
                var i = p
                while (i < f.size - 1 && f[i] != 0) { sb.append(f[i].toChar()); i++ }
                onFrame(Telemetry.FlightMode(sb.toString()))
            }
            CrsfTypes.LINK_STATISTICS -> onFrame(Telemetry.Link(LinkStats(
                upRssiAnt1 = -f[p], upRssiAnt2 = -f[p + 1], upLinkQuality = f[p + 2],
                upSnr = f[p + 3].toByte().toInt(), rfMode = f[p + 5], txPower = f[p + 6]
            )))
            else -> onFrame(Telemetry.Unknown(type))
        }
    }

    // --- big-endian field readers ---
    private fun u16(f: IntArray, i: Int) = (f[i] shl 8) or f[i + 1]
    private fun s16(f: IntArray, i: Int) = u16(f, i).toShort().toInt()
    private fun u24(f: IntArray, i: Int) = (f[i] shl 16) or (f[i + 1] shl 8) or f[i + 2]
    private fun i32(f: IntArray, i: Int) =
        (f[i] shl 24) or (f[i + 1] shl 16) or (f[i + 2] shl 8) or f[i + 3]

    private fun crc8(f: IntArray, start: Int, count: Int): Int {
        var crc = 0
        for (i in start until start + count) {
            crc = crc xor f[i]
            repeat(8) { crc = if (crc and 0x80 != 0) ((crc shl 1) xor 0xD5) and 0xFF else (crc shl 1) and 0xFF }
        }
        return crc
    }
}
