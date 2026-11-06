package com.elrs.ghost

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class CrsfParserTest {

    private fun crc8(b: List<Int>): Int {
        var crc = 0
        for (x in b) {
            crc = crc xor x
            repeat(8) { crc = if (crc and 0x80 != 0) ((crc shl 1) xor 0xD5) and 0xFF else (crc shl 1) and 0xFF }
        }
        return crc
    }

    private fun frame(type: Int, payload: List<Int>): ByteArray {
        val body = listOf(type) + payload
        return (listOf(0xC8, body.size + 1) + body + crc8(body)).map { it.toByte() }.toByteArray()
    }

    private fun parse(vararg chunks: ByteArray): List<Telemetry> {
        val out = mutableListOf<Telemetry>()
        val p = CrsfParser { out += it }
        chunks.forEach { p.feed(it) }
        return out
    }

    @Test fun shortFrameIsDropped() {
        assertTrue(parse(frame(CrsfTypes.GPS, List(5) { 0 })).isEmpty())
    }

    @Test fun baroDecimetres() {
        val r = parse(frame(CrsfTypes.BARO_ALT, listOf(0x27, 0x10 + 0x0A))) // 10000 + 10 -> 1.0 m
        assertEquals(1.0, (r[0] as Telemetry.BaroAlt).altitudeM, 1e-9)
    }

    @Test fun baroHighBitMetres() {
        val r = parse(frame(CrsfTypes.BARO_ALT, listOf(0x80, 0x64))) // 100 m
        assertEquals(100.0, (r[0] as Telemetry.BaroAlt).altitudeM, 1e-9)
    }

    @Test fun badLengthDoesNotSwallowGoodFrame() {
        val good = frame(CrsfTypes.VARIO, listOf(0, 100))
        val bad = byteArrayOf(0xC8.toByte(), 30) // claims 32 bytes total
        val r = parse(bad + good + good + good + good + good + good)
        assertEquals(6, r.size)
        assertEquals(1.0, (r[0] as Telemetry.Vario).verticalSpeedMs, 1e-9)
    }
}
