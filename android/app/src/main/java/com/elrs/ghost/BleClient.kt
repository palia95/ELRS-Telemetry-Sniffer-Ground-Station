package com.elrs.ghost

import android.annotation.SuppressLint
import android.bluetooth.*
import android.content.Context
import java.util.UUID

/**
 * BLE GATT client for the Ghost RX (Nordic UART Service).
 * Connects, subscribes to TX notifications (telemetry), and can write the
 * binding phrase to the RX characteristic ("P:<phrase>\n").
 */
@SuppressLint("MissingPermission")
class BleClient(
    private val context: Context,
    private val onBytes: (ByteArray) -> Unit,
    private val onState: (String) -> Unit
) {
    companion object {
        val SVC: UUID = UUID.fromString("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
        val RX: UUID  = UUID.fromString("6E400002-B5A3-F393-E0A9-E50E24DCCA9E") // phone -> ESP
        val TX: UUID  = UUID.fromString("6E400003-B5A3-F393-E0A9-E50E24DCCA9E") // ESP -> phone
        val CCCD: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
        const val DEVICE_NAME = "T3S3 Ghost RX"
    }

    private var gatt: BluetoothGatt? = null
    private var rxChar: BluetoothGattCharacteristic? = null

    fun connect(device: BluetoothDevice) {
        onState("Connecting…")
        gatt = device.connectGatt(context, false, cb, BluetoothDevice.TRANSPORT_LE)
    }

    fun disconnect() { gatt?.disconnect(); gatt?.close(); gatt = null }

    fun setPhrase(phrase: String) {
        val c = rxChar ?: return
        c.value = ("P:$phrase\n").toByteArray()
        c.writeType = BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE
        gatt?.writeCharacteristic(c)
    }

    private val cb = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(g: BluetoothGatt, status: Int, newState: Int) {
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                onState("Discovering…"); g.requestMtu(247)
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                onState("Disconnected")
            }
        }

        override fun onMtuChanged(g: BluetoothGatt, mtu: Int, status: Int) { g.discoverServices() }

        override fun onServicesDiscovered(g: BluetoothGatt, status: Int) {
            val svc = g.getService(SVC) ?: run { onState("No NUS service"); return }
            rxChar = svc.getCharacteristic(RX)
            val tx = svc.getCharacteristic(TX) ?: run { onState("No TX char"); return }
            g.setCharacteristicNotification(tx, true)
            tx.getDescriptor(CCCD)?.let {
                it.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                g.writeDescriptor(it)
            }
            onState("Connected")
        }

        @Deprecated("Deprecated in Java")
        override fun onCharacteristicChanged(g: BluetoothGatt, c: BluetoothGattCharacteristic) {
            if (c.uuid == TX) onBytes(c.value)
        }

        // API 33+ overload
        override fun onCharacteristicChanged(
            g: BluetoothGatt, c: BluetoothGattCharacteristic, value: ByteArray
        ) { if (c.uuid == TX) onBytes(value) }
    }
}
