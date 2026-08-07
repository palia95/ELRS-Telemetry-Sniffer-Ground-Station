package com.elrs.ghost

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.content.Context
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewmodel.compose.viewModel

/**
 * Holds decoded telemetry and drives the CRSF parser from the BLE byte stream.
 */
class TelemetryViewModel : ViewModel() {
    var status by mutableStateOf("Idle"); private set
    var gps by mutableStateOf<GpsData?>(null); private set
    var battery by mutableStateOf<BatteryData?>(null); private set
    var attitude by mutableStateOf<AttitudeData?>(null); private set
    var flightMode by mutableStateOf(""); private set
    var link by mutableStateOf<LinkStats?>(null); private set
    var baroAltM by mutableStateOf<Double?>(null); private set

    private val parser = CrsfParser { t ->
        when (t) {
            is Telemetry.Gps -> gps = t.d
            is Telemetry.Battery -> battery = t.d
            is Telemetry.Attitude -> attitude = t.d
            is Telemetry.FlightMode -> flightMode = t.mode
            is Telemetry.Link -> link = t.d
            is Telemetry.BaroAlt -> baroAltM = t.altitudeM
            else -> {}
        }
    }

    fun onBytes(b: ByteArray) = parser.feed(b)
    fun setStatus(s: String) { status = s }
}

class MainActivity : ComponentActivity() {

    private lateinit var vm: TelemetryViewModel
    private var ble: BleClient? = null

    private val permLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { scanAndConnect() }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            vm = viewModel()
            MaterialTheme { GhostScreen(vm, onConnect = { ensurePermsThenConnect() }, onPhrase = { ble?.setPhrase(it) }) }
        }
    }

    private fun ensurePermsThenConnect() {
        val perms = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S)
            arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
        else arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)
        permLauncher.launch(perms)
    }

    @SuppressLint("MissingPermission")
    private fun scanAndConnect() {
        val mgr = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        val adapter: BluetoothAdapter = mgr.adapter ?: run { vm.setStatus("No Bluetooth"); return }
        ble = BleClient(this, onBytes = { vm.onBytes(it) }, onState = { runOnUiThread { vm.setStatus(it) } })

        vm.setStatus("Scanning…")
        val scanner = adapter.bluetoothLeScanner
        val cb = object : android.bluetooth.le.ScanCallback() {
            override fun onScanResult(t: Int, r: android.bluetooth.le.ScanResult) {
                val dev: BluetoothDevice = r.device
                if (dev.name == BleClient.DEVICE_NAME) {
                    scanner.stopScan(this)
                    ble?.connect(dev)
                }
            }
        }
        scanner.startScan(cb)
    }
}
