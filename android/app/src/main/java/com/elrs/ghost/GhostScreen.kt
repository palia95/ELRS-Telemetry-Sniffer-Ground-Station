package com.elrs.ghost

import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.unit.dp

@Composable
fun GhostScreen(
    vm: TelemetryViewModel,
    onConnect: () -> Unit,
    onPhrase: (String) -> Unit
) {
    var phrase by remember { mutableStateOf("") }
    Column(
        Modifier.fillMaxSize().padding(16.dp).verticalScroll(rememberScrollState()),
        verticalArrangement = Arrangement.spacedBy(12.dp)
    ) {
        Text("ELRS Ghost Telemetry", style = MaterialTheme.typography.headlineSmall)
        Text("Status: ${vm.status}", style = MaterialTheme.typography.bodyMedium)

        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Button(onClick = onConnect) { Text("Scan & Connect") }
        }

        OutlinedTextField(
            value = phrase, onValueChange = { phrase = it },
            label = { Text("Binding phrase") }, singleLine = true,
            modifier = Modifier.fillMaxWidth()
        )
        Button(onClick = { onPhrase(phrase) }, enabled = phrase.isNotBlank()) {
            Text("Send phrase to sniffer")
        }

        Divider()

        Card(Modifier.fillMaxWidth()) {
            Column(Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                Text("Battery", style = MaterialTheme.typography.titleMedium)
                vm.battery?.let {
                    Text("${it.voltage} V   ${it.current} A   ${it.capacityMah} mAh   ${it.remainingPct}%",
                        fontFamily = FontFamily.Monospace)
                } ?: Text("—")
            }
        }

        Card(Modifier.fillMaxWidth()) {
            Column(Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                Text("GPS", style = MaterialTheme.typography.titleMedium)
                vm.gps?.let {
                    Text("lat ${it.latDeg}  lon ${it.lonDeg}", fontFamily = FontFamily.Monospace)
                    Text("alt ${it.altitudeM} m  spd ${it.groundSpeedKmh} km/h  sats ${it.satellites}",
                        fontFamily = FontFamily.Monospace)
                } ?: Text("—")
            }
        }

        Card(Modifier.fillMaxWidth()) {
            Column(Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                Text("Attitude / Mode", style = MaterialTheme.typography.titleMedium)
                vm.attitude?.let {
                    Text("pitch ${"%.1f".format(it.pitchDeg)}  roll ${"%.1f".format(it.rollDeg)}  yaw ${"%.1f".format(it.yawDeg)}",
                        fontFamily = FontFamily.Monospace)
                } ?: Text("—")
                Text("mode: ${vm.flightMode.ifBlank { "—" }}")
                vm.baroAltM?.let { Text("baro alt ${"%.1f".format(it)} m") }
            }
        }

        Card(Modifier.fillMaxWidth()) {
            Column(Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                Text("Link", style = MaterialTheme.typography.titleMedium)
                vm.link?.let {
                    Text("RSSI ${it.upRssiAnt1}/${it.upRssiAnt2} dBm  LQ ${it.upLinkQuality}  SNR ${it.upSnr}",
                        fontFamily = FontFamily.Monospace)
                } ?: Text("—")
            }
        }
    }
}
