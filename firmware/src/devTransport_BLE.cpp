// =====================================================================
// Ghost RX transport: BLE GATT (Nordic UART Service)
// Streams reassembled CRSF telemetry frames to an Android phone as GATT
// notifications, and accepts commands (e.g. set passphrase) on the RX
// characteristic. Uses NimBLE-Arduino (already an ELRS ESP32 dependency).
//
// Service/char UUIDs = Nordic UART Service (NUS), so generic BLE-UART
// apps work too:
//   Service : 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
//   TX (notify, ESP -> phone): 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
//   RX (write,  phone -> ESP): 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
// =====================================================================
#if defined(GHOST_TRANSPORT_BLE) && defined(PLATFORM_ESP32)

#include <NimBLEDevice.h>
#include <string.h>

// NimBLE's os/endian.h defines htobe16/be16toh/htobe32/be32toh as MACROS. These
// collide with ExpressLRS crsf_protocol.h, which declares inline FUNCTIONS of the
// exact same names (pulled in transitively via sniffer.h). If the macros are still
// live when crsf_protocol.h is parsed, they expand over the function definitions
// ("expected ')' before '(' token"). We don't use these macros in this file, and
// NimBLE's own .cpp translation units are unaffected, so undefine them here.
#undef htobe16
#undef be16toh
#undef htobe32
#undef be32toh

#include "sniffer.h"   // Ghost_Transport_SendCRSF override + Sniffer_SetUidFromPhrase
#include "logging.h"   // DBGLN

#define NUS_SERVICE "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_CHAR "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // phone writes here
#define NUS_TX_CHAR "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // we notify here

static NimBLECharacteristic *s_txChar = nullptr;
static volatile bool s_connected = false;
static void BLE_sink(const uint8_t *frame, uint8_t len);   // fwd decl

// Phrase change deferred from the NimBLE host task to the loop task: applying it
// rewrites the FHSS hop table the radio ISR is reading. The callback only fills
// this buffer and sets the flag; BLE_Transport_Poll() applies it.
static char s_pendingPhrase[96] = {0};
static volatile bool s_pendingPhraseSet = false;

// ---- Commands from the phone (simple line/binary protocol) ----
//   "P:<phrase>\n"  -> set binding phrase (recompute UID live, applied in the loop task)
//   Binary 0xEC 0xAC 0x32 0x62 -> enter bind mode (Ghost RX style), etc.
class RxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *c) override {
        std::string v = c->getValue();
        // "P:" is off unless built with -D GHOST_BLE_ALLOW_PHRASE: this link is
        // unpaired and unencrypted, so anyone in range could read the phrase off
        // the air or retarget the sniffer. Use the serial "P:" command instead.
#if !defined(GHOST_BLE_ALLOW_PHRASE)
        if (v.rfind("P:", 0) == 0) { DBGLN("[TLM RX] BLE phrase write ignored (serial only)"); return; }
#endif
        if (v.rfind("P:", 0) == 0) {
            std::string phrase = v.substr(2);
            // strip trailing newline
            while (!phrase.empty() && (phrase.back() == '\n' || phrase.back() == '\r'))
                phrase.pop_back();
            if (s_pendingPhraseSet) {
                DBGLN("[TLM RX] phrase change already pending, ignoring");
            } else if (phrase.length() >= sizeof(s_pendingPhrase)) {
                DBGLN("[TLM RX] phrase too long, ignoring");   // never truncate: would bind the wrong UID
            } else {
                memcpy(s_pendingPhrase, phrase.c_str(), phrase.length() + 1);   // fill buffer BEFORE flag
                s_pendingPhraseSet = true;
            }
        }
        // extend here: BOOT / BIND / WIFI binary backdoor commands
    }
};

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *s) override    { s_connected = true;  DBGLN("[TLM RX] BLE client connected"); }
    void onDisconnect(NimBLEServer *s) override { s_connected = false; DBGLN("[TLM RX] BLE client disconnected, re-advertising"); NimBLEDevice::startAdvertising(); }
};

void BLE_Transport_Init(const char *deviceName)
{
    const char *name = deviceName ? deviceName : "ELRS TLM RX";
    DBGLN("[TLM RX] BLE init as '%s'", name);
    NimBLEDevice::init(name);
    NimBLEDevice::setMTU(247);  // fit a whole CRSF frame in one notify

    NimBLEServer *server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());

    NimBLEService *svc = server->createService(NUS_SERVICE);

    s_txChar = svc->createCharacteristic(NUS_TX_CHAR, NIMBLE_PROPERTY::NOTIFY);

    NimBLECharacteristic *rx =
        svc->createCharacteristic(NUS_RX_CHAR, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    rx->setCallbacks(new RxCallbacks());

    svc->start();

    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(NUS_SERVICE);
    adv->setName(name);          // ensure the name is in the scan response
    adv->setScanResponse(true);
    bool ok = NimBLEDevice::startAdvertising();
    DBGLN("[TLM RX] BLE advertising %s (NUS 6E400001-...)", ok ? "STARTED" : "FAILED");

    Ghost_Transport_Register(&BLE_sink);   // receive CRSF frames from Sniffer_Poll
}

// Loop task: apply a phrase change the write callback queued.
void BLE_Transport_Poll(void)
{
    if (!s_pendingPhraseSet) return;
    char phrase[sizeof(s_pendingPhrase)];
    memcpy(phrase, s_pendingPhrase, sizeof(phrase));
    s_pendingPhraseSet = false;   // callback may queue the next one from here on
    Sniffer_SetUidFromPhrase(phrase);
}

// ---- Sink: called (from loop context) for every completed CRSF frame ----
static void BLE_sink(const uint8_t *frame, uint8_t len)
{
    if (s_connected && s_txChar) {
        s_txChar->setValue(frame, len);
        s_txChar->notify();
    }
}

#endif // GHOST_TRANSPORT_BLE
