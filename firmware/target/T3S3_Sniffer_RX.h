#pragma once
// =====================================================================
// LILYGO T3-S3 (ESP32-S3 + SX1280 2.4GHz) - STATIC pin target header
// for the ELRS Telemetry Sniffer (Ghost RX).
//
// This is the "no runtime layout" fallback. The RECOMMENDED path on
// ELRS 3.5.x ESP32 is the Unified firmware + a hardware-layout JSON
// (firmware/hardware/lilygo_t3s3_sniffer_rx.json). Use this header only
// if you want a fully self-contained compile-time build; to do so,
// replace `-include target/Unified_ESP_RX.h` in the PlatformIO env
// with `-include target/T3S3_Sniffer_RX.h` and remove the layout upload.
//
// >>> VERIFY the RF-switch pins (RX_ENABLE/TX_ENABLE) against YOUR board
// >>> revision. Wrong values = a deaf receiver.
// =====================================================================

#define DEVICE_NAME "T3S3 Ghost RX"

// ---- SX1280 SPI + control ----
#define GPIO_PIN_SCK        5
#define GPIO_PIN_MISO       3
#define GPIO_PIN_MOSI       6
#define GPIO_PIN_NSS        7
#define GPIO_PIN_RST        8
#define GPIO_PIN_BUSY       36
#define GPIO_PIN_DIO1       9

// ---- Front-end / PA RF switch (T3-S3 rev 1.1/1.2 SX1280 PA) ----
// RXEN=21, TXEN=10. ELRS drives RXEN HIGH during receive; without these the
// LNA/RX path is never enabled and the radio is deaf. (Bare-SX1280 v1.0 with
// no FEM: set both to UNDEF_PIN — override with -D GPIO_PIN_RX_ENABLE=UNDEF_PIN
// / -D GPIO_PIN_TX_ENABLE=UNDEF_PIN in the env, e.g. the non-PA V1 board.)
#ifndef GPIO_PIN_RX_ENABLE
#define GPIO_PIN_RX_ENABLE  21
#endif
#ifndef GPIO_PIN_TX_ENABLE
#define GPIO_PIN_TX_ENABLE  10
#endif

// ---- UART (USB-CDC is primary on S3; these are the header pins) ----
#define GPIO_PIN_RCSIGNAL_RX 44
#define GPIO_PIN_RCSIGNAL_TX 43

// ---- 0.96" SSD1306 I2C OLED ----
#define GPIO_PIN_OLED_SDA   18
#define GPIO_PIN_OLED_SCK   17   // aka SCL
#define OLED_I2C_ADDR       0x3C

// ---- Status LED ----
#define GPIO_PIN_LED        37   // adjust per board rev

// ---- Power table (SX1280). RX-only, but ELRS wants bounds. ----
#define MinPower            PWR_10mW
#define DefaultPower        PWR_10mW
#define MaxPower            PWR_25mW
