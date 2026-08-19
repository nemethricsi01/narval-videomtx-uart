#pragma once

// ---- LCD — ST7789 1.9" 170x320 (SPI2) ------------------------------------
#define BOARD_LCD_SPI_HOST        SPI2_HOST
#define BOARD_LCD_PIXEL_CLOCK_HZ  (80 * 1000 * 1000)
#define BOARD_PIN_LCD_SCLK        36
#define BOARD_PIN_LCD_MOSI        35
#define BOARD_PIN_LCD_DC          18
#define BOARD_PIN_LCD_RST         38
#define BOARD_PIN_LCD_CS           7
#define BOARD_PIN_LCD_BL           6
#define BOARD_LCD_BL_ON_LEVEL      1
#define BOARD_LCD_H_RES          170
#define BOARD_LCD_V_RES          320

// ---- DS3231 RTC (I2C) -----------------------------------------------------
#define BOARD_I2C_PORT            I2C_NUM_0
#define BOARD_PIN_RTC_SDA          5
#define BOARD_PIN_RTC_SCL          4

// ---- WS2812 RGB LED -------------------------------------------------------
#define BOARD_PIN_WS2812          48

// ---- W5500 Ethernet (SPI3) ------------------------------------------------
#define BOARD_ETH_SPI_HOST        SPI3_HOST
#define BOARD_ETH_CLOCK_HZ        (25 * 1000 * 1000)
#define BOARD_PIN_ETH_MOSI        11
#define BOARD_PIN_ETH_MISO        13
#define BOARD_PIN_ETH_SCLK        12
#define BOARD_PIN_ETH_CS          10
#define BOARD_PIN_ETH_RST          9
// Locally-administered MAC (W5500 modules often ship without a factory MAC)
#define BOARD_ETH_MAC             {0x02, 0xAB, 0xCD, 0x12, 0x34, 0x56}

// ---- TWAI (CAN) -----------------------------------------------------------
#define BOARD_PIN_CAN_RX          15
#define BOARD_PIN_CAN_TX          16

// ---- DS3231 SQW (1 Hz interrupt) ------------------------------------------
#define BOARD_PIN_RTC_SQW  17

// ---- Video routing UART output ---------------------------------------------
// TX-only, one frame per crosspoint change: [0x55][addr][channel][0xAA].
// UART1 chosen deliberately (not UART0) — this project doesn't open any
// UART itself, but UART0 may be claimed by the system console depending on
// sdkconfig, so UART1 avoids that ambiguity entirely.
#define BOARD_VIDEO_UART_PORT      UART_NUM_1
#define BOARD_VIDEO_UART_BAUD      115200  // TODO: confirm against whatever's listening
#define BOARD_PIN_VIDEO_UART_TX    39

// ---- Rotary encoder -------------------------------------------------------
#define BOARD_PIN_ENC_A           40
#define BOARD_PIN_ENC_B           41
#define BOARD_PIN_ENC_BTN         42
#define BOARD_ENC_BTN_ACTIVE_LVL   0  // active-low (pulled up, closes to GND)

// ---- Debug switch ---------------------------------------------------------
// Comment out to disable the USB CDC bridge and free the USB OTG peripheral
// for the built-in USB-Serial/JTAG debug interface.
#define USB_BRIDGE_ENABLED
