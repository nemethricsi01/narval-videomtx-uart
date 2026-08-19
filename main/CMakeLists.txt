idf_component_register(
    SRCS
        "main.c"
        "drivers/lcd_st7789.c"
        "drivers/twai_can.c"
        "drivers/encoder.c"
        "drivers/ws2812.c"
        "drivers/video_uart.c"
        "services/display.c"
        "services/can.c"
        "services/can_latest.c"
        "services/can_monitor.c"
        "services/ui.c"
        "services/settings.c"
        "services/usb_bridge.c"
        "services/prog.c"
        "services/videomtx.c"
        "services/column_properties.c"
        "services/matrix_config.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_lcd driver esp_timer freertos esp_system esp_rom
        nvs_flash
        esp_driver_twai esp_driver_ledc esp_driver_gpio esp_driver_rmt esp_driver_uart
        knob button cherryusb)
