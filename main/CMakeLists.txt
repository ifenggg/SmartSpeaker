idf_component_register(SRCS "amp.c" "audio_vol.c" "ble_client.c" "ble_debug.c" "adc.c" "sd.c" "uart.c" "led_strip.c" "ble.c" "bt_app_av.c"
                            "bt_app_core.c"
                            "main.c"
                            "bt_a2dp.c"
                            "led_strip.c"
                            "uart.c"
                            "adc.c"
                    PRIV_REQUIRES esp_driver_i2s bt nvs_flash esp_ringbuf esp_driver_dac driver esp_adc fatfs sdmmc
                    INCLUDE_DIRS ".")
