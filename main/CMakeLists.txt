idf_component_register(SRCS "main.c"
                        INCLUDE_DIRS "."
                        REQUIRES led_strip esp_driver_twai esp_driver_spi esp_driver_gpio esp_timer)
