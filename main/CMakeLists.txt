idf_component_register(SRCS "hello_world_main.c" "rc5_receiver.c"
                    PRIV_REQUIRES spi_flash esp_driver_rmt
                    INCLUDE_DIRS ""
                    REQUIRES driver)
