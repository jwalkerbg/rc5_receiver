idf_component_register(SRCS "hello_world_main.c"
                    PRIV_REQUIRES spi_flash esp_driver_rmt
                    INCLUDE_DIRS ""
                    REQUIRES driver)
