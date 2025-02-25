idf_component_register(SRCS "rc5_receiver.c"
                    INCLUDE_DIRS "." "../../main"
                    PRIV_REQUIRES esp_driver_rmt
                    REQUIRES driver
)
