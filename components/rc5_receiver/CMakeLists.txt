idf_component_register(SRCS "rc5_receiver.c"
                    INCLUDE_DIRS "." "../../main"
                    PRIV_REQUIRES esp_driver_rmt
                    REQUIRES driver
)

# Pass the version to the build system
set(RC5_RECEIVER_VERSION "${CONFIG_RC5_RECEIVER_VERSION}")

message(STATUS "RC5_RECEIVER_VERSION: ${RC5_RECEIVER_VERSION}")
