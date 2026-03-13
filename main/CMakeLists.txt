idf_component_register(SRCS "spp_client_demo.c"
                    PRIV_REQUIRES bt esp_driver_uart nvs_flash
                    INCLUDE_DIRS "."
                    REQUIRES MT6816
                            esp_driver_gpio
                            esp_driver_ledc
                    )
