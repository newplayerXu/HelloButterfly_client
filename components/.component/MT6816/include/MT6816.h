#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif



#define MT6816_ON
#define PIN_NUM_MISO  18
#define PIN_NUM_MOSI  16
#define PIN_NUM_CLK   17
#define PIN_NUM_CS    15
#define PIN_NUM_CS2   35

extern spi_bus_config_t buscfg; // Moved definition to source file

typedef struct {
    spi_device_handle_t dev;
    int cs_gpio;
} mt6816_t;

extern mt6816_t enc; // Moved definition to source file

esp_err_t mt6816_init(mt6816_t *out, spi_host_device_t host, int cs_gpio, int clk_hz);
esp_err_t mt6816_read_angle14(mt6816_t *dev, uint16_t *angle14);
float mt6816_angle14_to_deg(uint16_t angle14);
esp_err_t mt6816_read_angle_deg(mt6816_t *dev, float *angle_deg);

#ifdef __cplusplus
}
#endif