#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif



#define MT6816_ON
#define PIN_NUM_MISO  15
#define PIN_NUM_MISO2  6
#define PIN_NUM_MISO3  17
#define PIN_NUM_MISO4  8
#define PIN_NUM_MOSI  5
#define PIN_NUM_CLK   7

#define PIN_NUM_CS1   18
// #define PIN_NUM_CS2   6
// #define PIN_NUM_CS3   17
// #define PIN_NUM_CS4   8

extern spi_bus_config_t buscfg; // Moved definition to source file
extern spi_bus_config_t buscfg2; // Second bus for alternative MISO

typedef struct {
    spi_device_handle_t dev;
    int cs_gpio;
    spi_host_device_t host;
    int clk_hz;
    int miso_gpio;
} mt6816_t;

extern mt6816_t enc;  // Primary encoder handle
extern mt6816_t enc2; // Secondary encoder handle

esp_err_t mt6816_init(mt6816_t *out, spi_host_device_t host, int cs_gpio, int clk_hz, int miso_gpio);
esp_err_t mt6816_select_miso(mt6816_t *dev, int miso_gpio);
esp_err_t mt6816_read_angle14(mt6816_t *dev, uint16_t *angle14);
float mt6816_angle14_to_deg(uint16_t angle14);
esp_err_t mt6816_read_angle_deg(mt6816_t *dev, float *angle_deg);
esp_err_t mt6816_read_angle_deg_dev(mt6816_t *dev, int dev_index, float *angle_deg);

#ifdef __cplusplus
}
#endif