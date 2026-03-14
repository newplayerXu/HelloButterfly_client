#include "MT6816.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <string.h>
#include <stdbool.h>

#define MT6816_REG_ANGLE_13_6   0x03
#define MT6816_REG_ANGLE_5_0_PC 0x04
#define MT6816_FLAG_NO_MAG      (1u << 1)

static const char *TAG = "MT6816_STD";

spi_bus_config_t buscfg = {
    .miso_io_num = PIN_NUM_MISO,
    .mosi_io_num = PIN_NUM_MOSI,
    .sclk_io_num = PIN_NUM_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 2,
};

mt6816_t enc = {0};

static void mt6816_log_idle_levels(int active_cs_gpio)
{
    if (active_cs_gpio < 0) {
        // ESP_LOGW(TAG, "CS GPIO not configured yet");
        return;
    }
    #ifdef DEBUG
    int cs1 = gpio_get_level((gpio_num_t)active_cs_gpio);
    int cs2 = gpio_get_level((gpio_num_t)PIN_NUM_CS2);
    int sclk = gpio_get_level((gpio_num_t)PIN_NUM_CLK);
    int mosi = gpio_get_level((gpio_num_t)PIN_NUM_MOSI);
    int miso = gpio_get_level((gpio_num_t)PIN_NUM_MISO);
    #endif
    // ESP_LOGI(TAG, "Idle levels: CS(%d)=%d CS2(%d)=%d SCLK=%d MOSI=%d MISO=%d",
    //          active_cs_gpio, cs, PIN_NUM_CS2, cs2, sclk, mosi, miso);
}

static inline void mt6816_set_other_cs_high(int active_cs_gpio)
{
    if (PIN_NUM_CS1 >= 0 && PIN_NUM_CS1 != active_cs_gpio) {
        gpio_set_level((gpio_num_t)PIN_NUM_CS1, 1);
    }
    if (PIN_NUM_CS2 >= 0 && PIN_NUM_CS2 != active_cs_gpio) {
        gpio_set_level((gpio_num_t)PIN_NUM_CS2, 1);
    }
    if (PIN_NUM_CS3 >= 0 && PIN_NUM_CS3 != active_cs_gpio) {
        gpio_set_level((gpio_num_t)PIN_NUM_CS3, 1);
    }
    if (PIN_NUM_CS4 >= 0 && PIN_NUM_CS4 != active_cs_gpio) {
        gpio_set_level((gpio_num_t)PIN_NUM_CS4, 1);
    }
}

static inline int mt6816_cs_from_index(int dev_index)
{
    switch (dev_index) {
    case 1:
        return PIN_NUM_CS1;
    case 2:
        return PIN_NUM_CS2;
    case 3:
        return PIN_NUM_CS3;
    case 4:
        return PIN_NUM_CS4;
    default:
        return -1;
    }
}

static inline void mt6816_config_cs_pin(int gpio)
{
    if (gpio < 0) return;

    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&conf));
    gpio_set_level((gpio_num_t)gpio, 1);
}

// STM32 working impl: tx word = ((0x80 | addr) << 8) | 0x00
static inline uint16_t mt6816_make_cmd_read_word(uint8_t addr)
{
    return (uint16_t)(((uint16_t)(0x80u | (addr & 0x7Fu))) << 8);
}

// static esp_err_t mt6816_xfer16(spi_device_handle_t dev, uint16_t txw, uint8_t rx[2])
// {
//     uint8_t tx[2] = { (uint8_t)(txw >> 8), (uint8_t)(txw & 0xFF) };

//     spi_transaction_t t = {0};
//     t.length = 16;
//     t.tx_buffer = tx;
//     t.rx_buffer = rx;

//     return spi_device_transmit(dev, &t);
// }

static esp_err_t mt6816_read_reg8(mt6816_t *dev, uint8_t addr, uint8_t *out)
{
    if (!dev || !dev->dev || !out) return ESP_ERR_INVALID_ARG;
    if (dev->cs_gpio < 0) {
        ESP_LOGW(TAG, "cs_gpio invalid: %d", dev->cs_gpio);
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t cmd = mt6816_make_cmd_read_word(addr);
    uint8_t tx[2] = {
        (uint8_t)(cmd >> 8),
        (uint8_t)(cmd & 0xFF),
    };
    uint8_t rx[2] = {0};

    spi_transaction_t t = {0};
    t.length = 16;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    // Match STM32 flow: manual CS low/high per 16-bit transaction.
    mt6816_set_other_cs_high(dev->cs_gpio);
    gpio_set_level((gpio_num_t)dev->cs_gpio, 0);
    esp_err_t err = spi_device_transmit(dev->dev, &t);
    gpio_set_level((gpio_num_t)dev->cs_gpio, 1);
    mt6816_set_other_cs_high(dev->cs_gpio);
    if (err != ESP_OK) return err;

    // On ESP-IDF byte-buffer transfer, register payload is in the second byte.
    *out = rx[1];
    return ESP_OK;
}

static bool mt6816_even_parity16(uint16_t x)
{
    // true if number of 1s is even
    x ^= x >> 8;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    return ((x & 1u) == 0);
}

esp_err_t mt6816_init(mt6816_t *out, spi_host_device_t host, int cs_gpio, int clk_hz)
{
    if (!out) return ESP_ERR_INVALID_ARG;

    out->cs_gpio = cs_gpio;
    mt6816_config_cs_pin(PIN_NUM_CS1);
    mt6816_config_cs_pin(PIN_NUM_CS2);
    mt6816_config_cs_pin(PIN_NUM_CS3);
    mt6816_config_cs_pin(PIN_NUM_CS4);

    mt6816_set_other_cs_high(cs_gpio);

    mt6816_log_idle_levels(cs_gpio);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = clk_hz,   // 建议先 100k~500k，稳定后再升
        .mode = 3,                  // Try mode-1 first (valid range is 0..3)
        .spics_io_num = -1,         // Use software-controlled CS to match STM32 behavior
        .queue_size = 1,
        .flags = 0,
        // 如果你线比较长/模块慢，可加一点 CS 前后延时（单位：SPI bit 周期）
        // .cs_ena_pretrans = 1,
        // .cs_ena_posttrans = 1,
    };

    spi_device_handle_t dev = NULL;
    esp_err_t err = spi_bus_add_device(host, &devcfg, &dev);
    if (err != ESP_OK) return err;

    out->dev = dev;

    mt6816_log_idle_levels(cs_gpio);

    return ESP_OK;
}

esp_err_t mt6816_read_angle14(mt6816_t *dev, uint16_t *angle14)
{
    if (!dev || !dev->dev || !angle14) return ESP_ERR_INVALID_ARG;

    bool no_mag_seen = false;

    // like your STM32: try up to 3 times until parity OK
    for (int i = 0; i < 3; i++) {
        uint8_t r03 = 0, r04 = 0;

        esp_err_t err = mt6816_read_reg8(dev, MT6816_REG_ANGLE_13_6, &r03);
        if (err != ESP_OK) return err;

        err = mt6816_read_reg8(dev, MT6816_REG_ANGLE_5_0_PC, &r04);
        if (err != ESP_OK) return err;

        uint16_t sample = ((uint16_t)r03 << 8) | (uint16_t)r04;

        if (mt6816_even_parity16(sample)) {
            if ((sample & MT6816_FLAG_NO_MAG) != 0u) {
                no_mag_seen = true;
                continue;
            }

            *angle14 = (sample >> 2) & 0x3FFF;
            return ESP_OK;
        }
    }

    if (no_mag_seen) return ESP_ERR_INVALID_STATE; // parity ok but no magnet detected

    return ESP_ERR_INVALID_CRC; // parity failed
}

float mt6816_angle14_to_deg(uint16_t angle14)
{
    // 14-bit full scale is 16384 counts per mechanical revolution.
    uint16_t a = (uint16_t)(angle14 & 0x3FFFu);
    return ((float)a * 360.0f) / 16384.0f;
}

esp_err_t mt6816_read_angle_deg(mt6816_t *dev, float *angle_deg)
{
    if (!angle_deg) return ESP_ERR_INVALID_ARG;

    uint16_t a14 = 0;
    esp_err_t err = mt6816_read_angle14(dev, &a14);
    if (err != ESP_OK) return err;

    *angle_deg = mt6816_angle14_to_deg(a14);
    return ESP_OK;
}

esp_err_t mt6816_read_angle_deg_dev(mt6816_t *dev, int dev_index, float *angle_deg)
{
    if (!dev || !angle_deg) return ESP_ERR_INVALID_ARG;

    int cs_gpio = mt6816_cs_from_index(dev_index);
    if (cs_gpio < 0) return ESP_ERR_INVALID_ARG;

    int prev_cs = dev->cs_gpio;
    dev->cs_gpio = cs_gpio;

    esp_err_t err = mt6816_read_angle_deg(dev, angle_deg);

    dev->cs_gpio = prev_cs;
    return err;
}