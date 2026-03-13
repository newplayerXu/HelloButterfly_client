/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/****************************************************************************
*
* This file is for ble spp client demo.
*
****************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "driver/uart.h"

#include "esp_bt.h"
#include "nvs_flash.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_system.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"

#define DEBUG
/*--------------------------------------*/
#include "MT6816.h"

#define MT6816_CS1_PIN GPIO_NUM_15
#define MT6816_CS2_PIN GPIO_NUM_6
#define MT6816_CS3_PIN GPIO_NUM_17
#define MT6816_CS4_PIN GPIO_NUM_8

#define MT6816_MISO_PIN GPIO_NUM_18
#define MT6816_MOSI_PIN GPIO_NUM_5
#define MT6816_SCK_PIN GPIO_NUM_7
/*--------------------------------------*/
#include "driver/ledc.h"
#include "driver/gpio.h"

#define _1_PWMA GPIO_NUM_10
#define _1_AIN2 GPIO_NUM_11
#define _1_AIN1 GPIO_NUM_12
#define _1_PWMB GPIO_NUM_3
#define _1_BIN2 GPIO_NUM_46
#define _1_BIN1 GPIO_NUM_9

#define _2_PWMA GPIO_NUM_13
#define _2_AIN2 GPIO_NUM_14
#define _2_AIN1 GPIO_NUM_21
#define _2_PWMB GPIO_NUM_45
#define _2_BIN2 GPIO_NUM_48
#define _2_BIN1 GPIO_NUM_47

//------------------------------电机引脚配置------------------------------//
void gpio_init(void)
{
    // gpio_set_direction(_1_PWMA, GPIO_MODE_OUTPUT);//配置左侧两电机为输出模式
    // gpio_set_direction(_1_PWMB, GPIO_MODE_OUTPUT);
    gpio_set_direction(_1_AIN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(_1_AIN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(_1_BIN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(_1_BIN1, GPIO_MODE_OUTPUT);

    // gpio_set_direction(_2_PWMA, GPIO_MODE_OUTPUT);//配置右侧两电机为输出模式
    // gpio_set_direction(_2_PWMB, GPIO_MODE_OUTPUT);
    gpio_set_direction(_2_AIN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(_2_AIN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(_2_BIN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(_2_BIN1, GPIO_MODE_OUTPUT);

    //------------------------------配置定时器和通道------------------------------//
    gpio_config_t motor_cfg = {
        //.pin_bit_mask = (1ULL << _2A_LED_GPIO),
        .pin_bit_mask = (1ULL << _1_PWMA) | (1ULL << _1_PWMB) |
                        (1ULL << _2_PWMA) | (1ULL << _2_PWMB) |
                        (1ULL << _1_AIN2) | (1ULL << _1_AIN1) |
                        (1ULL << _1_BIN2) | (1ULL << _1_BIN1) |
                        (1ULL << _2_AIN2) | (1ULL << _2_AIN1) |
                        (1ULL << _2_BIN2) | (1ULL << _2_BIN1),

        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&motor_cfg);

    ledc_timer_config_t motor_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
        .freq_hz = 500,
        .duty_resolution = LEDC_TIMER_13_BIT,

    };
    ledc_timer_config(&motor_timer);
    // 配置4个独立的PWM通道
    // 通道0: 左侧A电机
    ledc_channel_config_t channel_0 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .gpio_num = _1_PWMA,
        .duty = 0,
        .intr_type = LEDC_INTR_DISABLE,
    };
    ledc_channel_config(&channel_0);
    // 通道1: 左侧B电机
    ledc_channel_config_t channel_1 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0,
        .gpio_num = _1_PWMB,
        .duty = 0,
        .intr_type = LEDC_INTR_DISABLE,
    };
    ledc_channel_config(&channel_1);

    // 通道2: 右侧A电机
    ledc_channel_config_t channel_2 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .timer_sel = LEDC_TIMER_0,
        .gpio_num = _2_PWMA,
        .duty = 0,
        .intr_type = LEDC_INTR_DISABLE,
    };
    ledc_channel_config(&channel_2);

    // 通道3: 右侧B电机
    ledc_channel_config_t channel_3 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_3,
        .timer_sel = LEDC_TIMER_0,
        .gpio_num = _2_PWMB,
        .duty = 0,
        .intr_type = LEDC_INTR_DISABLE,
    };
    ledc_channel_config(&channel_3);

    // ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 2192); // 25%初始占空比
    // ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    ledc_fade_func_install(0); // 开启硬件pwm
}
//------------------------------电机控制逻辑------------------------------//
void motor_speed_lr(uint8_t lr, uint8_t motor, uint32_t speed, uint8_t dir)
// lr:1-左侧，2-右侧 motor:
// motor:1-A电机，2-B电机；
// speed：0-8191
// dir:1-正转，2-反转
{
    ledc_channel_t channel=0;

    if (lr == 1)
    { // 左侧
        channel = (motor == 1) ? LEDC_CHANNEL_0 : LEDC_CHANNEL_1;
    }
    else if(lr == 2)
    { // 右侧
        channel = (motor == 1) ? LEDC_CHANNEL_2 : LEDC_CHANNEL_3;
    }
    if (lr == 1)
    {
        if (motor == 1)
        {
            if (dir == 1)
            {
                gpio_set_level(_1_AIN2, 1);
                gpio_set_level(_1_AIN1, 0);
            }
            else if(dir == 2)
            {
                gpio_set_level(_1_AIN2, 0);
                gpio_set_level(_1_AIN1, 1);
            }
        }
        else if (motor == 2)
        {
            if (dir == 1)
            {
                gpio_set_level(_1_BIN2, 1);
                gpio_set_level(_1_BIN1, 0);
            }
            else if(dir == 2)
            {
                gpio_set_level(_1_BIN2, 0);
                gpio_set_level(_1_BIN1, 1);
            }
        }
    }else if (lr == 2)
    {
        if (motor == 1)
        {
            if (dir == 1)
            {
                gpio_set_level(_2_AIN2, 1);
                gpio_set_level(_2_AIN1, 0);
            }
            else if(dir == 2)
            {
                gpio_set_level(_2_AIN2, 0);
                gpio_set_level(_2_AIN1, 1);
            }
        }
        else if (motor == 2)
        {
            if (dir == 1)
            {
                gpio_set_level(_2_BIN2, 1);
                gpio_set_level(_2_BIN1, 0);
            }
            else if(dir == 2)
            {
                gpio_set_level(_2_BIN2, 0);
                gpio_set_level(_2_BIN1, 1);
            }
        }
    }
    if(speed > 8191)
    {
        speed = 8191;
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, speed);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
    // ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

#define GATTC_TAG                   "GATTC_SPP_DEMO"
#define PROFILE_NUM                 1
#define PROFILE_APP_ID              0
#define ESP_GATT_SPP_SERVICE_UUID   0xABF0
#define SCAN_ALL_THE_TIME           0
#define SPP_GATT_MTU_SIZE           (512)

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

enum{
    SPP_IDX_SVC,

    SPP_IDX_SPP_DATA_RECV_VAL,

    SPP_IDX_SPP_DATA_NTY_VAL,
    SPP_IDX_SPP_DATA_NTF_CFG,

    SPP_IDX_SPP_COMMAND_VAL,

    SPP_IDX_SPP_STATUS_VAL,
    SPP_IDX_SPP_STATUS_CFG,

#ifdef SUPPORT_HEARTBEAT
    SPP_IDX_SPP_HEARTBEAT_VAL,
    SPP_IDX_SPP_HEARTBEAT_CFG,
#endif

    SPP_IDX_NB,
};

///Declare static functions
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

 static const char device_name[] = "ESP_SPP_SERVER";
static bool is_connect = false;
static uint16_t spp_conn_id = 0;
static uint16_t spp_mtu_size = 23;
static uint16_t cmd = 0;
static uint16_t spp_srv_start_handle = 0;
static uint16_t spp_srv_end_handle = 0;
static uint16_t spp_gattc_if = 0xff;
static uint16_t count = SPP_IDX_NB;
static esp_gattc_db_elem_t *db = NULL;
static QueueHandle_t cmd_reg_queue = NULL;
QueueHandle_t spp_uart_queue = NULL;
static bool connect = false;
static char * notify_value_p = NULL;
static int notify_value_offset = 0;
static int notify_value_count = 0;
static bool start = false;
static uint64_t notify_len = 0;
static uint64_t start_time = 0;
static uint64_t current_time = 0;

/*-----------------------------------------------*/
#define RC_FRAME_HEADER                  0xA5
#define RC_FRAME_TAIL                    0x5A
#define RC_FRAME_LEN                     11
#define RC_NOTIFY_MAX_LEN                32
#define RC_RX_QUEUE_LEN                  16
#define RC_CTRL_QUEUE_LEN                1
#define RC_LINK_TIMEOUT_US               (300000)
#define RC_NEUTRAL_US                    1500
#define RC_HALF_RANGE_US                 500
#define MOTOR_PWM_MAX_DUTY               8191
#define RC_DEBUG_LOG_RAW                 1
#define ANGLE_CTRL_KP_DUTY_PER_DEG       45.0f
#define ANGLE_CTRL_DEADBAND_DEG          1.5f
#define ANGLE_CTRL_LOG_PERIOD_US         (200000)

typedef struct {
    uint16_t len;
    uint8_t data[RC_NOTIFY_MAX_LEN];
} rc_rx_packet_t;

typedef struct {
    uint8_t seq;
    uint16_t ch1;
    uint16_t ch2;
    uint16_t ch3;
    uint8_t flags;
    uint64_t rx_timestamp_us;
} rc_cmd_t;

static QueueHandle_t rc_rx_queue = NULL;
static QueueHandle_t rc_ctrl_queue = NULL;
static uint64_t g_last_rx_us = 0;
static bool g_mt6816_ready = false;

/*-----------------------------------------------*/
#ifdef SUPPORT_HEARTBEAT
static uint8_t  heartbeat_s[9] = {'E','s','p','r','e','s','s','i','f'};
static QueueHandle_t cmd_heartbeat_queue = NULL;
#endif

static esp_bt_uuid_t spp_service_uuid = {
    .len  = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_SPP_SERVICE_UUID,},
};

/*-----------------------------------------------*/
static uint8_t rc_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static void motor_stop_all(void)
{
    motor_speed_lr(1, 1, 0, 1);
    motor_speed_lr(1, 2, 0, 1);
    motor_speed_lr(2, 1, 0, 1);
    motor_speed_lr(2, 2, 0, 1);
}

static inline int16_t rc_centered(uint16_t ch)
{
    int32_t v = (int32_t)ch - RC_NEUTRAL_US;
    if (v > RC_HALF_RANGE_US) {
        v = RC_HALF_RANGE_US;
    } else if (v < -RC_HALF_RANGE_US) {
        v = -RC_HALF_RANGE_US;
    }
    return (int16_t)v;
}

static uint32_t rc_to_pwm(int16_t v)
{
    uint32_t mag = (uint32_t)(v >= 0 ? v : -v);
    return (mag * MOTOR_PWM_MAX_DUTY) / RC_HALF_RANGE_US;
}

static float rc_ch_to_target_deg(uint16_t ch)
{
    if (ch < 1000) {
        ch = 1000;
    } else if (ch > 2000) {
        ch = 2000;
    }
    return ((float)(ch - 1000) * 360.0f) / 1000.0f;
}

static float angle_error_deg(float target_deg, float current_deg)
{
    float err = target_deg - current_deg;
    while (err > 180.0f) {
        err -= 360.0f;
    }
    while (err < -180.0f) {
        err += 360.0f;
    }
    return err;
}

static bool enqueue_notify_packet(const uint8_t *data, uint16_t len)
{
    if (rc_rx_queue == NULL || data == NULL || len == 0 || len > RC_NOTIFY_MAX_LEN) {
        #ifdef DEBUG    
        ESP_LOGW(GATTC_TAG, "Drop notify packet, queue=%p len=%u", rc_rx_queue, len);
        #endif
        return false;
    }

    rc_rx_packet_t pkt = {
        .len = len,
    };
    memcpy(pkt.data, data, len);

#if RC_DEBUG_LOG_RAW
    ESP_LOGI(GATTC_TAG, "RX notify len=%u", len);
    ESP_LOG_BUFFER_HEX_LEVEL(GATTC_TAG, pkt.data, len, ESP_LOG_INFO);
#endif

    g_last_rx_us = esp_timer_get_time();
    if (xQueueSend(rc_rx_queue, &pkt, 0) == pdPASS) {
        return true;
    }

    rc_rx_packet_t drop_pkt;
    (void)xQueueReceive(rc_rx_queue, &drop_pkt, 0);
    return xQueueSend(rc_rx_queue, &pkt, 0) == pdPASS;
}

static bool parse_rc_frame(const uint8_t *buf, uint16_t len, rc_cmd_t *cmd)
{
    if (buf == NULL || cmd == NULL || len != RC_FRAME_LEN) {
        #ifdef DEBUG
        ESP_LOGW(GATTC_TAG, "Invalid RC frame len=%u", len);
        #endif
        return false;
    }
    if (buf[0] != RC_FRAME_HEADER || buf[10] != RC_FRAME_TAIL) {
        #ifdef DEBUG
        ESP_LOGW(GATTC_TAG, "Invalid RC frame header/tail: %02X ... %02X", buf[0], buf[10]);
        #endif
        return false;
    }

    const uint8_t calc_crc = rc_crc8(&buf[1], 7);
    if (calc_crc != buf[8]) {
        #ifdef DEBUG
        ESP_LOGW(GATTC_TAG, "Invalid RC frame crc calc=%02X recv=%02X", calc_crc, buf[8]);
        #endif
        return false;
    }

    cmd->seq = buf[1];
    cmd->ch1 = (uint16_t)(buf[2] | (buf[3] << 8));
    cmd->ch2 = (uint16_t)(buf[4] | (buf[5] << 8));
    cmd->ch3 = (uint16_t)(buf[6] | (buf[7] << 8));
    cmd->flags = buf[9];
    cmd->rx_timestamp_us = esp_timer_get_time();
    return true;
}

static void rc_parse_task(void *arg)
{
    rc_rx_packet_t pkt;
    rc_cmd_t cmd;

    for (;;) {
        if (xQueueReceive(rc_rx_queue, &pkt, portMAX_DELAY) != pdPASS) {
            continue;
        }
        if (!parse_rc_frame(pkt.data, pkt.len, &cmd)) {
            continue;
        }

        ESP_LOGI(GATTC_TAG, "RC seq=%u ch1=%u ch2=%u ch3=%u flags=0x%02X",
                 cmd.seq, cmd.ch1, cmd.ch2, cmd.ch3, cmd.flags);

        (void)xQueueOverwrite(rc_ctrl_queue, &cmd);
    }
}

static void rc_control_task(void *arg)
{
    rc_cmd_t cmd = {0};
    const TickType_t loop_ticks = pdMS_TO_TICKS(10);
    uint64_t last_log_us = 0;

    for (;;) {
        (void)xQueueReceive(rc_ctrl_queue, &cmd, 0);

        const uint64_t now_us = esp_timer_get_time();
        if ((now_us - g_last_rx_us) > RC_LINK_TIMEOUT_US) {
            motor_stop_all();
            vTaskDelay(loop_ticks);
            continue;
        }

        if (!g_mt6816_ready) {
            motor_stop_all();
            vTaskDelay(loop_ticks);
            continue;
        }

        float current_deg = 0.0f;
        esp_err_t angle_err = mt6816_read_angle_deg(&enc, &current_deg);
        if (angle_err != ESP_OK) {
            ESP_LOGW(GATTC_TAG, "MT6816 read failed: %s", esp_err_to_name(angle_err));
            motor_stop_all();
            vTaskDelay(loop_ticks);
            continue;
        }

        const float target_deg = rc_ch_to_target_deg(cmd.ch1);
        const float err_deg = angle_error_deg(target_deg, current_deg);
        const float abs_err_deg = (err_deg >= 0.0f) ? err_deg : -err_deg;

        uint32_t pwm = 0;
        uint8_t dir = 1;
        if (abs_err_deg >= ANGLE_CTRL_DEADBAND_DEG) {
            float duty_f = abs_err_deg * ANGLE_CTRL_KP_DUTY_PER_DEG;
            if (duty_f > (float)MOTOR_PWM_MAX_DUTY) {
                duty_f = (float)MOTOR_PWM_MAX_DUTY;
            }
            pwm = (uint32_t)duty_f;
            dir = (err_deg >= 0.0f) ? 1 : 2;
        }

        motor_speed_lr(1, 1, pwm, dir);
        motor_speed_lr(1, 2, pwm, dir);
        motor_speed_lr(2, 1, pwm, dir);
        motor_speed_lr(2, 2, pwm, dir);

        uint64_t now_us = esp_timer_get_time();
        if ((now_us - last_log_us) >= ANGLE_CTRL_LOG_PERIOD_US) {
            last_log_us = now_us;
            ESP_LOGI(GATTC_TAG, "ANGLE target=%.2f current=%.2f err=%.2f pwm=%u dir=%u",
                     target_deg, current_deg, err_deg, pwm, dir);
        }

        vTaskDelay(loop_ticks);
    }
}

/*-----------------------------------------------*/
static void notify_event_handler(esp_ble_gattc_cb_param_t * p_data)
{
    uint8_t handle = 0;

    handle = p_data->notify.handle;
    if (db == NULL) {
        ESP_LOGE(GATTC_TAG, " %s db is NULL", __func__);
        return;
    }

    if (handle == db[SPP_IDX_SPP_DATA_NTY_VAL].attribute_handle) {
        /*------------------------------------*/
          if (p_data->notify.value_len == 0) {
            return;
        }
        /*------------------------------------*/
        if ((p_data->notify.value[0] == '#') && (p_data->notify.value[1] == '#')) {
            /*--------------------------------------------------*/
            if (p_data->notify.value_len < 4) {
                #ifdef DEBUG
                ESP_LOGW(GATTC_TAG, "Fragment frame too short");
                #endif
                return;
            }
            /*--------------------------------------------------*/
            if ((++notify_value_count) != p_data->notify.value[3]) {
                if(notify_value_p != NULL){
                    free(notify_value_p);
                }
                notify_value_count = 0;
                notify_value_p = NULL;
                notify_value_offset = 0;
                #ifdef DEBUG
                ESP_LOGE(GATTC_TAG,"notify value count is not continuous, %s", __func__);
                #endif
                return;
            }
            if (p_data->notify.value[3] == 1) {
                notify_value_p = (char *)malloc(((spp_mtu_size-7)*(p_data->notify.value[2]))*sizeof(char));
                if (notify_value_p == NULL) {
                    #ifdef DEBUG
                    ESP_LOGE(GATTC_TAG, "malloc failed, %s L#%d", __func__, __LINE__);
                    #endif
                    notify_value_count = 0;
                    return;
                }
                memcpy((notify_value_p + notify_value_offset), (p_data->notify.value + 4), (p_data->notify.value_len - 4));
                if (p_data->notify.value[2] == p_data->notify.value[3]) {
                    //uart_write_bytes(UART_NUM_0, (char *)(notify_value_p), (p_data->notify.value_len - 4 + notify_value_offset));
                    /*-----------------------------------------------------------------------------------------*/
                    const uint16_t payload_len = (uint16_t)(p_data->notify.value_len - 4 + notify_value_offset);
                    (void)enqueue_notify_packet((uint8_t *)notify_value_p, payload_len);
                    /*-----------------------------------------------------------------------------------------*/
                    free(notify_value_p);
                    notify_value_p = NULL;
                    notify_value_offset = 0;
                    return;
                }
                notify_value_offset += (p_data->notify.value_len - 4);
            } else if (p_data->notify.value[3] <= p_data->notify.value[2]) {
                memcpy((notify_value_p + notify_value_offset), (p_data->notify.value + 4), (p_data->notify.value_len - 4));
                if (p_data->notify.value[3] == p_data->notify.value[2]) {
                    //uart_write_bytes(UART_NUM_0, (char *)(notify_value_p), (p_data->notify.value_len - 4 + notify_value_offset));
                    /*-----------------------------------------------------------------------------------------*/
                    const uint16_t payload_len = (uint16_t)(p_data->notify.value_len - 4 + notify_value_offset);
                    (void)enqueue_notify_packet((uint8_t *)notify_value_p, payload_len);
                   /*------------------------------------------------------------------------------------------*/
                    free(notify_value_p);
                    notify_value_count = 0;
                    notify_value_p = NULL;
                    notify_value_offset = 0;
                    return;
                }
                notify_value_offset += (p_data->notify.value_len - 4);
            }
        } else {
            // uart_write_bytes(UART_NUM_0, (char *)(p_data->notify.value), p_data->notify.value_len);
            /*-----------------------------------------------------------------------------------------*/
            (void)enqueue_notify_packet(p_data->notify.value, p_data->notify.value_len);
            /*-----------------------------------------------------------------------------------------*/
        }   
    } else if (handle == ((db+SPP_IDX_SPP_STATUS_VAL)->attribute_handle)) {
        ESP_LOG_BUFFER_CHAR(GATTC_TAG, (char *)p_data->notify.value, p_data->notify.value_len);
        //TODO:server notify status characteristic
    } else {
        ESP_LOG_BUFFER_CHAR(GATTC_TAG, (char *)p_data->notify.value, p_data->notify.value_len);
    }
}

static void free_gattc_srv_db(void)
{
    is_connect = false;
    connect = false;
    spp_gattc_if = 0xff;
    spp_conn_id = 0;
    spp_mtu_size = 23;
    cmd = 0;
    spp_srv_start_handle = 0;
    spp_srv_end_handle = 0;
    notify_value_p = NULL;
    notify_value_offset = 0;
    notify_value_count = 0;
    g_last_rx_us = 0;
    if (db) {
        free(db);
        db = NULL;
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    esp_err_t err;

    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        if((err = param->scan_param_cmpl.status) != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "Scan param set failed: %s", esp_err_to_name(err));
            break;
        }
        // the unit of the duration is second, 0 means scan permanently
        uint32_t duration = 0;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if ((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "Scanning start failed, err %s", esp_err_to_name(err));
            break;
        }
        ESP_LOGI(GATTC_TAG, "Scanning start successfully");
        break;
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if ((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "Scanning stop failed, err %s", esp_err_to_name(err));
            break;
        }
        ESP_LOGI(GATTC_TAG, "Scanning stop successfully");
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            adv_name = esp_ble_resolve_adv_data_by_type(scan_result->scan_rst.ble_adv,
                            scan_result->scan_rst.adv_data_len + scan_result->scan_rst.scan_rsp_len,
                            ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            ESP_LOGI(GATTC_TAG, "Scan result, device "ESP_BD_ADDR_STR", name len %u", ESP_BD_ADDR_HEX(scan_result->scan_rst.bda), adv_name_len);
            ESP_LOG_BUFFER_CHAR(GATTC_TAG, adv_name, adv_name_len);
            if (adv_name != NULL && strncmp((char *)adv_name, device_name, adv_name_len) == 0) {
                if (connect == false) {
                    connect = true;
                    esp_ble_gap_stop_scanning();
                    ESP_LOGI(GATTC_TAG, "Connect to the remote device.");
                    esp_ble_conn_params_t phy_1m_conn_params = {0};
                    phy_1m_conn_params.interval_max = 32;
                    phy_1m_conn_params.interval_min = 32;
                    phy_1m_conn_params.latency = 0;
                    phy_1m_conn_params.supervision_timeout = 600;
                    esp_ble_gatt_creat_conn_params_t creat_conn_params = {0};
                    memcpy(&creat_conn_params.remote_bda, scan_result->scan_rst.bda,ESP_BD_ADDR_LEN);
                    creat_conn_params.remote_addr_type = scan_result->scan_rst.ble_addr_type;
                    creat_conn_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
                    creat_conn_params.is_direct = true;
                    creat_conn_params.is_aux = false;
                    creat_conn_params.phy_mask = ESP_BLE_PHY_1M_PREF_MASK;
                    creat_conn_params.phy_1m_conn_params = &phy_1m_conn_params;
                    esp_ble_gattc_enh_open(gl_profile_tab[PROFILE_APP_ID].gattc_if,
                                        &creat_conn_params);
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            ESP_LOGI(GATTC_TAG, "Scan complete");
            break;
        default:
            break;
        }
        break;
    }
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if ((err = param->adv_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "Advertising stop failed, err %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(GATTC_TAG, "Advertising stop successfully");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTC_TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
                  param->update_conn_params.status,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{

    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(GATTC_TAG, "Reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
            return;
        }
    }
    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "GATT client register, status %d, app_id %d, gattc_if %d", param->reg.status, param->reg.app_id, gattc_if);
        esp_ble_gap_set_scan_params(&ble_scan_params);
        break;
    case ESP_GATTC_CONNECT_EVT:
        ESP_LOGI(GATTC_TAG, "Connected, conn_id %d, remote "ESP_BD_ADDR_STR"", p_data->connect.conn_id,
                 ESP_BD_ADDR_HEX(p_data->connect.remote_bda));
        memcpy(gl_profile_tab[PROFILE_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        spp_gattc_if = gattc_if;
        is_connect = true;
        spp_conn_id = p_data->connect.conn_id;
        esp_ble_gattc_search_service(spp_gattc_if, spp_conn_id, &spp_service_uuid);
        break;
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Open failed, status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Open successfully, MTU %u", p_data->open.mtu);
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGI(GATTC_TAG, "Disconnected, remote "ESP_BD_ADDR_STR", reason 0x%02x",
                 ESP_BD_ADDR_HEX(p_data->disconnect.remote_bda), p_data->disconnect.reason);
        free_gattc_srv_db();
        start = false;
        start_time = 0;
        current_time = 0;
        notify_len = 0;
        esp_ble_gap_start_scanning(SCAN_ALL_THE_TIME);
        break;
    case ESP_GATTC_SEARCH_RES_EVT:
        ESP_LOGI(GATTC_TAG, "Service search result, start_handle %d, end_handle %d, UUID:0x%04x",
                 p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.uuid.uuid.uuid16);
        spp_srv_start_handle = p_data->search_res.start_handle;
        spp_srv_end_handle = p_data->search_res.end_handle;
        break;
    case ESP_GATTC_SEARCH_CMPL_EVT:
        ESP_LOGI(GATTC_TAG, "Service search complete, conn_id %x, status %d", spp_conn_id, p_data->search_cmpl.status);
        esp_ble_gattc_send_mtu_req(gattc_if, spp_conn_id);
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGI(GATTC_TAG,"Notification register, index %d, status %d, handle %d", cmd, p_data->reg_for_notify.status, p_data->reg_for_notify.handle);
        if (p_data->reg_for_notify.status != ESP_GATT_OK) {
            break;
        }
        uint16_t notify_en = 0x01;
#ifdef CONFIG_EXAMPLE_SPP_RELIABLE
        if (cmd == SPP_IDX_SPP_DATA_NTY_VAL) {
            notify_en = 0x02;
        }
#endif
        esp_ble_gattc_write_char_descr(
                spp_gattc_if,
                spp_conn_id,
                (db+cmd+1)->attribute_handle,
                sizeof(notify_en),
                (uint8_t *)&notify_en,
                ESP_GATT_WRITE_TYPE_RSP,
                ESP_GATT_AUTH_REQ_NONE);
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify){
            // ESP_LOGI(GATTC_TAG, "Notification received, handle %d", param->notify.handle);
        }else{
            // ESP_LOGI(GATTC_TAG, "Indication received, handle %d", param->notify.handle);
        }
        notify_event_handler(p_data);
        break;
    case ESP_GATTC_READ_CHAR_EVT:
        ESP_LOGI(GATTC_TAG,"Characteristic read");
        break;
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (param->write.status) {
            ESP_LOGI(GATTC_TAG,"Characteristic write, status %d, handle %d", param->write.status, param->write.handle);
        }
        break;
    case ESP_GATTC_PREP_WRITE_EVT:
        ESP_LOGI(GATTC_TAG, "Prepare write");
        break;
    case ESP_GATTC_EXEC_EVT:
        ESP_LOGI(GATTC_TAG, "Execute write %d", param->exec_cmpl.status);
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        ESP_LOGI(GATTC_TAG,"Descriptor write, status %d, handle %d", p_data->write.status, p_data->write.handle);
        if(p_data->write.status != ESP_GATT_OK){
            break;
        }
        switch(cmd){
        case SPP_IDX_SPP_DATA_NTY_VAL:
            cmd = SPP_IDX_SPP_STATUS_VAL;
            xQueueSend(cmd_reg_queue, &cmd,10/portTICK_PERIOD_MS);
            break;
        case SPP_IDX_SPP_STATUS_VAL:
    #ifdef SUPPORT_HEARTBEAT
            cmd = SPP_IDX_SPP_HEARTBEAT_VAL;
            xQueueSend(cmd_reg_queue, &cmd, 10/portTICK_PERIOD_MS);
    #endif
            break;
    #ifdef SUPPORT_HEARTBEAT
        case SPP_IDX_SPP_HEARTBEAT_VAL:
            xQueueSend(cmd_heartbeat_queue, &cmd, 10/portTICK_PERIOD_MS);
            break;
    #endif
        default:
            break;
        };
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        ESP_LOGI(GATTC_TAG, "MTU exchange, status %d, MTU %d", param->cfg_mtu.status, param->cfg_mtu.mtu);
        if(p_data->cfg_mtu.status != ESP_OK){
            break;
        }
        spp_mtu_size = p_data->cfg_mtu.mtu;

        db = (esp_gattc_db_elem_t *)malloc(count*sizeof(esp_gattc_db_elem_t));
        if(db == NULL){
            ESP_LOGE(GATTC_TAG, "Malloc db failed");
            break;
        }
        if(esp_ble_gattc_get_db(spp_gattc_if, spp_conn_id, spp_srv_start_handle, spp_srv_end_handle, db, &count) != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Get db failed");
            break;
        }
        if(count != SPP_IDX_NB){
            ESP_LOGE(GATTC_TAG, "Get db count != SPP_IDX_NB, count = %d, SPP_IDX_NB = %d", count, SPP_IDX_NB);
            break;
        }
        for(int i = 0;i < SPP_IDX_NB;i++){
            switch((db+i)->type){
            case ESP_GATT_DB_PRIMARY_SERVICE:
                ESP_LOGI(GATTC_TAG, "PRIMARY_SERVICE, attribute_handle %d, start_handle %d, end_handle %d, properties 0x%x, uuid 0x%04x",
                        (db+i)->attribute_handle, (db+i)->start_handle, (db+i)->end_handle, (db+i)->properties, (db+i)->uuid.uuid.uuid16);
                break;
            case ESP_GATT_DB_SECONDARY_SERVICE:
                ESP_LOGI(GATTC_TAG, "SECONDARY_SERVICE, attribute_handle %d, start_handle %d, end_handle %d, properties 0x%x, uuid 0x%04x",
                        (db+i)->attribute_handle, (db+i)->start_handle, (db+i)->end_handle, (db+i)->properties, (db+i)->uuid.uuid.uuid16);
                break;
            case ESP_GATT_DB_CHARACTERISTIC:
                ESP_LOGI(GATTC_TAG, "CHARACTERISTIC, attribute_handle %d, start_handle %d, end_handle %d, properties 0x%x, uuid 0x%04x",
                        (db+i)->attribute_handle, (db+i)->start_handle, (db+i)->end_handle, (db+i)->properties, (db+i)->uuid.uuid.uuid16);
                break;
            case ESP_GATT_DB_DESCRIPTOR:
                ESP_LOGI(GATTC_TAG, "DESCRIPTOR, attribute_handle %d, start_handle %d, end_handle %d, properties 0x%x, uuid 0x%04x",
                        (db+i)->attribute_handle, (db+i)->start_handle, (db+i)->end_handle, (db+i)->properties, (db+i)->uuid.uuid.uuid16);
                break;
            case ESP_GATT_DB_INCLUDED_SERVICE:
                ESP_LOGI(GATTC_TAG, "INCLUDED_SERVICE, attribute_handle %d, start_handle %d, end_handle %d, properties 0x%x, uuid 0x%04x",
                        (db+i)->attribute_handle, (db+i)->start_handle, (db+i)->end_handle, (db+i)->properties, (db+i)->uuid.uuid.uuid16);
                break;
            case ESP_GATT_DB_ALL:
                ESP_LOGI(GATTC_TAG, "GATT_DB_ALL, attribute_handle %d, start_handle %d, end_handle %d, properties 0x%x, uuid 0x%04x",
                        (db+i)->attribute_handle, (db+i)->start_handle, (db+i)->end_handle, (db+i)->properties, (db+i)->uuid.uuid.uuid16);
                break;
            default:
                break;
            }
        }
        cmd = SPP_IDX_SPP_DATA_NTY_VAL;
        xQueueSend(cmd_reg_queue, &cmd, 10/portTICK_PERIOD_MS);
        break;
    case ESP_GATTC_SRVC_CHG_EVT:
        ESP_LOGI(GATTC_TAG, "Service change from "ESP_BD_ADDR_STR"", ESP_BD_ADDR_HEX(p_data->srvc_chg.remote_bda));
        break;
    default:
        break;
    }
}

void spp_client_reg_task(void* arg)
{
    uint16_t cmd_id;
    for (;;) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        if (xQueueReceive(cmd_reg_queue, &cmd_id, portMAX_DELAY)) {
            if (db != NULL) {
                if (cmd_id == SPP_IDX_SPP_DATA_NTY_VAL) {
                    ESP_LOGI(GATTC_TAG,"Index %d, UUID 0x%04x, handle %d", cmd_id, (db+SPP_IDX_SPP_DATA_NTY_VAL)->uuid.uuid.uuid16, (db+SPP_IDX_SPP_DATA_NTY_VAL)->attribute_handle);
                    esp_ble_gattc_register_for_notify(spp_gattc_if, gl_profile_tab[PROFILE_APP_ID].remote_bda, (db+SPP_IDX_SPP_DATA_NTY_VAL)->attribute_handle);
                } else if(cmd_id == SPP_IDX_SPP_STATUS_VAL) {
                    ESP_LOGI(GATTC_TAG,"Index %d, UUID 0x%04x, handle %d", cmd_id, (db+SPP_IDX_SPP_STATUS_VAL)->uuid.uuid.uuid16, (db+SPP_IDX_SPP_STATUS_VAL)->attribute_handle);
                    esp_ble_gattc_register_for_notify(spp_gattc_if, gl_profile_tab[PROFILE_APP_ID].remote_bda, (db+SPP_IDX_SPP_STATUS_VAL)->attribute_handle);
                }
#ifdef SUPPORT_HEARTBEAT
                else if (cmd_id == SPP_IDX_SPP_HEARTBEAT_VAL) {
                    ESP_LOGI(GATTC_TAG,"Index %d, UUID 0x%04x, handle %d", cmd_id, (db+SPP_IDX_SPP_HEARTBEAT_VAL)->uuid.uuid.uuid16, (db+SPP_IDX_SPP_HEARTBEAT_VAL)->attribute_handle);
                    esp_ble_gattc_register_for_notify(spp_gattc_if, gl_profile_tab[PROFILE_APP_ID].remote_bda, (db+SPP_IDX_SPP_HEARTBEAT_VAL)->attribute_handle);
                }
#endif
            }
        }
    }
}

#ifdef SUPPORT_HEARTBEAT
void spp_heart_beat_task(void * arg)
{
    uint16_t cmd_id;

    for (;;) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if (xQueueReceive(cmd_heartbeat_queue, &cmd_id, portMAX_DELAY)) {
            while(1){
                if ((is_connect == true) && (db != NULL) && ((db+SPP_IDX_SPP_HEARTBEAT_VAL)->properties & (ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_WRITE))) {
                    esp_ble_gattc_write_char( spp_gattc_if,
                                              spp_conn_id,
                                              (db+SPP_IDX_SPP_HEARTBEAT_VAL)->attribute_handle,
                                              sizeof(heartbeat_s),
                                              (uint8_t *)heartbeat_s,
                                              ESP_GATT_WRITE_TYPE_RSP,
                                              ESP_GATT_AUTH_REQ_NONE);
                    vTaskDelay(5000 / portTICK_PERIOD_MS);
                } else {
                    ESP_LOGI(GATTC_TAG,"disconnect");
                    break;
                }
            }
        }
    }
}
#endif

void ble_client_appRegister(void)
{
    esp_err_t status;
    char err_msg[20];

    ESP_LOGI(GATTC_TAG, "register callback");

    //register the scan callback function to the gap module
    if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(GATTC_TAG, "gap register error: %s", esp_err_to_name_r(status, err_msg, sizeof(err_msg)));
        return;
    }
    //register the callback function to the gattc module
    if ((status = esp_ble_gattc_register_callback(esp_gattc_cb)) != ESP_OK) {
        ESP_LOGE(GATTC_TAG, "gattc register error: %s", esp_err_to_name_r(status, err_msg, sizeof(err_msg)));
        return;
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(SPP_GATT_MTU_SIZE);
    if (local_mtu_ret) {
        ESP_LOGE(GATTC_TAG, "set local  MTU failed: %s", esp_err_to_name_r(local_mtu_ret, err_msg, sizeof(err_msg)));
    }

    cmd_reg_queue = xQueueCreate(10, sizeof(uint16_t));
    xTaskCreate(spp_client_reg_task, "spp_client_reg_task", 2048, NULL, 10, NULL);

#ifdef SUPPORT_HEARTBEAT
    cmd_heartbeat_queue = xQueueCreate(10, sizeof(uint16_t));
    xTaskCreate(spp_heart_beat_task, "spp_heart_beat_task", 2048, NULL, 10, NULL);
#endif
    esp_ble_gattc_app_register(PROFILE_APP_ID);
}

void uart_task(void *pvParameters)
{
    uart_event_t event;
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(spp_uart_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            switch (event.type) {
            //Event of UART receiving data
            case UART_DATA:
                if (event.size && (is_connect == true) && (db != NULL) && ((db+SPP_IDX_SPP_DATA_RECV_VAL)->properties & (ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_WRITE))) {
                    uint8_t * temp = NULL;
                    size_t offset = 0;
                    size_t send_len = 0;
                    temp = (uint8_t *)malloc(sizeof(uint8_t)*event.size);
                    if(temp == NULL){
                        ESP_LOGE(GATTC_TAG, "malloc failed,%s L#%d", __func__, __LINE__);
                        break;
                    }
                    uart_read_bytes(UART_NUM_0, temp, event.size, portMAX_DELAY);
                    while (offset < event.size) {
                        send_len = MIN(spp_mtu_size - 3, event.size - offset);
#ifdef CONFIG_EXAMPLE_SPP_THROUGHPUT
                        if (esp_ble_get_cur_sendable_packets_num(spp_conn_id) > 0) {
                            esp_ble_gattc_write_char(spp_gattc_if,
                                                spp_conn_id,
                                                (db+SPP_IDX_SPP_DATA_RECV_VAL)->attribute_handle,
                                                send_len,
                                                temp + offset,
                                                ESP_GATT_WRITE_TYPE_NO_RSP,
                                                ESP_GATT_AUTH_REQ_NONE);
                        } else {
                            //Add the vTaskDelay to prevent this task from consuming the CPU all the time, causing low-priority tasks to not be executed at all.
                            vTaskDelay(10 / portTICK_PERIOD_MS);
                        }
#else
                        esp_ble_gattc_write_char(spp_gattc_if,
                                            spp_conn_id,
                                            (db+SPP_IDX_SPP_DATA_RECV_VAL)->attribute_handle,
                                            send_len,
                                            temp + offset,
                                            ESP_GATT_WRITE_TYPE_RSP,
                                            ESP_GATT_AUTH_REQ_NONE);
#endif
                        offset += send_len;
                    }
                    free(temp);
                }
                break;
            default:
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

static void spp_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
        .rx_flow_ctrl_thresh = 124,
        .source_clk = UART_SCLK_DEFAULT,
    };

    //Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_0, 4096, 8192, 10, &spp_uart_queue, 0);
    //Set UART parameters
    uart_param_config(UART_NUM_0, &uart_config);
    //Set UART pins
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreate(uart_task, "uTask", 4096, (void*)UART_NUM_0, 8, NULL);
}

void app_main(void)
{
    esp_err_t ret;
    /*-----------------------------------------------------------------*/
    gpio_init();
    motor_stop_all();

#ifdef MT6816_ON
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ret = mt6816_init(&enc, SPI2_HOST, PIN_NUM_CS, 100 * 1000);
    if (ret == ESP_OK) {
        g_mt6816_ready = true;
        ESP_LOGI(GATTC_TAG, "MT6816 init OK");
    } else {
        ESP_LOGE(GATTC_TAG, "MT6816 init failed: %s", esp_err_to_name(ret));
        return;
    }
#else
    ESP_LOGW(GATTC_TAG, "MT6816_ON not defined, angle closed-loop disabled");
#endif

    rc_rx_queue = xQueueCreate(RC_RX_QUEUE_LEN, sizeof(rc_rx_packet_t));
    rc_ctrl_queue = xQueueCreate(RC_CTRL_QUEUE_LEN, sizeof(rc_cmd_t));
    if (rc_rx_queue == NULL || rc_ctrl_queue == NULL) {
        ESP_LOGE(GATTC_TAG, "Create RC queues failed");
        return;
    }
    g_last_rx_us = esp_timer_get_time();

    xTaskCreate(rc_parse_task, "rc_parse_task", 3072, NULL, 9, NULL);
    xTaskCreate(rc_control_task, "rc_ctrl_task", 2048, NULL, 8, NULL);
    /*------------------------------------------------------------------*/

    spp_uart_init();

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    nvs_flash_init();

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(GATTC_TAG, "%s init bluetooth", __func__);

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ble_client_appRegister();
}
