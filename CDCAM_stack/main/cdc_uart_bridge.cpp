/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "sdkconfig.h"

#include "driver/uart.h"

static const char *TAG = "example";
static uint8_t rx_buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];
static uint8_t tx_buf[CONFIG_TINYUSB_CDC_TX_BUFSIZE + 1];

/**
 * @brief Application Queue
 */
static QueueHandle_t app_queue;
typedef struct
{
    uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1]; // Data buffer
    size_t buf_len;                                 // Number of bytes received
    uint8_t itf;                                    // Index of CDC device interface
} app_message_t;

QueueHandle_t uart_queue; // Possibly not needed

/**
 * @brief CDC device RX callback
 *
 * CDC device signals, that new data were received
 *
 * @param[in] itf   CDC device index
 * @param[in] event CDC event type
 */
void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    /* initialization */
    size_t rx_size = 0;

    /* read */
    esp_err_t ret = tinyusb_cdcacm_read((tinyusb_cdcacm_itf_t)itf, rx_buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    if (ret == ESP_OK && uart_is_driver_installed(UART_NUM_1))
    {

        // app_message_t tx_msg = {
        //     .buf_len = rx_size,
        //     .itf = itf,
        // };

        // memcpy(tx_msg.buf, rx_buf, rx_size);
        // xQueueSend(app_queue, &tx_msg, 0);
        // Write to UART
        uart_write_bytes(UART_NUM_1, (const char *)rx_buf, rx_size);
    }
    else
    {
        ESP_LOGE(TAG, "Read Error");
    }
}

/**
 * @brief CDC device line change callback
 *
 * CDC device signals, that the DTR, RTS states changed
 *
 * @param[in] itf   CDC device index
 * @param[in] event CDC event type
 */
void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rts = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "Line state changed on channel %d: DTR:%d, RTS:%d", itf, dtr, rts);
    uart_set_rts(UART_NUM_1, rts);
    uart_set_dtr(UART_NUM_1, dtr);
}

void tinyusb_cdc_line_coding_changed_callback(int itf, cdcacm_event_t *event)
{
    uint32_t baudrate = event->line_coding_changed_data.p_line_coding->bit_rate;
    uint8_t data_bits = event->line_coding_changed_data.p_line_coding->data_bits;
    uint8_t stop_bits = event->line_coding_changed_data.p_line_coding->stop_bits;
    uint8_t parity = event->line_coding_changed_data.p_line_coding->parity;
    if (uart_is_driver_installed(UART_NUM_1))
    {

        uart_set_baudrate(UART_NUM_1, baudrate); // Should boundary check it
        uart_set_word_length(UART_NUM_1, (uart_word_length_t)(data_bits - 5));
        uart_set_stop_bits(UART_NUM_1, (uart_stop_bits_t)(stop_bits + 1)); // Should do switch-cases instead.
        switch (parity)
        {
        case 0:
            uart_set_parity(UART_NUM_1, UART_PARITY_DISABLE);
            break;
        case 1:
            uart_set_parity(UART_NUM_1, UART_PARITY_ODD);
            break;
        case 2:
            uart_set_parity(UART_NUM_1, UART_PARITY_ODD);
            break;
        default:
            ESP_LOGE(__func__, "Unknown parity");
            break;
        }
    }
}

extern "C" void app_main(void)
{
    // Create FreeRTOS primitives
    app_queue = xQueueCreate(5, sizeof(app_message_t));
    assert(app_queue);
    app_message_t msg;

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, 5, 6, 7, 8));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 2048, 2048, 10, &uart_queue, 0));

    ESP_LOGI(TAG, "USB initialization");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
#if (TUD_OPT_HIGH_SPEED)
        .fs_configuration_descriptor = NULL,
        .hs_configuration_descriptor = NULL,
        .qualifier_descriptor = NULL,
#else
        .configuration_descriptor = NULL,
#endif // TUD_OPT_HIGH_SPEED
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = &tinyusb_cdc_rx_callback, // the first way to register a callback
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = &tinyusb_cdc_line_state_changed_callback,
        .callback_line_coding_changed = &tinyusb_cdc_line_coding_changed_callback};

    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));



    ESP_LOGI(TAG, "USB initialization DONE");
    while (1)
    {
        // Loop checking if there is anything coming from uart and write-flush it to the CDC

        size_t tx_len = uart_read_bytes(UART_NUM_1, &tx_buf, sizeof(tx_buf), pdMS_TO_TICKS(10));
        if (tx_len > 0)
        {
            tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, (const uint8_t *)&tx_buf, tx_len);
            esp_err_t err = tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, pdMS_TO_TICKS(1));
            if (err != ESP_OK)
            {
                ESP_LOGW(__func__, "CDC ACM write flush error: %s", esp_err_to_name(err));
            }
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        // if (xQueueReceive(app_queue, &msg, portMAX_DELAY))
        // {
        //     if (msg.buf_len)
        //     {

        //         /* Print received data*/
        //         ESP_LOGI(TAG, "Data from channel %d:", msg.itf);
        //         ESP_LOG_BUFFER_HEXDUMP(TAG, msg.buf, msg.buf_len, ESP_LOG_INFO);

        //         /* write back */
        //         tinyusb_cdcacm_write_queue((tinyusb_cdcacm_itf_t)msg.itf, msg.buf, msg.buf_len);
        //         esp_err_t err = tinyusb_cdcacm_write_flush((tinyusb_cdcacm_itf_t)msg.itf, 0);
        //         if (err != ESP_OK)
        //         {
        //             ESP_LOGE(TAG, "CDC ACM write flush error: %s", esp_err_to_name(err));
        //         }
        //     }
        // }
    }
}
