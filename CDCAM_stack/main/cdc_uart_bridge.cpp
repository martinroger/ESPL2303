#include <stdint.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "sdkconfig.h"

#include "driver/uart.h"
#include "driver/gpio.h"

// Bitty banggity
#define DTR_GPIO (gpio_num_t) CONFIG_DTR_PIN
#define RTS_GPIO (gpio_num_t) CONFIG_RTS_PIN

static uint8_t rx_buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];
static uint8_t tx_buf[CONFIG_TINYUSB_CDC_TX_BUFSIZE + 1];

esp_err_t init_bridge_control_pins(void)
{
    esp_err_t ret;
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << DTR_GPIO) | (1ULL << RTS_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(__func__, "GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = (gpio_set_level(DTR_GPIO, 1) | gpio_set_level(RTS_GPIO, 1));
    if (ret != ESP_OK)
    {
        ESP_LOGE(__func__, "GPIO set level failed: %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

/// @brief USB-> UART callback
/// @param itf Interface
/// @param event Type of CDC event
void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    size_t rx_size = 0;
    esp_err_t ret = tinyusb_cdcacm_read((tinyusb_cdcacm_itf_t)itf, rx_buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    if (ret == ESP_OK && uart_is_driver_installed(UART_NUM_1))
    {
        uart_write_bytes(UART_NUM_1, (const char *)rx_buf, rx_size);
    }
    else
    {
        ESP_LOGE(__func__, "Read Error");
    }
}

/// @brief CDC line state change callback, sets the DTR and RTS line state on the UART side
/// @param itf Interface
/// @param event Type of CDC event
void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = ((event->line_state_changed_data.dtr ) ? 0 : 1);
    int rts = ((event->line_state_changed_data.rts ) ? 0 : 1);
    ESP_LOGD(__func__, "Line state changed on channel %d: DTR:%d, RTS:%d", itf, dtr, rts);
    // uart_set_rts(UART_NUM_1, rts);
    // uart_set_dtr(UART_NUM_1, dtr);
    // Is inverted !
    esp_err_t ret = gpio_set_level(DTR_GPIO, dtr);
    if (ret != ESP_OK)
    {
        ESP_LOGE(__func__, "GPIO set level failed: %s", esp_err_to_name(ret));
    }
    ret = gpio_set_level(RTS_GPIO, rts);
    if (ret != ESP_OK)
    {
        ESP_LOGE(__func__, "GPIO set level failed: %s", esp_err_to_name(ret));
    }
}

/// @brief Line coding change callback, sets the baudrate, data bits, stop bits and parity on the UART side
/// @param itf Interface
/// @param event Type of CDC event
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
        uart_set_stop_bits(UART_NUM_1, (uart_stop_bits_t)(stop_bits + 1)); // Should do switch-cases instead?
        switch (parity)
        {
        case 0:
            uart_set_parity(UART_NUM_1, UART_PARITY_DISABLE);
            break;
        case 1:
            uart_set_parity(UART_NUM_1, UART_PARITY_ODD);
            break;
        case 2:
            uart_set_parity(UART_NUM_1, UART_PARITY_EVEN);
            break;
        default:
            ESP_LOGE(__func__, "Unknown parity");
            break;
        }
    }
}

extern "C" void app_main(void)
{
    // Set up control pins
    ESP_LOGD(__func__, "Initializing control pins");
    ESP_ERROR_CHECK(init_bridge_control_pins());

    // Set up the target UART
    ESP_LOGD(__func__, "UART initialization");
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, CONFIG_UART_TX_PIN, CONFIG_UART_RX_PIN, -1, -1));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, CONFIG_UART_RX_BUFSIZE, CONFIG_UART_TX_BUFSIZE, 0, NULL, 0));

    ESP_LOGD(__func__, "USB initialization");
    const tinyusb_config_t tusb_cfg = {0};
    // const tinyusb_config_t tusb_cfg = {
    //     .device_descriptor = NULL,
    //     .string_descriptor = NULL,
    //     .external_phy = false,
    //     .configuration_descriptor = NULL,
    // };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = &tinyusb_cdc_rx_callback,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = &tinyusb_cdc_line_state_changed_callback,
        .callback_line_coding_changed = &tinyusb_cdc_line_coding_changed_callback};

    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
    ESP_LOGD(__func__, "USB initialization DONE");
    while (1)
    {
        // Loop checking if there is anything coming from uart and write-flush it to the CDC
        size_t tx_len = uart_read_bytes(UART_NUM_1, &tx_buf, sizeof(tx_buf), pdMS_TO_TICKS(1));
        if (tx_len > 0) // Not sure what should be done if length is 0, for now it is ignored
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
            vTaskDelay(pdMS_TO_TICKS(5)); // Stability yield
        }
    }
}
