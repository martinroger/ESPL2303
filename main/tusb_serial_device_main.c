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
#include "sdkconfig.h"
#include "driver/uart.h"

static const char *TAG = "example";
static uint8_t rx_buf[CFG_TUD_VENDOR_RX_BUFSIZE + 1];

/**
 * @brief Application Queue
 */
static QueueHandle_t app_queue;
typedef struct {
    uint8_t buf[CFG_TUD_VENDOR_RX_BUFSIZE + 1];     // Data buffer
    size_t buf_len;                                     // Number of bytes received
    uint8_t itf;                                        // Index of CDC device interface
} app_message_t;

/**
 * @brief CDC device RX callback
 *
 * CDC device signals, that new data were received
 *
 * @param[in] itf   CDC device index
 * @param[in] event CDC event type
 */
// Vendor class control transfer callback
static uint8_t set_line_buf[7];
static uint8_t line_control = 0;

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request)
{
    ESP_LOGI(TAG, "Vendor CTRL (stage %d): bm=0x%02x bReq=0x%02x wVal=0x%04x wIdx=0x%04x wLen=%d",
             stage, request->bmRequestType, request->bRequest, request->wValue, request->wIndex, request->wLength);

    if (stage == CONTROL_STAGE_SETUP) {
        // Handle vendor read/write request (bRequest == 0x01)
        if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR && request->bRequest == 0x01) {
            // IN vendor read (device->host)
            if ((request->bmRequestType & TUSB_DIR_IN) && request->wValue == 0x8484) {
                uint8_t resp = 0x4C; // value observed in PCAP
                return tud_control_xfer(rhport, request, &resp, sizeof(resp));
            } else if ((request->bmRequestType & TUSB_DIR_IN) && request->wValue == 0x8383) {
                uint8_t resp = 0xF8; // value observed in PCAP
                return tud_control_xfer(rhport, request, &resp, sizeof(resp));
            }

            // OUT vendor write (host->device) - acknowledge
            if (!(request->bmRequestType & TUSB_DIR_IN)) {
                return tud_control_status(rhport, request);
            }

            return false;
        }

            // Handle GET_LINE class request (some hosts may fall back to class requests)
        if ((request->bmRequestType == (TUSB_DIR_IN | TUSB_REQ_TYPE_CLASS | TUSB_REQ_RCPT_INTERFACE)) && request->bRequest == 0x21) {
            uint8_t linebuf[7] = {0};
            linebuf[6] = 8; // 8 data bits
            return tud_control_xfer(rhport, request, linebuf, sizeof(linebuf));
        }

        // CDC_SET_LINE_CODING (host -> device) - request payload of 7 bytes
        if ((request->bmRequestType == (TUSB_DIR_OUT | TUSB_REQ_TYPE_CLASS | TUSB_REQ_RCPT_INTERFACE)) && request->bRequest == 0x20) {
            // schedule buffer to receive 7 bytes from host
            return tud_control_xfer(rhport, request, set_line_buf, sizeof(set_line_buf));
        }

        // CDC_SET_CONTROL_LINE_STATE (host -> device) - used for DTR/RTS
        if ((request->bmRequestType == (TUSB_DIR_OUT | TUSB_REQ_TYPE_CLASS | TUSB_REQ_RCPT_INTERFACE)) && request->bRequest == 0x22) {
            // record line control immediately and ack
            line_control = request->wValue & 0xff;
            ESP_LOGI(TAG, "Set control lines: DTR=%d RTS=%d", (bool)(line_control & 0x01), (bool)(line_control & 0x02));
            tud_control_status(rhport, request);

            // send an interrupt-in status packet (9 bytes like PL2303)
            uint8_t status[9] = {0};
            uint8_t status_byte = 0;
            if (line_control & 0x02) status_byte |= 0x80; // indicate CTS
            if (line_control & 0x01) status_byte |= 0x01; // indicate DCD
            status[8] = status_byte;

            tud_vendor_n_int_write(0, status, sizeof(status));
            tud_vendor_n_int_write_flush(0);
            return true;
        }

        // Unknown request - stall
        return false;
    }

    // DATA / ACK stages
    if (stage == CONTROL_STAGE_ACK) {
        // If this was a SET_LINE (0x20) request, apply the new UART settings
        if (request->bRequest == 0x20) {
            // parse set_line_buf: [0..3] baud (LE), [4] stop, [5] parity, [6] data bits
            uint32_t baud = (uint32_t)set_line_buf[0] | ((uint32_t)set_line_buf[1] << 8) | ((uint32_t)set_line_buf[2] << 16) | ((uint32_t)set_line_buf[3] << 24);
            uint8_t stop  = set_line_buf[4];
            uint8_t parity = set_line_buf[5];
            uint8_t databits = set_line_buf[6];

            ESP_LOGI(TAG, "SET_LINE received: baud=%u stop=%u parity=%u data_bits=%u", baud, stop, parity, databits);

            // Map to ESP-IDF uart_config_t
            uart_config_t uart_cfg = {
                .baud_rate = baud ? baud : 115200,
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
            };

            // data bits
            if (databits == 5) uart_cfg.data_bits = UART_DATA_5_BITS;
            else if (databits == 6) uart_cfg.data_bits = UART_DATA_6_BITS;
            else if (databits == 7) uart_cfg.data_bits = UART_DATA_7_BITS;
            else uart_cfg.data_bits = UART_DATA_8_BITS;

            // parity
            if (parity == 1) uart_cfg.parity = UART_PARITY_ODD;
            else if (parity == 2) uart_cfg.parity = UART_PARITY_EVEN;
            else uart_cfg.parity = UART_PARITY_DISABLE;

            // stop bits (0 -> 1, 1 -> 1.5, 2 -> 2) - ESP-IDF supports 1 or 2
            if (stop == 2) uart_cfg.stop_bits = UART_STOP_BITS_2;
            else uart_cfg.stop_bits = UART_STOP_BITS_1;

            // apply config to UART_NUM_0
            esp_err_t ret = uart_param_config(UART_NUM_0, &uart_cfg);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to set UART params: %s", esp_err_to_name(ret));
            }
            if (baud) {
                ret = uart_set_baudrate(UART_NUM_0, baud);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to set UART baud: %s", esp_err_to_name(ret));
                }
            }

            return true;
        }

        return true;
    }

    return true;
}

// Called when vendor interface receives data (host -> device)
void tud_vendor_rx_cb(uint8_t itf, uint8_t const* buffer, uint16_t bufsize)
{
    app_message_t tx_msg = {
        .buf_len = bufsize,
        .itf = itf,
    };

    memcpy(tx_msg.buf, buffer, bufsize > sizeof(tx_msg.buf) ? sizeof(tx_msg.buf) : bufsize);
    xQueueSend(app_queue, &tx_msg, 0);

    #if CFG_TUD_VENDOR_RX_BUFSIZE > 0
    tud_vendor_read_flush();
    #endif
}

static void uart_task(void *arg)
{
    (void) arg;
    while (1) {
        int len = uart_read_bytes(UART_NUM_0, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(100));
        if (len > 0) {
            ESP_LOGI(TAG, "UART -> USB: %d bytes", len);
            tud_vendor_n_write(0, rx_buf, len);
            tud_vendor_n_write_flush(0);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    // Create FreeRTOS primitives
    app_queue = xQueueCreate(5, sizeof(app_message_t));
    assert(app_queue);
    app_message_t msg;

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

    /* Initialize UART (bridge to vendor serial) */
    uart_config_t uart_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_cfg));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 512, 0, 0, NULL, 0));

    /* start UART rx task */
    xTaskCreate(uart_task, "uart_task", 2048, NULL, 10, NULL);

    /* Vendor-class in use: vendor control handler and RX callbacks are implemented via tud_vendor_control_xfer_cb and tud_vendor_rx_cb. */
    /* No additional class init required here. */
    
    ESP_LOGI(TAG, "USB initialization DONE");
    while (1) {
        if (xQueueReceive(app_queue, &msg, portMAX_DELAY)) {
            if (msg.buf_len) {

                /* Print received data*/
                ESP_LOGI(TAG, "USB -> UART, channel %d:", msg.itf);
                ESP_LOG_BUFFER_HEXDUMP(TAG, msg.buf, msg.buf_len, ESP_LOG_INFO);

                /* forward to UART */
                int written = uart_write_bytes(UART_NUM_0, (const char*)msg.buf, msg.buf_len);
                if (written < 0) {
                    ESP_LOGE(TAG, "UART write error: %d", written);
                }
            }
        }
    }
}
