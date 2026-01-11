/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/*
 * Simple PL2303-like vendor serial example
 * - Implements TinyUSB vendor class control handler to reply to a few PL2303 vendor requests
 * - Bridges vendor bulk IN/OUT to UART (UART0)
 * - Handles SET_LINE and SET_CONTROL_LINE_STATE to configure UART parameters and send a
 *   status packet back to the host
 * Notes:
 * - We avoid modifying managed TinyUSB components. For status notifications that PL2303 uses
 *   via an Interrupt IN endpoint we currently send the same 9-byte status packet over the
 *   vendor bulk IN endpoint so hosts still receive it. If strict interrupt behavior is required
 *   we can add an application-provided configuration descriptor or revisit the managed
 *   components with your approval.
 */

#include <stdint.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "tinyusb.h"
#include <stdlib.h>
#include "sdkconfig.h"
#include "driver/uart.h"

/* Private TinyUSB header used only for low-level USB helpers when necessary. Keep usage minimal. */
#include "device/usbd_pvt.h"

static const char *TAG = "example";
static uint8_t rx_buf[CFG_TUD_VENDOR_RX_BUFSIZE + 1];

/* Bridge UART selection: use UART1 instead of UART0 per request. */
#define BRIDGE_UART_NUM UART_NUM_1

/* Ensure our tag prints INFO-level logs at runtime; this helps debug when ESP_LOG_BUFFER_HEXDUMP
 * seems not to produce output (it respects the set log level). */



/**
 * @brief Application Queue
 */
static QueueHandle_t app_queue;
/* Queue for messages coming from USB (host->device) */
typedef struct {
    uint8_t buf[CFG_TUD_VENDOR_RX_BUFSIZE + 1];     // Data buffer
    size_t buf_len;                                     // Number of bytes received
    uint8_t itf;                                        // Index of vendor interface
} app_message_t;

/* Queue for messages originating from the application (to be sent to host over USB) */
static QueueHandle_t usb_tx_queue;
typedef struct {
    uint8_t buf[CFG_TUD_VENDOR_TX_BUFSIZE + 1];
    size_t len;
    uint8_t source; /* optional source id */
} usb_out_message_t;

/* Helper API: allow other tasks to push data to the PL2303 USB interface. */
void app_send_to_usb(const uint8_t *data, size_t len)
{
    usb_out_message_t msg;
    if (!usb_tx_queue) return;

    if (len > sizeof(msg.buf)) len = sizeof(msg.buf);
    memcpy(msg.buf, data, len);
    msg.len = len;
    msg.source = 0;
    xQueueSend(usb_tx_queue, &msg, 0);
}

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

            /* Send the 9-byte PL2303-like status packet over the vendor bulk IN endpoint.
             * We use vendor bulk instead of interrupt to avoid changing managed components' descriptors.
             * This is functionally acceptable for many hosts; if strict interrupt endpoint behavior
             * is required we can revisit adding an application-provided configuration descriptor.
             */
            tud_vendor_n_write(0, status, sizeof(status));
            tud_vendor_n_write_flush(0);
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

            // apply config to the configured bridge UART (UART1)
            esp_err_t ret = uart_param_config(BRIDGE_UART_NUM, &uart_cfg);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to set UART params: %s", esp_err_to_name(ret));
            }
            if (baud) {
                ret = uart_set_baudrate(BRIDGE_UART_NUM, baud);
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
// We copy the received packet into a small FreeRTOS queue and wake the main loop to
// forward it to UART. This decouples USB RX (interrupt context) from potentially
// blocking UART writes.
void tud_vendor_rx_cb(uint8_t itf, uint8_t const* buffer, uint16_t bufsize)
{
    app_message_t tx_msg = {
        .buf_len = bufsize,
        .itf = itf,
    };

    memcpy(tx_msg.buf, buffer, bufsize > sizeof(tx_msg.buf) ? sizeof(tx_msg.buf) : bufsize);
    xQueueSend(app_queue, &tx_msg, 0);

    /* If using a FIFO RX buffer, clear and rearm the stream transfer so the next
     * packet can be received. */
    #if CFG_TUD_VENDOR_RX_BUFSIZE > 0
    tud_vendor_read_flush();
    #endif
}

static void uart_task(void *arg)
{
    (void) arg;
    /* Continuously read from BRIDGE_UART_NUM (UART1) and forward to host over vendor bulk IN.
     * Also log the received bytes to the monitor so a copy exists locally.
     */
    while (1) {
        int len = uart_read_bytes(BRIDGE_UART_NUM, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(100));
        if (len > 0) {
            ESP_LOGI(TAG, "UART -> USB: %d bytes", len);
            ESP_LOG_BUFFER_HEXDUMP(TAG, rx_buf, len, ESP_LOG_INFO);
            /* Also print as ASCII for convenience (non-printables may appear as garbage). */
            ESP_LOGI(TAG, "UART -> USB (ASCII): %.*s", len, (char*)rx_buf);

            tud_vendor_n_write(0, rx_buf, len);
            tud_vendor_n_write_flush(0);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* Task that sends queued messages (from any source) out to the host over the vendor interface. */
static void usb_out_task(void *arg)
{
    (void)arg;
    usb_out_message_t msg;
    while (1) {
        if (xQueueReceive(usb_tx_queue, &msg, portMAX_DELAY)) {
            ESP_LOGI(TAG, "OUTGOING -> USB: %d bytes", msg.len);
            ESP_LOG_BUFFER_HEXDUMP(TAG, msg.buf, msg.len, ESP_LOG_INFO);
            ESP_LOGI(TAG, "OUTGOING -> USB (ASCII): %.*s", msg.len, (char*)msg.buf);
            tud_vendor_n_write(0, msg.buf, msg.len);
            tud_vendor_n_write_flush(0);
        }
    }
}

void app_main(void)
{
    // Create FreeRTOS primitives
    app_queue = xQueueCreate(5, sizeof(app_message_t));
    assert(app_queue);
    app_message_t msg;

    ESP_LOGI(TAG, "USB initialization");

    /* Force INFO level for this tag so ESP_LOG_BUFFER_HEXDUMP prints at INFO in case the
     * global log level is higher. This helps diagnose cases where hex dump output is missing.
     */
    esp_log_level_set(TAG, ESP_LOG_INFO);

    /* Quick hexdump test to confirm ESP_LOG_BUFFER_HEXDUMP is active at INFO level. Look for:
     * "example: 01 02 03 04"
     */
    const uint8_t _hexdump_test[] = { 0x01, 0x02, 0x03, 0x04 };
    ESP_LOG_BUFFER_HEXDUMP(TAG, _hexdump_test, sizeof(_hexdump_test), ESP_LOG_INFO);
    /* Use default configuration descriptor from managed components (no changes to managed components).
     * We intentionally avoid adding an interrupt endpoint in descriptors here to keep the managed
     * components untouched. Instead, status/notification packets (like the PL2303 status) will be
     * sent over the vendor bulk IN endpoint so the host still receives the data without requiring
     * changes to TinyUSB's managed components.
     */
    /* Create a modified configuration descriptor by copying the managed default
     * configuration descriptor at runtime and appending a small Interrupt-IN
     * endpoint (7 bytes) that PL2303 expects. This avoids referencing symbols
     * like ITF_NUM_TOTAL (which are defined in the managed component's
     * usb_descriptors.c) and keeps changes minimal.
     */

    extern const uint8_t descriptor_fs_cfg_default[]; /* provided by managed component */
#if (TUD_OPT_HIGH_SPEED)
    extern const uint8_t descriptor_hs_cfg_default[]; /* provided by managed component */
#endif

    /* Helper to extend a descriptor by appending an endpoint descriptor */
    uint8_t *pl2303_fs_configuration = NULL;
#if (TUD_OPT_HIGH_SPEED)
    uint8_t *pl2303_hs_configuration = NULL;
#endif

    do {
        const uint8_t *base = descriptor_fs_cfg_default;
        uint16_t base_len = (uint16_t)base[2] | ((uint16_t)base[3] << 8);
        const uint8_t irq_ep[] = { 0x07, /* bLength */ 0x05 /* TUSB_DESC_ENDPOINT */ , (0x80 | 0x03), /* EP3 IN */ 0x03 /* Interrupt */ , 0x08, 0x00, /* wMaxPacketSize */ 10 /* bInterval */ };
        const size_t extra = sizeof(irq_ep);
        size_t new_len = base_len + extra;

        pl2303_fs_configuration = malloc(new_len);
        if (!pl2303_fs_configuration) {
            ESP_LOGE(TAG, "Failed to allocate FS descriptor buffer");
            abort();
        }
        memcpy(pl2303_fs_configuration, base, base_len);

        /* Find the vendor interface descriptor (bDescriptorType==INTERFACE && bInterfaceClass==0xFF)
         * and increment its bNumEndpoints (at offset +4) so it matches the appended endpoint.
         */
        for (uint16_t i = 9; i + 2 < base_len; ) {
            uint8_t bLength = pl2303_fs_configuration[i];
            uint8_t bDescriptorType = pl2303_fs_configuration[i + 1];
            if (bLength < 2) break;
            if (bDescriptorType == 0x04 /* TUSB_DESC_INTERFACE */) {
                uint8_t bInterfaceClass = pl2303_fs_configuration[i + 5];
                if (bInterfaceClass == 0xFF) {
                    /* increment bNumEndpoints */
                    pl2303_fs_configuration[i + 4] = (uint8_t)(pl2303_fs_configuration[i + 4] + 1);
                    ESP_LOGI(TAG, "Updated vendor interface bNumEndpoints to %d", pl2303_fs_configuration[i + 4]);
                    break;
                }
            }
            i += bLength;
        }

        memcpy(pl2303_fs_configuration + base_len, irq_ep, extra);
        pl2303_fs_configuration[2] = (uint8_t)(new_len & 0xff);
        pl2303_fs_configuration[3] = (uint8_t)((new_len >> 8) & 0xff);

    #if (TUD_OPT_HIGH_SPEED)
        const uint8_t *base_hs = descriptor_hs_cfg_default;
        uint16_t base_hs_len = (uint16_t)base_hs[2] | ((uint16_t)base_hs[3] << 8);
        size_t new_hs_len = base_hs_len + extra;
        pl2303_hs_configuration = malloc(new_hs_len);
        if (!pl2303_hs_configuration) {
            ESP_LOGE(TAG, "Failed to allocate HS descriptor buffer");
            abort();
        }
        memcpy(pl2303_hs_configuration, base_hs, base_hs_len);

        for (uint16_t i = 9; i + 2 < base_hs_len; ) {
            uint8_t bLength = pl2303_hs_configuration[i];
            uint8_t bDescriptorType = pl2303_hs_configuration[i + 1];
            if (bLength < 2) break;
            if (bDescriptorType == 0x04 /* TUSB_DESC_INTERFACE */) {
                uint8_t bInterfaceClass = pl2303_hs_configuration[i + 5];
                if (bInterfaceClass == 0xFF) {
                    pl2303_hs_configuration[i + 4] = (uint8_t)(pl2303_hs_configuration[i + 4] + 1);
                    ESP_LOGI(TAG, "Updated HS vendor interface bNumEndpoints to %d", pl2303_hs_configuration[i + 4]);
                    break;
                }
            }
            i += bLength;
        }

        memcpy(pl2303_hs_configuration + base_hs_len, irq_ep, extra);
        pl2303_hs_configuration[2] = (uint8_t)(new_hs_len & 0xff);
        pl2303_hs_configuration[3] = (uint8_t)((new_hs_len >> 8) & 0xff);
    #endif

    } while (0);

    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
#if (TUD_OPT_HIGH_SPEED)
        .fs_configuration_descriptor = pl2303_fs_configuration,
        .hs_configuration_descriptor = pl2303_hs_configuration,
        .qualifier_descriptor = NULL,
#else
        .configuration_descriptor = pl2303_fs_configuration,
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

    ESP_ERROR_CHECK(uart_param_config(BRIDGE_UART_NUM, &uart_cfg));
    ESP_ERROR_CHECK(uart_driver_install(BRIDGE_UART_NUM, 512, 512, 0, NULL, 0));

    /* start UART rx task */
    xTaskCreate(uart_task, "uart_task", 2048, NULL, 10, NULL);

    /* Create outgoing USB queue and start the USB-out task so other tasks may push messages to
     * the PL2303 interface without touching the vendor class internals directly. */
    usb_tx_queue = xQueueCreate(8, sizeof(usb_out_message_t));
    xTaskCreate(usb_out_task, "usb_out_task", 2048, NULL, 10, NULL);

    /* Vendor-class in use: vendor control handler and RX callbacks are implemented via tud_vendor_control_xfer_cb and tud_vendor_rx_cb. */
    /* No additional class init required here. */
    
    ESP_LOGI(TAG, "USB initialization DONE");
    while (1) {
        if (xQueueReceive(app_queue, &msg, portMAX_DELAY)) {
            if (msg.buf_len) {

                /* Print received data*/
                ESP_LOGI(TAG, "USB -> UART, channel %d:", msg.itf);
                ESP_LOG_BUFFER_HEXDUMP(TAG, msg.buf, msg.buf_len, ESP_LOG_INFO);

                /* forward to bridge UART (UART1) */
                ESP_LOG_BUFFER_HEXDUMP(TAG, msg.buf, msg.buf_len, ESP_LOG_INFO);
                ESP_LOGI(TAG, "USB -> UART (ASCII): %.*s", msg.buf_len, (char*)msg.buf);

                int written = uart_write_bytes(BRIDGE_UART_NUM, (const char*)msg.buf, msg.buf_len);
                if (written < 0) {
                    ESP_LOGE(TAG, "UART write error: %d", written);
                }
            }
        }
    }
}
