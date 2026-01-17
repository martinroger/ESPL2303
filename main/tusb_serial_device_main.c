#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "tinyusb.h"
#include "driver/uart.h"

static const char *TAG = "PL2303_Bridge";

#define BRIDGE_UART_NUM UART_NUM_1
#define EPNUM_VENDOR_OUT 0x02
#define EPNUM_VENDOR_IN 0x83
#define EPNUM_VENDOR_IRQ 0x81

// --- USB Descriptors ---

/**
 * Device Descriptor: Set to Prolific PL2303 Identifiers
 */
tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0110,
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x067B,  // Prolific
    .idProduct = 0x2303, // PL2303
    .bcdDevice = 0x0400,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x00,
    .bNumConfigurations = 0x01};

/**
 * Configuration Descriptor: 1 Interface, 3 Endpoints (Bulk IN, Bulk OUT, Interrupt IN)
 */
uint8_t const desc_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    // Total length = Config(9) + Vendor(9) + 3*Endpoint(7) = 39 bytes
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, (9 + 9 + 7 + 7 + 7), 0x00, 100),

    // Vendor Interface Descriptor
    9, TUSB_DESC_INTERFACE, 0x00, 0x00, 0x03, 0xFF, 0x00, 0x00, 0x00,

    // Endpoint Interrupt In (The "Status" EP for PL2303)
    7, TUSB_DESC_ENDPOINT, EPNUM_VENDOR_IRQ, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(10), 0x01,

    // Endpoint Out (Bulk)
    7, TUSB_DESC_ENDPOINT, EPNUM_VENDOR_OUT, TUSB_XFER_BULK, U16_TO_U8S_LE(64), 0,

    // Endpoint In (Bulk)
    7, TUSB_DESC_ENDPOINT, EPNUM_VENDOR_IN, TUSB_XFER_BULK, U16_TO_U8S_LE(64), 0};

/**
 * String Descriptors
 */
char const *string_desc_arr[] = {
    (const char[]){0x09, 0x04}, // 0: is supported language is English (0x0409)
    "Spoolific",                // 1: Manufacturer
    "USB-Serial Controller",    // 2: Product
    "",                         // 3: Serials
    "ESPL2303",                 // 4: Interface

};

// --- Globals & Queues ---

static uint8_t rx_buf[CFG_TUD_VENDOR_RX_BUFSIZE];
static uint8_t set_line_buf[7];
static uint8_t line_control = 0;
static uint8_t status_override = 0;

typedef struct
{
    uint8_t buf[CFG_TUD_VENDOR_RX_BUFSIZE];
    size_t buf_len;
} usb_rx_msg_t;

static QueueHandle_t usb_rx_queue;

// --- PL2303 Logic ---

static void pl2303_send_status(void)
{
    uint8_t status[9] = {0};
    uint8_t combined = line_control | status_override;
    if (combined & 0x02)
        status[8] |= 0x80; // CTS
    if (combined & 0x01)
        status[8] |= 0x01; // DCD

    if (tud_vendor_mounted())
    {
        // tud_vendor_n_write ?
        tud_vendor_write(status, sizeof(status));
        tud_vendor_write_flush();
    }
    ESP_LOGI(__func__,"Sent status : %02X",status[8]);
}

// --- TinyUSB Callbacks ---

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    static uint8_t req_0404_wIndex = 0x00;
    // Allow TinyUSB Core to handle Standard Requests (Address/Enumeration)
    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD)
        return false;

    if (stage != CONTROL_STAGE_SETUP)
    {
        if (stage == CONTROL_STAGE_ACK)
        {
            ESP_LOGI(__func__, "ACK");
            if (request->bRequest == 0x22)
                pl2303_send_status();
            if (request->bRequest == 0x20)
            {
                // uint32_t baud = (uint32_t)set_line_buf[0] | (set_line_buf[1] << 8) | (set_line_buf[2] << 16) | (set_line_buf[3] << 24);
                // ESP_LOGI(__func__,"SET LINE NOT SETUP");
                uint32_t baud = *(uint32_t *)(set_line_buf);
                ESP_LOGI(__func__, "Baud set to: %lu", baud);
                if (baud > 0)
                    uart_set_baudrate(BRIDGE_UART_NUM, baud);
            }
        }
        return true;
    }

    if (stage == CONTROL_STAGE_SETUP)
    {
        // Vendor Reads (0x01)
        if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR && request->bRequest == 0x01)
        {
            if (request->bmRequestType_bit.direction & TUSB_DIR_IN)
            {
                ESP_LOGW(__func__, "VENDOR READ %04X", request->wValue);
                static uint8_t resp;
                // Response to 0x8383 seems to vary to 0xFF after a (0x40 01) 0x0404 0x0100 0x00
                // There are more cases to handle, like 0x0080
                resp = (request->wValue == 0x8484) ? 0x02 : (request->wValue == 0x8383 ? (0xEF + req_0404_wIndex) : 0x00);
                return tud_control_xfer(rhport, request, &resp, 1);
            }
            else if ((request->bmRequestType_bit.direction == TUSB_DIR_OUT))
            {
                if (request->wValue == 0x0404)
                {
                    if (request->wIndex == 0x0001)
                        req_0404_wIndex = 0x10;
                    else
                        req_0404_wIndex = 0x00;
                }
                else
                    ESP_LOGW(__func__, "VENDOR WRITE %04X", request->wValue);
            }
            return tud_control_status(rhport, request);
        }

        // Class Requests (Line Coding / Control)
        if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS)
        {
            if (request->bRequest == 0x21)
            { // GET_LINE, stalled by PL2303 sometimes
                ESP_LOGI(__func__,"GET_LINE");
                static uint8_t linebuf[7] = {0, 0, 0, 0, 0, 0, 8};
                *((uint32_t*)&linebuf[0]) = *((uint32_t*)&set_line_buf);
                return tud_control_xfer(rhport, request, linebuf, 7);
            }
            if (request->bRequest == 0x20)
            { // SET_LINE
                // ESP_LOGI(__func__, "SET LINE SETUP");
                return tud_control_xfer(rhport, request, set_line_buf, 7);
            }
            if (request->bRequest == 0x22)
            { // SET_CONTROL_LINE
                line_control = request->wValue & 0xff;
                ESP_LOGI(__func__, "SET_CONTROL : DTR %d RTS %d", (line_control & 0x01) != 0, (line_control & 0x02) != 0);
                return tud_control_status(rhport, request);
            }
        }

        // Default : STALL
    }

    // Should not be reached
    ESP_LOGE(__func__, "Default return");
    return false;
}

// --- Tasks ---

static void uart_task(void *arg)
{
    while (1)
    {
        int len = uart_read_bytes(BRIDGE_UART_NUM, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(10));
        if (len > 0 && tud_vendor_mounted())
        {
            tud_vendor_write(rx_buf, len);
            tud_vendor_write_flush();
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // Really necessary ?
    }
}

void app_main(void)
{
    usb_rx_queue = xQueueCreate(10, sizeof(usb_rx_msg_t));

    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = &desc_device,
        .configuration_descriptor = desc_configuration,
        .string_descriptor = string_desc_arr,
        .string_descriptor_count = 5,
        .external_phy = false,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    uart_config_t uart_cfg = {.baud_rate = 115200, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(BRIDGE_UART_NUM, &uart_cfg);
    uart_set_pin(BRIDGE_UART_NUM, 17, 18, -1, -1);
    uart_driver_install(BRIDGE_UART_NUM, 512, 512, 0, NULL, 0);

    xTaskCreate(uart_task, "uart", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "USB initialization DONE");

    uint8_t rx_temp_buf[CFG_TUD_VENDOR_RX_BUFSIZE];

    while (1)
    {
        // 1. Handle Outgoing Data (USB -> UART)
        // Instead of a callback, we check if there is data available in the TinyUSB buffer
        if (tud_vendor_available())
        {
            uint32_t count = tud_vendor_read(rx_temp_buf, sizeof(rx_temp_buf));
            if (count > 0)
            {
                ESP_LOGI(TAG, "USB -> UART: %lu bytes", count);
                uart_write_bytes(BRIDGE_UART_NUM, (const char *)rx_temp_buf, count);
            }
        }

        // 2. You can still use a small delay to prevent watchdog issues,
        // though tud_task (handled by the driver) is doing the heavy lifting.
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}