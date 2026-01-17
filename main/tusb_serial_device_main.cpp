#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "tinyusb.h"
#include "device/usbd_pvt.h" // Required for low-level access
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
static uint8_t line_control = 0;

// typedef struct
// {
//     uint8_t buf[CFG_TUD_VENDOR_RX_BUFSIZE];
//     size_t buf_len;
// } usb_rx_msg_t;

// static QueueHandle_t usb_rx_queue;

// --- PL2303 Logic ---

static void pl2303_send_status(bool DCD_state, bool CTS_state, bool DSR_state, bool RI_state)
{
    uint8_t status[9] = {0};
    uint8_t final_status = 0;
    final_status |= (DCD_state? 0x01 : 0x00);
    final_status |= (DSR_state? 0x02 : 0x00);
    final_status |= (RI_state? 0x08 : 0x00);
    final_status |= (CTS_state? 0x80 : 0x00);
    status[8] = final_status;
    if (tud_vendor_mounted())
    {
        // tud_vendor_n_write ?
        tud_vendor_write(status, sizeof(status));
        tud_vendor_write_flush();
    }
    ESP_LOGD(__func__, "Sent status : %02X", status[8]);
}

static void pl2303_send_status(void)
{
    pl2303_send_status(true,true,true,false);
}

// --- TinyUSB Callbacks ---

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    static uint8_t req_0404_wIndex = 0x00; // Toggler for write and read requests around 0x0404 and 0x8383
    static uint8_t resp_read_0000 = 0x01;
    static uint8_t set_line_buf[7];        // Reception buffer for the set_line request
    // Allow TinyUSB Core to handle stray standard Requests (Address/Enumeration)
    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD)
        return false;

    if (stage != CONTROL_STAGE_SETUP)
    {
        if (stage == CONTROL_STAGE_ACK)
        {
            if (request->bRequest == 0x22) // SET CONTROL request ?
                pl2303_send_status();
            if (request->bRequest == 0x20) // SET LINE request, only handling the baudrate
            {
                uint32_t baud = *(uint32_t *)(set_line_buf);
                if (baud > 0)
                {
                    esp_err_t uart_set_err = uart_set_baudrate(BRIDGE_UART_NUM, baud);
                    if (uart_set_err != ESP_OK)
                        ESP_LOGE(__func__, "Could not set baud to %lu", baud);
                }
                pl2303_send_status();
            }
        }
        return true;
    }

    if (stage == CONTROL_STAGE_SETUP)
    {
        if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR && request->bRequest == 0x01) // Could be replaced by request->bmRequestType == 0x40 or 0xc0 (write and read respectively)
        {
            if (request->bmRequestType_bit.direction & TUSB_DIR_IN) // Vendor Reads
            {

                static uint8_t resp;
                // Response to 0x8383 seems to vary to 0xFF after a (0x40 01) 0x0404 0x0100 0x00
                // There are more cases to handle, like 0x0080
                switch (request->wValue)
                {
                case 0x8484:
                    resp = 0x02;
                    break;
                case 0x8383:
                    resp = 0xEF + req_0404_wIndex;
                    break;
                case 0x8080: // Supports HX status
                    resp = 0x01;
                    break;
                case 0x0080:
                    resp = resp_read_0000;
                    break;
                default:
                    resp = 0x00;
                    ESP_LOGW(__func__, "VENDOR READ %04X len %04X", request->wValue, request->wLength);
                    break;
                }
                return tud_control_xfer(rhport, request, &resp, request->wLength);
            }
            else if ((request->bmRequestType_bit.direction == TUSB_DIR_OUT)) // Vendor Writes
            {
                switch (request->wValue)
                {
                case 0x0404:
                    if (request->wIndex == 0x0001)
                        req_0404_wIndex = 0x10;
                    else
                        req_0404_wIndex = 0x00;
                    break;
                case 0x0000:
                    resp_read_0000 = request->wIndex & 0xFF;
                    break;
                default:
                ESP_LOGW(__func__, "VENDOR WRITE %04X len %04X", request->wValue, request->wLength);
                    break;
                }

                    
            }
            return tud_control_status(rhport, request);
        }

        // Class Requests (Line Coding / Control)
        if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS)
        {
            static uint8_t linebuf[7] = {0, 0, 0, 0, 0, 0, 8};
            static uart_stop_bits_t U_stop_bits;
            static uart_parity_t U_parity;
            static uart_word_length_t U_data_bits;
            static uint32_t U_baudrate;

            switch (request->bRequest)
            {
            case 0x21: // GET LINE
            {
                ESP_LOGI(__func__, "GET_LINE");
                uart_get_baudrate(BRIDGE_UART_NUM, &U_baudrate);
                uart_get_stop_bits(BRIDGE_UART_NUM, &U_stop_bits);
                uart_get_parity(BRIDGE_UART_NUM, &U_parity);
                uart_get_word_length(BRIDGE_UART_NUM, &U_data_bits);
                linebuf[4] = U_stop_bits - 1;
                switch (U_parity)
                {
                case UART_PARITY_ODD:
                    linebuf[5] = 1;
                    break;
                case UART_PARITY_EVEN:
                    linebuf[5] = 2;
                    break;
                default:
                    linebuf[5] = 0;
                    break;
                }
                switch (U_data_bits)
                {
                case UART_DATA_5_BITS:
                    linebuf[6] = 5;
                    break;
                case UART_DATA_6_BITS:
                    linebuf[6] = 6;
                    break;
                case UART_DATA_7_BITS:
                    linebuf[6] = 7;
                    break;
                case UART_DATA_8_BITS:
                    linebuf[6] = 8;
                    break;
                default:
                    linebuf[6] = 8;
                    break;
                }
                *((uint32_t *)&linebuf[0]) = U_baudrate;
                return tud_control_xfer(rhport, request, linebuf, 7);
                break;
            }
            case 0x20: // SET LINE
            {
                ESP_LOGD(__func__, "SET_LINE SETUP");
                // pl2303_send_status(); // Questionable
                return tud_control_xfer(rhport, request, set_line_buf, 7);
                break;
            }
            case 0x22: // SET CONTROL
            {
                line_control = request->wValue & 0xff;
                ESP_LOGD(__func__, "SET_CONTROL : DTR %d RTS %d", (line_control & 0x01) != 0, (line_control & 0x02) != 0);
                return tud_control_status(rhport, request);
                break;
            }
            case 0x23: // BREAK
            {
                ESP_LOGI(__func__, "BREAK : %s", request->wValue == 0XFFFF ? "ON" : "OFF");
                return tud_control_status(rhport, request);
                break;
            }
            default:
                break;
            }
        }
    }

    // Should not be reached
    ESP_LOGE(__func__, "Default return");
    return false;
}

// --- Tasks ---

/// @brief Uart to USB transfer task. Checks for any data on the UART, and if so, goes to push them on the USB
/// @param arg
static void uart_task(void *arg)
{
    while (1)
    {
        int len = uart_read_bytes(BRIDGE_UART_NUM, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(10));
        if (len > 0 && tud_vendor_mounted())
        {
            // Check if Endpoint 0x83 (Bulk IN) is ready for a new transfer
            if (!usbd_edpt_busy(0, EPNUM_VENDOR_IN))
            {
                // Manually push UART data to the Bulk IN pipe (0x83)
                usbd_edpt_xfer(0, 0x83, rx_buf, (uint16_t)len);
            }
        }
        // vTaskDelay(pdMS_TO_TICKS(1)); // Really necessary ?
    }
}

extern "C" void app_main(void)
{
    // usb_rx_queue = xQueueCreate(10, sizeof(usb_rx_msg_t));

    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = &desc_device,
        .string_descriptor = string_desc_arr,
        .string_descriptor_count = 5,
        .external_phy = false,
        .configuration_descriptor = desc_configuration,
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
        // This will need to move to a task
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