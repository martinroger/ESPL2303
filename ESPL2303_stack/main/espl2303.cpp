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

#include "driver/gpio.h"

// Bitty banggity
#define DTR_GPIO (gpio_num_t) CONFIG_DTR_PIN
#define RTS_GPIO (gpio_num_t) CONFIG_RTS_PIN

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

#define BRIDGE_UART_NUM UART_NUM_1

#pragma region Descriptors
/// @brief Device descriptors
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

/// @brief Configuration descriptors
uint8_t const desc_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    // Total length = Config(9) + Vendor(9) + 3*Endpoint(7) = 39 bytes
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, (9 + 9 + 7 + 7 + 7), 0x00, 100),

    // Vendor Interface Descriptor
    9, TUSB_DESC_INTERFACE, 0x00, 0x00, 0x03, 0xFF, 0x00, 0x00, 0x00,

    // Endpoint Interrupt In (The "Status" EP for PL2303)
    7, TUSB_DESC_ENDPOINT, CONFIG_EP_VENDOR_IRQ, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(10), 0x01,

    // Endpoint Out (Bulk)
    7, TUSB_DESC_ENDPOINT, CONFIG_EP_VENDOR_BULK_OUT, TUSB_XFER_BULK, U16_TO_U8S_LE(64), 0,

    // Endpoint In (Bulk)
    7, TUSB_DESC_ENDPOINT, CONFIG_EP_VENDOR_BULK_IN, TUSB_XFER_BULK, U16_TO_U8S_LE(64), 0};

/// @brief String descriptors
char const *string_desc_arr[] = {
    (const char[]){0x09, 0x04},              // 0: is supported language is English (0x0409)
    CONFIG_TINYUSB_DESC_MANUFACTURER_STRING, // 1: Manufacturer
    "USB-Serial Controller",                 // 2: Product
    "",                                      // 3: Serial
    CONFIG_TINYUSB_DESC_PRODUCT_STRING,      // 4: Interface

};
#pragma endregion

#pragma region Globals
static uint8_t tusb_rx_buf[CFG_TUD_VENDOR_RX_BUFSIZE]; // USB receive buffer (UART -> USB)
static uint8_t tusb_tx_buf[CFG_TUD_VENDOR_TX_BUFSIZE]; // USB transmit buffer (USB-> UART)
static uint8_t line_control = 0;                       // Line control bitfield byte
static TaskHandle_t UART2USB_hdl;                      // Task handle for the UART->USB bridging.
#pragma endregion

#pragma region PL2303 Vendor functions
/// @brief Sends line status to the host, allowing to set some of the line indicators direction
/// @param DCD_state Device Carrier Detect state, needs to be active to have sustained connection to device
/// @param CTS_state Clear to Send
/// @param DSR_state Data Send Ready
/// @param RI_state Ring Indicator
static inline void pl2303_send_status(bool DCD_state, bool CTS_state, bool DSR_state, bool RI_state)
{
    uint8_t status[9] = {0};
    uint8_t final_status = 0;
    final_status |= (DCD_state ? 0x01 : 0x00);
    final_status |= (DSR_state ? 0x02 : 0x00);
    final_status |= (RI_state ? 0x08 : 0x00);
    final_status |= (CTS_state ? 0x80 : 0x00);
    status[8] = final_status;
    if (tud_vendor_mounted())
    {
        tud_vendor_write(status, sizeof(status));
        tud_vendor_write_flush();
    }
    ESP_LOGD(__func__, "Sent status : %02X", status[8]);
}

/// @brief Short-hand for sending active status to allow comms to persist
/// @param
static inline void pl2303_send_status(void)
{
    pl2303_send_status(true, true, true, false);
}

/// @brief TinyUSB callback for Vendor Control and transfer requests handling
/// @param rhport Port
/// @param stage Control communication stage (Setup, ACK...)
/// @param request USB request
/// @return True when a reaction occurs, false otherwise
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    static uint8_t req_0404_wIndex = 0x00; // Toggler for write and read requests around 0x0404 and 0x8383
    static uint8_t resp_read_0000 = 0x01;
    static uint8_t set_line_buf[7]; // Reception buffer for the set_line request
    // Allow TinyUSB Core to handle stray standard Requests (Address/Enumeration)
    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD)
        return false;
    // If we are not in a SETUP stage, this is probably a follow-up of a request
    if (stage != CONTROL_STAGE_SETUP)
    {
        if (stage == CONTROL_STAGE_ACK)
        {
            if (request->bRequest == 0x22) // SET CONTROL request ?
                pl2303_send_status();      // Send line status, just in case
            if (request->bRequest == 0x20) // SET LINE request
            {
                uint32_t baud = *(uint32_t *)(set_line_buf); // Retrieved in SETUP stage
                uart_stop_bits_t stop_bits = (uart_stop_bits_t)(set_line_buf[4] + 1);
                uart_parity_t parity = (set_line_buf[5] == 1 ? UART_PARITY_ODD : (set_line_buf[5] == 2 ? UART_PARITY_EVEN : UART_PARITY_DISABLE));
                uart_word_length_t data_bits = (uart_word_length_t)(set_line_buf[6] - 5);
                if (baud > 0)
                {
                    esp_err_t uart_set_err = uart_set_baudrate(BRIDGE_UART_NUM, baud);
                    if (uart_set_err != ESP_OK)
                        ESP_LOGE(__func__, "Could not set baud to %lu", baud);
                }
                uart_set_stop_bits(BRIDGE_UART_NUM, stop_bits);
                uart_set_parity(BRIDGE_UART_NUM, parity);
                uart_set_word_length(BRIDGE_UART_NUM, data_bits);
                pl2303_send_status(); // Just in case
            }
        }
        return true;
    }
    // Start of Vendor control in SETUP stage
    if (stage == CONTROL_STAGE_SETUP)
    {
        if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR && request->bRequest == 0x01) // Could be replaced by request->bmRequestType == 0x40 or 0xc0 (write and read respectively)
        {
            if (request->bmRequestType_bit.direction & TUSB_DIR_IN) // Vendor Reads
            {
                static uint8_t resp;
                // Response to 0x8383 seems to vary to 0xFF after a (0x40 01) 0x0404 0x0100 0x00
                // There are sometimes more cases to handle, like 0x0080
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
                    ESP_LOGD(__func__, "VENDOR READ %04X len %04X", request->wValue, request->wLength);
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
                // Wrong method, should use GPIO bitbanging instead
                // uart_set_dtr(BRIDGE_UART_NUM, ((line_control & 0x01) != 0));
                // uart_set_rts(BRIDGE_UART_NUM, ((line_control & 0x02) != 0));
                // Might require inversion
                int dtr = ((line_control & 0x01) != 0) ? 0 : 1; // Invert DTR for active low
                int rts = ((line_control & 0x02) != 0) ? 0 : 1; // Invert RTS for active low
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
                return tud_control_status(rhport, request);
                break;
            }
            case 0x23: // BREAK
            {
                ESP_LOGD(__func__, "BREAK : %s", request->wValue == 0XFFFF ? "ON" : "OFF");
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
#pragma endregion

/// @brief UART->USB transfer task. Checks for any data on the UART, and if so, goes to push them on the USB
/// @param arg
static void UART2USB_task(void *arg)
{
    while (1)
    {
        int len = uart_read_bytes(BRIDGE_UART_NUM, tusb_rx_buf, sizeof(tusb_rx_buf), pdMS_TO_TICKS(1));
        if (len > 0 && tud_vendor_mounted())
        {
            // Check if Endpoint 0x83 (Bulk IN) is ready for a new transfer (should never be not ready but you know)
            if (!usbd_edpt_busy(0, CONFIG_EP_VENDOR_BULK_IN))
            {
                // Manually push UART data to the Bulk IN pipe (0x83)
                usbd_edpt_xfer(0, 0x83, tusb_rx_buf, (uint16_t)len);
            }
        }
    }
}

extern "C" void app_main(void)
{
    // Tiny USB configuration
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = &desc_device,
        .string_descriptor = string_desc_arr,
        .string_descriptor_count = 5,
        .external_phy = false,
        .configuration_descriptor = desc_configuration,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    // Bridge UART configuration
    uart_config_t uart_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(BRIDGE_UART_NUM, &uart_cfg);
    uart_set_pin(BRIDGE_UART_NUM, CONFIG_BRIDGE_TX_PIN, CONFIG_BRIDGE_RX_PIN, -1, -1);
    // Should actually use event queue for RX_DATA in to trigger the UART->USB task
    ESP_ERROR_CHECK(uart_driver_install(BRIDGE_UART_NUM, CONFIG_BRIDGE_UART_RX_BUFFER_SIZE, CONFIG_BRIDGE_UART_TX_BUFFER_SIZE, 0, NULL, 0));
    // Callbacks are not available for the vendor class, so they are implemented manually above

    xTaskCreate(UART2USB_task, "UART2USB", 8192, NULL, 10, &UART2USB_hdl);
    if (UART2USB_hdl == NULL)
    {
        ESP_LOGE(__func__, "Could not create UART->USB task, aborting.");
        return;
    }

    while (1)
    {
        // This is fine here
        if (tud_vendor_available())
        {
            uint32_t count = tud_vendor_read(tusb_tx_buf, sizeof(tusb_tx_buf));
            if (count > 0)
            {
                ESP_LOGD(__func__, "USB -> UART: %lu bytes", count);
                uart_write_bytes(BRIDGE_UART_NUM, (const char *)tusb_tx_buf, count);
            }
        }
        // Precautionary yield, 10 ticks
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}