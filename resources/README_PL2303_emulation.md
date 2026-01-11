# PL2303 Emulation Notes

This repository implements a TinyUSB "vendor" class device that emulates enough of a PL2303-like device for a host driver to enumerate and configure it.

Files added:
- `resources/pl2303_vendor_requests.md` - table of vendor control requests observed in the PCAP and suggested responses.

Testing
- Build and flash to the ESP32-S3 board with USB connected to the test host.
- On the host, observe kernel logs (`dmesg`) to see `pl2303` driver binding (this repo sets VID/PID to 0x067b/0x2303 via sdkconfig by default).
- Use `tools/test_pl2303.py /dev/ttyUSB0` to open the serial port, set baud, and send test data.
- Connect the ESP32's UART TX/RX pins to a serial loopback adapter to validate host <-> UART data flows.

Notes
- The vendor control handlers are implemented in `main/tusb_serial_device_main.c` in `tud_vendor_control_xfer_cb`.
- An interrupt-IN endpoint has been added and a small status packet is sent when the host toggles DTR/RTS (CDC_SET_CONTROL_LINE_STATE).
- The mapping from SET_LINE coding to UART parameters is a best-effort mapping based on CDC line coding format (baud 4 bytes LE, stop, parity, data bits).

If you want me to add an automated host test that exercises the exact vendor requests captured in the PCAP, I can parse the PCAP and emit the sequence as a test harness.