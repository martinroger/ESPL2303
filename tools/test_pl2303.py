#!/usr/bin/env python3
"""
Simple test script to exercise the emulated PL2303 device from the host side.
Usage: python3 tools/test_pl2303.py /dev/ttyUSB0
"""
import sys
import serial
import time

if len(sys.argv) < 2:
    print("Usage: {} <serial-port>").format(sys.argv[0])
    sys.exit(1)

port = sys.argv[1]
print('Opening', port)
ser = serial.Serial(port, baudrate=115200, timeout=1)
print('Opened', ser)
print('Setting baud to 57600')
ser.baudrate = 57600
ser.write(b'hello from host\n')
print('Wrote hello, waiting for reply...')
try:
    for _ in range(5):
        data = ser.read(128)
        if data:
            print('Received:', data)
            break
        time.sleep(0.2)
finally:
    ser.close()
    print('Closed')
