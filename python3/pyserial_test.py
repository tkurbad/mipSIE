#!/usr/bin/python3

import serial
from time import sleep

ser = serial.Serial(
    port='/dev/ttyAMA0',
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout = 0,
    xonxoff = False,
    #inter_byte_timeout = 0.001
)

sleep(0.2)

inFile = open('README.md', 'rb')
for line in inFile:
    ser.write(line)
    sleep(0.03)

ser.close()
inFile.close()
