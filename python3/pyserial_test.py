#!/usr/bin/python3

import serial

ser = serial.Serial(
    port='/dev/ttyAMA0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

if ser.isOpen():
    ser.close()

ser.open()

inFile = open('README.md', 'r')

for line in inFile:
    ser.write(b'%s\n\r' % line)
ser.close()
