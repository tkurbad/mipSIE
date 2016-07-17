#!/usr/bin/python

from time import sleep

from lsm6ds33 import LSM6DS33
from lis3mdl import LIS3MDL

imu = LSM6DS33()
imu.enable()

magnet = LIS3MDL()
magnet.enable()

while True:
    print imu.getTemperatureCelsius()
    sleep(0.1)
    print magnet.getMagnetometerRaw()
    sleep(0.1)
