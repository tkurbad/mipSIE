#!/usr/bin/python

from time import sleep

from lsm6ds33 import LSM6DS33

imu = LSM6DS33()
imu.enable()

while True:
    print imu.getTemperatureCelsius()
    sleep(0.1)
