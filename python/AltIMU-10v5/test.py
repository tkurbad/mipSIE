#!/usr/bin/python

from time import sleep

from lsm6 import LSM6

imu = LSM6()
imu.enable(autoIncrementRegisters = True)

while True:
    print imu.getAccelerometer()
    sleep(0.1)
