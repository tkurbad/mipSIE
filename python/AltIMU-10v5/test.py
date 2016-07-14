#!/usr/bin/python

from time import sleep

from lsm6 import LSM6

imu = LSM6()
imu.enable(autoIncrementRegisters = False)

while True:
    print imu.getAll()
    sleep(0.1)
