#!/usr/bin/python

from time import sleep

from lsm6 import LSM6

imu = LSM6()
imu.enable()

while True:
    imu.getAll()
    sleep(0.1)
