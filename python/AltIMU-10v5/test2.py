#!/usr/bin/python

from time import sleep

from altimu import AltIMU

imu = AltIMU()
imu.enable()

while True:
    print "Gyro Angles:", imu.trackGyroAngle()
