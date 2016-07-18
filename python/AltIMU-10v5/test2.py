#!/usr/bin/python

from datetime import datetime
from time import sleep

from altimu import AltIMU

imu = AltIMU()
imu.enable()

imu.calibrateGyroAngles()

for x in range(1000):
    startTime = datetime.now()
    print "Gyro Angles:", imu.trackGyroAngle(deltaT = 0.005)
