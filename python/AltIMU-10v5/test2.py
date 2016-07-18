#!/usr/bin/python

from datetime import datetime
from time import sleep

from altimu import AltIMU

imu = AltIMU()
imu.enable()

imu.calibrateGyroAngles()

for x in range(1000):
    startTime = datetime.now()
    print "Gyro Angles:", imu.trackGyroAngle()
    while (datetime.now() - startTime).microseconds < 21000:
        sleep(0.005)
