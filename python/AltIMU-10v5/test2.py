#!/usr/bin/python

from datetime import datetime
from time import sleep

from altimu import AltIMU

imu = AltIMU()
imu.enable()

imu.calibrateGyroAngles()

for x in range(1000):
    startTime = datetime.now()
    angles = imu.trackGyroAngles(deltaT = 0.0002)

print angles

for x in range(100):
    print "Accel:", imu.getAccelerometerAngles()
