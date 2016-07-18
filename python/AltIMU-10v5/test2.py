#!/usr/bin/python

from datetime import datetime
from time import sleep

from altimu import AltIMU

imu = AltIMU()
imu.enable()

imu.calibrateGyroAngles()

for x in range(50000):
    startTime = datetime.now()
    angles = imu.trackGyroAngle(deltaT = 0.0005)

print angles
