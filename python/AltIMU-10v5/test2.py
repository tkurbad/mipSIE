#!/usr/bin/python

from datetime import datetime
from time import sleep

from altimu import AltIMU

imu = AltIMU()
imu.enable()

imu.calibrateGyroAngles()

#for x in range(1000):
#    startTime = datetime.now()
#    angles = imu.trackGyroAngles(deltaT = 0.0002)

#print angles

start = datetime.now()

while True:
    stop = datetime.now() - start
    start = datetime.now()
    deltaT = stop.microseconds/1000000.0
    print " "
    print "Loop:", deltaT
    print "Accel:", imu.getAccelerometerAngles()
    print "Gyro:", imu.trackGyroAngles(deltaT = deltaT)
    sleep(0.02)
