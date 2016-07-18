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
    print "Accel:", imu.getAccelerometerAngles()
    print "Kalman:", imu.getKalmanAngles(deltaT = stop.microseconds/500000.0)
    print "Compl:", imu.getComplementaryAngles(deltaT = stop.microseconds/500000.0)
    sleep(0.3)
