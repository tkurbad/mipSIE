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

while True:
    start = datetime.now()
    print "Accel:", imu.getAccelerometerAngles()
    print "Kalman:", imu.getKalmanAngles()
    while (datetime.now() - start).microseconds < 50000:
        sleep(0.001)
