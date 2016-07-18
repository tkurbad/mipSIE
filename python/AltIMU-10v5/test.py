#!/usr/bin/python

from time import sleep

from lsm6ds33 import LSM6DS33
from lis3mdl import LIS3MDL
from lps25h import LPS25H

imu = LSM6DS33()
imu.enable()

magnet = LIS3MDL()
magnet.enable()

baro = LPS25H()
baro.enable()

while True:
    print "Gyro:", imu.getGyroscopeRaw()
    print "Accelerometer:", imu.getAccelerometerRaw()
    print "Magnet:", magnet.getMagnetometerRaw()
    print "hPa:", baro.getBarometerMillibars()
    print "Altitude:", baro.getAltitude()
    sleep(0.2)
    print "Gyro Temperature:", imu.getTemperatureCelsius()
    print "Magnet Temperature:", magnet.getTemperatureCelsius()
    print "Baro Temperature:", baro.getTemperatureCelsius()
    sleep(0.1)
