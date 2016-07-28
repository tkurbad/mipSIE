#!/usr/bin/python3

import atexit

from math import degrees, pi, radians
from time import sleep

from PicoBorgRev.PicoBorgRev import PicoBorgRev
from RTIMU import RTIMU, RTPressure, Settings

altIMUSettings = Settings('AltIMU')
altIMU = RTIMU(altIMUSettings)
altIMUPressure = RTPressure(altIMUSettings)

print ('IMU Name:', altIMU.IMUName())
print ('Pressure Sensor:', altIMUPressure.pressureName())

if not altIMU.IMUInit():
    raise Exception('IMU initialization failed!')

if not altIMUPressure.pressureInit():
    raise Exception('Pressure sensor initialization failed!')

altIMU.setSlerpPower(0.02)
altIMU.setGyroEnable(True)
altIMU.setAccelEnable(True)
altIMU.setCompassEnable(True)

poll_interval = altIMU.IMUGetPollInterval()

pbr = PicoBorgRev()
pbr.Init()

atexit.register(pbr.MotorsOff)

# PID values
KP = 1.0
KI = 0.0
KD = 0.0

lastFusionRollX = 0.0

while True:
    if altIMU.IMURead():
        data = altIMU.getIMUData()
        (data['pressureValid'],
         data['pressure'],
         data['temperatureValid'],
         data['temperature']) = altIMUPressure.pressureRead()

        (fusionRollX, fusionPitchY, fusionYawZ) = data['fusionPose']
        print ('Raw r: %f p: %f y: %f' %
               (fusionRollX, 
                fusionPitchY,
                fusionYawZ))

        Pvalue = KP * fusionRollX
        Ivalue += KI * fusionRollX
        Dvalue = KD * (fusionRollX - lastFusionRollX)
        lastFusionRollX = fusionRollX

        motorPWM = (Pvalue + Ivalue + Dvalue) / pi

        if Ivalue > pi:
            Ivalue = pi
        if Ivalue < -pi:
            Ivalue = -pi

        pbr.SetMotor1(motorPWM)

        #print ('r: %f p: %f y: %f' %
        #       (degrees(fusionPose[0]), 
        #        degrees(fusionPose[1]),
        #        degrees(fusionPose[2])))

        if data['pressureValid']:
            print('Pressure: %f' % (
                  data['pressure']))
        if data['temperatureValid']:
            print('Temperature: %f' % (data['temperature']))

        sleep(poll_interval * 0.001)
