#!/usr/bin/python3

from math import degrees, pi
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

while True:
    if altIMU.IMURead():
        data = altIMU.getIMUData()
        (data['pressureValid'],
         data['pressure'],
         data['temperatureValid'],
         data['temperature']) = altIMUPressure.pressureRead()

        fusionPose = data['fusionPose']
        print ('Raw r: %f p: %f y: %f' %
               (fusionPose[0], 
                fusionPose[1],
                fusionPose[2]))

        pbr.SetMotor1(fusionPose[0] / pi)
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
