#!/usr/bin/python3

import atexit

from math import pi
from time import sleep

from PicoBorgRev.PicoBorgRev import PicoBorgRev
from RTIMU import RTIMU, RTPressure, Settings

altIMUSettings = Settings('AltIMU')
altIMU = RTIMU(altIMUSettings)
altIMUPressure = RTPressure(altIMUSettings)

print ('IMU Name:', altIMU.IMUName())
print ('Pressure Sensor:', altIMUPressure.pressureName())

# Initialize the AltIMUv5
if not altIMU.IMUInit():
    raise Exception('IMU initialization failed!')

if not altIMUPressure.pressureInit():
    raise Exception('Pressure sensor initialization failed!')

altIMU.setSlerpPower(0.02)
altIMU.setGyroEnable(True)
altIMU.setAccelEnable(True)
altIMU.setCompassEnable(True)

# Get minimum poll interval
poll_interval = altIMU.IMUGetPollInterval()

pbr = PicoBorgRev()
pbr.Init()

# Stop all motors upon program exit
atexit.register(pbr.MotorsOff)

# PID coefficients - still need to be tuned
KP = 2.5 * pi
KI = 0.01
KD = 0.1

# Initialize some values
lastFusionRollX = 0.0
Ivalue = 0.0

while True:
    # Check if AltIMU is ready
    if altIMU.IMURead():
        # Get a new set of data from IMU and pressure sensor
        data = altIMU.getIMUData()
        (data['pressureValid'],
         data['pressure'],
         data['temperatureValid'],
         data['temperature']) = altIMUPressure.pressureRead()

        # Extract fused roll, pitch and yaw from the data (values are in radians)
        (fusionRollX, fusionPitchY, fusionYawZ) = data['fusionPose']
        #print ('Raw r: %f p: %f y: %f' %
        #       (fusionRollX, 
        #        fusionPitchY,
        #        fusionYawZ))

        # Calculate PID term
        Pvalue = KP * fusionRollX
        Ivalue += KI * fusionRollX
        Dvalue = KD * (fusionRollX - lastFusionRollX)
        lastFusionRollX = fusionRollX

        PID = Pvalue + Ivalue + Dvalue

        # Calculate motor PWM. Divide PID by pi to normalize radians
        motorPWM = PID / pi

        # If integral part gets over-excited, limit to pi
        if Ivalue > pi:
            Ivalue = pi
        if Ivalue < -pi:
            Ivalue = -pi

        # Set motor PWM, i.e. set motors running
        pbr.SetMotor1(motorPWM)

        #print ('r: %f p: %f y: %f' %
        #       (degrees(fusionPose[0]), 
        #        degrees(fusionPose[1]),
        #        degrees(fusionPose[2])))

        #if data['pressureValid']:
        #    print('Pressure: %f' % (
        #          data['pressure']))
        #if data['temperatureValid']:
        #    print('Temperature: %f' % (data['temperature']))

        # Sleep for a short time
        sleep(poll_interval * 0.001)
