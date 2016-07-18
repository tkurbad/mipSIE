#!/usr/bin/python

### Python module to control several aspects of Pololu's AltIMU-10v5 ###
#
# This module enables the Raspberry PI embedded computer to set up and
# use various aspects of Pololu's AltIMU-10v5 Inertial Measurement Unit
# (IMU).
# [https://www.pololu.com/product/2739]
#
# This Python code was initially derived from Pololu's own C++ Arduino
# libraries, available at [https://github.com/pololu]
#
# The Python code is developed and maintained by
# Torsten Kurbad <github@tk-webart.de>
#
########################################################################

# Imports
from time import sleep

from lis3mdl import LIS3MDL as Magnet
from lps25h import LPS25H as BaroTemp
from lsm6ds33 import LSM6DS33 as AccelGyro


# Code
class AltIMU(object):
    """ Class to control Pololu's AltIMU-10v5. """

    # Class variables and constants
    GYRO_GAIN = 0.00875    # Gyroscope dps/LSB for 245 dps full scale

    def __init__(self):
        """ Initialize some flags and values. """
        self.accelGyroSensor = None
        self.baroTempSensor = None
        self.magnetSensor = None

        self.accelerometer = False
        self.barometer = False
        self.gyroscope = False
        self.magnetometer = False
        self.temperature = False

        # Initialize tracked gyroscope angles
        self.gyrAngles = []


    def __del__(self):
        """ Cleanup routine. """
        for device in [self.accelGyroSensor,
                       self.baroTempSensor,
                       self.magnetSensor]:
            if device is not None:
                try:
                    del(device)
                except:
                    pass


    def enable(self, accelerometer = True, barometer = True,
               gyroscope = True, magnetometer = True,
               temperature = True, autoIncrementRegisters = True):
        """ Enable the given devices. """
        if accelerometer:
            self.accelerometer = True
        if barometer:
            self.barometer = True
        if gyroscope:
            self.gyroscope = True
        if magnetometer:
            self.magnetometer = True
        if temperature:
            self.temperature = True

        # Enable LSM6DS33 if accelerometer and/or gyroscope are requested
        if self.accelerometer or self.gyroscope:
            self.accelGyroSensor = AccelGyro()
            self.accelGyroSensor.enable(
                accelerometer = self.accelerometer,
                gyroscope = self.gyroscope,
                temperature = self.temperature,
                autoIncrementRegisters = autoIncrementRegisters)

            if self.gyroscope:
                # "calibrate" tracked gyroscope angles
                self.calibrateGyroAngles()

        # Enable LPS25H if barometric pressure or temperature sensors
        # are requested
        if self.barometer or self.temperature:
            self.baroTempSensor = BaroTemp()
            self.baroTempSensor.enable(
                barometer = self.barometer,
                temperature = self.temperature,
                autoIncrementRegisters = autoIncrementRegisters)

        # Enable LIS3MDL if magnetometer is requested
        if self.magnetometer:
            self.magnetSensor = Magnet()
            self.magnetSensor.enable(
                magnetometer = self.magnetometer,
                temperature = self.temperature,
                autoIncrementRegisters = autoIncrementRegisters)


    def calibrateGyroAngles(self, xCal = 0.0, yCal = 0.0, zCal = 0.0):
        """ Calibrate (i.e. set to '0') the tracked gyroscope
            angles. (cf. self.trackGyroAngle())
        """
        self.gyrAngles = [xCal, yCal, zCal]


    def getGyroRotationRate(self, x = True, y = True, z = True):
        """ Get the rotation rate of the gyroscope for the requested
            axes. The result is returned as a vector (3-tuple) of
            floating point numbers representing the angular velocity
            in degrees/second.
        """
        # If gyroscope is not enabled or none of the dimensions is
        # requested make a quick turnaround
        if not (self.gyroscope and (x or y or z)):
            return (None, None, None)

        # Get raw data from gyroscope
        gyrRaw = self.accelGyroSensor.getGyroscopeRaw(
                                        x = x, y = y, z = z)

        # Calculate requested values
        gyrRates = [None if gyrRawDimension is None else gyrRawDimension * self.GYRO_GAIN for gyrRawDimension in gyrRaw]

        # Return result vector
        return tuple(gyrRates)


    def trackGyroAngle(self, x = True, y = True, z = True, deltaT = 0.02):
        """ Track gyrometer angle change over time delta deltaT.
            deltaT has to be extremely accurate, otherwise the gyroscope
            values will drift.
            The result is returned as a vector (3-tuple) of floating
            point numbers representing the angle in degrees.
        """
        # Assert if angles had been calibrated
        try:
            assert(len(self.gyrAngles) == 3)
        except AssertionError, e:
            raise(Exception('Gyroscope must be calibrated before angle tracking!'))

        # If gyroscope is not enabled or none of the dimensions is
        # requested make a quick turnaround
        if not (self.gyroscope and (x or y or z)):
            return tuple(self.gyrAngles)

        # Get current gyroscope rotation rate
        gyrRates = self.getGyroRotationRate(x = x, y = y, z = z)

        # Sum up and multiply by deltaT for angle tracking
        self.gyrAngles = [self.gyrAngles[i] if gyrRates is None else self.gyrAngles[i] + gyrRates[i] * deltaT for i in range(3)]

        return tuple(self.gyrAngles)
