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
import math

from time import sleep

from lis3mdl import LIS3MDL as Magnet
from lps25h import LPS25H as BaroTemp
from lsm6ds33 import LSM6DS33 as AccelGyro


# Code
class AltIMU(object):
    """ Class to control Pololu's AltIMU-10v5. """

    # Class variables and constants
    GYRO_GAIN       = 0.0175  # Gyroscope dps/LSB for 500 dps full scale

    # Used by complementary filter
    C_FILTER_CONST  = 0.98    # Complementary filter constant

    # Used by the Kalman filter
    Q_ANGLE   = 0.01
    Q_GYRO    = 0.0003
    R_ANGLE   = 0.01


    # Private methods
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

        ## Initialize Kalman filter variables
        # Bias values for X, Y, and Z
        self.kalmanBias = [0.0, 0.0, 0.0]
        # State vectors for X, Y, and Z (XP_00, XP_01, XP_10, XP_11)
        self.kalmanXP = self.kalmanYP = self.kalmanZP = [0.0, 0.0, 0.0, 0.0]
        # Kalman filtered angle values
        self.kalmanAngles = [0.0, 0.0, 0.0]

        ## Initialize complementary filter variables
        self.complementaryAngles = [0.0, 0.0, 0.0]

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


    def _calculateKalmanAngle(self, kalmanP, accelAngles, axis, deltaT):
        """ Calculate Kalman filtered angle and return updated filter
            matrix for one dimension.
        """
        kalmanP[0] += -deltaT * (kalmanP[1] + kalmanP[2]) + self.Q_ANGLE * deltaT
        kalmanP[1] += -deltaT * kalmanP[3]
        kalmanP[2] += -deltaT * kalmanP[3]
        kalmanP[3] += self.Q_GYRO * deltaT

        kalY = accelAngles[axis] - self.kalmanAngles[axis]
        kalS = kalmanP[0] + self.R_ANGLE
        kal0 = kalmanP[0] / kalS
        kal1 = kalmanP[2] / kalS

        # Set Kalman filtered angle
        self.kalmanAngles[axis] +=  kal0 * kalY

        self.kalmanBias[axis] +=  kal1 * kalY
        kalmanP[0] -= kal0 * kalmanP[0]
        kalmanP[1] -= kal0 * kalmanP[1]
        kalmanP[2] -= kal1 * kalmanP[0]
        kalmanP[3] -= kal1 * kalmanP[1]

        return kalmanP


    # Public methods
    def enable(self, accelerometer = True, barometer = True,
               gyroscope = True, magnetometer = True,
               temperature = True, autoIncrementRegisters = True,
               initFiltersFromAccel = True):
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

        # Determine wether Kalman/complementary filters should be
        # initialized with accelerometer readings
        self.initFiltersFromAccel = initFiltersFromAccel

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

        # Disable Kalman filter initialization if accelerometer is
        # deactivated.
        if not self.accelerometer:
            self.initFiltersFromAccel = False


    def calibrateGyroAngles(self, xCal = 0.0, yCal = 0.0, zCal = 0.0):
        """ Calibrate (i.e. set to '0') the tracked gyroscope
            angles. (cf. self.trackGyroAngle())
        """
        self.gyrAngles = [xCal, yCal, zCal]


    def getGyroRotationRates(self, x = True, y = True, z = True):
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
        gyrRates = [None if gyrRawDimension is None
                        else gyrRawDimension * self.GYRO_GAIN
                        for gyrRawDimension in gyrRaw]

        # Return result vector
        return tuple(gyrRates)


    def trackGyroAngles(self, x = True, y = True, z = True, deltaT = 0.0002):
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
        gyrRates = self.getGyroRotationRates(x = x, y = y, z = z)

        # Sum up and multiply by deltaT for angle tracking
        self.gyrAngles = [self.gyrAngles[i] if gyrRates is None
                            else self.gyrAngles[i] + gyrRates[i] * deltaT
                            for i in range(3)]

        return tuple(self.gyrAngles)


    def getAccelerometerAngles(self, x = True, y = True, z = True,
        initKalman = False):
        """ Calculate accelerometer angles.
            If initKalman parameter is set to True, initialize Kalman
            filter values with accelerometer readings.
        """
        # If accelerometer is not enabled or none of the dimensions is
        # requested make a quick turnaround
        if not (self.accelerometer and (x or y or z)):
            return (None, None, None)

        # Get raw accelerometer data
        (accelXRaw, accelYRaw, accelZRaw) = self.accelGyroSensor.getAccelerometerRaw(
                                                x = x, y = y, z = z)

        # Calculate angles
        accelXAngle = math.degrees(math.atan2(accelYRaw, accelZRaw) + math.pi)
        accelYAngle = math.degrees(math.atan2(accelXRaw, accelZRaw) + math.pi)
        accelZAngle = math.degrees(math.atan2(accelXRaw, accelYRaw) + math.pi)

        # Return vector
        return (accelXAngle, accelYAngle, accelZAngle)


    def getComplementaryAngles(self, x = True, y = True, z = True, deltaT = 0.05):
        """ Calculate combined angles of accelerometer and gyroscope
            using a complementary filter.
        """
        # If accelerometer or gyroscope is not enabled or none of the
        # dimensions is requested make a quick turnaround
        if not (self.accelerometer and self.gyroscope and (x or y or z)):
            return (None, None, None)
        
        # Get gyroscope rotation rates and accelerometer angles
        gyrRates = self.getGyroRotationRates(x = x, y = y, z = z)
        accelAngles = self.getAccelerometerAngles(x = x, y = y, z = z)

        # Determine wether to initialize the Kalman angles from the
        # accelerometer readings in the first iteration
        if self.initFiltersFromAccel:
            self.complementaryAngles = list(accelAngles)
            self.initFiltersFromAccel = False

        # Calculate complementary filtered angles
        self.complementaryAngles = [None if (gyrRates[i] is None or accAngles[i] is None)
            else self.C_FILTER_CONST * (self.complementaryAngles[i] + gyrRates[i] * deltaT)
            + (1 - self.C_FILTER_CONST) * accAngles[i]
            for i in range(3)]

        # Return vector
        return tuple(self.complementaryAngles)


    def getKalmanAngles(self, x = True, y = True, z = True, deltaT = 0.05):
        """ Calculate combined angles of accelerometer and gyroscope
            using a Kalman filter.
        """
        # If accelerometer or gyroscope is not enabled or none of the
        # dimensions is requested make a quick turnaround
        if not (self.accelerometer and self.gyroscope and (x or y or z)):
            return (None, None, None)

        # Get gyroscope rotation rates and accelerometer angles
        gyrRates = self.getGyroRotationRates(x = x, y = y, z = z)
        accelAngles = self.getAccelerometerAngles(x = x, y = y, z = z)

        # Determine wether to initialize the Kalman angles from the
        # accelerometer readings in the first iteration
        if self.initFiltersFromAccel:
            self.kalmanAngles = list(accelAngles)
            self.initFiltersFromAccel = False

        # Calculate gyroscope parts
        self.kalmanAngles = [None if gyrRates[i] is None
                                else self.kalmanAngles[i] + (gyrRates[i] - self.kalmanBias[i]) * deltaT
                                for i in range(3)]

        # Calculate accelerometer parts
        if x:
            self._calculateKalmanAngle(self.kalmanXP, accelAngles, 0, deltaT)
        if y:
            self._calculateKalmanAngle(self.kalmanYP, accelAngles, 1, deltaT)
        if z:
            self._calculateKalmanAngle(self.kalmanZP, accelAngles, 2, deltaT)

        # Return vector
        return tuple(self.kalmanAngles)
