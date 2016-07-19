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

from constants import *
from lis3mdl import LIS3MDL
from lps25h import LPS25H
from lsm6ds33 import LSM6DS33


# Code
class AltIMU(LIS3MDL, LPS25H, LSM6DS33):
    """ Class to control Pololu's AltIMU-10v5. """

    # Private methods
    def __init__(self, busId = 1):
        """ Initialize some flags and values. """
        super(AltIMU, self).__init__()

        # Initialize tracked gyroscope angles
        self.gyrAngleX = 0.0
        self.gyrAngleY = 0.0
        self.gyrAngleZ = 0.0

        ## Initialize Kalman filter variables
        # Bias values for X, Y, and Z
        self.kalmanBiasX = 0.0
        self.kalmanBiasY = 0.0
        self.kalmanBiasZ = 0.0
        # State vectors for X, Y, and Z (XP_00, XP_01, XP_10, XP_11)
        self.kalmanXP_00 = self.kalmanXP_01 = 0.0
        self.kalmanXP_10 = self.kalmanXP_11 = 0.0
        self.kalmanYP_00 = self.kalmanYP_01 = 0.0
        self.kalmanYP_10 = self.kalmanYP_11 = 0.0
        self.kalmanZP_00 = self.kalmanZP_01 = 0.0
        self.kalmanZP_10 = self.kalmanZP_11 = 0.0
        # Kalman filtered angle values
        self.kalmanAngleX = 0.0
        self.kalmanAngleY = 0.0
        self.kalmanAngleZ = 0.0

        ## Initialize complementary filter variables
        self.complementaryAngleX = 0.0
        self.complementaryAngleY = 0.0
        self.complementaryAngleZ = 0.0

    def __del__(self):
        """ Cleanup routine. """
        super(AltIMU, self).__del__()


    # Public methods
    def enable(self, accelerometer = True, barometer = True,
               gyroscope = True, magnetometer = True,
               temperature = True):
        """ Enable the given devices. """
        # Enable LSM6DS33 if accelerometer and/or gyroscope are requested
        if accelerometer or gyroscope:
            self.enableLSM(accelerometer = accelerometer,
                           gyroscope = gyroscope,
                           temperature = temperature)

            if gyroscope:
                # "calibrate" tracked gyroscope angles
                self.calibrateGyroAngles()

        # Enable LPS25H if barometric pressure or temperature sensors
        # are requested
        if barometer or temperature:
            self.enableLPS()

        # Enable LIS3MDL if magnetometer is requested
        if magnetometer:
            self.enableLIS(magnetometer = magnetometer,
                           temperature = temperature)


    def calibrateGyroAngles(self, xCal = 0.0, yCal = 0.0, zCal = 0.0):
        """ Calibrate (i.e. set to '0') the tracked gyroscope
            angles. (cf. self.trackGyroAngle())
        """
        self.gyrAngles = [xCal, yCal, zCal]


    def getGyroRotationRates(self):
        """ Get the rotation rate of the gyroscope for the requested
            axes. The result is returned as a vector (list) of
            floating point numbers representing the angular velocity
            in degrees/second.
        """
        # Get raw data from gyroscope
        [gyrRawX, gyrRawY, gyrRawZ] = self.getGyroscopeRaw()

        # Calculate requested values
        gyrRateX = gyrRawX * GYRO_GAIN
        gyrRateY = gyrRawY * GYRO_GAIN
        gyrRateZ = gyrRawZ * GYRO_GAIN

        # Return result vector
        return [gyrRateX, gyrRateY, gyrRateZ]


    def trackGyroAngles(self, deltaT = 0.02):
        """ Track gyrometer angle change over time delta deltaT.
            deltaT has to be extremely accurate, otherwise the gyroscope
            values will drift.
            The result is returned as a vector (list) of floating
            point numbers representing the angle in degrees.
        """
        # Get current gyroscope rotation rate
        [gyrRateX, gyrRateY, gyrRateZ] = self.getGyroRotationRates()

        # Sum up and multiply by deltaT for angle tracking
        self.gyrAngleX += gyrRateX * deltaT
        self.gyrAngleY += gyrRateY * deltaT
        self.gyrAngleZ += gyrRateZ * deltaT

        return [self.gyrAngleX, self.gyrAngleY, self.gyrAngleZ]


    def getAccelerometerAngles(self):
        """ Calculate accelerometer angles. """
        # Get raw accelerometer data
        [accelXRaw, accelYRaw, accelZRaw] = self.getAccelerometerRaw()

        # Calculate angles
        accelXAngle = math.degrees(math.atan2(accelYRaw, accelZRaw) + math.pi)
        accelYAngle = math.degrees(math.atan2(accelXRaw, accelZRaw) + math.pi)
        accelZAngle = math.degrees(math.atan2(accelXRaw, accelYRaw) + math.pi)

        # Return vector
        return [accelXAngle, accelYAngle, accelZAngle]


    def getComplementaryAngles(self, deltaT = 0.05):
        """ Calculate combined angles of accelerometer and gyroscope
            using a complementary filter.
            Note: This filter is very cheap CPU-wise, but the result
            follows the drift of the gyroscope.
        """
        # If accelerometer or gyroscope is not enabled or none of the
        # dimensions is requested make a quick turnaround
        #if not (self.accelerometer and self.gyroscope and (x or y or z)):
        #    return (None, None, None)
        
        # Get gyroscope rotation rates and accelerometer angles
        gyrRates = self.getGyroRotationRates()
        accelAngles = self.getAccelerometerAngles()

        # Determine wether to initialize the complementary filter angles
        # from the accelerometer readings in the first iteration
        #if self.initComplementaryFromAccel:
        #    self.complementaryAngles = list(accelAngles)
        #    self.initComplementaryFromAccel = False

        # Calculate complementary filtered angles
        self.complementaryAngles = [None if (gyrRates[i] is None or accelAngles[i] is None)
            else self.C_FILTER_CONST * (self.complementaryAngles[i] + gyrRates[i] * deltaT)
            + (1 - self.C_FILTER_CONST) * accelAngles[i]
            for i in range(3)]

        # Return vector
        return tuple(self.complementaryAngles)


    def getKalmanAngles(self, deltaT = 0.05):
        """ Calculate combined angles of accelerometer and gyroscope
            using a Kalman filter.
            Note: This filter is complex, but eliminates gyroscope drift
            altogether.
        """
        def _calculateKalmanAngle(kalmanP_00,
                                  kalmanP_01,
                                  kalmanP_10,
                                  kalmanP_11,
                                  gyrRate,
                                  accAngle,
                                  kalmanBias,
                                  kalmanAngle,
                                  deltaT):
            """ Calculate Kalman filtered angle and return updated filter
                matrix for one dimension.
            """

            # Gyroscope part
            kalmanAngle += (gyrRate - kalmanBias) * deltaT

            kalmanP_00 += -deltaT * (kalmanP_01 + kalmanP_10) + K_Q_ANGLE * deltaT
            kalmanP_01 += -deltaT * kalmanP_11
            kalmanP_10 += -deltaT * kalmanP_11
            kalmanP_11 += K_Q_GYRO * deltaT

            # Accelerometer part
            kalY = accAngle - kalmanAngle
            kalS = kalmanP_00 + K_R_ANGLE
            kal0 = kalmanP_00 / kalS
            kal1 = kalmanP_10 / kalS

            # Set Kalman filtered angle
            kalmanAngle +=  kal0 * kalY

            # Re-calculate Kalman parameters
            kalmanBias +=  kal1 * kalY
            kalmanP_00 -= kal0 * kalmanP_00
            kalmanP_01 -= kal0 * kalmanP_01
            kalmanP_10 -= kal1 * kalmanP_10
            kalmanP_11 -= kal1 * kalmanP_11

            return (kalmanP_00, kalmanP_01, kalmanP_10, kalmanP_11,
                    kalmanBias, kalmanAngle)

        # If accelerometer or gyroscope is not enabled or none of the
        # dimensions is requested make a quick turnaround
        #if not (self.accelerometer and self.gyroscope and (x or y or z)):
        #    return (None, None, None)

        # Get gyroscope rotation rates and accelerometer angles
        [gyrRateX, gyrRateY, gyrRateZ] = self.getGyroRotationRates()
        [accAngleX, accAngleY, accAngleZ] = self.getAccelerometerAngles()

        # Determine wether to initialize the Kalman angles from the
        # accelerometer readings in the first iteration
        #if self.initKalmanFromAccel:
        #    self.kalmanAngles = list(accelAngles)
        #    self.initKalmanFromAccel = False

        # X axis
        (self.kalmanXP_00,
         self.kalmanXP_01,
         self.kalmanXP_10,
         self.kalmanXP_11,
         self.kalmanBiasX,
         self.kalmanX) = _calculateKalmanAngle(accAngleX,
                                               gyrRateX,
                                               self.kalmanXP_00,
                                               self.kalmanXP_01,
                                               self.kalmanXP_10,
                                               self.kalmanXP_11,
                                               self.kalmanBiasX,
                                               self.kalmanX,
                                               deltaT)

        # Y axis
        (self.kalmanYP_00,
         self.kalmanYP_01,
         self.kalmanYP_10,
         self.kalmanYP_11,
         self.kalmanBiasY,
         self.kalmanY) = _calculateKalmanAngle(accAngleY,
                                               gyrRateY,
                                               self.kalmanYP_00,
                                               self.kalmanYP_01,
                                               self.kalmanYP_10,
                                               self.kalmanYP_11,
                                               self.kalmanBiasY,
                                               self.kalmanY,
                                               deltaT)
        # Z axis
        (self.kalmanZP_00,
         self.kalmanZP_01,
         self.kalmanZP_10,
         self.kalmanZP_11,
         self.kalmanBiasZ,
         self.kalmanZ) = _calculateKalmanAngle(accAngleZ,
                                               gyrRateZ,
                                               self.kalmanZP_00,
                                               self.kalmanZP_01,
                                               self.kalmanZP_10,
                                               self.kalmanZP_11,
                                               self.kalmanBiasZ,
                                               self.kalmanZ,
                                               deltaT)

        # Return vector
        return [self.kalmanX, self.kalmanY, self.kalmanZ]
