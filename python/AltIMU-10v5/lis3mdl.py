#!/usr/bin/python

########### Python library module for ST LIS3MDL magnetometer ##########
#
# This module enables the Raspberry PI embedded computer to set up the
# ST LIS3MDL digital magnetometer.
#
# The ST LIS3MDL is an integral part of Pololu's AltIMU-10v5 and
# MinIMU-9v5 Inertial Measurement Units (IMUs).
# [https://www.pololu.com/product/2739]
# [https://www.pololu.com/product/2738]
#
# The datasheet for the ST LIS3MDL is available at
# [https://www.pololu.com/file/download/LIS3MDL.pdf?file_id=0J1089]
#
# This Python code was initially derived from Pololu's own C++ Arduino
# library, available at [https://github.com/pololu/lis3mdl-arduino]
#
# The Python code is developed and maintained by
# Torsten Kurbad <github@tk-webart.de>
#
########################################################################

# Imports
from i2c import I2C

from constants import *


# Code
class LIS3MDL(I2C):
    """ Class to set up and access LIS3MDL magnetometer.
    """

    ##
    ## Class variables and constants
    ##

    # Register addresses
    #  ([+] = used in the code, [-] = not used or useful, [ ] = TBD)
    LIS_WHO_AM_I    = 0x0F   # [-] Returns 0x3d (read only)

    LIS_CTRL_REG1   = 0x20   # [+] Control register to enable device, set
                         #     operating modes and rates for X and Y axes
    LIS_CTRL_REG2   = 0x21   # [+] Set gauss scale
    LIS_CTRL_REG3   = 0x22   # [+] Set operating/power modes
    LIS_CTRL_REG4   = 0x23   # [+] Set operating mode and rate for Z-axis
    LIS_CTRL_REG5   = 0x24   # [ ] Set fast read, block data update modes

    LIS_STATUS_REG  = 0x27   # [ ] Read device status (Is new data available?)

    LIS_OUT_X_L     = 0x28   # [+] X output, low byte
    LIS_OUT_X_H     = 0x29   # [+] X output, high byte
    LIS_OUT_Y_L     = 0x2A   # [+] Y output, low byte
    LIS_OUT_Y_H     = 0x2B   # [+] Y output, high byte
    LIS_OUT_Z_L     = 0x2C   # [+] Z output, low byte
    LIS_OUT_Z_H     = 0x2D   # [+] Z output, high byte

    LIS_TEMP_OUT_L  = 0x2E   # [+] Temperature output, low byte
    LIS_TEMP_OUT_H  = 0x2F   # [+] Temperature output, high byte

    LIS_INT_CFG     = 0x30   # [-] Interrupt generation config
    LIS_INT_SRC     = 0x31   # [-] Interrupt sources config
    LIS_INT_THS_L   = 0x32   # [-] Interrupt threshold, low byte
    LIS_INT_THS_H   = 0x33   # [-] Interrupt threshold, high byte

    # Output registers used by the magnetometer
    magRegisters = [
        LIS_OUT_X_L,    # low byte of X value
        LIS_OUT_X_H,    # high byte of X value
        LIS_OUT_Y_L,    # low byte of Y value
        LIS_OUT_Y_H,    # high byte of Y value
        LIS_OUT_Z_L,    # low byte of Z value
        LIS_OUT_Z_H,    # high byte of Z value
    ]

    # Output registers used by the temperature sensor
    lisTempRegisters = [
        LIS_TEMP_OUT_L, # low byte of temperature value
        LIS_TEMP_OUT_H, # high byte of temperature value
    ]


    ##
    ## Class methods
    ##

    ## Private methods
    def __init__(self, busId = 1):
        """ Set up I2C connection and initialize some flags and values.
        """
        super(LIS3MDL, self).__init__(busId)
        self.magEnabled = False
        self.lisTempEnabled = False


    def __del__(self):
        """ Clean up routines. """
        try:
            # Power down magnetometer
            self._writeRegister(LIS3MDL_ADDR, self.LIS_CTRL_REG3, 0x03)
            super(LIS3MDL, self).__del__()
        except:
            pass


    ## Public methods
    def enableLIS(self, magnetometer = True, temperature = True):
        """ Enable and set up the given sensors in the magnetometer
            device and determine whether to auto increment registers
            during I2C read operations.
        """
        # Disable magnetometer and temperature sensor first
        self._writeRegister(LIS3MDL_ADDR, self.LIS_CTRL_REG1, 0x00)
        self._writeRegister(LIS3MDL_ADDR, self.LIS_CTRL_REG3, 0x03)

        # Initialize flags
        self.magEnabled = False
        self.lisTempEnabled = False

        # Enable device in continuous conversion mode
        self._writeRegister(LIS3MDL_ADDR, self.LIS_CTRL_REG3, 0x00)

        # Initial value for CTRL_REG1
        ctrl_reg1 = 0x00

        if magnetometer:
            # Magnetometer

            # CTRL_REG1
            # Ultra-high-performance mode for X and Y
            # Output data rate 10Hz
            # 01110000b
            ctrl_reg1 += 0x70

            # CTRL_REG2
            # +/- 4 gauss full scale
            self._writeRegister(LIS3MDL_ADDR, self.LIS_CTRL_REG2, 0x00);

            # CTRL_REG4
            # Ultra-high-performance mode for Z
            # 00001100b
            self._writeRegister(LIS3MDL_ADDR, self.LIS_CTRL_REG4, 0x0c);

            self.magEnabled = True

        if temperature:
            # Temperature sensor enabled
            # 10000000b
            ctrl_reg1 += 0x80
            self.lisTempEnabled = True

        # Write calculated value to the CTRL_REG1 register
        self._writeRegister(LIS3MDL_ADDR, self.LIS_CTRL_REG1, ctrl_reg1)


    def getMagnetometerRaw(self):
        """ Return a 3-dimensional vector (list) of raw magnetometer
            data.
        """
        # Check if magnetometer has been enabled
        if not self.magEnabled:
            raise(Exception('Magnetometer has to be enabled first'))

        # Return raw sensor data
        return self._getSensorRawLoHi3(LIS3MDL_ADDR, self.magRegisters)


    def getLISTemperatureRaw(self):
        """ Return the raw temperature value. """
        # Check if device has been set up
        if not self.tempEnabled:
            raise(Exception('Temperature sensor has to be enabled first'))

        # Return raw sensor data
        return self._getSensorRawLoHi1(LIS3MDL_ADDR, self.lisTempRegisters)


    def getAllRaw(self, x = True, y = True, z = True):
        """ Return a 4-tuple of the raw output of the two sensors,
            magnetometer and temperature.
        """
        return self.getMagnetometerRaw() + [self.getTemperatureRaw()]


    def getLISTemperatureCelsius(self, rounded = True):
        """ Return the temperature sensor reading in C as a floating
            point number rounded to one decimal place.
        """
        # According to the datasheet, the raw temperature value is 0
        # @ 25 degrees Celsius and the resolution of the sensor is 8
        # steps per degree Celsius.
        # Thus, the following statement should return the temperature in
        # degrees Celsius.
        if rounded:
            return round(25.0 + self.getLISTemperatureRaw() / 8.0, 1)
        return 25.0 + self.getLISTemperatureRaw() / 8.0
