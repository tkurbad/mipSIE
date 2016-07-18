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


# Code
class LIS3MDL(I2C):
    """ Class to set up and access LIS3MDL magnetometer.
    """

    ##
    ## Class variables and constants
    ##

    # Register addresses
    #  ([+] = used in the code, [-] = not used or useful, [ ] = TBD)
    WHO_AM_I    = 0x0F   # [-] Returns 0x3d (read only)

    CTRL_REG1   = 0x20   # [+] Control register to enable device, set
                         #     operating modes and rates for X and Y axes
    CTRL_REG2   = 0x21   # [+] Set gauss scale
    CTRL_REG3   = 0x22   # [+] Set operating/power modes
    CTRL_REG4   = 0x23   # [+] Set operating mode and rate for Z-axis
    CTRL_REG5   = 0x24   # [ ] Set fast read, block data update modes

    STATUS_REG  = 0x27   # [ ] Read device status (Is new data available?)

    OUT_X_L     = 0x28   # [+] X output, low byte
    OUT_X_H     = 0x29   # [+] X output, high byte
    OUT_Y_L     = 0x2A   # [+] Y output, low byte
    OUT_Y_H     = 0x2B   # [+] Y output, high byte
    OUT_Z_L     = 0x2C   # [+] Z output, low byte
    OUT_Z_H     = 0x2D   # [+] Z output, high byte

    TEMP_OUT_L  = 0x2E   # [+] Temperature output, low byte
    TEMP_OUT_H  = 0x2F   # [+] Temperature output, high byte

    INT_CFG     = 0x30   # [-] Interrupt generation config
    INT_SRC     = 0x31   # [-] Interrupt sources config
    INT_THS_L   = 0x32   # [-] Interrupt threshold, low byte
    INT_THS_H   = 0x33   # [-] Interrupt threshold, high byte

    # Output registers used by the magnetometer
    magRegisters = dict (
        xl = OUT_X_L,    # low byte of X value
        xh = OUT_X_H,    # high byte of X value
        yl = OUT_Y_L,    # low byte of Y value
        yh = OUT_Y_H,    # high byte of Y value
        zl = OUT_Z_L,    # low byte of Z value
        zh = OUT_Z_H,    # high byte of Z value
    )

    # Output registers used by the temperature sensor
    tempRegisters = dict (
        tl = TEMP_OUT_L, # low byte of temperature value
        th = TEMP_OUT_H, # high byte of temperature value
    )


    ##
    ## Class methods
    ##

    ## Private methods
    def __init__(self, busId = 1, address = 0x1e):
        """ Initialize the I2C bus and store device slave address.
            Set initial sensor activation flags to False.
        """
        super(LIS3MDL, self).__init__(busId, address)
        self._autoIncrementRegisters = False
        self.magEnabled = False
        self.tempEnabled = False


    def __del__(self):
        """ Clean up routines. """
        try:
            # Power down magnetometer
            self._writeRegister(self.CTRL_REG3, 0x03)
            super(LIS3MDL, self).__del__()
        except:
            pass


    ## Public methods
    def enable(self, magnetometer = True, temperature = True,
               autoIncrementRegisters = True):
        """ Enable and set up the given sensors in the magnetometer
            device and determine whether to auto increment registers
            during I2C read operations.
        """
        # Disable magnetometer and temperature sensor first
        self._writeRegister(self.CTRL_REG1, 0x00)
        self._writeRegister(self.CTRL_REG3, 0x03)

        # Initialize flags
        self._autoIncrementRegisters = False
        self.magEnabled = False
        self.tempEnabled = False

        # Enable device in continuous conversion mode
        self._writeRegister(self.CTRL_REG3, 0x00)

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
            self._writeRegister(self.CTRL_REG2, 0x00);

            # CTRL_REG4
            # Ultra-high-performance mode for Z
            # 00001100b
            self._writeRegister(self.CTRL_REG4, 0x0c);

            self.magEnabled = True

        if temperature:
            # Temperature sensor enabled
            # 10000000b
            ctrl_reg1 += 0x80
            self.tempEnabled = True

        if autoIncrementRegisters:
            # Auto increment register address during read
            # There is no control register for this setting on the
            # LIS3MDL. Instead, this feature is enabled by doing a raw
            # write to the desired start register | 0x80.
            self._autoIncrementRegisters = True

        # Write calculated value to the CTRL_REG1 register
        self._writeRegister(self.CTRL_REG1, ctrl_reg1)


    def getMagnetometerRaw(self, x = True, y = True, z = True):
        """ Return a 3-dimensional vector (3-tuple) of raw magnetometer
            data.
            Booleans 'x', 'y', and 'z' can be used to request specific
                vector dimensions.
        """
        # Check if magnetometer has been enabled
        if not self.magEnabled:
            raise(Exception('Magnetometer has to be enabled first'))

        if self._autoIncrementRegisters:
            if (not x and not y and not z):
                # In case none of the values is requested, we can make
                # a quick turnaround
                return (None, None, None)

            # In any other case we read all the values first, because
            # this is a reasonably fast operation

            # To read all the magnetometer values in one go, the start
            # register has to be ORed by 0x80 (10000000b)
            [xl, xh, yl, yh, zl, zh] = self._readRegister(
                (self.magRegisters['xl'] | 0x80), count = 6)

            # In the return step we assess the requested vector
            # dimensions and return None for the ones that weren't
            # requested.
            return (self._combineSignedLoHi(xl, xh) if x else None,
                    self._combineSignedLoHi(yl, yh) if y else None,
                    self._combineSignedLoHi(zl, zh) if z else None,)

        ## In case auto increment of registers is not enabled we have to
        #  read all registers consecutively.

        # Initialize return values
        xVal = yVal = zVal = None

        # Read register outputs for the requested dimensions and combine
        # low and high byte values
        if x:
            xl = self._readRegister(self.magRegisters['xl'])
            xh = self._readRegister(self.magRegisters['xh'])

            xVal = self._combineSignedLoHi(xl, xh)

        if y:
            yl = self._readRegister(self.magRegisters['yl'])
            yh = self._readRegister(self.magRegisters['yh'])

            yVal = self._combineSignedLoHi(yl, yh)

        if z:
            zl = self._readRegister(self.magRegisters['zl'])
            zh = self._readRegister(self.magRegisters['zh'])

            zVal = self._combineSignedLoHi(zl, zh)

        # Return the vector
        return (xVal, yVal, zVal)


    def getTemperatureRaw(self):
        """ Return the raw temperature value. """
        # Check if device has been set up
        if not self.tempEnabled:
            raise(Exception('Temperature sensor has to be enabled first'))

        if self._autoIncrementRegisters:
            # To read both temperature bytes in one go, the start
            # register has to be ORed by 0x80 (10000000b)
            [tl, th] = self._readRegister(
                (self.tempRegisters['tl'] | 0x80), count = 2)

            # Return combined signed 16 bit value
            return self._combineSignedLoHi(tl, th)

        ## In case auto increment of registers is not enabled we have to
        #  read all registers consecutively.
        tl = self._readRegister(self.tempRegisters['tl'])
        th = self._readRegister(self.tempRegisters['th'])

        # Return combined result
        return self._combineSignedLoHi(tl, th)


    def getAllRaw(self, x = True, y = True, z = True):
        """ Return a 4-tuple of the raw output of the two sensors,
            magnetometer and temperature.
        """
        return self.getMagnetometerRaw(x, y, z) \
                + (self.getTemperatureRaw(), )


    def getTemperatureCelsius(self, rounded = True):
        """ Return the temperature sensor reading in C as a floating
            point number rounded to one decimal place.
        """
        # According to the datasheet, the raw temperature value is 0
        # @ 25 degrees Celsius and the resolution of the sensor is 8
        # steps per degree Celsius.
        # Thus, the following statement should return the temperature in
        # degrees Celsius.
        if rounded:
            return round(25.0 + self.getTemperatureRaw() / 8.0, 1)
        return 25.0 + self.getTemperatureRaw() / 8.0
