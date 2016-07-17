#!/usr/bin/python

######### Python library module for ST LPS25H digital barometer ########
#
# This module enables the Raspberry PI embedded computer to set up the
# ST LPS25H digital barometer.
#
# The ST LPS25H is an integral part of Pololu's AltIMU-10v5 Inertial
# Measurement Units (IMUs).
# [https://www.pololu.com/product/2739]
#
# The datasheet for the ST LPS25H is available at
# [https://www.pololu.com/file/download/LPS25H.pdf?file_id=0J761]
#
# This Python code was initially derived from Pololu's own C++ Arduino
# library, available at [https://github.com/pololu/lps-arduino]
#
# The Python code is developed and maintained by
# Torsten Kurbad <github@tk-webart.de>
#
########################################################################

# Imports
from smbus import SMBus

# Code
class LPS25H(object):
    """ Class to set up and access LIS3MDL magnetometer.
    """

    ##
    ## Class variables and constants
    ##


    # Register addresses
    #  ([+] = used in the code, [-] = not used or useful, [ ] = TBD)
    REF_P_XL        = 0x08  # [ ] Reference pressure, lowest byte
    REF_P_L         = 0x09  # [ ] Reference pressure, low byte
    REF_P_H         = 0x0A  # [ ] Reference pressure, high byte

    WHO_AM_I        = 0x0F  # [-] Returns 0xbd (read only)

    RES_CONF        = 0x10  # [ ] Set pressure and temperature resolution

    CTRL_REG1       = 0x20  # [+] Set device power mode / ODR / BDU
    CTRL_REG2       = 0x21  # [-] FIFO / I2C configuration
    CTRL_REG3       = 0x22  # [-] Interrupt configuration
    CTRL_REG4       = 0x23  # [-] Interrupt configuration
              
    INTERRUPT_CFG   = 0x24  # [-] Interrupt configuration
    INT_SOURCE      = 0x25  # [-] Interrupt source configuration

    STATUS_REG      = 0x27  # [ ] Status (new pressure/temperature data
                            #     available)
                            
    PRESS_OUT_XL    = 0x28  # [+] Pressure output, loweste byte
    PRESS_OUT_L     = 0x29  # [+] Pressure output, low byte
    PRESS_OUT_H     = 0x2A  # [+] Pressure output, high byte

    TEMP_OUT_L      = 0x2B  # [+] Temperature output, low byte
    TEMP_OUT_H      = 0x2C  # [+] Temperature output, high byte
      
    FIFO_CTRL       = 0x2E  # [ ] FIFO control / mode selection
    FIFO_STATUS     = 0x2F  # [-] FIFO status
      
    THS_P_L         = 0x30  # [-] Pressure interrupt threshold, low byte
    THS_P_H         = 0x31  # [-] Pressure interrupt threshold, high byte

    # The next two registers need special soldering and are not
    # available on the AltIMU
    RPDS_L          = 0x39  # [-] Pressure offset for differential
                            #     pressure computing, low byte
    RPDS_H          = 0x3A  # [-] Differential offset, high byte

    # Registers used for reference pressure
    refRegisters = dict(
        rxl = REF_P_XL, # lowest byte of reference pressure value
        rl  = REF_P_L,  # low byte of reference pressure value
        rh  = REF_P_H,  # high byte of reference pressure value
    )

    # Output registers used by the pressure sensor
    pressRegisters = dict (
        pxl = PRESS_OUT_XL, # lowest byte of pressure value
        pl  = PRESS_OUT_L,  # low byte of pressure value
        ph  = PRESS_OUT_H,  # high byte of pressure value
    )

    # Output registers used by the temperature sensor
    tempRegisters = dict (
        tl = TEMP_OUT_L, # low byte of temperature value
        th = TEMP_OUT_H, # high byte of temperature value
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
    def __init__(self, busId = 1, address = 0x5d):
        """ Initialize the I2C bus and store device slave address.
            Set initial sensor activation flags to False.
        """
        self._i2c = SMBus(busId)
        self._address = address
        self._autoIncrementRegisters = False
        self.pressEnabled = False
        self.tempEnabled = False


    def __del__(self):
        """ Clean up routines. """
        try:
            # Power down device
            self._writeRegister(self.CTRL_REG1, 0x00)
            # Remove SMBus connection
            del(self._i2c)
        except:
            pass


    def _combineLoHi(self, loByte, hiByte):
        """ Combine high and low bytes to a signed 16 bit value. """
        combined = (loByte | hiByte << 8)
        return combined if combined < 32768 else (combined - 65536)


    def _readRegister(self, register, count = None):
        """ Read I2C register(s).
            If count is a positive integer, do 'count' consecutive
            readings. If auto increment of register addresses is enabled
            for the device, the values of up to count = 32 consecutive
            registers can be read in one call.
        """
        if (count is None) or (count <= 1):
            # A single value has been requested
            return self._i2c.read_byte_data(self._address, register)
        else:
            # Read 'count' consecutive bytes, i.e. to read the output
            # of several registers at once
            return self._i2c.read_i2c_block_data(self._address, register, count)


    def _read(self):
        """ Read a single byte from the I2C device without specifying a
            register.
        """
        return self._i2c.read_byte(self._address)


    def _writeRegister(self, register, value):
        """ Write a single byte to a I2C register. Return the value the
            register had before the write.
        """
        valueOld = self._readRegister(register)
        self._i2c.write_byte_data(self._address, register, value)
        return valueOld


    def _write(self, value):
        """ Write a single byte to the I2C device without specifying a
            register.
        """
        return self._i2c.write_byte(self._address, value)


    def _testRegister(self, register):
        """ Check, if a I2C register is readable/accessible. """
        try:
            return self._readRegister(register)
        except:
            return -1


    ## Public methods
    def enable(self, magnetometer = True, temperature = True,
               autoIncrementRegisters = True):
        """ Enable and set up the given sensors in the IMU device and
            determine whether to auto increment registers during I2C
            read operations.
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
            # LISM3MDL. Instead, this feature is enabled by doing a raw
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

            # First write to OUT_X_L | 10000000b register address to
            # enable auto increment of registers
            self._write(self.magRegisters['xl'] | 0x80)

            # Now read all the magnetometer values in one go
            [xl, xh, yl, yh, zl, zh] = self._readRegister(
                self.magRegisters['xl'], count = 6)

            # In the return step we assess the requested vector
            # dimensions and return None for the ones that weren't
            # requested.
            return (self._combineLoHi(xl, xh) if x else None,
                    self._combineLoHi(yl, yh) if y else None,
                    self._combineLoHi(zl, zh) if z else None,)

        ## In case auto increment of registers is not enabled we have to
        #  read all registers consecutively.

        # Initialize return values
        xVal = yVal = zVal = None

        # Read register outputs for the requested dimensions and combine
        # low and high byte values
        if x:
            xl = self._readRegister(self.magRegisters['xl'])
            xh = self._readRegister(self.magRegisters['xh'])

            xVal = self._combineLoHi(xl, xh)

        if y:
            yl = self._readRegister(self.magRegisters['yl'])
            yh = self._readRegister(self.magRegisters['yh'])

            yVal = self._combineLoHi(yl, yh)

        if z:
            zl = self._readRegister(self.magRegisters['zl'])
            zh = self._readRegister(self.magRegisters['zh'])

            zVal = self._combineLoHi(zl, zh)

        # Return the vector
        return (xVal, yVal, zVal)


    def getTemperatureRaw(self):
        """ Return the raw temperature value. """
        # Check if device has been set up
        if not self.tempEnabled:
            raise(Exception('Temperature sensor has to be enabled first'))

        # Read temperature sensor data
        tl = self._readRegister(self.tempRegisters['tl'])
        th = self._readRegister(self.tempRegisters['th'])

        # Return combined result
        return self._combineLoHi(tl, th)


    def getAllRaw(self, x = True, y = True, z = True):
        """ Return a 7-tuple of the raw output of all three sensors,
            accelerometer, gyroscope, temperature.
        """
        return self.getMagnetometerRaw(x, y, z) \
                + (self.getTemperatureRaw(), )


    def getTemperatureCelsius(self):
        """ Return the temperature sensor reading in C as a floating
            point number rounded to one decimal place.
        """
        # According to the datasheet, the raw temperature value is 0
        # @ 25 degrees Celsius and the resolution of the sensor is 8
        # steps per degree Celsius.
        # Thus, the following statement should return the temperature in
        # degrees Celsius.
        return round(25.0 + self.getTemperatureRaw() / 8.0, 1)
