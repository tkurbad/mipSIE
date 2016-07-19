#!/usr/bin/python

######################## Python I2C library module #####################
#
# This module collects classes and helper methods for I2C SMBus access
# on a Raspberry PI computer.
#
# It is developed and maintained by
# Torsten Kurbad <github@tk-webart.de>
#
########################################################################

# Imports
from smbus import SMBus


# Code
class I2C(object):
    """ Class to set up and access I2C devices.
    """

    ##
    ## Class methods
    ##

    ## Private methods
    def __init__(self, busId = 1):
        """ Initialize the I2C bus. """
        self._i2c = SMBus(busId)


    def __del__(self):
        """ Clean up routines. """
        try:
            # Remove SMBus connection
            del(self._i2c)
        except:
            pass


    def _combineLoHi(self, loByte, hiByte):
        """ Combine low and high bytes to an unsigned 16 bit value. """
        return (loByte | hiByte << 8)


    def _combineSignedLoHi(self, loByte, hiByte):
        """ Combine low and high bytes to a signed 16 bit value. """
        combined = self._combineLoHi (loByte, hiByte)
        return combined if combined < 32768 else (combined - 65536)


    def _combineXLoLoHi(self, xloByte, loByte, hiByte):
        """ Combine extra low, low, and high bytes to an unsigned 24 bit
            value.
        """
        return (xloByte | loByte << 8 | hiByte << 16)


    def _combineSignedXLoLoHi(self, xloByte, loByte, hiByte):
        """ Combine extra low, low, and high bytes to a signed 24 bit
            value.
        """
        combined = self._combineXLoLoHi(xloByte, loByte, hiByte)
        return combined if combined < 8388608 else (combined - 16777216)


    def _getSensorRawLoHi1(self, address, outRegs):
        """ Return a scalar representing the combined raw signed 16 bit
            value of the output registers of a one-dimensional sensor,
            e.g. temperature.
            'address' is the I2C slave address.
            'outRegs' is a list of the output registers to read.
        """
        # Read register outputs and combine low and high byte values
        xl = self._readRegister(address, outRegs[0])
        xh = self._readRegister(address, outRegs[1])

        xVal = self._combineSignedLoHi(xl, xh)
        # Return the scalar
        return xVal


    def _getSensorRawXLoLoHi1(self, address, outRegs):
        """ Return a scalar representing the combined raw signed 24 bit
            value of the output registers of a one-dimensional sensor,
            e.g. temperature.
            'address' is the I2C slave address.
            'outRegs' is a list of the output registers to read.
        """
        # Read register outputs and combine low and high byte values
        xxl = self._readRegister(address, outRegs[0])
        xl = self._readRegister(address, outRegs[1])
        xh = self._readRegister(address, outRegs[2])

        xVal = self._combineSignedXLoLoHi(xxl, xl, xh)
        # Return the scalar
        return xVal


    def _getSensorRawLoHi3(self, address, outRegs):
        """ Return a vector (i.e. list) representing the combined
            raw signed 16 bit values of the output registers of a
            3-dimensional (IMU) sensor.
            'address' is the I2C slave address.
            'outRegs' is a list of the output registers to read.
        """
        # Read register outputs and combine low and high byte values
        xl = self._readRegister(address, outRegs[0])
        xh = self._readRegister(address, outRegs[1])
        yl = self._readRegister(address, outRegs[2])
        yh = self._readRegister(address, outRegs[3])
        zl = self._readRegister(address, outRegs[4])
        zh = self._readRegister(address, outRegs[5])

        xVal = self._combineSignedLoHi(xl, xh)
        yVal = self._combineSignedLoHi(yl, yh)
        zVal = self._combineSignedLoHi(zl, zh)

        # Return the vector
        return [xVal, yVal, zVal]


    def _readRegister(self, address, register):
        """ Read a single I2C register. """
        return self._i2c.read_byte_data(address, register)


    def _readRegisters(self, address, register, count):
        """ Read (up to 32) 'count' consecutive I2C registers. """
        return self._i2c.read_i2c_block_data(address, register, count)


    def _read(self, address):
        """ Read a single byte from the I2C device without specifying a
            register.
        """
        return self._i2c.read_byte(address)


    def _writeRegister(self, address, register, value):
        """ Write a single byte to a I2C register. Return the value the
            register had before the write.
        """
        valueOld = self._readRegister(address, register)
        self._i2c.write_byte_data(address, register, value)
        return valueOld


    def _write(self, address, value):
        """ Write a single byte to the I2C device without specifying a
            register.
        """
        return self._i2c.write_byte(address, value)


    def _testRegister(self, address, register):
        """ Check, if a I2C register is readable/accessible. """
        try:
            return self._readRegister(address, register)
        except:
            return -1
