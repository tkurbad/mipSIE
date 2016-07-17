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
    def __init__(self, busId, address):
        """ Initialize the I2C bus and store device slave address. """
        self._i2c = SMBus(busId)
        self._address = address



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
