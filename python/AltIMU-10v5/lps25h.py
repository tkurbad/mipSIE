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
from i2c import I2C


# Code
class LPS25H(I2C):
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
        super(LPS25H, self).__init__(busId, address)
        self._autoIncrementRegisters = False
        self.pressEnabled = False
        self.tempEnabled = False


    def __del__(self):
        """ Clean up routines. """
        try:
            # Power down device
            self._writeRegister(self.CTRL_REG1, 0x00)
            super(LPS25H, self).__del__()
        except:
            pass


    ## Public methods
    def enable(self, barometer = True, temperature = True,
               autoIncrementRegisters = True):
        """ Enable and set up the given sensors in the IMU device and
            determine whether to auto increment registers during I2C
            read operations.
        """
        # Power down device first
        self._writeRegister(self.CTRL_REG1, 0x00)

        # Initialize flags
        self._autoIncrementRegisters = False
        self.pressEnabled = False
        self.tempEnabled = False

        if barometer or temperature:
            # Barometer and temperature sensor
            # (Both sensors are enabled together on the LPS25H)
            # CTRL_REG1
            # Power up
            # Output data rate for both sensors 12.5Hz
            # 10110000
            self._writeRegister(self.CTRL_REG1, 0xb0)

            self.pressEnabled = True
            self.tempEnabled = True

        if autoIncrementRegisters:
            # Auto increment register address during read
            # There is no control register for this setting on the
            # LPS25H. Instead, this feature is enabled by doing a raw
            # write to the desired start register | 0x80.
            self._autoIncrementRegisters = True


    def getBarometerRaw(self):
        """ Return the raw pressure sensor data.
        """
        # Check if barometer has been enabled
        if not self.pressEnabled:
            raise(Exception('Barometer has to be enabled first'))

        if self._autoIncrementRegisters:
            # First write to PRESS_OUT_XL | 10000000b register address
            # to enable auto increment of registers
            self._write(self.pressRegisters['pxl'] | 0x80)

            # Now read all the pressure bytes in one go
            [pxl, pl, ph] = self._readRegister(
                self.pressRegisters['pxl'], count = 3)

            # Return the combined signed 24 bit value
            return self._combineSignedXLoLoHi(pxl, pl, ph)

        ## In case auto increment of registers is not enabled we have to
        #  read all registers consecutively.
        pxl = self._readRegister(self.pressRegisters['pxl'])
        pl = self._readRegister(self.pressRegisters['pl'])
        ph = self._readRegister(self.pressRegisters['ph'])

        # Return the combined signed 24 bit value
        return self._combineSignedXLoLoHi(pxl, pl, ph)


    def getTemperatureRaw(self):
        """ Return the raw temperature value. """
        # Check if device has been set up
        if not self.tempEnabled:
            raise(Exception('Temperature sensor has to be enabled first'))

        # Read temperature sensor data
        tl = self._readRegister(self.tempRegisters['tl'])
        th = self._readRegister(self.tempRegisters['th'])

        # Return combined result
        return self._combineSignedLoHi(tl, th)


    def getAllRaw(self, x = True, y = True, z = True):
        """ Return a tuple of the raw output of the two sensors,
            pressure and temperature.
        """
        return (self.getBarometerRaw(), self.getTemperatureRaw(), )


    def getBarometerMillibars(self, rounded = True):
        """ Return the barometric pressure in millibars (mbar)
            (same as hectopascals (hPa)).
        """
        if rounded:
            return round(self.getBarometerRaw() / 4096.0, 1)
        return self.getBarometerRaw / 4096.0


    def getTemperatureCelsius(self, rounded = True):
        """ Return the temperature sensor reading in C as a floating
            point number rounded to one decimal place.
        """
        # According to the datasheet, the raw temperature value is 0
        # @ 42.5 degrees Celsius and the resolution of the sensor is 480
        # steps per degree Celsius.
        # Thus, the following statement should return the temperature in
        # degrees Celsius.
        if rounded:
            return round(42.5 + self.getTemperatureRaw() / 480.0, 1)
        return 42.5 + self.getTemperatureRaw() / 480.0


    def getAltitude(self, altimeterMbar = 1013.25, rounded = True):
        """ Return the altitude in meters above the standard pressure
            level of 1013.25 hPa, calculated using the 1976 US Standard
            Atmosphere model.
            altimeterMbar can be adjusted to the actual pressure
            "adjusted to sea level" (QNH) to compensate for regional
            and/or weather-based variations.
        """
        altitude = (1 - pow(self.getBarometerMillibars(rounded = False) / altimeterMbar, 0.190263)) * 44330.8
        if rounded:
            return round(altitude, 2)
        return altitude
