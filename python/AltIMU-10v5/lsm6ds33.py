#!/usr/bin/python

#### Python library module for LSM6DS33 accelerometer and gyroscope ####
#
# This module enables the Raspberry PI embedded computer to set up the
# STM LSM6DS33 integrated accelerometer, gyroscope and temperature MEMS.
#
# The STM LSM6DS33 is an integral part of Pololu's AltIMU-10v5 and
# MinIMU-9v5 Inertial Measurement Units (IMUs).
# [https://www.pololu.com/product/2739]
# [https://www.pololu.com/product/2738]
#
# The datasheet for the STM LSM6DS33 is available at
# [https://www.pololu.com/file/download/LSM6DS33.pdf?file_id=0J1087]
#
# This Python code was initially derived from Pololu's own C++ Arduino
# library, available at [https://github.com/pololu/lsm6-arduino]
#
# The Python code is developed and maintained by
# Torsten Kurbad <github@tk-webart.de>
#
########################################################################

# Imports
from smbus import SMBus

# Code
class LSM6DS33(object):
    """ Class to set up and access LSM6DS33 accelerometer and gyroscope.
    """

    ##
    ## Class variables and constants
    ##

    # Register addresses
    #  ([+] = used in the code, [-] = not used or useful, [ ] = TBD)
    FUNC_CFG_ACCESS   = 0x01  # [-] Configuration of embedded
                              #     functions, e.g. pedometer

    FIFO_CTRL1        = 0x06  # [-] FIFO threshold setting
    FIFO_CTRL2        = 0x07  # [-] FIFO control register
    FIFO_CTRL3        = 0x08  # [-] Gyro/Acceleromter-specific FIFO settings
    FIFO_CTRL4        = 0x09  # [-] FIFO data storage control
    FIFO_CTRL5        = 0x0A  # [-] FIFO ODR/Mode selection

    ORIENT_CFG_G      = 0x0B  # [ ] Gyroscope sign/orientation

    INT1_CTRL         = 0x0D  # [-] INT1 pad control - unavailable for AltIMU
    INT2_CTRL         = 0x0E  # [-] INT2 pad control - unavailable for AltIMU
    WHO_AM_I          = 0x0F  # [-] Returns 0x69 (read only)
    CTRL1_XL          = 0x10  # [+] Acceleration sensor control
    CTRL2_G           = 0x11  # [+] Angular rate sensor (gyroscope) control
    CTRL3_C           = 0x12  # [+] Device/communication settings
    CTRL4_C           = 0x13  # [ ] Bandwith/sensor/communication settings
    CTRL5_C           = 0x14  # [ ] Rounding/self-test control
    CTRL6_C           = 0x15  # [ ] Gyroscope settings
    CTRL7_G           = 0x16  # [ ] Gyroscope settings
    CTRL8_XL          = 0x17  # [ ] Acceleration sensor settings
    CTRL9_XL          = 0x18  # [ ] Acceleration sensor axis control
    CTRL10_C          = 0x19  # [ ] Gyroscope axis control / misc. settings

    WAKE_UP_SRC       = 0x1B  # [-] Wake up interrupt source register
    TAP_SRC           = 0x1C  # [-] Tap source register
    D6D_SRC           = 0x1D  # [-] Orientation sensing for Android devices

    STATUS_REG        = 0x1E  # [ ] Status register. Shows if new data
                              #     is available from one or more of the
                              #     sensors

    OUT_TEMP_L        = 0x20  # [+] Temperature output, low byte
    OUT_TEMP_H        = 0x21  # [+] Temperature output, high byte
    OUTX_L_G          = 0x22  # [+] Gyroscope X output, low byte
    OUTX_H_G          = 0x23  # [+] Gyroscope X output, high byte
    OUTY_L_G          = 0x24  # [+] Gyroscope Y output, low byte
    OUTY_H_G          = 0x25  # [+] Gyroscope Y output, high byte
    OUTZ_L_G          = 0x26  # [+] Gyroscope Z output, low byte
    OUTZ_H_G          = 0x27  # [+] Gyroscope Z output, high byte
    OUTX_L_XL         = 0x28  # [+] Accelerometer X output, low byte
    OUTX_H_XL         = 0x29  # [+] Accelerometer X output, high byte
    OUTY_L_XL         = 0x2A  # [+] Accelerometer Y output, low byte
    OUTY_H_XL         = 0x2B  # [+] Accelerometer Y output, high byte
    OUTZ_L_XL         = 0x2C  # [+] Accelerometer Z output, low byte
    OUTZ_H_XL         = 0x2D  # [+] Accelerometer Z output, high byte

    FIFO_STATUS1      = 0x3A  # [-] Number of unread words in FIFO
    FIFO_STATUS2      = 0x3B  # [-] FIFO status control register
    FIFO_STATUS3      = 0x3C  # [-] FIFO status control register
    FIFO_STATUS4      = 0x3D  # [-] FIFO status control register
    FIFO_DATA_OUT_L   = 0x3E  # [-] FIFO data output, low byte
    FIFO_DATA_OUT_H   = 0x3F  # [-] FIFO data output, high byte

    TIMESTAMP0_REG    = 0x40  # [-] Time stamp first byte data output
    TIMESTAMP1_REG    = 0x41  # [-] Time stamp second byte data output
    TIMESTAMP2_REG    = 0x42  # [-] Time stamp third byte data output

    STEP_TIMESTAMP_L  = 0x49  # [-] Time stamp of last step (for pedometer)
    STEP_TIMESTAMP_H  = 0x4A  # [-] Time stamp of last step, high byte
    STEP_COUNTER_L    = 0x4B  # [-] Step counter output, low byte
    STEP_COUNTER_H    = 0x4C  # [-] Step counter output, high byte

    FUNC_SRC          = 0x53  # [-] Interrupt source register for
                              #     embedded functions

    TAP_CFG           = 0x58  # [-] Configuration of embedded functions
    TAP_THS_6D        = 0x59  # [-] Orientation and tap threshold
    INT_DUR2          = 0x5A  # [-] Tap recognition settings
    WAKE_UP_THS       = 0x5B  # [-] Wake up threshold settings
    WAKE_UP_DUR       = 0x5C  # [-] Wake up function settings
    FREE_FALL         = 0x5D  # [-] Free fall duration settings
    MD1_CFG           = 0x5E  # [-] Function routing for INT1
    MD2_CFG           = 0x5F  # [-] Function routing for INT2


    # Output registers used by the accelerometer
    accRegisters = dict (
        xl = OUTX_L_XL,     # low byte of X value
        xh = OUTX_H_XL,     # high byte of X value
        yl = OUTY_L_XL,     # low byte of Y value
        yh = OUTY_H_XL,     # high byte of Y value
        zl = OUTZ_L_XL,     # low byte of Z value
        zh = OUTZ_H_XL,     # high byte of Z value
    )

    # Output registers used by the gyroscope
    gyroRegisters = dict (
        xl = OUTX_L_G,     # low byte of X value
        xh = OUTX_H_G,     # high byte of X value
        yl = OUTY_L_G,     # low byte of Y value
        yh = OUTY_H_G,     # high byte of Y value
        zl = OUTZ_L_G,     # low byte of Z value
        zh = OUTZ_H_G,     # high byte of Z value
    )

    # Output registers used by the temperature sensor
    tempRegisters = dict (
        tl = OUT_TEMP_L,   # low byte of temperature value
        th = OUT_TEMP_H,   # high byte of temperature value
    )


    ##
    ## Class methods
    ##

    ## Private methods
    def __init__(self, busId = 1, address = 0x6b):
        """ Initialize the I2C bus and store device slave address.
            Set initial sensor activation flags to False.
        """
        self._i2c = SMBus(busId)
        self._address = address
        self._autoIncrementRegisters = False
        self._ready = False
        self.accEnabled = False
        self.gyroEnabled = False


    def __del__(self):
        """ Clean up routines. """
        try:
            # Power down accelerometer
            self._writeRegister(self.CTRL1_XL, 0x00)
            # Power down gyroscope
            self._writeRegister(self.CTRL2_G, 0x00)
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


    def _testRegister(self, register):
        """ Check, if a I2C register is readable/accessible. """
        try:
            return self._readRegister(register)
        except:
            return -1


    def _getIMUSensorRaw(self, x, y, z, mode):
        """ Return a vector (i.e., a 3-tuple) representing the raw
            values of the output registers of one of the two IMU
            sensors, accelerometer or gyroscope.
            'mode' decides which of the two is to be read:
                mode == 0   -> accelerometer
                mode == 1   -> gyroscope
            'x', 'y', and 'z' are booleans to determine the requested
                vector dimensions.
        """
        try:
            # Does 'mode' contain an accepted value
            assert(mode in [0, 1])
        except AssertionError, e:
            raise ('getSensor accepts mode=0 (accelerometer) and mode=1 (gyroscope) only')

        # Accelerometer
        registers = self.accRegisters

        if mode == 1:
            # Gyroscope
            registers = self.gyroRegisters

        if self._autoIncrementRegisters:
            if (not x and not y and not z):
                # In case none of the values is requested, we can make
                # a quick turnaround
                return (None, None, None)

            # In any other case we read all the values first, because
            # this is a reasonably fast operation
            [xl, xh, yl, yh, zl, zh] = self._readRegister(registers['xl'], count = 6)

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
            xl = self._readRegister(registers['xl'])
            xh = self._readRegister(registers['xh'])

            xVal = self._combineLoHi(xl, xh)

        if y:
            yl = self._readRegister(registers['yl'])
            yh = self._readRegister(registers['yh'])

            yVal = self._combineLoHi(yl, yh)

        if z:
            zl = self._readRegister(registers['zl'])
            zh = self._readRegister(registers['zh'])

            zVal = self._combineLoHi(zl, zh)

        # Return the vector
        return (xval, yval, zval)


    def _vectorCross(vectorA, vectorB):
        """ Calculate vector cross product for a 3-dimensional vector.
        """
        try:
            # Check if input vectors are 3-dimensional
            assert((len(vectorA) == 3) and (len(vectorB) == 3))
        except AssertionError, e:
            raise(Exception('Input vectors have to be 3-dimensional'))

        try:
            # Check if all vector dimensions are set
            assert((dimension is not None) and isinstance(dimension, (int, long, float)) for dimension in vectorA)
            assert((dimension is not None) and isinstance(dimension, (int, long, float)) for dimension in vectorB)
        except AssertionError, e:
            raise(Exception('At least one dimension is not a number for one of the input vectors'))

        # Calculate and return cross product
        outX = vectorA[1] * vectorB[2] - vectorA[2] * vectorB[1]
        outY = vectorA[2] * vectorB[0] - vectorA[0] * vectorB[2]
        outZ = vectorA[0] * vectorB[1] - vectorA[1] * vectorB[0]

        return (outX, outY, outZ)


    def _vectorDot(vectorA, vectorB):
        """ Calculate vector dot product for a 3-dimensional vector. """
        try:
            # Check if input vectors are 3-dimensional
            assert((len(vectorA) == 3) and (len(vectorB) == 3))
        except AssertionError, e:
            raise(Exception('Input vectors have to be 3-dimensional'))

        try:
            # Check if all vector dimensions are set
            assert((dimension is not None) and isinstance(dimension, (int, long, float)) for dimension in vectorA)
            assert((dimension is not None) and isinstance(dimension, (int, long, float)) for dimension in vectorB)
        except AssertionError, e:
            raise(Exception('At least one dimension is not a number for one of the input vectors'))

        # Calculate and return dot product
        return vectorA[0] * vectorB[0] + vectorA[1] * vectorB[1] + vectorA[2] * vectorB[2]


    ## Public methods
    def enable(self, accelerometer = True, gyro = True,
               autoIncrementRegisters = True):
        """ Enable and set up the given sensors in the IMU device and
            determine whether to auto increment registers during I2C
            read operations.
        """
        # Disable accelerometer and gyroscope at first
        self._writeRegister(self.CTRL1_XL, 0x00)
        self._writeRegister(self.CTRL2_G, 0x00)
        self._writeRegister(self.CTRL3_C, 0x00)

        # Initialize flags
        self._autoIncrementRegisters = False
        self._ready = False
        self.accEnabled = False
        self.gyroEnabled = False

        # Disable FIFO
        self._writeRegister(self.FIFO_CTRL5, 0x00)

        # Prepare value for CTRL3_C register
        # Output not updated until MSB and LSB are read
        # 01000000b
        ctrl3_c = 0x40

        if accelerometer:
            # Accelerometer
            # 1.66 kHz / +/- 4g
            # 10001000b
            self._writeRegister(self.CTRL1_XL, 0x88)
            self.accEnabled = True

        if gyro:
            # Gyro
            # 1.66 kHz / 245 dps
            # 10000000b
            self._writeRegister(self.CTRL2_G, 0x80)
            self.gyroEnabled = True

        if autoIncrementRegisters:
            # Auto increment register address during read
            # 00000100b
            ctrl3_c = ctrl3_c + 0x04
            self._autoIncrementRegisters = True

        # Write calculated value to the CTRL3_C register
        self._writeRegister(self.CTRL3_C, ctrl3_c)

        # Set flag that device is ready
        self._ready = True


    def getAccelerometerRaw(self, x = True, y = True, z = True):
        """ Return a 3-dimensional vector (3-tuple) of raw accelerometer
            data.
            Booleans 'x', 'y', and 'z' can be used to request specific
                vector dimensions.
        """
        # Check if accelerometer has been enabled
        if not self.accEnabled:
            raise(Exception('Accelerometer has to be enabled first'))

        # Read sensor data
        return self._getIMUSensorRaw(x, y, z, mode = 0)


    def getGyroscopeRaw(self, x = True, y = True, z = True):
        """ Return a 3-dimensional vector (3-tuple) of raw accelerometer
            data.
            Booleans 'x', 'y', and 'z' can be used to request specific
                vector dimensions.
        """
        # Check if gyroscope has been enabled
        if not self.gyroEnabled:
            raise(Exception('Gyroscope has to be enabled first'))

        # Read sensor data
        return self._getIMUSensorRaw(x, y, z, mode = 1)


    def getTemperatureRaw(self):
        """ Return the raw temperature value. """
        # Check if device has been set up
        if not self._ready:
            raise(Exception('Device has to be enabled first'))

        # Read temperature sensor data
        if self._autoIncrementRegisters:
            # In case register auto increment as enabled, read both
            # temperature bytes in one go
            [tl, th] = self._readRegister(self.tempRegisters['tl'], count = 2)

            # Return combined result
            return self._combineLoHi(tl, th)

        # If register auto increment is disabled, read temperature bytes
        # discretely
        tl = self._readRegister(self.tempRegisters['tl'])
        th = self._readRegister(self.tempRegisters['th'])

        # Return combined result
        return self._combineLoHi(tl, th)


    def getIMURaw(self, x = True, y = True, z = True):
        """ Return a 6-tuple of the raw output values of both IMU
            sensors, accelerometer and gyroscope.
        """
        return self.getAccelerometerRaw(x, y, z) \
               + self.getGyroscopeRaw(x, y, z)


    def getAllRaw(self, x = True, y = True, z = True):
        """ Return a 7-tuple of the raw output of all three sensors,
            accelerometer, gyroscope, temperature.
        """
        return self.getAccelerometerRaw(x, y, z) \
                + self.getGyroscopeRaw(x, y, z) \
                + (self.getTemperatureRaw(), )


    def getTemperatureCelsius(self):
        """ Return the temperature sensor reading in C as a floating
            point number rounded to one decimal place.
        """
        # According to the datasheet, the raw temperature value is 0
        # @ 25 degrees Celsius and the resolution of the sensor is 16
        # steps per degree Celsius.
        # Thus, the following statement should return the temperature in
        # degrees Celsius.
        return round(25.0 + self.getTemperatureRaw() / 16.0, 1)


    def normalizeVector(self, vector):
        """ Do a vector normalization using dot product. """
        mag = sqrt(self._vectorDot(vector, vector))
        normVector = [dimension / mag for dimension in vector]
        return tuple(normVector)
