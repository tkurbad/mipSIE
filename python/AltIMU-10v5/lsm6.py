#!/usr/bin/python

## Python library for LSM6DS33 accelerometer and gyroscope

from smbus import SMBus


class LSM6(object):
    """ Class to set up and access LSM6DS33 accelerometer and gyroscope.
    """

    ## Class variables and constants

    # Register addresses
    FUNC_CFG_ACCESS   = 0x01

    FIFO_CTRL1        = 0x06
    FIFO_CTRL2        = 0x07
    FIFO_CTRL3        = 0x08
    FIFO_CTRL4        = 0x09
    FIFO_CTRL5        = 0x0A
    ORIENT_CFG_G      = 0x0B

    INT1_CTRL         = 0x0D
    INT2_CTRL         = 0x0E
    WHO_AM_I          = 0x0F
    CTRL1_XL          = 0x10
    CTRL2_G           = 0x11
    CTRL3_C           = 0x12
    CTRL4_C           = 0x13
    CTRL5_C           = 0x14
    CTRL6_C           = 0x15
    CTRL7_G           = 0x16
    CTRL8_XL          = 0x17
    CTRL9_XL          = 0x18
    CTRL10_C          = 0x19

    WAKE_UP_SRC       = 0x1B
    TAP_SRC           = 0x1C
    D6D_SRC           = 0x1D
    STATUS_REG        = 0x1E

    OUT_TEMP_L        = 0x20
    OUT_TEMP_H        = 0x21
    OUTX_L_G          = 0x22
    OUTX_H_G          = 0x23
    OUTY_L_G          = 0x24
    OUTY_H_G          = 0x25
    OUTZ_L_G          = 0x26
    OUTZ_H_G          = 0x27
    OUTX_L_XL         = 0x28
    OUTX_H_XL         = 0x29
    OUTY_L_XL         = 0x2A
    OUTY_H_XL         = 0x2B
    OUTZ_L_XL         = 0x2C
    OUTZ_H_XL         = 0x2D

    FIFO_STATUS1      = 0x3A
    FIFO_STATUS2      = 0x3B
    FIFO_STATUS3      = 0x3C
    FIFO_STATUS4      = 0x3D
    FIFO_DATA_OUT_L   = 0x3E
    FIFO_DATA_OUT_H   = 0x3F
    TIMESTAMP0_REG    = 0x40
    TIMESTAMP1_REG    = 0x41
    TIMESTAMP2_REG    = 0x42

    STEP_TIMESTAMP_L  = 0x49
    STEP_TIMESTAMP_H  = 0x4A
    STEP_COUNTER_L    = 0x4B
    STEP_COUNTER_H    = 0x4C

    FUNC_SRC          = 0x53

    TAP_CFG           = 0x58
    TAP_THS_6D        = 0x59
    INT_DUR2          = 0x5A
    WAKE_UP_THS       = 0x5B
    WAKE_UP_DUR       = 0x5C
    FREE_FALL         = 0x5D
    MD1_CFG           = 0x5E
    MD2_CFG           = 0x5F

    # Registers used by the accelerometer
    accRegisters = dict (
        xl = OUTX_L_XL,     # low byte of X value
        xh = OUTX_H_XL,     # high byte of X value
        yl = OUTY_L_XL,     # low byte of Y value
        yh = OUTY_H_XL,     # high byte of Y value
        zl = OUTZ_L_XL,     # low byte of Z value
        zh = OUTZ_H_XL,     # high byte of Z value
    )

    # Registers used by the gyroscope
    gyroRegisters = dict (
        xl = OUTX_L_G,     # low byte of X value
        xh = OUTX_H_G,     # high byte of X value
        yl = OUTY_L_G,     # low byte of Y value
        yh = OUTY_H_G,     # high byte of Y value
        zl = OUTZ_L_G,     # low byte of Z value
        zh = OUTZ_H_G,     # high byte of Z value
    )

    def __init__(self, busId = 1, address = 0x6b):
        self._i2c = SMBus(busId)
        self._address = address
        self.accEnabled = False
        self.gyroEnabled = False


    def __del__(self):
        try:
            # Power down accelerometer
            self._writeRegister(self.CTRL1_XL, 0x00)
            # Power down gyroscope
            self._writeRegister(self.CTRL2_G, 0x00)
            # Remove SMBus connection
            del(self._i2c)
        except:
            pass


    def _combineHiLo(self, hiByte, loByte):
        combined = (loByte | hiByte << 8)
        return combined if combined < 32768 else (combined - 65536)


    def _readRegister(self, register):
        return self._i2c.read_byte_data(self._address, register)


    def _read(self):
        value = self._i2c.read_byte(self._address)
        return value


    def _writeRegister(self, register, value):
        valueOld = self._readRegister(register)
        self._i2c.write_byte_data(self._address, register, value)


    def _testRegister(self, register):
        try:
            return self._readRegister(register)
        except:
            return -1


    def _getSensor(self, x, y, z, mode):

        try:
            assert(mode in [0, 1])
        except AssertionError, e:
            raise ('getSensor accepts mode=0 (accelerometer) and mode=1 (gyroscope) only')

        # Accelerometer
        registers = self.accRegisters

        if mode == 1:
            # Gyroscope
            registers = self.gyroRegisters

        xval = yval = zval = None

        if x:
            xl = self._readRegister(registers['xl'])
            xh = self._readRegister(registers['xh'])

            xval = self._combineHiLo(xh, xl)

        if y:
            yl = self._readRegister(registers['yl'])
            yh = self._readRegister(registers['yh'])

            yval = self._combineHiLo(yh, yl)

        if z:
            zl = self._readRegister(registers['zl'])
            zh = self._readRegister(registers['zh'])

            zval = self._combineHiLo(zh, zl)

        return (xval, yval, zval)


    def _vectorCross(vectorA, vectorB):
        try:
            assert((len(vectorA) == 3) and (len(vectorB) == 3))
        except AssertionError, e:
            raise(Exception('Input vectors have to be 3-dimensional'))

        try:
            assert(val is not None for val in vectorA)
            assert(val is not None for val in vectorB)
        except AssertionError, e:
            raise(Exception('At least one dimension is None for one of the input vectors'))

        outX = vectorA[1] * vectorB[2] - vectorA[2] * vectorB[1]
        outY = vectorA[2] * vectorB[0] - vectorA[0] * vectorB[2]
        outZ = vectorA[0] * vectorB[1] - vectorA[1] * vectorB[0]

        return (outX, outY, outZ)


    def _vectorDot(vectorA, vectorB):
        try:
            assert((len(vectorA) == 3) and (len(vectorB) == 3))
        except AssertionError, e:
            raise(Exception('Input vectors have to be 3-dimensional'))

        try:
            assert(val is not None for val in vectorA)
            assert(val is not None for val in vectorB)
        except AssertionError, e:
            raise(Exception('At least one dimension is None for one of the input vectors'))

        return vectorA[0] * vectorB[0] + vectorA[1] * vectorB[1] + vectorA[2] * vectorB[2]


    def enable(self, accelerometer = True, gyro = True):
        # Disable accelerometer and gyro at first
        self._writeRegister(self.CTRL1_XL, 0x00)
        self._writeRegister(self.CTRL2_G, 0x00)
        self._writeRegister(self.CTRL3_C, 0x00)

        self.accEnabled = False
        self.gyroEnabled = False

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

        # Generic
        # Disable FIFO
        self._writeRegister(self.FIFO_CTRL5, 0x00)
        # Output not updated until MSB and LSB are read
        # 01000000b
        self._writeRegister(self.CTRL3_C, 0x40)


    def getAccelerometer(self, x = True, y = True, z = True):
        if not self.accEnabled:
            raise(Exception('Accelerometer has to be enabled first'))

        return self._getSensor(x, y, z, mode = 0)


    def getGyroscope(self, x = True, y = True, z = True):
        if not self.gyroEnabled:
            raise(Exception('Gyroscope has to be enabled first'))

        return self._getSensor(x, y, z, mode = 1)


    def getAll(self, x = True, y = True, z = True):
        return self.getAccelerometer(x, y, z) + self.getGyroscope(x, y, z)


    def normalizeVector(self, vector):
        mag = sqrt(self._vectorDot(vector, vector))
        normVector = [dimension / mag for dimension in vector]
        return tuple(normVector)
