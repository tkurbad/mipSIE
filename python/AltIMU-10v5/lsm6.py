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
        self._autoIncrementRegisters = False
        self.accEnabled = False
        self.gyroEnabled = False


    def __del__(self):
        del(self._i2c)


    def _combineHiLo(self, hiByte, loByte):
        return (hiByte << 8 | loByte)


    def _writeRegister(self, register, value):
        valueOld = self._readRegister(register)
        self._i2c.write_byte_data(self._address, register, value)


    def _readRegister(self, register):
        value = self._i2c.read_byte_data(self._address, register)
        return value


    def _read(self):
        value = self._i2c.read_byte(self._address)
        return value


    def getSensor(self, x, y, z, mode):

        try:
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

            # TODO: Handle timeouts
            xl = self._readRegister(registers['xl'])
            xh = self._read()
            yl = self._read()
            yh = self._read()
            zl = self._read()
            zh = self._read()

            # The permutations are listed canonically to provide maximum
            # performance
            if (x and y and z):
                return(self._combineHiLo(xh, xl),
                       self._combineHiLo(yh, yl),
                       self._combineHiLo(zh, zl))

            if (x and y and not z):
                return(self._combineHiLo(xh, xl),
                       self._combineHiLo(yh, yl),
                       None)

            if (x and not y and z):
                return(self._combineHiLo(xh, xl),
                       None,
                       self._combineHiLo(zh, zl))

            if (not x and y and z):
                return(None,
                       self._combineHiLo(yh, yl),
                       self._combineHiLo(zh, zl))

            if (x and not y and not z):
                return(self._combineHiLo(xh, xl),
                       None,
                       None)

            if (not x and y and not z):
                return(None,
                       self._combineHiLo(yh, yl),
                       None)

            if (not x and not y and z):
                return(None,
                       None,
                       self._combineHiLo(zh, zl))


        # Auto increment of registers is not enabled
        xval = yval = zval = None

        if x:
            xl = self._readRegister(registers('xl'))
            xh = self._readRegister(registers('xh'))

            xval = self._combineHiLo(xh, xl)

        if y:
            yl = self._readRegister(registers('yl'))
            yh = self._readRegister(registers('yh'))

            yval = self._combineHiLo(yh, yl)

        if z:
            zl = self._readRegister(registers('zl'))
            zh = self._readRegister(registers('zh'))

            zval = self._combineHiLo(zh, zl)

        return (xval, yval, zval)


    def enable(self, accelerometer = True, gyro = True, autoIncrementRegisters = True):
        # Disable accelerometer and gyro at first
        self._writeRegister(self.CTRL1_XL, 0x00)
        self._writeRegister(self.CTRL2_G, 0x00)
        self._writeRegister(self.CTRL3_C, 0x00)

        self._autoIncrementRegisters = False
        self.accEnabled = False
        self.gyroEnabled = False

        if autoIcrementRegisters:
            self._writeRegister(CTRL3_C, 0x04)
            self._autoIncrementRegisters = True

        if accelerometer:
            # Accelerometer
            self._writeRegister(CTRL1_XL, 0x80)
            self.accEnabled = True

        if gyro:
            # Gyro
            self._writeRegister(CTRL2_G, 0x80)
            self.gyroEnabled = True



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



#void LSM6::vector_normalize(vector<float> *a)
#{
  #float mag = sqrt(vector_dot(a, a));
  #a->x /= mag;
  #a->y /= mag;
  #a->z /= mag;
#}

#// Private Methods //////////////////////////////////////////////////////////////

#int16_t LSM6::testReg(uint8_t address, regAddr reg)
#{
  #Wire.beginTransmission(address);
  #Wire.write((uint8_t)reg);
  #if (Wire.endTransmission() != 0)
  #{
    #return TEST_REG_ERROR;
  #}

  #Wire.requestFrom(address, (uint8_t)1);
  #if (Wire.available())
  #{
    #return Wire.read();
  #}
  #else
  #{
    #return TEST_REG_ERROR;
  #}
#}

#{
  #public:
    #template <typename T> struct vector
    #{
      #T x, y, z;
    #};


    #vector<int16_t> a; // accelerometer readings
    #vector<int16_t> g; // gyro readings

    #uint8_t last_status; // status of last I2C transmission

    #LSM6(void);

    #bool init(deviceType device = device_auto, sa0State sa0 = sa0_auto);
    #deviceType getDeviceType(void) { return _device; }

    #void enableDefault(void);


    #void writeReg(uint8_t reg, uint8_t value);
    #uint8_t readReg(uint8_t reg);

    #void readAcc(void);
    #void readGyro(void);
    #void read(void);

    #void setTimeout(uint16_t timeout);
    #uint16_t getTimeout(void);
    #bool timeoutOccurred(void);

    #// vector functions
    #template <typename Ta, typename Tb, typename To> static void vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out);
    #template <typename Ta, typename Tb> static float vector_dot(const vector<Ta> *a, const vector<Tb> *b);
    #static void vector_normalize(vector<float> *a);

  #private:
    #deviceType _device; // chip type
    #uint8_t address;

    #uint16_t io_timeout;
    #bool did_timeout;

    #int16_t testReg(uint8_t address, regAddr reg);
#};


#template <typename Ta, typename Tb, typename To> void LSM6::vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out)
#{
  #out->x = (a->y * b->z) - (a->z * b->y);
  #out->y = (a->z * b->x) - (a->x * b->z);
  #out->z = (a->x * b->y) - (a->y * b->x);
#}

#template <typename Ta, typename Tb> float LSM6::vector_dot(const vector<Ta> *a, const vector<Tb> *b)
#{
  #return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
#}

##endif
