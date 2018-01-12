/*
MPU9250.h
Brian R Taylor
brian.taylor@bolderflight.com
2017-01-03

Copyright (c) 2016 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef MPU9250_h
#define MPU9250_h

#include "Arduino.h"
#include "i2c_t3.h"  // I2C library
#include "SPI.h" // SPI Library

#ifndef SPI_MOSI_PIN
#define SPI_MOSI_PIN
    // Teensy 3.0 || Teensy 3.1/3.2
    #if defined(__MK20DX128__) || defined(__MK20DX256__)
    enum spi_mosi_pin
    {
      MOSI_PIN_7,
      MOSI_PIN_11
    };
    #endif
    // Teensy 3.5 || Teensy 3.6
    #if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    enum spi_mosi_pin
    {
      MOSI_PIN_0,
      MOSI_PIN_7,
      MOSI_PIN_11,
      MOSI_PIN_21,
      MOSI_PIN_28,
      MOSI_PIN_44,
      MOSI_PIN_52
    };
    #endif
    // Teensy LC
    #if defined(__MKL26Z64__)
    enum spi_mosi_pin
    {
      MOSI_PIN_0,
      MOSI_PIN_7,
      MOSI_PIN_11,
      MOSI_PIN_21
    };
    #endif
#endif

enum mpu9250_gyro_range
{
    GYRO_RANGE_250DPS,
    GYRO_RANGE_500DPS,
    GYRO_RANGE_1000DPS,
    GYRO_RANGE_2000DPS
};

enum mpu9250_accel_range
{
    ACCEL_RANGE_2G,
    ACCEL_RANGE_4G,
    ACCEL_RANGE_8G,
    ACCEL_RANGE_16G
};

enum mpu9250_dlpf_bandwidth
{
    DLPF_BANDWIDTH_184HZ,
    DLPF_BANDWIDTH_92HZ,
    DLPF_BANDWIDTH_41HZ,
    DLPF_BANDWIDTH_20HZ,
    DLPF_BANDWIDTH_10HZ,
    DLPF_BANDWIDTH_5HZ
};

        // MPU9250 registers
#define	 ACCEL_OUT  0x3B
#define	 GYRO_OUT  0x43
#define	 TEMP_OUT  0x41
#define	 EXT_SENS_DATA_00  0x49

#define	 ACCEL_CONFIG  0x1C
#define	 ACCEL_FS_SEL_2G  0x00
#define	 ACCEL_FS_SEL_4G  0x08
#define	 ACCEL_FS_SEL_8G  0x10
#define	 ACCEL_FS_SEL_16G  0x18

#define	 GYRO_CONFIG  0x1B
#define	 GYRO_FS_SEL_250DPS  0x00
#define	 GYRO_FS_SEL_500DPS  0x08
#define	 GYRO_FS_SEL_1000DPS  0x10
#define	 GYRO_FS_SEL_2000DPS  0x18

#define	 ACCEL_CONFIG2  0x1D
#define	 ACCEL_DLPF_184  0x01
#define	 ACCEL_DLPF_92  0x02
#define	 ACCEL_DLPF_41  0x03
#define	 ACCEL_DLPF_20  0x04
#define	 ACCEL_DLPF_10  0x05
#define	 ACCEL_DLPF_5  0x06

#define	 CONFIG  0x1A
#define	 GYRO_DLPF_184  0x01
#define	 GYRO_DLPF_92  0x02
#define	 GYRO_DLPF_41  0x03
#define	 GYRO_DLPF_20  0x04
#define	 GYRO_DLPF_10  0x05
#define	 GYRO_DLPF_5  0x06

#define	 SMPDIV  0x19

#define	 INT_PIN_CFG  0x37
#define	 INT_ENABLE  0x38
#define	 INT_DISABLE  0x00
#define	 INT_PULSE_50US  0x00
#define	 INT_PULSE_READ  0x20
#define	 INT_RAW_RDY_EN  0x01
#define  DMP_INT_STATUS   0x39  // Check DMP interrupt
#define  INT_STATUS       0x3A


#define	 PWR_MGMNT_1  0x6B
#define	 PWR_RESET  0x80
#define	 CLOCK_SEL_PLL  0x01

#define	 PWR_MGMNT_2  0x6C
#define	 SEN_ENABLE  0x00

#define	 USER_CTRL  0x6A
#define	 I2C_MST_EN  0x20
#define	 I2C_MST_CLK  0x0D
#define	 I2C_MST_CTRL  0x24
#define	 I2C_SLV0_ADDR  0x25
#define	 I2C_SLV0_REG  0x26
#define	 I2C_SLV0_DO  0x63
#define	 I2C_SLV0_CTRL  0x27
#define	 I2C_SLV0_EN  0x80
#define	 I2C_READ_FLAG  0x80

#define	 WHO_AM_I  0x75

        // AK8963 registers
#define	 AK8963_I2C_ADDR  0x0C

#define	 AK8963_HXL  0x03

#define	 AK8963_CNTL1  0x0A
#define	 AK8963_PWR_DOWN  0x00
#define	 AK8963_CNT_MEAS1  0x12 // 0b00010010
#define	 AK8963_CNT_MEAS2  0x16 // 0b00010110
#define	 AK8963_FUSE_ROM  0x0F  // 0b00001111

#define	 AK8963_CNTL2  0x0B
#define	 AK8963_RESET  0x01

#define	 AK8963_ASA  0x10

#define	 AK8963_WHO_AM_I  0x00

       // SPI constants
#define	 SPI_READ  0x80
#define	 SPI_LS_CLOCK  1000000 // 1 MHz
#define	 SPI_HS_CLOCK  20000000 // 20 MHz

        // i2c bus frequency
#define	 _i2cRate  400000

class MPU9250{
    public:
        MPU9250(uint8_t address, uint8_t bus);
        MPU9250(uint8_t address, uint8_t bus, i2c_pins pins);
        MPU9250(uint8_t address, uint8_t bus, i2c_pins pins, i2c_pullup pullups);
        MPU9250(uint8_t csPin);
        MPU9250(uint8_t csPin, spi_mosi_pin pin);
        int begin(mpu9250_accel_range accelRange, mpu9250_gyro_range gyroRange, float regionalMagDeclination =0.);
        int setFilt(mpu9250_dlpf_bandwidth bandwidth, uint8_t SRD);
        int enableInt(bool enable);
        void getAccel(float* ax, float* ay, float* az);
        void getGyro(float* gx, float* gy, float* gz);
        void getMag(float* hx, float* hy, float* hz);
        void getTemp(float *t);
        void getMotion6(float* ax, float* ay, float* az, float* gx, float* gy, float* gz);
        void getMotion7(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* t);
        void getMotion9(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy, float* hz);
        void getMotion10(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy, float* hz, float* t);

        void getAccelCounts(int16_t* ax, int16_t* ay, int16_t* az);
        void getGyroCounts(int16_t* gx, int16_t* gy, int16_t* gz);
        void getMagCounts(int16_t* hx, int16_t* hy, int16_t* hz);
        void getTempCounts(int16_t* t);
        void getMotion6Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
        void getMotion7Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* t);
        void getMotion9Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* hx, int16_t* hy, int16_t* hz);
        void getMotion10Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* hx, int16_t* hy, int16_t* hz, int16_t* t);
        void setOffsets(int16_t* offsets);
        void getOffsets(int16_t* offsets);
        void setMagTMandBias(float* matrix, float* bias);
        void setTransformMatrix(int16_t **tm);
        void setBeta(float b) { beta = b; }
        void resetQuaternion(float *src);
        float getRoll();
        float getPitch();
        float getYaw();
        bool  isAccelGyroDataReady();
        void  setTestMode(bool mode=true) { _test_mode=mode; }
        void  setTestData(float* d);
		
//        void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);
//        void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);
        void MadgwickQuaternionUpdate(float deltat);
        void MadgwickQuaternionUpdateIMU(float deltat);
        void MahonyQuaternionUpdate(float deltat);
        const float *getQuaternion() { return q; }
        
      private:
	    void adjustMagData(float* hx, float* hy, float* hz);
        void calculateAngles();
        float invSqrt(float x);
		
        uint8_t _address;
        uint8_t _bus;
        i2c_pins _pins;
        i2c_pullup _pullups;
        bool _userDefI2C;
        uint8_t _csPin;
        spi_mosi_pin _mosiPin;
        bool _useSPI;
        bool _useSPIHS;
        float _accelScale;
        float _gyroScale;
        float _magScaleX, _magScaleY, _magScaleZ;
        const float _tempScale = 333.87f;
        const float _tempOffset = 21.0f;
        
        float mag_tm[9]; // m11, m12, m13, m21, m22 ...
        float mag_bias[3]; // bx, by, bz
        float mag_declination;
        float mag_scaler;
        bool  _tm_loaded;
        bool  _tm_scaler;
        bool  _calc_angles = false;
        bool  _test_mode = false;
        float _test_data[9]; 
        float yaw, roll, pitch;


// These are the free parameters in the Mahony filter and fusion scheme, Kp
// for proportional feedback, Ki for integral
#define Kp 2.0f * 5.0f
#define Ki 0.0f

        float GyroMeasError = PI * (40.0f / 180.0f);
// gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
        float GyroMeasDrift = PI * (0.0f  / 180.0f);
// There is a tradeoff in the beta parameter between accuracy and response
// speed. In the original Madgwick study, beta of 0.041 (corresponding to
// GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds
// to a stable initial quaternion. Subsequent changes also require a
// longish lag time to a stable output, not fast enough for a quadcopter or
// robot car! By increasing beta (GyroMeasError) by about a factor of
// fifteen, the response time constant is reduced to ~2 sec. I haven't
// noticed any reduction in solution accuracy. This is essentially the I
// coefficient in a PID control sense; the bigger the feedback coefficient,
// the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and
// fusion scheme.
        float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // Compute beta
// Compute zeta, the other free parameter in the Madgwick scheme usually
// set to a small or zero value
        float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;

// Vector to hold integral error for Mahony method
        float eInt[3] = {0.0f, 0.0f, 0.0f};
// Vector to hold quaternion
        float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

        // constants
//        const float G = 9.807f;
        const float _d2r = 3.14159265359f/180.0f;


        // // transformation matrix
        // /* transform the accel and gyro axes to match the magnetometer axes */
        // const int16_t tX[3] = {0,  1,  0};
        // const int16_t tY[3] = {1,  0,  0};
        // const int16_t tZ[3] = {0,  0, -1};

        // HACK: get acc and gyro data in ordinary frame
        // NB: This does not transform mag data.
        // const int16_t tX[3] = {1,  0,  0};
        // const int16_t tY[3] = {0,  1,  0};
        // const int16_t tZ[3] = {0,  0,  1};

        int16_t tM[3][3] = { \
                              {1,  0,  0}, \
                              {0,  1,  0}, \
                              {0,  0,  1}, \
							};
        bool writeRegister(uint8_t subAddress, uint8_t data);
        void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
        bool writeAK8963Register(uint8_t subAddress, uint8_t data);
        void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest);
        uint8_t whoAmI();
        uint8_t whoAmIAK8963();
};

#endif
