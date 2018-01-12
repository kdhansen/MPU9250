/*
MPU9250.cpp
Brian R Taylor
brian.taylor@bolderflight.com
2017-01-04

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

// Teensy 3.0 || Teensy 3.1/3.2 || Teensy 3.5 || Teensy 3.6 || Teensy LC
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || \
	defined(__MK66FX1M0__) || defined(__MKL26Z64__)

#include "Arduino.h"
#include "MPU9250_t3.h"
#include "i2c_t3.h"  // I2C library
#include "SPI.h" // SPI Library

/* MPU9250 object, input the I2C address and I2C bus */
MPU9250::MPU9250(uint8_t address, uint8_t bus){
    _address = address; // I2C address
    _bus = bus; // I2C bus
    _userDefI2C = false; // automatic I2C setup
    _useSPI = false; // set to use I2C instead of SPI
}

/* MPU9250 object, input the I2C address, I2C bus, and I2C pins */
MPU9250::MPU9250(uint8_t address, uint8_t bus, i2c_pins pins){
    _address = address; // I2C address
    _bus = bus; // I2C bus
    _pins = pins; // I2C pins
    _pullups = I2C_PULLUP_EXT; // I2C pullups
    _userDefI2C = true; // user defined I2C
    _useSPI = false; // set to use I2C instead of SPI
}

/* MPU9250 object, input the I2C address, I2C bus, I2C pins, and I2C pullups */
MPU9250::MPU9250(uint8_t address, uint8_t bus, i2c_pins pins, i2c_pullup pullups){
    _address = address; // I2C address
    _bus = bus; // I2C bus
    _pins = pins; // I2C pins
    _pullups = pullups; // I2C pullups
    _userDefI2C = true; // user defined I2C
    _useSPI = false; // set to use I2C instead of SPI
}

/* MPU9250 object, input the SPI CS Pin */
MPU9250::MPU9250(uint8_t csPin){
    _csPin = csPin; // SPI CS Pin
    _mosiPin = MOSI_PIN_11;	// SPI MOSI Pin, set to default
    _useSPI = true; // set to use SPI instead of I2C
    _useSPIHS = false; // defaul to low speed SPI transactions until data reads start to occur
}

/* MPU9250 object, input the SPI CS Pin and MOSI Pin */
MPU9250::MPU9250(uint8_t csPin, spi_mosi_pin pin){
    _csPin = csPin; // SPI CS Pin
    _mosiPin = pin;	// SPI MOSI Pin
    _useSPI = true; // set to use SPI instead of I2C
    _useSPIHS = false; // defaul to low speed SPI transactions until data reads start to occur
}

/* starts I2C communication and sets up the MPU-9250 */
int MPU9250::begin(mpu9250_accel_range accelRange, mpu9250_gyro_range gyroRange, float regionalMagDeclination){
    uint8_t buff[3];
    uint8_t data[7];
    
    _tm_loaded = false; // magnetometer transportation matrix not loaded
    _tm_scaler = false;
    
    if( _useSPI ){ // using SPI for communication

        // setting CS pin to output
        pinMode(_csPin,OUTPUT);

        // setting CS pin high
        digitalWriteFast(_csPin,HIGH);

        // Teensy 3.0 || Teensy 3.1/3.2
		#if defined(__MK20DX128__) || defined(__MK20DX256__)

	        // configure and begin the SPI
	        switch( _mosiPin ){

				case MOSI_PIN_7:	// SPI bus 0 alternate 1
				    SPI.setMOSI(7);
	        		SPI.setMISO(8);
	        		SPI.setSCK(14);
	        		SPI.begin();
	        		break;
				case MOSI_PIN_11:	// SPI bus 0 default
					SPI.setMOSI(11);
	        		SPI.setMISO(12);
	        		SPI.setSCK(13);
	        		SPI.begin();
	        		break;
	        }

        #endif

        // Teensy 3.5 || Teensy 3.6
		#if defined(__MK64FX512__) || defined(__MK66FX1M0__)

	        // configure and begin the SPI
	        switch( _mosiPin ){

	        	case MOSI_PIN_0:	// SPI bus 1 default
	        		SPI1.setMOSI(0);
	        		SPI1.setMISO(1);
	        		SPI1.setSCK(32);
	        		SPI1.begin();
	        		break;
				case MOSI_PIN_7:	// SPI bus 0 alternate 1
				    SPI.setMOSI(7);
	        		SPI.setMISO(8);
	        		SPI.setSCK(14);
	        		SPI.begin();
	        		break;
				case MOSI_PIN_11:	// SPI bus 0 default
					SPI.setMOSI(11);
	        		SPI.setMISO(12);
	        		SPI.setSCK(13);
	        		SPI.begin();
	        		break;
				case MOSI_PIN_21:	// SPI bus 1 alternate
		        	SPI1.setMOSI(21);
	        		SPI1.setMISO(5);
	        		SPI1.setSCK(20);
	        		SPI1.begin();
	        		break;
				case MOSI_PIN_28:	// SPI bus 0 alternate 2
	        		SPI.setMOSI(28);
	        		SPI.setMISO(39);
	        		SPI.setSCK(27);
	        		SPI.begin();
	        		break;
				case MOSI_PIN_44:	// SPI bus 2 default
	        		SPI2.setMOSI(44);
	        		SPI2.setMISO(45);
	        		SPI2.setSCK(46);
	        		SPI2.begin();
	        		break;
				case MOSI_PIN_52:	// SPI bus 2 alternate
	        		SPI2.setMOSI(52);
	        		SPI2.setMISO(51);
	        		SPI2.setSCK(53);
	        		SPI2.begin();
	        		break;
	        }

        #endif

        // Teensy LC
		#if defined(__MKL26Z64__)

			// configure and begin the SPI
	        switch( _mosiPin ){

	        	case MOSI_PIN_0:	// SPI bus 1 default
	        		SPI1.setMOSI(0);
	        		SPI1.setMISO(1);
	        		SPI1.setSCK(20);
	        		SPI1.begin();
	        		break;
				case MOSI_PIN_7:	// SPI bus 0 alternate 1
				    SPI.setMOSI(7);
	        		SPI.setMISO(8);
	        		SPI.setSCK(14);
	        		SPI.begin();
	        		break;
				case MOSI_PIN_11:	// SPI bus 0 default
					SPI.setMOSI(11);
	        		SPI.setMISO(12);
	        		SPI.setSCK(13);
	        		SPI.begin();
	        		break;
				case MOSI_PIN_21:	// SPI bus 1 alternate
		        	SPI1.setMOSI(21);
	        		SPI1.setMISO(5);
	        		SPI1.setSCK(20);
	        		SPI1.begin();
	        		break;
	        }

		#endif
    }
    else{ // using I2C for communication

        if( !_userDefI2C ) { // setup the I2C pins and pullups based on bus number if not defined by user
            /* setting the I2C pins, pullups, and protecting against _bus out of range */
            _pullups = I2C_PULLUP_EXT; // default to external pullups

            #if defined(__MK20DX128__) // Teensy 3.0
                _pins = I2C_PINS_18_19;
                _bus = 0;
            #endif

            #if defined(__MK20DX256__) // Teensy 3.1/3.2
                if(_bus == 1) {
                    _pins = I2C_PINS_29_30;
                }
                else{
                    _pins = I2C_PINS_18_19;
                    _bus = 0;
                }

            #endif

            #if defined(__MK64FX512__) // Teensy 3.5
                if(_bus == 2) {
                    _pins = I2C_PINS_3_4;
                }
                else if(_bus == 1) {
                    _pins = I2C_PINS_37_38;
                }
                else{
                    _pins = I2C_PINS_18_19;
                    _bus = 0;
                }

            #endif

            #if defined(__MK66FX1M0__) // Teensy 3.6
                if(_bus == 3) {
                    _pins = I2C_PINS_56_57;
                }
                else if(_bus == 2) {
                    _pins = I2C_PINS_3_4;
                }
                else if(_bus == 1) {
                    _pins = I2C_PINS_37_38;
                }
                else{
                    _pins = I2C_PINS_18_19;
                    _bus = 0;
                }

            #endif

            #if defined(__MKL26Z64__) // Teensy LC
                if(_bus == 1) {
                    _pins = I2C_PINS_22_23;
                }
                else{
                    _pins = I2C_PINS_18_19;
                    _bus = 0;
                }

            #endif
        }

        // starting the I2C bus
        i2c_t3(_bus).begin(I2C_MASTER, 0, _pins, _pullups, _i2cRate);
        i2c_t3(_bus).setDefaultTimeout(10000);
    }

    // select clock source to gyro
    if( !writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) ){
        return -1;
    }

    // enable I2C master mode
    if( !writeRegister(USER_CTRL,I2C_MST_EN) ){
        return -1;
    }

    // set the I2C bus speed to 400 kHz
    if( !writeRegister(I2C_MST_CTRL,I2C_MST_CLK) ){
        return -1;
    }

    // set AK8963 to Power Down
    if( !writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) ){
        return -1;
    }

    // reset the MPU9250
    writeRegister(PWR_MGMNT_1,PWR_RESET);

    // wait for MPU-9250 to come back up
    delay(1);

    // reset the AK8963
    writeAK8963Register(AK8963_CNTL2,AK8963_RESET);
	delay(10);
	
    // select clock source to gyro
    if( !writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) ){
        return -1;
    }

    // check the WHO AM I byte, expected value is 0x71 (decimal 113)
    if( whoAmI() != 113 ){
        return -1;
    }

    // enable accelerometer and gyro
    if( !writeRegister(PWR_MGMNT_2,SEN_ENABLE) ){
        return -1;
    }

    /* setup the accel and gyro ranges */
    switch(accelRange) {

        case ACCEL_RANGE_2G:
            // setting the accel range to 2G
            if( !writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_2G) ){
                return -1;
            }
            _accelScale = 2.0f/32767.5f; //* G; // setting the accel scale to 2G
            break;

        case ACCEL_RANGE_4G:
            // setting the accel range to 4G
            if( !writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_4G) ){
                return -1;
            }
            _accelScale = 4.0f/32767.5f; //* G; ; // setting the accel scale to 4G
            break;

        case ACCEL_RANGE_8G:
            // setting the accel range to 8G
            if( !writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_8G) ){
                return -1;
            }
            _accelScale = 8.0f/32767.5f; //* G; ; // setting the accel scale to 8G
            break;

        case ACCEL_RANGE_16G:
            // setting the accel range to 16G
            if( !writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_16G) ){
                return -1;
            }
            _accelScale = 16.0f/32767.5f; //* G; ; // setting the accel scale to 16G
            break;
    }

    switch(gyroRange) {
        case GYRO_RANGE_250DPS:
            // setting the gyro range to 250DPS
            if( !writeRegister(GYRO_CONFIG,GYRO_FS_SEL_250DPS) ){
                return -1;
            }
            _gyroScale = 250.0f/32767.5f;// * _d2r; // setting the gyro scale to 250DPS
            break;

        case GYRO_RANGE_500DPS:
            // setting the gyro range to 500DPS
            if( !writeRegister(GYRO_CONFIG,GYRO_FS_SEL_500DPS) ){
                return -1;
            }
            _gyroScale = 500.0f/32767.5f;// * _d2r; // setting the gyro scale to 500DPS
            break;

        case GYRO_RANGE_1000DPS:
            // setting the gyro range to 1000DPS
            if( !writeRegister(GYRO_CONFIG,GYRO_FS_SEL_1000DPS) ){
                return -1;
            }
            _gyroScale = 1000.0f/32767.5f;// * _d2r; // setting the gyro scale to 1000DPS
            break;

        case GYRO_RANGE_2000DPS:
            // setting the gyro range to 2000DPS
            if( !writeRegister(GYRO_CONFIG,GYRO_FS_SEL_2000DPS) ){
                return -1;
            }
            _gyroScale = 2000.0f/32767.5f;// * _d2r; // setting the gyro scale to 2000DPS
            break;
    }

    // set regional magnetic declination
    mag_declination = regionalMagDeclination;
    
    // enable I2C master mode
    if( !writeRegister(USER_CTRL,I2C_MST_EN) ){
    	return -1;
    }

	// set the I2C bus speed to 400 kHz
	if( !writeRegister(I2C_MST_CTRL,I2C_MST_CLK) ){
		return -1;
	}

	// check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
	if( whoAmIAK8963() != 72 ){
        return -1;
	}

    /* get the magnetometer calibration */

    // set AK8963 to Power Down
    if( !writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) ){
        return -1;
    }
    delay(100); // long wait between AK8963 mode changes

    // set AK8963 to FUSE ROM access
    if( !writeAK8963Register(AK8963_CNTL1,AK8963_FUSE_ROM) ){
        return -1;
    }
    delay(100); // long wait between AK8963 mode changes

    // read the AK8963 ASA registers and compute magnetometer scale factors
    readAK8963Registers(AK8963_ASA,sizeof(buff),&buff[0]);
    _magScaleX = ((((float)buff[0]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
    _magScaleY = ((((float)buff[1]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
    _magScaleZ = ((((float)buff[2]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla

    // set AK8963 to Power Down
    if( !writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) ){
        return -1;
    }
    delay(100); // long wait between AK8963 mode changes

    // set AK8963 to 16 bit resolution, 100 Hz update rate
    if( !writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2) ){
        return -1;
    }
    delay(100); // long wait between AK8963 mode changes

    // select clock source to gyro
    if( !writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) ){
        return -1;
    }

    
 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
    // if( !writeRegister(CONFIG, 0x03) ) {
        // return -1;
    // }
    
 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
 // Use a 200 Hz rate; a rate consistent with the filter update rate 
 // determined inset in CONFIG above
    // if( !writeRegister(SMPDIV, 0x04) ) {
        // return -1;
    // }
    
    
  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and do not enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
    // if( !writeRegister(INT_PIN_CFG, 0x20) ) {
        // return -1;
    // }
    
   // Enable data ready (bit 0) interrupt
    // if( !writeRegister(INT_ENABLE, 0x01) ) {
        // return -1;
    // }
    // delay(100);
    
    // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
    readAK8963Registers(AK8963_HXL,sizeof(data),&data[0]);

    // successful init, return 0
    return 0;
}

bool MPU9250::isAccelGyroDataReady() {
    uint8_t buffer;
    // if ( i2c_t3(_bus).status() == I2C_TIMEOUT ) i2c_t3(_bus).resetBus_(_bus);
    readRegisters( INT_STATUS, 1, &buffer );
    return (buffer & 0x01);
}

#define MPU9250_RA_XA_OFFS_H        0x77 //[15:0] XA_OFFS
#define MPU9250_RA_XA_OFFS_L_TC     0x78
#define MPU9250_RA_YA_OFFS_H        0x7A //[15:0] YA_OFFS
#define MPU9250_RA_YA_OFFS_L_TC     0x7B
#define MPU9250_RA_ZA_OFFS_H        0x7D //[15:0] ZA_OFFS
#define MPU9250_RA_ZA_OFFS_L_TC     0x7E

#define MPU9250_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU9250_RA_XG_OFFS_USRL     0x14
#define MPU9250_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU9250_RA_YG_OFFS_USRL     0x16
#define MPU9250_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU9250_RA_ZG_OFFS_USRL     0x18

void MPU9250::getOffsets(int16_t* offsets) {
    uint8_t buffer[2];
    int16_t *p = offsets;
    
    readRegisters( MPU9250_RA_XA_OFFS_H, 2, buffer );
    *p++ = ((((int16_t)buffer[0]) << 8) | buffer[1]);

    readRegisters( MPU9250_RA_YA_OFFS_H, 2, buffer );
    *p++ = ((((int16_t)buffer[0]) << 8) | buffer[1]);

    readRegisters( MPU9250_RA_ZA_OFFS_H, 2, buffer );
    *p++ = ((((int16_t)buffer[0]) << 8) | buffer[1]);

    readRegisters( MPU9250_RA_XG_OFFS_USRH, 2, buffer );
    *p++ = ((((int16_t)buffer[0]) << 8) | buffer[1]);

    readRegisters( MPU9250_RA_YG_OFFS_USRH, 2, buffer );
    *p++ = ((((int16_t)buffer[0]) << 8) | buffer[1]);

    readRegisters( MPU9250_RA_ZG_OFFS_USRH, 2, buffer );
    *p++ = ((((int16_t)buffer[0]) << 8) | buffer[1]);    
}
/* sets accel and gyro offsets */
void MPU9250::setOffsets(int16_t* offsets) {
    uint8_t bit0;
    int16_t value;
    int16_t *p = offsets;

// For ax, ay, and az offsets bit 0 should be preserved
// due to temperature compensation something
//ax    
    readRegisters( MPU9250_RA_XA_OFFS_L_TC, 1, &bit0 );
    bit0 &= 1;
    value = (*p++ & 0xFFFE) | bit0;
    writeRegister( MPU9250_RA_XA_OFFS_H, (uint8_t) (value>>8)&0xFF );
    writeRegister( MPU9250_RA_XA_OFFS_L_TC, (uint8_t) value&0xFF );
//ay    
    readRegisters( MPU9250_RA_YA_OFFS_L_TC, 1, &bit0 );
    bit0 &= 1;
    value = (*p++ & 0xFFFE) | bit0;
    writeRegister( MPU9250_RA_YA_OFFS_H, (uint8_t) (value>>8)&0xFF );
    writeRegister( MPU9250_RA_YA_OFFS_L_TC, (uint8_t) value&0xFF );
//az    
    readRegisters( MPU9250_RA_ZA_OFFS_L_TC, 1, &bit0 );
    bit0 &= 1;
    value = (*p++ & 0xFFFE) | bit0;
    writeRegister( MPU9250_RA_ZA_OFFS_H, (uint8_t) (value>>8)&0xFF );
    writeRegister( MPU9250_RA_ZA_OFFS_L_TC, (uint8_t) value&0xFF );
//gx
    value = *p++;
    writeRegister( MPU9250_RA_XG_OFFS_USRH, (uint8_t) (value>>8)&0xFF );
    writeRegister( MPU9250_RA_XG_OFFS_USRL, (uint8_t) value&0xFF );
//gy
    value = *p++;
    writeRegister( MPU9250_RA_YG_OFFS_USRH, (uint8_t) (value>>8)&0xFF );
    writeRegister( MPU9250_RA_YG_OFFS_USRL, (uint8_t) value&0xFF );
//gz
    value = *p++;
    writeRegister( MPU9250_RA_ZG_OFFS_USRH, (uint8_t) (value>>8)&0xFF );
    writeRegister( MPU9250_RA_ZG_OFFS_USRL, (uint8_t) value&0xFF );
}


void MPU9250::setMagTMandBias(float* matrix, float* bias) {
    float* p = matrix;
    
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            mag_tm[i*3+j] = *(p+(i*3+j));
    
    p = bias;
    for (int i=0; i<3; i++) 
        mag_bias[i] = *p++;

    _tm_loaded = true;   
}

void MPU9250::setTransformMatrix(int16_t **tm) {
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            tM[i][j] = tm[i][j];
}

/* sets the DLPF and interrupt settings */
int MPU9250::setFilt(mpu9250_dlpf_bandwidth bandwidth, uint8_t SRD){
    uint8_t data[7];

    switch(bandwidth) {
        case DLPF_BANDWIDTH_184HZ:
            if( !writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_184) ){ // setting accel bandwidth to 184Hz
                return -1;
            }
            if( !writeRegister(CONFIG,GYRO_DLPF_184) ){ // setting gyro bandwidth to 184Hz
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_92HZ:
            if( !writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_92) ){ // setting accel bandwidth to 92Hz
                return -1;
            }
            if( !writeRegister(CONFIG,GYRO_DLPF_92) ){ // setting gyro bandwidth to 92Hz
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_41HZ:
            if( !writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_41) ){ // setting accel bandwidth to 41Hz
                return -1;
            }
            if( !writeRegister(CONFIG,GYRO_DLPF_41) ){ // setting gyro bandwidth to 41Hz
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_20HZ:
            if( !writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_20) ){ // setting accel bandwidth to 20Hz
                return -1;
            }
            if( !writeRegister(CONFIG,GYRO_DLPF_20) ){ // setting gyro bandwidth to 20Hz
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_10HZ:
            if( !writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_10) ){ // setting accel bandwidth to 10Hz
                return -1;
            }
            if( !writeRegister(CONFIG,GYRO_DLPF_10) ){ // setting gyro bandwidth to 10Hz
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_5HZ:
            if( !writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_5) ){ // setting accel bandwidth to 5Hz
                return -1;
            }
            if( !writeRegister(CONFIG,GYRO_DLPF_5) ){ // setting gyro bandwidth to 5Hz
                return -1;
            }
            break;
    }

    /* setting the sample rate divider */
    if( !writeRegister(SMPDIV,SRD) ){ // setting the sample rate divider
        return -1;
    }

    if(SRD > 9){

        // set AK8963 to Power Down
        if( !writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) ){
            return -1;
        }
        delay(100); // long wait between AK8963 mode changes

        // set AK8963 to 16 bit resolution, 8 Hz update rate
        if( !writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS1) ){
            return -1;
        }
        delay(100); // long wait between AK8963 mode changes

        // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
        readAK8963Registers(AK8963_HXL,sizeof(data),&data[0]);
    }

    /* setting the interrupt */
    if( !writeRegister(INT_PIN_CFG,INT_PULSE_READ) ){ // setup interrupt, high until read
        return -1;
    }
    if( !writeRegister(INT_ENABLE,INT_RAW_RDY_EN) ){ // set to data ready
        return -1;
    }

    // successful filter setup, return 0
    return 0;
}

/* enables and disables the interrupt */
int MPU9250::enableInt(bool enable){

	if(enable){
		/* setting the interrupt */
	    if( !writeRegister(INT_PIN_CFG,INT_PULSE_50US) ){ // setup interrupt, 50 us pulse
	        return -1;
	    }
	    if( !writeRegister(INT_ENABLE,INT_RAW_RDY_EN) ){ // set to data ready
	        return -1;
	    }
	}
	else{
	    if( !writeRegister(INT_ENABLE,INT_DISABLE) ){ // disable interrupt
	        return -1;
	    }
	}

    // successful interrupt setup, return 0
    return 0;
}


/* get accelerometer data given pointers to store the three values, return data as counts */
void MPU9250::getAccelCounts(int16_t* ax, int16_t* ay, int16_t* az){
    uint8_t buff[6];
    int16_t axx, ayy, azz;
    _useSPIHS = true; // use the high speed SPI for data readout

    readRegisters(ACCEL_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

    axx = (((int16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit values
    ayy = (((int16_t)buff[2]) << 8) | buff[3];
    azz = (((int16_t)buff[4]) << 8) | buff[5];

    *ax = tM[0][0]*axx + tM[0][1]*ayy + tM[0][2]*azz; // transform axes
    *ay = tM[1][0]*axx + tM[1][1]*ayy + tM[1][2]*azz;
    *az = tM[2][0]*axx + tM[2][1]*ayy + tM[2][2]*azz;
}

/* get accelerometer data given pointers to store the three values */
void MPU9250::getAccel(float* ax, float* ay, float* az){
    int16_t accel[3];

    getAccelCounts(&accel[0], &accel[1], &accel[2]);

    *ax = ((float) accel[0]) * _accelScale; // typecast and scale to values
    *ay = ((float) accel[1]) * _accelScale;
    *az = ((float) accel[2]) * _accelScale;
}

/* get gyro data given pointers to store the three values, return data as counts */
void MPU9250::getGyroCounts(int16_t* gx, int16_t* gy, int16_t* gz){
    uint8_t buff[6];
    int16_t gxx, gyy, gzz;
    _useSPIHS = true; // use the high speed SPI for data readout

    readRegisters(GYRO_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

    gxx = (((int16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit values
    gyy = (((int16_t)buff[2]) << 8) | buff[3];
    gzz = (((int16_t)buff[4]) << 8) | buff[5];

    *gx = tM[0][0]*gxx + tM[0][1]*gyy + tM[0][2]*gzz; // transform axes
    *gy = tM[1][0]*gxx + tM[1][1]*gyy + tM[1][2]*gzz;
    *gz = tM[2][0]*gxx + tM[2][1]*gyy + tM[2][2]*gzz;
}

/* get gyro data given pointers to store the three values. In DPS */
void MPU9250::getGyro(float* gx, float* gy, float* gz){
    int16_t gyro[3];

    getGyroCounts(&gyro[0], &gyro[1], &gyro[2]);

    *gx = ((float) gyro[0]) * _gyroScale; // typecast and scale to DPS values
    *gy = ((float) gyro[1]) * _gyroScale;
    *gz = ((float) gyro[2]) * _gyroScale;
}

/* get magnetometer data given pointers to store the three values, return data as counts */
void MPU9250::getMagCounts(int16_t* hx, int16_t* hy, int16_t* hz){
    uint8_t buff[7];
    _useSPIHS = true; // use the high speed SPI for data readout

    // read the magnetometer data off the external sensor buffer
    readRegisters(EXT_SENS_DATA_00,sizeof(buff),&buff[0]);

    if( buff[6] == 0x10 ) { // check for overflow
        *hx = (((int16_t)buff[1]) << 8) | buff[0];  // combine into 16 bit values
        *hy = (((int16_t)buff[3]) << 8) | buff[2];
        *hz = (((int16_t)buff[5]) << 8) | buff[4];
    }
    else{
        *hx = 0;
        *hy = 0;
        *hz = 0;
    }
}

/* get magnetometer data given pointers to store the three values */
void MPU9250::getMag(float* hx, float* hy, float* hz){
    int16_t mag[3];

    getMagCounts(&mag[0], &mag[1], &mag[2]);

    *hx = ((float) mag[0]) * _magScaleX; // typecast and scale to values
    *hy = ((float) mag[1]) * _magScaleY;
    *hz = ((float) mag[2]) * _magScaleZ;

    if (_tm_loaded) adjustMagData(hx, hy, hz);
}

/* Adjust magnetometer data if biases are loaded */
void MPU9250::adjustMagData(float* hx, float* hy, float* hz) {
	    float temp[3];
        float result[3] = {0, 0, 0};

        temp[0] = *hx - mag_bias[0];
        temp[1] = *hy - mag_bias[1];
        temp[2] = *hz - mag_bias[2];
        for (int i=0; i<3; ++i)
          for (int j=0; j<3; ++j)
            result[i] += mag_tm[i*3+j] * temp[j];
        if (!_tm_scaler) {
            mag_scaler = sqrt(result[0]*result[0] + result[1]*result[1] + result[2]*result[2]);
            _tm_scaler = true;
        }
        float c = mag_scaler / sqrt(result[0]*result[0] + result[1]*result[1] + result[2]*result[2]);
        *hx = result[0]*c;
        *hy = result[1]*c;
        *hz = result[2]*c;
}


/* get temperature data given pointer to store the value, return data as counts */
void MPU9250::getTempCounts(int16_t* t){
    uint8_t buff[2];
    _useSPIHS = true; // use the high speed SPI for data readout

    readRegisters(TEMP_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

    *t = (((int16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit value and return
}

/* get temperature data given pointer to store the values */
void MPU9250::getTemp(float* t){
    int16_t tempCount;

    getTempCounts(&tempCount);

    *t = (( ((float) tempCount) - _tempOffset )/_tempScale) + _tempOffset;
}

/* get accelerometer and gyro data given pointers to store values, return data as counts */
void MPU9250::getMotion6Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz){
    uint8_t buff[14];
    int16_t axx, ayy, azz, gxx, gyy, gzz;
    _useSPIHS = true; // use the high speed SPI for data readout

    if ( _test_mode ) {
        *ax = _test_data[0];
        *ay = _test_data[1];
        *az = _test_data[2];
        *gx = _test_data[3];
        *gy = _test_data[4];
        *gz = _test_data[5];
    }
    else {
        readRegisters(ACCEL_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

        axx = (((int16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit values
        ayy = (((int16_t)buff[2]) << 8) | buff[3];
        azz = (((int16_t)buff[4]) << 8) | buff[5];

        gxx = (((int16_t)buff[8]) << 8) | buff[9];
        gyy = (((int16_t)buff[10]) << 8) | buff[11];
        gzz = (((int16_t)buff[12]) << 8) | buff[13];

        *ax = tM[0][0]*axx + tM[0][1]*ayy + tM[0][2]*azz; // transform axes
        *ay = tM[1][0]*axx + tM[1][1]*ayy + tM[1][2]*azz;
        *az = tM[2][0]*axx + tM[2][1]*ayy + tM[2][2]*azz;

        *gx = tM[0][0]*gxx + tM[0][1]*gyy + tM[0][2]*gzz; // transform axes
        *gy = tM[1][0]*gxx + tM[1][1]*gyy + tM[1][2]*gzz;
        *gz = tM[2][0]*gxx + tM[2][1]*gyy + tM[2][2]*gzz;
    }
}

/* get accelerometer and gyro data given pointers to store values */
void MPU9250::getMotion6(float* ax, float* ay, float* az, float* gx, float* gy, float* gz){
    int16_t accel[3];
    int16_t gyro[3];

    getMotion6Counts(&accel[0], &accel[1], &accel[2], &gyro[0], &gyro[1], &gyro[2]);

    *ax = ((float) accel[0]) * _accelScale; // typecast and scale to values
    *ay = ((float) accel[1]) * _accelScale;
    *az = ((float) accel[2]) * _accelScale;

    *gx = ((float) gyro[0]) * _gyroScale;
    *gy = ((float) gyro[1]) * _gyroScale;
    *gz = ((float) gyro[2]) * _gyroScale;
}

/* get accelerometer, gyro and temperature data given pointers to store values, return data as counts */
void MPU9250::getMotion7Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* t){
    uint8_t buff[14];
    int16_t axx, ayy, azz, gxx, gyy, gzz;
    _useSPIHS = true; // use the high speed SPI for data readout

    readRegisters(ACCEL_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

    axx = (((int16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit values
    ayy = (((int16_t)buff[2]) << 8) | buff[3];
    azz = (((int16_t)buff[4]) << 8) | buff[5];

    *t = (((int16_t)buff[6]) << 8) | buff[7];

    gxx = (((int16_t)buff[8]) << 8) | buff[9];
    gyy = (((int16_t)buff[10]) << 8) | buff[11];
    gzz = (((int16_t)buff[12]) << 8) | buff[13];

    *ax = tM[0][0]*axx + tM[0][1]*ayy + tM[0][2]*azz; // transform axes
    *ay = tM[1][0]*axx + tM[1][1]*ayy + tM[1][2]*azz;
    *az = tM[2][0]*axx + tM[2][1]*ayy + tM[2][2]*azz;

    *gx = tM[0][0]*gxx + tM[0][1]*gyy + tM[0][2]*gzz; // transform axes
    *gy = tM[1][0]*gxx + tM[1][1]*gyy + tM[1][2]*gzz;
    *gz = tM[2][0]*gxx + tM[2][1]*gyy + tM[2][2]*gzz;
}

/* get accelerometer, gyro, and temperature data given pointers to store values */
void MPU9250::getMotion7(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* t){
    int16_t accel[3];
    int16_t gyro[3];
    int16_t tempCount;

    getMotion7Counts(&accel[0], &accel[1], &accel[2], &gyro[0], &gyro[1], &gyro[2], &tempCount);

    *ax = ((float) accel[0]) * _accelScale; // typecast and scale to values
    *ay = ((float) accel[1]) * _accelScale;
    *az = ((float) accel[2]) * _accelScale;

    *gx = ((float) gyro[0]) * _gyroScale;
    *gy = ((float) gyro[1]) * _gyroScale;
    *gz = ((float) gyro[2]) * _gyroScale;

    *t = (( ((float) tempCount) - _tempOffset )/_tempScale) + _tempOffset;
}

/* get accelerometer, gyro and magnetometer data given pointers to store values, return data as counts */
void MPU9250::getMotion9Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* hx, int16_t* hy, int16_t* hz){
    uint8_t buff[21];
    int16_t axx, ayy, azz, gxx, gyy, gzz;
    _useSPIHS = true; // use the high speed SPI for data readout

        if ( _test_mode ) {
        *ax = _test_data[0];
        *ay = _test_data[1];
        *az = _test_data[2];
        *gx = _test_data[3];
        *gy = _test_data[4];
        *gz = _test_data[5];
        *hx = _test_data[6];
        *hy = _test_data[7];
        *hz = _test_data[8];    }
    else {
        readRegisters(ACCEL_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

        axx = (((int16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit values
        ayy = (((int16_t)buff[2]) << 8) | buff[3];
        azz = (((int16_t)buff[4]) << 8) | buff[5];

        gxx = (((int16_t)buff[8]) << 8) | buff[9];
        gyy = (((int16_t)buff[10]) << 8) | buff[11];
        gzz = (((int16_t)buff[12]) << 8) | buff[13];

        *hx = (((int16_t)buff[15]) << 8) | buff[14];
        *hy = (((int16_t)buff[17]) << 8) | buff[16];
        *hz = (((int16_t)buff[19]) << 8) | buff[18];

        *ax = tM[0][0]*axx + tM[0][1]*ayy + tM[0][2]*azz; // transform axes
        *ay = tM[1][0]*axx + tM[1][1]*ayy + tM[1][2]*azz;
        *az = tM[2][0]*axx + tM[2][1]*ayy + tM[2][2]*azz;

        *gx = tM[0][0]*gxx + tM[0][1]*gyy + tM[0][2]*gzz; // transform axes
        *gy = tM[1][0]*gxx + tM[1][1]*gyy + tM[1][2]*gzz;
        *gz = tM[2][0]*gxx + tM[2][1]*gyy + tM[2][2]*gzz;
    }
}

/* get accelerometer, gyro, and magnetometer data given pointers to store values */
void MPU9250::getMotion9(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy, float* hz){
    int16_t accel[3];
    int16_t gyro[3];
    int16_t mag[3];

    getMotion9Counts(&accel[0], &accel[1], &accel[2], &gyro[0], &gyro[1], &gyro[2], &mag[0], &mag[1], &mag[2]);

    *ax = ((float) accel[0]) * _accelScale; // typecast and scale to values
    *ay = ((float) accel[1]) * _accelScale;
    *az = ((float) accel[2]) * _accelScale;

    *gx = ((float) gyro[0]) * _gyroScale;
    *gy = ((float) gyro[1]) * _gyroScale;
    *gz = ((float) gyro[2]) * _gyroScale;

    *hx = ((float) mag[0]) * _magScaleX; // typecast and scale to values
    *hy = ((float) mag[1]) * _magScaleY;
    *hz = ((float) mag[2]) * _magScaleZ;
		
    if (_tm_loaded) adjustMagData(hx, hy, hz);
}

/* get accelerometer, magnetometer, and temperature data given pointers to store values, return data as counts */
void MPU9250::getMotion10Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* hx, int16_t* hy, int16_t* hz, int16_t* t){
    uint8_t buff[21];
    int16_t axx, ayy, azz, gxx, gyy, gzz;
    _useSPIHS = true; // use the high speed SPI for data readout

    readRegisters(ACCEL_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

    axx = (((int16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit values
    ayy = (((int16_t)buff[2]) << 8) | buff[3];
    azz = (((int16_t)buff[4]) << 8) | buff[5];

    *t = (((int16_t)buff[6]) << 8) | buff[7];

    gxx = (((int16_t)buff[8]) << 8) | buff[9];
    gyy = (((int16_t)buff[10]) << 8) | buff[11];
    gzz = (((int16_t)buff[12]) << 8) | buff[13];

    *hx = (((int16_t)buff[15]) << 8) | buff[14];
    *hy = (((int16_t)buff[17]) << 8) | buff[16];
    *hz = (((int16_t)buff[19]) << 8) | buff[18];

    *ax = tM[0][0]*axx + tM[0][1]*ayy + tM[0][2]*azz; // transform axes
    *ay = tM[1][0]*axx + tM[1][1]*ayy + tM[1][2]*azz;
    *az = tM[2][0]*axx + tM[2][1]*ayy + tM[2][2]*azz;

    *gx = tM[0][0]*gxx + tM[0][1]*gyy + tM[0][2]*gzz; // transform axes
    *gy = tM[1][0]*gxx + tM[1][1]*gyy + tM[1][2]*gzz;
    *gz = tM[2][0]*gxx + tM[2][1]*gyy + tM[2][2]*gzz;
}

void MPU9250::getMotion10(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy, float* hz, float* t){
    int16_t accel[3];
    int16_t gyro[3];
    int16_t mag[3];
    int16_t tempCount;

    getMotion10Counts(&accel[0], &accel[1], &accel[2], &gyro[0], &gyro[1], &gyro[2], &mag[0], &mag[1], &mag[2], &tempCount);

    *ax = ((float) accel[0]) * _accelScale; // typecast and scale to values
    *ay = ((float) accel[1]) * _accelScale;
    *az = ((float) accel[2]) * _accelScale;

    *gx = ((float) gyro[0]) * _gyroScale;
    *gy = ((float) gyro[1]) * _gyroScale;
    *gz = ((float) gyro[2]) * _gyroScale;

    *hx = ((float) mag[0]) * _magScaleX; // typecast and scale to values
    *hy = ((float) mag[1]) * _magScaleY;
    *hz = ((float) mag[2]) * _magScaleZ;
		
    if (_tm_loaded) adjustMagData(hx, hy, hz);

    *t = (( ((float) tempCount) - _tempOffset )/_tempScale) + _tempOffset;
}

/* writes a byte to MPU9250 register given a register address and data */
bool MPU9250::writeRegister(uint8_t subAddress, uint8_t data){
    uint8_t buff[1];

    /* write data to device */
    if( _useSPI ){

    	// Teensy 3.0 || Teensy 3.1/3.2
		#if defined(__MK20DX128__) || defined(__MK20DX256__)

	    	if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
		        SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
		        digitalWriteFast(_csPin,LOW); // select the MPU9250 chip
		        SPI.transfer(subAddress); // write the register address
		        SPI.transfer(data); // write the data
		        digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
		        SPI.endTransaction(); // end the transaction
	    	}

    	#endif

        // Teensy 3.5 || Teensy 3.6
		#if defined(__MK64FX512__) || defined(__MK66FX1M0__)

	    	if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)||(_mosiPin == MOSI_PIN_28)){
		        SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
		        digitalWriteFast(_csPin,LOW); // select the MPU9250 chip
		        SPI.transfer(subAddress); // write the register address
		        SPI.transfer(data); // write the data
		        digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
		        SPI.endTransaction(); // end the transaction
	    	}
	    	else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
		        SPI1.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
		        digitalWriteFast(_csPin,LOW); // select the MPU9250 chip
		        SPI1.transfer(subAddress); // write the register address
		        SPI1.transfer(data); // write the data
		        digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
		        SPI1.endTransaction(); // end the transaction
	    	}
	    	else if((_mosiPin == MOSI_PIN_44)||(_mosiPin == MOSI_PIN_52)){
		    	SPI2.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
		        digitalWriteFast(_csPin,LOW); // select the MPU9250 chip
		        SPI2.transfer(subAddress); // write the register address
		        SPI2.transfer(data); // write the data
		        digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
		        SPI2.endTransaction(); // end the transaction
	    	}

    	#endif

        // Teensy LC
		#if defined(__MKL26Z64__)

	    	if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
		        SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
		        digitalWriteFast(_csPin,LOW); // select the MPU9250 chip
		        SPI.transfer(subAddress); // write the register address
		        SPI.transfer(data); // write the data
		        digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
		        SPI.endTransaction(); // end the transaction
	    	}
	    	else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
		        SPI1.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
		        digitalWriteFast(_csPin,LOW); // select the MPU9250 chip
		        SPI1.transfer(subAddress); // write the register address
		        SPI1.transfer(data); // write the data
		        digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
		        SPI1.endTransaction(); // end the transaction
	    	}

    	#endif
    }
    else{
      	i2c_t3(_bus).beginTransmission(_address); // open the device
      	i2c_t3(_bus).write(subAddress); // write the register address
      	i2c_t3(_bus).write(data); // write the data
      	i2c_t3(_bus).endTransmission();
    }
    delay(10); // need to slow down how fast I write to MPU9250

  	/* read back the register */
  	readRegisters(subAddress,sizeof(buff),&buff[0]);

  	/* check the read back register against the written register */
  	if(buff[0] == data) {
  		return true;
  	}
  	else{
  		return false;
  	}
}

/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
void MPU9250::readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){

    if( _useSPI ){

    	// Teensy 3.0 || Teensy 3.1/3.2
		#if defined(__MK20DX128__) || defined(__MK20DX256__)

	    	if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
		        // begin the transaction
		        if(_useSPIHS){
		            SPI.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
		        }
		        else{
		            SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
		        }
		        digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

		        SPI.transfer(subAddress | SPI_READ); // specify the starting register address

                digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
                delayMicroseconds(1);
                digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

                SPI.transfer(subAddress | SPI_READ); // specify the starting register address

		        for(uint8_t i = 0; i < count; i++){
		            dest[i] = SPI.transfer(0x00); // read the data
		        }

		        digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
		        SPI.endTransaction(); // end the transaction
	    	}

    	#endif

        // Teensy 3.5 || Teensy 3.6
		#if defined(__MK64FX512__) || defined(__MK66FX1M0__)

	    	if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)||(_mosiPin == MOSI_PIN_28)){
		        // begin the transaction
		        if(_useSPIHS){
		            SPI.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
		        }
		        else{
		            SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
		        }
		        digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

		        SPI.transfer(subAddress | SPI_READ); // specify the starting register address

                digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
                delayMicroseconds(1);
                digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

                SPI.transfer(subAddress | SPI_READ); // specify the starting register address

		        for(uint8_t i = 0; i < count; i++){
		            dest[i] = SPI.transfer(0x00); // read the data
		        }

		        digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
		        SPI.endTransaction(); // end the transaction
	    	}
	    	else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
		        // begin the transaction
		        if(_useSPIHS){
		            SPI1.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
		        }
		        else{
		            SPI1.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
		        }
		        digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

		        SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

                digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
                delayMicroseconds(1);
                digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

                SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

		        for(uint8_t i = 0; i < count; i++){
		            dest[i] = SPI1.transfer(0x00); // read the data
		        }

		        digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
		        SPI1.endTransaction(); // end the transaction
	    	}
	    	else if((_mosiPin == MOSI_PIN_44)||(_mosiPin == MOSI_PIN_52)){
		        // begin the transaction
		        if(_useSPIHS){
		            SPI2.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
		        }
		        else{
		            SPI2.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
		        }
		        digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

		        SPI2.transfer(subAddress | SPI_READ); // specify the starting register address

                digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
                delayMicroseconds(1);
                digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

                SPI2.transfer(subAddress | SPI_READ); // specify the starting register address

		        for(uint8_t i = 0; i < count; i++){
		            dest[i] = SPI.transfer(0x00); // read the data
		        }

		        digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
		        SPI2.endTransaction(); // end the transaction
	    	}

    	#endif

        // Teensy LC
		#if defined(__MKL26Z64__)

	    	if((_mosiPin == MOSI_PIN_11)||(_mosiPin == MOSI_PIN_7)){
		        // begin the transaction
		        if(_useSPIHS){
		            SPI.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
		        }
		        else{
		            SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
		        }
		        digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

		        SPI.transfer(subAddress | SPI_READ); // specify the starting register address

                digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
                delayMicroseconds(1);
                digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

                SPI.transfer(subAddress | SPI_READ); // specify the starting register address

		        for(uint8_t i = 0; i < count; i++){
		            dest[i] = SPI.transfer(0x00); // read the data
		        }

		        digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
		        SPI.endTransaction(); // end the transaction
	    	}
	    	else if((_mosiPin == MOSI_PIN_0)||(_mosiPin == MOSI_PIN_21)){
		        // begin the transaction
		        if(_useSPIHS){
		            SPI1.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
		        }
		        else{
		            SPI1.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
		        }
		        digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

		        SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

                digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
                delayMicroseconds(1);
                digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

                SPI1.transfer(subAddress | SPI_READ); // specify the starting register address

		        for(uint8_t i = 0; i < count; i++){
		            dest[i] = SPI1.transfer(0x00); // read the data
		        }

		        digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
		        SPI1.endTransaction(); // end the transaction
	    	}

    	#endif
    }
    else{
        i2c_t3(_bus).beginTransmission(_address); // open the device
        i2c_t3(_bus).write(subAddress); // specify the starting register address
        i2c_t3(_bus).endTransmission(false);

        i2c_t3(_bus).requestFrom(_address, count); // specify the number of bytes to receive

        uint8_t i = 0; // read the data into the buffer
        while( i2c_t3(_bus).available() ){
            dest[i++] = i2c_t3(_bus).readByte();
        }
    }
}

/* writes a register to the AK8963 given a register address and data */
bool MPU9250::writeAK8963Register(uint8_t subAddress, uint8_t data){
	uint8_t count = 1;
	uint8_t buff[1];

	writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR); // set slave 0 to the AK8963 and set for write
	writeRegister(I2C_SLV0_REG,subAddress); // set the register to the desired AK8963 sub address
	writeRegister(I2C_SLV0_DO,data); // store the data for write
	writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | count); // enable I2C and send 1 byte

	// read the register and confirm
	readAK8963Registers(subAddress, sizeof(buff), &buff[0]);

	if(buff[0] == data) {
  		return true;
  	}
  	else{
  		return false;
  	}
}

/* reads registers from the AK8963 */
void MPU9250::readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest){

	writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR | I2C_READ_FLAG); // set slave 0 to the AK8963 and set for read
	writeRegister(I2C_SLV0_REG,subAddress); // set the register to the desired AK8963 sub address
	writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | count); // enable I2C and request the bytes
	delayMicroseconds(100); // takes some time for these registers to fill
	readRegisters(EXT_SENS_DATA_00,count,dest); // read the bytes off the MPU9250 EXT_SENS_DATA registers
}

/* gets the MPU9250 WHO_AM_I register value, expected to be 0x71 */
uint8_t MPU9250::whoAmI(){
    uint8_t buff[1];

    // read the WHO AM I register
    readRegisters(WHO_AM_I,sizeof(buff),&buff[0]);

    // return the register value
    return buff[0];
}

/* gets the AK8963 WHO_AM_I register value, expected to be 0x48 */
uint8_t MPU9250::whoAmIAK8963(){
    uint8_t buff[1];

    // read the WHO AM I register
    readAK8963Registers(AK8963_WHO_AM_I,sizeof(buff),&buff[0]);

    // return the register value
    return buff[0];
}

void MPU9250::MadgwickQuaternionUpdateIMU(float deltat) {
    float ax,  ay,  az,  gx,  gy,  gz;
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Convert gyroscope degrees/sec to radians/sec
    _calc_angles = false;
    getMotion6(&ax, &ay, &az, &gx, &gy, &gz);    

    gx *= _d2r;
    gy *= _d2r;
    gz *= _d2r;

    
	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalize step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * deltat;
	q1 += qDot2 * deltat;
	q2 += qDot3 * deltat;
	q3 += qDot4 * deltat;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q[0] = q0*recipNorm;
	q[1] = q1*recipNorm;
	q[2] = q2*recipNorm;
	q[3] = q3*recipNorm;
}

void MPU9250::resetQuaternion(float* src) {
    for (int i=0; i<4; i++, src++) q[i] = src[i];
}

void MPU9250::MadgwickQuaternionUpdate(float deltat)
{
  float ax,  ay,  az,  gx,  gy,  gz,  mx,  my,  mz;
  

  // short name local variable for readability
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  _calc_angles = false;
  getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  
  // convert gyro angles to radians
  gx *= _d2r;
  gy *= _d2r;
  gz *= _d2r;
  
  // Normalise accelerometer measurement
  norm = ax * ax + ay * ay + az * az;
  if (norm == 0.0f) return; // handle NaN
  norm = invSqrt( norm );
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = mx * mx + my * my + mz * mz;
  if (norm == 0.0f) return; // handle NaN
  norm = invSqrt( norm );
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 +
       _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = invSqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = invSqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

void MPU9250::MahonyQuaternionUpdate(float deltat)
{
    
  float ax,  ay,  az,  gx,  gy,  gz,  mx,  my,  mz;
  

  // short name local variable for readability
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  _calc_angles = false;
  getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  
  // convert gyro angles to radians
  gx *= _d2r;
  gy *= _d2r;
  gz *= _d2r;
  
  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // Handle NaN
  norm = 1.0f / norm;       // Use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // Handle NaN
  norm = 1.0f / norm;       // Use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
  }
  else
  {
    eInt[0] = 0.0f;     // prevent integral wind up
    eInt[1] = 0.0f;
    eInt[2] = 0.0f;
  }

  // Apply feedback terms
  gx = gx + Kp * ex + Ki * eInt[0];
  gy = gy + Kp * ey + Ki * eInt[1];
  gz = gz + Kp * ez + Ki * eInt[2];
 
  // Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

float  MPU9250::getPitch() {
    if (!_calc_angles) calculateAngles();
    return pitch;
}

float  MPU9250::getRoll() {
    if (!_calc_angles) calculateAngles();
    return roll;
}

float  MPU9250::getYaw() {
    if (!_calc_angles) calculateAngles();
    return yaw;
}

void  MPU9250::calculateAngles() {
    // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
    // In this coordinate system, the positive z-axis is down toward Earth.
    // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
    // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
    // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
    // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
    // applied in the correct order which for this configuration is yaw, pitch, and then roll.
    // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI;
    yaw   += mag_declination; // Declination at current location
    roll  *= 180.0f / PI;
    _calc_angles = true;
}

//-------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float MPU9250::invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void MPU9250::setTestData( float* d ) {
    if (_test_mode) {
    	for (int i=0; i<9; i++) _test_data[i] = *d++;
    }
}

#endif
