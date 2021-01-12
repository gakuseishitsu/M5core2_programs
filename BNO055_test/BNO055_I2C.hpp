#ifndef BNO055_I2C_HPP
#define BNO055_I2C_HPP

#include "Arduino.h"
#include <Wire.h>

#define BNO055_ADDRESS_A (0x28)
#define BNO055_ADDRESS_B (0x29)

typedef struct{
	float x,y,z;
}EULAR;

typedef struct{ 
    float x,y,z,w;
}QUATERNION;

class BNO055_I2C{

public:
    BNO055_I2C(uint8_t address = BNO055_ADDRESS_A, TwoWire *theWire = &Wire1);

    bool BNO055_init();
    QUATERNION get_quaternion();
    EULAR get_eular();
    EULAR get_accel();
    EULAR get_gyro();

private:
    void I2C_read(uint8_t reg, uint8_t number_bytes, uint8_t *read_buffer);
    void I2C_write(uint8_t reg, uint8_t val);

    uint8_t _address;
    TwoWire *_wire;
};

BNO055_I2C::BNO055_I2C(uint8_t address, TwoWire *theWire){
    _address = address;
    _wire = theWire;
}

void BNO055_I2C::I2C_read(uint8_t reg, uint8_t number_bytes, uint8_t *read_buffer){
    _wire->beginTransmission(_address);
    _wire->write(reg);  
    _wire->endTransmission(false);
    
    uint8_t i = 0;
    _wire->requestFrom(_address, number_bytes);
  
    //! Put read results in the Rx buffer
    while (_wire->available()) {
        read_buffer[i++] = _wire->read();
    }     
}

void BNO055_I2C::I2C_write(uint8_t reg, uint8_t val){
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->write(val);
    _wire->endTransmission();
}

bool BNO055_I2C::BNO055_init(){
    byte error;
    Wire1.beginTransmission(_address);
    error = Wire1.endTransmission();
    if (error == 0){
        I2C_write(0x3d, 0x08);
        return true;
    }else{
        return false;
    }
}

QUATERNION BNO055_I2C::get_quaternion(){
  uint8_t bno_readquat_address = 0x20;
  uint8_t bno_receivedata[16];
  int16_t quat[4];

  I2C_read(bno_readquat_address, 16, bno_receivedata);
  quat[0] = bno_receivedata[1] << 8 | bno_receivedata[0];
  quat[1] = bno_receivedata[3] << 8 | bno_receivedata[2];
  quat[2] = bno_receivedata[5] << 8 | bno_receivedata[4];
  quat[3] = bno_receivedata[7] << 8 | bno_receivedata[6];

  const float NORM = 16384.0;
  QUATERNION q = { (float)quat[1]/NORM,(float)quat[2]/NORM,(float)quat[3]/NORM,(float)quat[0]/NORM };
  return q;
}

EULAR BNO055_I2C::get_eular(){

	QUATERNION q = get_quaternion();
	EULAR e;
	// roll (x-axis rotation)
	double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
	double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
	e.x = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w * q.y - q.z * q.x);
	if (fabs(sinp) >= 1)
		e.y = copysign(3.1415926535 / 2, sinp); // use 90 degrees if out of range
	else
		e.y = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
	double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
	e.z = atan2(siny_cosp, cosy_cosp);

	return e;
}

EULAR BNO055_I2C::get_accel(){
    uint8_t bno_readaccel_address = 0x08;
	uint8_t bno_receivedata[16];
	int16_t e_raw[3];
	const float div = 100.0;

    I2C_read(bno_readaccel_address, 16, bno_receivedata);
	e_raw[0] = bno_receivedata[1] << 8 | bno_receivedata[0];
	e_raw[1] = bno_receivedata[3] << 8 | bno_receivedata[2];
	e_raw[2] = bno_receivedata[5] << 8 | bno_receivedata[4];

	EULAR e = { (float)e_raw[0]/div,(float)e_raw[1]/div,(float)e_raw[2]/div };
	return e;
}

EULAR BNO055_I2C::get_gyro(){
    uint8_t bno_readgyro_address = 0x14;
	uint8_t bno_receivedata[16];
	int16_t e_raw[3];
	const float div = 16.0;

    I2C_read(bno_readgyro_address, 16, bno_receivedata);
	e_raw[0] = bno_receivedata[1] << 8 | bno_receivedata[0];
	e_raw[1] = bno_receivedata[3] << 8 | bno_receivedata[2];
	e_raw[2] = bno_receivedata[5] << 8 | bno_receivedata[4];

	EULAR e = { (float)e_raw[0]/div,(float)e_raw[1]/div,(float)e_raw[2]/div };
	return e;
}

#endif //BNO055_I2C_HPP