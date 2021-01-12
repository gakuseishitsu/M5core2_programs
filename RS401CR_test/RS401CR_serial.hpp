#ifndef RS401CR_SERIAL_HPP
#define RS401CR_SERIAL_HPP

#include "Arduino.h"
#include "HardwareSerial.h"

class RS401CR{
public:
    RS401CR(HardwareSerial *uart = &Serial);

    void RS401CR_serial_init();
    void RS401CR_FLAG_reset(int servo_id);
    void RS401CR_FLAG_write_to_flash_ROM(int servo_id);
    void RS401CR_FLAG_factory_initialization(int servo_id);
    void RS401CR_CMD04_change_id(int servo_id, int new_id);
    void RS401CR_CMD32_rotate(int servo_id, float angle, float duration);
    void RS401CR_CMD36_set_torque(int servo_id, int servo_mode);

private:
    void serial_write(uint8_t *write_buffer, uint8_t number_bytes);

    HardwareSerial *_uart;
};

RS401CR::RS401CR(HardwareSerial *uart){
    _uart = uart;
}

void RS401CR::RS401CR_serial_init(){
    _uart->begin(115200);
}

void RS401CR::serial_write(uint8_t *write_buffer, uint8_t number_bytes){
    for(int i = 0;i < number_bytes; i++){
        _uart->write(write_buffer[i]);
        delayMicroseconds(1); // Why we need it
    }
}

void RS401CR::RS401CR_FLAG_reset(int servo_id){
	unsigned char sendbuf[32];
	sendbuf[0] = (unsigned char) 0xFA; ///header1
	sendbuf[1] = (unsigned char) 0xAF; //header2
	sendbuf[2] = (unsigned char) servo_id; //servo ID
	sendbuf[3] = (unsigned char) 0x20; //flg
	sendbuf[4] = (unsigned char) 0xFF; //addres
	sendbuf[5] = (unsigned char) 0x00; //length
	sendbuf[6] = (unsigned char) 0x00; //Cnt

	// check sum
	unsigned char sum;
	sum = sendbuf[2];
	for (int i = 3; i < 7; i++) {
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[7] = sum;

    serial_write(sendbuf, 8);
    //_uart->write(sendbuf,8);
}

void RS401CR::RS401CR_FLAG_write_to_flash_ROM(int servo_id){
	unsigned char sendbuf[32];
	sendbuf[0] = (unsigned char) 0xFA; ///header1
	sendbuf[1] = (unsigned char) 0xAF; //header2
	sendbuf[2] = (unsigned char) servo_id; //servo ID
	sendbuf[3] = (unsigned char) 0x40; //flg
	sendbuf[4] = (unsigned char) 0xFF; //addres
	sendbuf[5] = (unsigned char) 0x00; //length
	sendbuf[6] = (unsigned char) 0x00; //Cnt

	// check sum
	unsigned char sum;
	sum = sendbuf[2];
	for (int i = 3; i < 7; i++) {
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[7] = sum;

    serial_write(sendbuf, 8);
    //_uart->write(sendbuf,8);
}

void RS401CR::RS401CR_FLAG_factory_initialization(int servo_id){
	unsigned char sendbuf[32];
	sendbuf[0] = (unsigned char) 0xFA; ///header1
	sendbuf[1] = (unsigned char) 0xAF; //header2
	sendbuf[2] = (unsigned char) servo_id; //servo ID
	sendbuf[3] = (unsigned char) 0x10; //flg
	sendbuf[4] = (unsigned char) 0xFF; //addres
	sendbuf[5] = (unsigned char) 0xFF; //length
	sendbuf[6] = (unsigned char) 0x00; //Cnt

	// check sum
	unsigned char sum;
	sum = sendbuf[2];
	for (int i = 3; i < 7; i++) {
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[7] = sum;

    serial_write(sendbuf, 8);
    //_uart->write(sendbuf,8);
}

void RS401CR::RS401CR_CMD04_change_id(int servo_id, int new_id){
	unsigned char sendbuf[32];
	sendbuf[0] = (unsigned char) 0xFA; ///header1
	sendbuf[1] = (unsigned char) 0xAF; //header2
	sendbuf[2] = (unsigned char) servo_id; //servo ID
	sendbuf[3] = (unsigned char) 0x00; //flg
	sendbuf[4] = (unsigned char) 0x04; //addres
	sendbuf[5] = (unsigned char) 0x01; //length
	sendbuf[6] = (unsigned char) 0x01; //Cnt
	sendbuf[7] = (unsigned char) new_id; //Dat

	// check sum
	unsigned char sum;
	sum = sendbuf[2];
	for (int i = 3; i < 8; i++) {
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[8] = sum;

    serial_write(sendbuf, 9);
    //_uart->write(sendbuf,9);
}

void RS401CR::RS401CR_CMD36_set_torque(int servo_id, int servo_mode){
    uint8_t sendbuf[32];
	sendbuf[0] = (unsigned char) (0xFA); //header1
	sendbuf[1] = (unsigned char) (0xAF); //header2
	sendbuf[2] = (unsigned char) (servo_id); //servo ID
	sendbuf[3] = (unsigned char) (0x00); //flg
	sendbuf[4] = (unsigned char) (0x24); //addres(0x24=36)
	sendbuf[5] = (unsigned char) (0x01); //length(4byte)
	sendbuf[6] = (unsigned char) (0x01); //numver
	sendbuf[7] = (unsigned char)((servo_mode&0x00FF)); // ON/OFF flg

	// check sum
    uint8_t sum;
	sum = sendbuf[2];
	for (int i = 3; i < 8; i++) {
        sum = sum ^ sendbuf[i];
	}
	sendbuf[8] = sum;
	serial_write(sendbuf, 9);
    //_uart->write(sendbuf,9);
}

void RS401CR::RS401CR_CMD32_rotate(int servo_id, float angle, float duration){

	// float to HEX
	// angle(from -90~90 into 0xFB50~0x0384)
	// duration (from sec into 10msec)
	int16_t pos,time;
	pos = (int16_t)(angle*10);
	time = (int16_t)(duration*100);

	unsigned char sendbuf[32];
	sendbuf[0] = (unsigned char) 0xFA; ///header1
	sendbuf[1] = (unsigned char) 0xAF; //header2
	sendbuf[2] = (unsigned char) servo_id; //servo ID
	sendbuf[3] = (unsigned char) 0x00; //flg
	sendbuf[4] = (unsigned char) 0x1E; //addres(0x1E=30)
	sendbuf[5] = (unsigned char) 0x04; //length(4byte)
	sendbuf[6] = (unsigned char) 0x01; //numver
	sendbuf[7] = (unsigned char) (pos & 0x00FF); //position1
	sendbuf[8] = (unsigned char) ((pos & 0xFF00) >> 8); //position2
	sendbuf[9] = (unsigned char) (time & 0x00FF); //duration1
	sendbuf[10] = (unsigned char) ((time & 0xFF00) >> 8); //duration1

	// check sum
	unsigned char sum;
	sum = sendbuf[2];
	for (int i = 3; i < 11; i++) {
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[11] = sum;

    serial_write(sendbuf, 12);
    //_uart->write(sendbuf,12);
}

#endif //RS401CR_SERIAL_HPP