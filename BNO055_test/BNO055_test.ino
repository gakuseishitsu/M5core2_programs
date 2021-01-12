#include <M5Core2.h>
#include "BNO055_I2C.hpp"

BNO055_I2C bno055 = BNO055_I2C(0x29, &Wire1);
EULAR e, w;

void setup() {
    // put your setup code here, to run once:
    M5.begin();

    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(0, 10);
    M5.Lcd.println("[INFO]start connection");

    //start BNO055
    if(bno055.BNO055_init()==true){
        M5.Lcd.println("[INFO]connection success");
    }else{
        M5.Lcd.println("[INFO]connection failed");
    }

}

void loop() {
    // put your main code here, to run repeatedly:

    e = bno055.get_eular();
    w = bno055.get_gyro();

    M5.Lcd.setCursor(0, 50);
    M5.Lcd.printf("roll:%f\r\n", e.x); // rad
    M5.Lcd.printf("pitch:%f\r\n", e.y); // rad
    M5.Lcd.printf("yaw:%f\r\n", e.z); // rad
    M5.Lcd.printf("wx:%f\r\n", w.x); // deg/sec
    M5.Lcd.printf("wy:%f\r\n", w.y); // deg/sec
    M5.Lcd.printf("wz:%f\r\n", w.z); // deg/sec

    delay(50);

}