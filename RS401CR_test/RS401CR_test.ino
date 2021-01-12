#include <M5Core2.h>
#include "RS401CR_serial.hpp"

RS401CR servo1 = RS401CR(&Serial);

void setup() {
    // put your setup code here, to run once:
    M5.begin();

    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(0, 10);
    M5.Lcd.println("[INFO]RS401CR test");

    servo1.RS401CR_serial_init();
    servo1.RS401CR_CMD36_set_torque(2, 1);

}

void loop() {
    // put your main code here, to run repeatedly:

    servo1.RS401CR_CMD36_set_torque(2, 1);
    servo1.RS401CR_CMD32_rotate(2, 45.0, 0);
    delay(1000);
    servo1.RS401CR_CMD36_set_torque(2, 1);
    servo1.RS401CR_CMD32_rotate(2, 0.0, 0);
    delay(1000);
    servo1.RS401CR_CMD36_set_torque(2, 1);
    servo1.RS401CR_CMD32_rotate(2, -45.0, 0);
    delay(1000);
    servo1.RS401CR_CMD36_set_torque(2, 1);
    servo1.RS401CR_CMD32_rotate(2, 0.0, 0);
    delay(1000);

}