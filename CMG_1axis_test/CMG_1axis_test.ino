#include <M5Core2.h>
#include "BNO055_I2C.hpp"
#include "RS401CR_serial.hpp"

TaskHandle_t LCD_control_handle;
TaskHandle_t CMG_control_handle;
void LCD_control_task(void *pvParameters);
void CMG_control_task(void *pvParameters);

BNO055_I2C bno055 = BNO055_I2C(0x29, &Wire1);
EULAR e, w;

RS401CR servo1 = RS401CR(&Serial);

void setup() {
	// put your setup code here, to run once:

	M5.begin();

	M5.Lcd.fillScreen(BLACK);
	M5.Lcd.setTextSize(3);
	M5.Lcd.setCursor(0, 20);
	M5.Lcd.printf("[INFO]CMG 1axis test");

	//init BNO055
    if(bno055.BNO055_init()==true){
        M5.Lcd.println("[INFO]connection success");
    }else{
        M5.Lcd.println("[INFO]connection failed");
    }

	//init RS401CR
	servo1.RS401CR_serial_init();
    servo1.RS401CR_CMD36_set_torque(2, 1);

	xTaskCreateUniversal(LCD_control_task, "LCD_control _task", 8192, NULL, 0, &LCD_control_handle, APP_CPU_NUM);
	xTaskCreateUniversal(CMG_control_task, "CMG_control _task", 8192, NULL, 2, &CMG_control_handle, PRO_CPU_NUM);

}

void loop() {
	// put your main code here, to run repeatedly:
	delay(1);
}

void LCD_control_task(void *pvParameters){

	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	while(1){
		
		M5.Lcd.setCursor(0, 50);
    	M5.Lcd.printf("roll:%f\r\n", e.x); // rad
    	M5.Lcd.printf("pitch:%f\r\n", e.y); // rad
    	M5.Lcd.printf("yaw:%f\r\n", e.z); // rad
    	M5.Lcd.printf("wx:%f\r\n", w.x); // deg/sec
    	M5.Lcd.printf("wy:%f\r\n", w.y); // deg/sec
    	M5.Lcd.printf("wz:%f\r\n", w.z); // deg/sec


		vTaskDelayUntil(&xLastWakeTime, 100/portTICK_PERIOD_MS);
	}
}

void CMG_control_task(void *pvParameters){

	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	while(1){

		e = bno055.get_eular();
    	w = bno055.get_gyro();

		servo1.RS401CR_CMD32_rotate(2, 90.0, 0);

		vTaskDelayUntil(&xLastWakeTime, 10/portTICK_PERIOD_MS);
	}
}