#include <M5Core2.h>

TaskHandle_t LCD_control_handle;
TaskHandle_t CMG_control_handle;
void LCD_control_task(void *pvParameters);
void CMG_control_task(void *pvParameters);

uint32_t global_val;

void setup() {
	// put your setup code here, to run once:

	M5.begin();

	M5.Lcd.fillScreen(BLACK);
	M5.Lcd.setTextSize(3);
	M5.Lcd.setCursor(0, 20);
	M5.Lcd.printf("VScode test");

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
		M5.Lcd.printf("%d", global_val);

		vTaskDelayUntil(&xLastWakeTime, 100/portTICK_PERIOD_MS);
	}
}

void CMG_control_task(void *pvParameters){

	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	while(1){
		global_val++;

		vTaskDelayUntil(&xLastWakeTime, 10/portTICK_PERIOD_MS);
	}
}