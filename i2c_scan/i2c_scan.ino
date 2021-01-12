#include <Wire.h>
#include <M5Core2.h>
 
 
void setup()
{
  Wire1.begin();
  M5.begin();
 
  /*
  Serial.begin(9600);
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");
  */

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN , BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 20);
  M5.Lcd.println("\nI2C Scanner");

}
 
 
void loop()
{
  byte error, address;
  int nDevices;
 
  //Serial.println("Scanning...");
  M5.Lcd.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();
 
    if (error == 0)
    {
      //Serial.print("I2C device found at address 0x");
      M5.Lcd.print("I2C device found at address 0x");
      if (address<16)
        M5.Lcd.print("0");//Serial.print("0");
      //Serial.print(address,HEX);
      M5.Lcd.print(address,HEX);
      //Serial.println("  !");
      M5.Lcd.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      //Serial.print("Unknown error at address 0x");
      M5.Lcd.print("Unknown error at address 0x");
      if (address<16)
        M5.Lcd.print("0");//Serial.print("0");
      //Serial.println(address,HEX);
      M5.Lcd.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    M5.Lcd.println("No I2C devices found\n");//Serial.println("No I2C devices found\n");
  else
    M5.Lcd.println("done\n");//Serial.println("done\n");
 
  delay(5000);           // wait 5 seconds for next scan
}