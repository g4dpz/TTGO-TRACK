#include <Arduino.h>
#include <common.h>

#ifdef I2C_SLAVE

#include <Wire.h>

extern struct TGPS gps;

void onRequest()
{
  if (*gps.UplinkText)
  {
    Wire.write(gps.UplinkText);
    Serial.print("onRequest - send '");
    Serial.print(gps.UplinkText);
    Serial.println("'");
    gps.UplinkText[0] = '\0';
  }
  else
  {
    Wire.write(0);
    Serial.println("onRequest - nothing to send");
  }
}

void onReceive(int len)
{
  int i=0;
  
  Serial.printf("onReceive[%d]: ", len);
  while(Wire.available())
  {
    gps.ExtraFields[i++] = Wire.read();
  }
  gps.ExtraFields[i] = '\0';    
  
  Serial.println(gps.ExtraFields);
}

void SetupSlave()
{
  Serial.println("Setting up I2C slave");
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_SLAVE_ADDR);  // , I2C_SLAVE_SDA_PIN, I2C_SLAVE_SCL_PIN);
}

#endif
