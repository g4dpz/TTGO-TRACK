#include "DFRobot_BMP280.h"
#include "Wire.h"
#include <common.h>

typedef DFRobot_BMP280_IIC BMP;    // ******** use abbreviations instead of full names ********

BMP bmp(&Wire, BMP::eSdoHigh);

#define SEA_LEVEL_PRESSURE    1015.0f   // sea level pressure

// show last sensor operate status
void printLastOperateStatus(BMP::eStatus_t eStatus)
{
  switch(eStatus) {
  case BMP::eStatusOK:    Serial.println("everything ok"); break;
  case BMP::eStatusErr:   Serial.println("unknow error"); break;
  case BMP::eStatusErrDeviceNotDetected:    Serial.println("device not detected"); break;
  case BMP::eStatusErrParameter:    Serial.println("parameter error"); break;
  default: Serial.println("unknow status"); break;
  }
}

void SetupBMP280()
{
  bmp.reset();
  Serial.println("bmp280 config test");
  while(bmp.begin() != BMP::eStatusOK) {
    Serial.println("bmp280 begin failed");
    printLastOperateStatus(bmp.lastOperateStatus);
    delay(2000);
  }
  Serial.println("bmp280 begin success");

  bmp.setConfigFilter(BMP::eConfigFilter_off);        // set config filter
  bmp.setConfigTStandby(BMP::eConfigTStandby_125);    // set standby time
  bmp.setCtrlMeasSamplingTemp(BMP::eSampling_X8);     // set temperature over sampling
  bmp.setCtrlMeasSamplingPress(BMP::eSampling_X8);    // set pressure over sampling
  bmp.setCtrlMeasMode(BMP::eCtrlMeasModeNormal);     // set control measurement mode to make these settings effective

  delay(100);
}

void CheckBMP280(TEN_DOF* tendof)
{
  float   temp = bmp.getTemperature();
  uint32_t    press = bmp.getPressure();
  float   alti = bmp.calAltitude(SEA_LEVEL_PRESSURE, press);

  tendof->Temperature = temp;
  tendof->Pressure = press/100.0f;
  tendof->Altitude = alti;
}