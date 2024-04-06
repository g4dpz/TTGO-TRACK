#include <common.h>

unsigned long NextADC=0;

void SetupADC(void)
{
  //
}

void CheckADC(TGPS gps)
{
  if (millis() >= NextADC)
  {
    gps.BatteryVoltage = (int)(axp.getBattVoltage());
    
    NextADC = millis() + 10000L;
  }
}