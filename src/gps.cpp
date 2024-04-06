#include <Arduino.h>
#include <common.h>

// Globals
uint8_t RequiredFlightMode=0;
uint8_t GlonassMode=0;
uint8_t RequiredPowerMode=-1;
uint8_t LastCommand1=0;
uint8_t LastCommand2=0;
uint8_t HaveHadALock=0;

char Hex(int Character)
{
  char HexTable[] = "0123456789ABCDEF";
	
  return HexTable[Character];
}

void FixUBXChecksum(unsigned char *Message, int Length)
{ 
  int i;
  unsigned char CK_A, CK_B;
  
  CK_A = 0;
  CK_B = 0;

  for (i=2; i<(Length-2); i++)
  {
    CK_A = CK_A + Message[i];
    CK_B = CK_B + CK_A;
  }
  
  Message[Length-2] = CK_A;
  Message[Length-1] = CK_B;
}

void SendUBX(unsigned char *Message, int Length)
{
  int i;

  LastCommand1 = Message[2];
  LastCommand2 = Message[3];
  
  for (i=0; i<Length; i++)
  {
    Serial1.write(Message[i]);
  }
}


void DisableNMEAProtocol(unsigned char Protocol)
{
  unsigned char Disable[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
  
  Disable[7] = Protocol;
  
  FixUBXChecksum(Disable, sizeof(Disable));
  
  SendUBX(Disable, sizeof(Disable));
  
  Serial.print("Disable NMEA "); Serial.println(Protocol);
}

void SetFlightMode(uint8_t NewMode)
{
  // Send navigation configuration command
  unsigned char setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};

  setNav[8] = NewMode;

  FixUBXChecksum(setNav, sizeof(setNav));
  
  SendUBX(setNav, sizeof(setNav));
}

void SetupGPS(void)
{
  // Switch GPS on, if we have control of that
#ifdef GPS_ON
  pinMode(GPS_ON, OUTPUT);
  digitalWrite(GPS_ON, 1);
#endif

  Serial.println("Open GPS port");
  Serial1.begin(9600, SERIAL_8N1, 34, 12); // Pins for T-Beam v0.8 (3 push buttons) and up
  // Serial1.begin(9600, SERIAL_8N1, 12, 15); // For version 0.7 (2 push buttons) and down
}

int GPSChecksumOK(char *Buffer, int Count)
{
  unsigned char XOR, i, c;

  XOR = 0;
  for (i = 1; i < (Count-4); i++)
  {
    c = Buffer[i];
    XOR ^= c;
  }

  return (Buffer[Count-4] == '*') && (Buffer[Count-3] == Hex(XOR >> 4)) && (Buffer[Count-2] == Hex(XOR & 15));
}

float FixPosition(float Position)
{
  float Minutes, Seconds;
  
  Position = Position / 100;
  
  Minutes = trunc(Position);
  Seconds = fmod(Position, 1);

  return Minutes + Seconds * 5 / 3;
}

void ProcessNMEA(TGPS *gps, TSettings settings,  char *Buffer, int Count)
{
  int Satellites, date;
  char ns, ew;
  char TimeString[16], LatString[16], LongString[16], Temp[4];

  //Serial.print(Buffer);

  if (GPSChecksumOK(Buffer, Count))
  {
    Satellites = 0;
  
    if (strncmp(Buffer+3, "GGA", 3) == 0)
    {
      int lock;
      char hdop[16], Altitude[16];
      
      // Serial.print(Buffer+1);
      
      if (sscanf(Buffer+7, "%16[^,],%16[^,],%c,%[^,],%c,%d,%d,%[^,],%[^,]", TimeString, LatString, &ns, LongString, &ew, &lock, &Satellites, hdop, Altitude) >= 1)
      { 
        // $GPGGA,124943.00,5157.01557,N,00232.66381,W,1,09,1.01,149.3,M,48.6,M,,*42
        Temp[0] = TimeString[0]; Temp[1] = TimeString[1]; Temp[2] = '\0';
        gps->Hours = atoi(Temp);
        Temp[0] = TimeString[2]; Temp[1] = TimeString[3]; Temp[2] = '\0';
        gps->Minutes = atoi(Temp);
        Temp[0] = TimeString[4]; Temp[1] = TimeString[5]; Temp[2] = '\0';
        gps->Seconds = atoi(Temp);
        gps->SecondsInDay = (unsigned long)gps->Hours * 3600L + (unsigned long)gps->Minutes * 60L + (unsigned long)gps->Seconds;

        if (gps->UseHostPosition)
        {
          gps->UseHostPosition--;
        }
        else if (Satellites >= 4)
        {
          gps->Latitude = FixPosition(atof(LatString));
          if (ns == 'S') gps->Latitude = -gps->Latitude;
          gps->Longitude = FixPosition(atof(LongString));
          if (ew == 'W') gps->Longitude = -gps->Longitude;
          gps->PreviousAltitude = gps->Altitude;
          gps->Altitude = (unsigned int)atof(Altitude);
          gps->AscentRate = gps->AscentRate * 0.7 + (gps->Altitude - gps->PreviousAltitude) * 0.3;
        }
        
        gps->Satellites = Satellites;

        if (gps->Altitude > gps->MaximumAltitude)
        {
          gps->MaximumAltitude = gps->Altitude;
        }
        
        if ((gps->Altitude < gps->MinimumAltitude) || (gps->MinimumAltitude == 0))
        {
          gps->MinimumAltitude = gps->Altitude;           
        }

        // Launched?
        if ((gps->AscentRate >= 1.0) && (gps->Altitude > (gps->MinimumAltitude+150)) && (gps->FlightMode == fmIdle))
        {
          gps->FlightMode = fmLaunched;
          Serial.printf("*** LAUNCHED ***\n");
        }

        // Burst?
        if ((gps->AscentRate < -10.0) && (gps->Altitude < (gps->MaximumAltitude+50)) && (gps->MaximumAltitude >= (gps->MinimumAltitude+2000)) && (gps->FlightMode == fmLaunched))
        {
          gps->FlightMode = fmDescending;
          Serial.printf("*** DESCENDING ***\n");
        }
        
        // Landed?
        if ((gps->AscentRate >= -0.1) && (gps->Altitude <= settings.LandingAltitude+2000) && (gps->FlightMode >= fmDescending) && (gps->FlightMode < fmLanded))
        {
          gps->FlightMode = fmLanded;
          Serial.printf("*** LANDED ***\n");
        }        

        #ifdef OLED
          ShowGPSStatus();
        #endif
      }
      
      // Serial.print(gps->Hours); Serial.print(":"); Serial.print(gps->Minutes); Serial.print(":"); Serial.print(gps->Seconds);Serial.print(" - ");
      // Serial.print(gps->Latitude, 6); Serial.print(',');Serial.print(gps->Longitude, 6);Serial.print(',');Serial.print(gps->Altitude);Serial.print(',');
      // Serial.println(gps->Satellites);
    }
    else if (strncmp((char *)Buffer+3, "RMC", 3) == 0)
    {
      float Speed, Direction;
      
      // Serial.print(Buffer+1);
      // GPRMC,154036.00,A,5157.00573,N,00232.66685,W,0.210,,160322,,,A*6F   

      Direction = 0;
      if (sscanf(Buffer+7, "%*[^,],%*[^,],%*[^,],%*[^,],%*[^,],%*[^,],%f,%f", &Speed, &Direction) >= 1)
      { 
        gps->Speed = Speed;
        gps->Direction = Direction;
        // Serial.printf("%f %f\n", Speed, Direction);
      }
    }
    else if (strncmp((char *)Buffer+3, "GSV", 3) == 0)
    {
      DisableNMEAProtocol(3);
    }
    else if (strncmp((char *)Buffer+3, "GLL", 3) == 0)
    {
      DisableNMEAProtocol(1);
    }
    else if (strncmp((char *)Buffer+3, "GSA", 3) == 0)
    {
      DisableNMEAProtocol(2);
    }
    else if (strncmp((char *)Buffer+3, "VTG", 3) == 0)
    {
      DisableNMEAProtocol(5);
    }
  }
  else
  {
    Serial.println("Bad checksum");
  }
}

  
void CheckGPS(TGPS *gps, TSettings settings)
{
  static unsigned long ModeTime=0;
  static char Line[128];
  static int Length=0;
  unsigned char Character;

  while (Serial1.available())
  {
    Character = Serial1.read();
  
    if (Character == '$')
    {
      Line[0] = Character;
      Length = 1;
    }
    else if (Length >= (sizeof(Line)-2))
    {
      Length = 0;
    }
    else if ((Length > 0) && (Character != '\r'))
    {
      Line[Length++] = Character;
      if (Character == '\n')
      {
        Line[Length] = '\0';
        ProcessNMEA(gps, settings, Line, Length);
        Length = 0;
      }
    }
  }

  if (millis() >= ModeTime)
  {
    if ((settings.GPSDynamicModel >= 2) && (settings.GPSDynamicModel <= 8))
    {
      // Specific mode use as-is
      RequiredFlightMode = settings.GPSDynamicModel;
    }
    else if (settings.GPSDynamicModel == 1)
    {
      // Portable mode
      RequiredFlightMode = 0;
    }
    else
    {
      RequiredFlightMode = (gps->Altitude > settings.FlightModeAltitude) ? 6 : 3;    // 6 is airborne <1g mode; 3=Pedestrian mode
    }
    
    if (RequiredFlightMode != gps->GPSFlightMode)
    {
      gps->GPSFlightMode = RequiredFlightMode;

      SetFlightMode(RequiredFlightMode);
      Serial.print("Setting flight mode ");
      Serial.println(RequiredFlightMode);
    }
    
    ModeTime = millis() + 60000;
  }
}