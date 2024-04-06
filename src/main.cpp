/*-------------------------------------------------------------------------------------------------------\
|                                                                                                        |
| FlexTrack specifically for the TTGO T-Beam which has ESP32, UBlox NEO-6M and LoRa transceiver          |
|                                                                                                        |
| Compared with usual AVR FlexTrack, it has landing prediction and EEPROM configuration.                 |
|                                                                                                        |
\*------------------------------------------------------------------------------------------------------*/

#include <common.h>

//------------------------------------------------------------------------------------------------------

// CONFIGURATION SECTION.  CHANGE AS REQUIRED.

// #define LORA_DONT_LISTEN                  // Saves power

// Optional devices - uncomment if present
// #define OLED                                // Enable OLED
// #define WIREBUS 4
// #define BMP085         // Also works with BMP180
// #define BNO085

// PRODUCT INFO

#define   VERSION     "V1.45"
#define   PRODUCT     "FlexTrack"
#define   DESCRIPTION "TTGO T-Beam ESP32"

// FIXED CONFIG

#define SIG_1   'D'
#define SIG_2   'E'


// HARDWARE DEFINITION FOR TTGO T-BEAM

#define OLED_SDA                21
#define OLED_SCL                22
#define OLED_ADDRESS            0x3C
#define SCREEN_WIDTH            128
#define SCREEN_HEIGHT           64


// Libraries

#include <EEPROM.h>

#ifdef OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#endif

#ifdef BMP085
#include <Adafruit_BMP085.h>
#endif

#ifdef BNO085
#include <Adafruit_BNO08x.h>
#include <sh2.h>
#include <sh2_err.h>
#include <sh2_hal.h>
#include <sh2_SensorValue.h>
#include <sh2_util.h>
#include <shtp.h>
#endif

#ifdef WIREBUS
#include "OneWireNg_CurrentPlatform.h"
#include "drivers/DSTherm.h"
#include "utils/Placeholder.h"
#endif

#ifdef SERVO_PIN
#include <ESP32Servo.h>
#endif

//------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------

// HARDWARE DEFINITION

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI

//------------------------------te------------------------------------------------------------------------

// OLED
#ifdef OLED
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
#endif

// Cutdown ...
#ifdef CUTDOWN
  #define CUTDOWN_FIELD_FORMAT    ",%d"
  #define CUTDOWN_FIELD_LIST       ,gps.CutdownStatus
#else
  #define CUTDOWN_FIELD_FORMAT    
  #define CUTDOWN_FIELD_LIST          
#endif

// IMU ...
#ifdef BNO085
  #define IMU_FIELD_FORMAT    ",%d,%d,%d"
  #define IMU_FIELD_LIST       ,gps.Heading, gps.Pitch, gps.Roll

#else
  #define IMU_FIELD_FORMAT    
  #define IMU_FIELD_LIST          
#endif

struct TSettings settings;
struct TGPS gps;

// function prototypes (needed for PlatformIO)
void LoadSettings();
void SetDefaults();
void SetupLEDs();
void SetupCutdown();
void SetupADC();
void SetupGPS();
void SetupLoRa(TSettings settings);
void SetupPrediction(TGPS gps, TSettings settings);
void CheckGPS(TGPS *gps, TSettings settings);
void CheckCutdown();
void CheckLoRa(TGPS gps, TSettings settings);
void CheckADC(TGPS gps);
void CheckLEDs();
void CheckHost();
void CheckPrediction(TGPS gps, TSettings settings);
void ProcessCommand(char *Line);
int ProcessGPSCommand(char *Line);
int ProcessCommonCommand(char *Line);
int ProcessLORACommand(char *Line);
int ProcessRTTYCommand(char *Line);
int ProcessPredictionCommand(char *Line);
int ProcessFieldCommand(TGPS gps, char *Line);
void CutdownNow(unsigned long Period);
void SaveSettings();
void SendSettings();
void setupRFM98(double Frequency, int Mode);
void SetupSlave();

//------------------------------------------------------------------------------------------------------

void setup()
{
  // Serial port(s)

  setCpuFrequencyMhz(80);
  
  Serial.begin(38400);

  delay(1000);
  
  Serial.println();
  Serial.println();
  Serial.printf("%s: %s %s\n", PRODUCT, DESCRIPTION, VERSION); 
  Serial.println();

  EEPROM.begin(EEPROM_SIZE);  

  if ((EEPROM.read(0) == SIG_1) && (EEPROM.read(1) == SIG_2))
  {
    // Store current (default) settings
    Serial.println("Loading settings from EEPROM");
    LoadSettings();
  }
  else
  {
    Serial.println("Placing default settings in EEPROM");
    SetDefaults();
  }
  
  Serial.print("Payload ID ");
  Serial.println(settings.PayloadID);

  gps.UplinkText[0] = '\0';
  gps.ExtraFields[0] = '\0';
        
#ifdef CUTDOWN
  gps.CutdownAltitude = Settings.CutdownAltitude;
  
  SetupCutdown();
#endif

  Wire.begin(21, 22);
  if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS))
  {
    Serial.println("AXP192 OK");
  } else {
    Serial.println("AXP192 FAIL");
  }
  axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);   // LoRa On
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);   // GPS On
  axp.setPowerOutPut(AXP192_DCDC2, AXP202_OFF);  // ??
  axp.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);  // ??
  axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);  // OLED Off
  // axp.setDCDC1Voltage(3300);
  
//  Serial.printf("DCDC1: %s\n", axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");
//  Serial.printf("DCDC2: %s\n", axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");
//  Serial.printf("DCDC3: %s\n", axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");
//  Serial.printf("LDO2: %s\n", axp.isLDO2Enable() ? "ENABLE" : "DISABLE");
//  Serial.printf("LDO3: %s\n", axp.isLDO3Enable() ? "ENABLE" : "DISABLE");
//  Serial.printf("Exten: %s\n", axp.isExtenEnable() ? "ENABLE" : "DISABLE");

  if (axp.isChargeing()) {
      Serial.println("Charging");
  }
  Wire.end();

#ifdef OLED
  SetupOLED();
#endif

#ifdef BNO085
  SetupBNO085();
#endif

  SetupLEDs();

  SetupADC();
  
  SetupGPS();
  
  SetupLoRa(settings);

#ifdef WIREBUS
  Setupds18b20();
#endif

#ifdef BMP085
  SetupBMP085();
#endif

  SetupPrediction(gps, settings);

#ifdef I2C_SLAVE
  SetupSlave();
#endif
}


void loop()
{  
  CheckGPS(&gps, settings);

#ifdef CUTDOWN
  CheckCutdown();
#endif

  CheckLoRa(gps, settings);

  CheckADC(gps);
   
  CheckLEDs();

#ifdef WIREBUS
  Checkds18b20();
#endif

#ifdef BMP085
  CheckBMP085();
#endif

#ifdef BNO085
  CheckBNO085();
#endif

  CheckHost();

  CheckPrediction(gps, settings);
}

void CheckHost(void)
{
  static char Line[COMMAND_BUFFER_LENGTH];
  static unsigned int Length=0;
  char Character;

  while (Serial.available())
  { 
    Character = Serial.read();

    if (Character == '~')
    {
      Line[0] = Character;
      Length = 1;
    }
    else if (Character == '\r' || Character == '\n')
    {
      Line[Length] = '\0';
      ProcessCommand(Line+1);
      Length = 0;
    }
    else if (Length >= sizeof(Line))
    {
      Length = 0;
    }
    else if (Length > 0)
    {
      Line[Length++] = Character;
    }
  }
}

void ProcessCommand(char *Line)
{
  int OK = 0;

  if (Line[0] == 'G')
  {
    OK = ProcessGPSCommand(Line+1);
  }
  else if (Line[0] == 'C')
  {
    OK = ProcessCommonCommand(Line+1);
  }
  else if (Line[0] == 'L')
  {
    OK = ProcessLORACommand(Line+1);
  }
  else if (Line[0] == 'R')
  {
    OK = ProcessRTTYCommand(Line+1);
  }
  else if (Line[0] == 'P')
  {
    OK = ProcessPredictionCommand(Line+1);
  }
  else if (Line[0] == 'F')
  {
    OK = ProcessFieldCommand(gps, Line+1);
  }
  #ifdef CUTDOWN
  else if (Line[0] == 'X')
  {
    CutdownNow(5000);
  }
  #endif

  if (OK)
  {
    Serial.println("*");
    #ifdef OLED
      ShowSettings();
    #endif
  }
  else
  {
    Serial.println("?");
  }
}

int ProcessFieldCommand(TGPS gps, char *Line)
{
  int OK = 0;

  if (Line[0] == 'P')
  {
    gps.PreviousAltitude = gps.Altitude;
    sscanf(Line+1,"%f,%f,%ld", &gps.Latitude, &gps.Longitude, &gps.Altitude);
    gps.UseHostPosition = 5;
    gps.AscentRate = gps.AscentRate * 0.7 + (gps.Altitude - gps.PreviousAltitude) * 0.3;
    OK = 1;
  }
  
  return OK;
}

void SetDefaults(void)
{
  // Common settings
  strcpy(settings.PayloadID, "TTGO");

  const static char DefaultFieldList[] = "0123456789";   // "0123456CD";
  strcpy(settings.FieldList, (char *)DefaultFieldList);

  // GPS Settings
  settings.FlightModeAltitude = 2000;
  settings.GPSDynamicModel = 0;

  // LoRa Settings
  settings.LoRaFrequency = 434.454;
  settings.LoRaMode = 2;
  settings.LoRaCycleTime = 5;
  settings.LoRaSlot = 1;
  settings.CallingCount = 0;
  settings.LoRaRepeatSlot1 = -1;
  settings.LoRaRepeatSlot2 = -1;
  settings.UseBinaryMode = 0;
  settings.BinaryNode = 0;

  // Cutdown Settings
  settings.CutdownAltitude = 0;     // Disables cutdown
  settings.CutdownPeriod = 5000;    // ms

  // Prediction Settings
  settings.CDA = 0.7;
  settings.PayloadWeight = 1.0;
  settings.LandingAltitude = 100;

  // RTTY Settings
  settings.RTTYFrequency = 434.400;
  settings.RTTYBaudRate = 300;
  settings.RTTYAudioShift = 488;
  settings.RTTYCount = 0;
  settings.RTTYEvery = 3;
  settings.RTTYPreamble = 8;
  
  // Uplink Settings
  settings.EnableUplink = 0;
  strcat(settings.UplinkCode, "");
  
  SaveSettings();
}

void LoadSettings(void)
{
  unsigned int i;
  unsigned char *ptr;

  ptr = (unsigned char *)(&settings);
  for (i=0; i<sizeof(settings); i++, ptr++)
  {
    *ptr = EEPROM.read(i+2);
  }
}

void SaveSettings(void)
{
  unsigned int i;
  unsigned char *ptr;
  
  // Signature
  EEPROM.write(0, SIG_1);
  EEPROM.write(1, SIG_2);

  // Settings
  ptr = (unsigned char *)(&settings);
  for (i=0; i<sizeof(settings); i++, ptr++)
  {
    EEPROM.write(i+2, *ptr);
  }

  EEPROM.commit();
}

int ProcessCommonCommand(char *Line)
{
  Serial.print("Processing command: ");
  Serial.println(Line);
  int OK = 0;

  if (Line[0] == 'P')
  {
    // Store payload ID
    if (strlen(Line+1) < PAYLOAD_LENGTH)
    {
      strcpy(settings.PayloadID, Line+1);
      OK = 1;
    }
  }
  else if (Line[0] == 'F')
  {
    // Store Field List
    if (strlen(Line+1) < FIELDLIST_LENGTH)
    {
      strcpy(settings.FieldList, Line+1);
      OK = 1;
    }
  }
  else if (Line[0] == 'R')
  {
    // Reset to default settings
    SetDefaults();
    OK = 1;
  }
  else if (Line[0] == 'S')
  {
    // Save settings to flash
    SaveSettings();
    OK = 1;
  }
  else if (Line[0] == 'E')
  {
    SendSettings();
    OK = 1;
  }
  else if (Line[0] == 'V')
  {
    Serial.printf("VER=%s\n", VERSION);
    OK = 1;
  }
  else if (Line[0] == 'C')
  {
    Serial.printf("PROD=%s\n", PRODUCT);
    OK = 1;
  }
  else if (Line[0] == 'D')
  {
    Serial.printf("DESC=%s\n", DESCRIPTION);
    OK = 1;
  }
  else if (Line[0] == 'A')
  {
    // Cutdown Altitude   
     settings.CutdownAltitude = atol(Line+1);
     OK = 1;
  }
  else if (Line[0] == 'T')
  {
    // Cutdown Time  
     settings.CutdownPeriod = atoi(Line+1);
     OK = 1;
  }

  return OK;
}

int ProcessLORACommand(char *Line)
{
  int OK = 0;

  if (Line[0] == 'F')
  {
    // New frequency
    double Frequency;
    
    Frequency = atof(Line+1);
    if ((Frequency >= 400) && (Frequency < 1000))
    {
      settings.LoRaFrequency = Frequency;
      setupRFM98(settings.LoRaFrequency, settings.LoRaMode);
      OK = 1;
    }
  }
  else if (Line[0] == 'M')
  {
    // Preset Mode
    int LoRaMode = atoi(Line+1);
    
    if ((LoRaMode >= 0) && (LoRaMode <= 5))
    {
      settings.LoRaMode = LoRaMode;
      setupRFM98(settings.LoRaFrequency, settings.LoRaMode);
      OK = 1;
    }
  }
  else if (Line[0] == 'C')
  {
    // Calling mode count
    int CallingCount = atoi(Line+1);
    
    if ((CallingCount >= 0) && (CallingCount <= 100))
    {
      settings.CallingCount = CallingCount;
      OK = 1;
    }
  }
  else if (Line[0] == 'T')
  {
    int LoRaCycleTime = atoi(Line+1);
    
    if ((LoRaCycleTime >= 0) && (LoRaCycleTime <= 60))
    {
      settings.LoRaCycleTime = LoRaCycleTime;
      OK = 1;
    }
  }
  else if (Line[0] == 'O')
  {
    int LoRaSlot = atoi(Line+1);
    
    if ((LoRaSlot >= -1) && (LoRaSlot < 60))
    {
      settings.LoRaSlot = LoRaSlot;
      OK = 1;
    }
  }
  else if (Line[0] == 'K')
  {
    settings.EnableUplink = atoi(Line+1);
    OK = 1;
  }
  else if (Line[0] == 'U')
  {
    strncpy(settings.UplinkCode, Line+1, sizeof(settings.UplinkCode));
    OK = 1;
    Serial.printf("Password = '%s'\n", settings.UplinkCode);
  }
  else if (Line[0] == '1')
  {
    settings.LoRaRepeatSlot1 = atoi(Line+1);
    OK = 1;
  }
  else if (Line[0] == '2')
  {
    settings.LoRaRepeatSlot2 = atoi(Line+1);
    OK = 1;
  }
  else if (Line[0] == 'Y')
  {
    settings.UseBinaryMode = atoi(Line+1);
    OK = 1;
  }
  else if (Line[0] == 'N')
  {
    settings.BinaryNode = atoi(Line+1);
    OK = 1;
  }

  return OK;
}

int ProcessPredictionCommand(char *Line)
{
  int OK = 0;

  if (Line[0] == 'C')
  {
    // CDA
    float CDA = atof(Line+1);
    
    if ((CDA > 0) && (CDA <= 10.0))
    {
      settings.CDA = CDA;
      OK = 1;
    }
  }
  else if (Line[0] == 'W')
  {
    // Payload Weight
    float PayloadWeight = atof(Line+1);
    
    if ((PayloadWeight > 0) && (PayloadWeight <= 10.0))
    {
      settings.PayloadWeight = PayloadWeight;
      OK = 1;
    }
  }
  else if (Line[0] == 'L')
  {
    // Landing Altitude
    int LandingAltitude = atol(Line+1);
    
    if ((LandingAltitude >= 0) && (LandingAltitude <= 20))
    {
      settings.LandingAltitude = LandingAltitude;
      OK = 1;
    }
  }

  return OK;
}

int ProcessGPSCommand(char *Line)
{
  int OK = 0;
  
  if (Line[0] == 'F')
  {
    // Flight mode altitude
    unsigned int Altitude;
    
    Altitude = atoi(Line+1);
    
    if (Altitude < 8000)
    {
      settings.FlightModeAltitude = Altitude;
      OK = 1;
    }
  }
  else if (Line[0] == 'M')
  {
    settings.GPSDynamicModel = atoi(Line+1);
    OK = 1;
  }

  return OK;
}

int ProcessRTTYCommand(char *Line)
{
  int OK = 0;

  if (Line[0] == 'F')
  {
    // New frequency
    double Frequency;
    
    Frequency = atof(Line+1);
    if ((Frequency >= 400) && (Frequency < 1000))
    {
      settings.RTTYFrequency = Frequency;
      OK = 1;
    }
  }
  else if (Line[0] == 'B')
  {
    int BaudRate = atoi(Line+1);
    
    if ((BaudRate >= 50) && (BaudRate <= 1200))
    {
      settings.RTTYBaudRate = BaudRate;
      OK = 1;
    }
  }
  else if (Line[0] == 'A')
  {
    int RTTYAudioShift = atoi(Line+1);
    
    if ((RTTYAudioShift >= 100) && (RTTYAudioShift <= 1200))
    {
      settings.RTTYAudioShift = RTTYAudioShift;
      OK = 1;
    }
  }
  else if (Line[0] == 'C')
  {
    int RTTYCount = atoi(Line+1);
    
    if ((RTTYCount >= 0) && (RTTYCount <= 5))
    {
      settings.RTTYCount = RTTYCount;
      OK = 1;
    }
  }
  else if (Line[0] == 'E')
  {
    int RTTYEvery = atoi(Line+1);
    
    if ((RTTYEvery > 0) && (RTTYEvery <= 100))
    {
      settings.RTTYEvery = RTTYEvery;
      OK = 1;
    }
  }
  else if (Line[0] == 'P')
  {
    int RTTYPreamble = atoi(Line+1);
    
    if ((RTTYPreamble >= 4) && (RTTYPreamble <= 100))
    {
      settings.RTTYPreamble = RTTYPreamble;
      OK = 1;
    }
  }

  return OK;
}

void SendSettings(void)
{
  Serial.printf("CP=%s\n", settings.PayloadID);
  Serial.printf("CF=%s\n", settings.FieldList);

  Serial.printf("CA=%ld\n", settings.CutdownAltitude);
  Serial.printf("CT=%u\n", settings.CutdownPeriod);

  Serial.printf("GF=%u\n", settings.FlightModeAltitude);
  Serial.printf("GM=%d\n", settings.GPSDynamicModel);

  Serial.printf("LF=%.4f\n", settings.LoRaFrequency);
  Serial.printf("LM=%u\n", settings.LoRaMode);

  Serial.printf("LT=%u\n", settings.LoRaCycleTime);
  Serial.printf("LO=%u\n", settings.LoRaSlot);
  Serial.printf("L1=%d\n", settings.LoRaRepeatSlot1);
  Serial.printf("L2=%d\n", settings.LoRaRepeatSlot2);
  
  Serial.printf("LB=%d\n", settings.UseBinaryMode);
  Serial.printf("LN=%d\n", settings.BinaryNode);

  Serial.printf("LC=%u\n", settings.CallingCount);

  Serial.printf("LE=%u\n", settings.EnableUplink);
  Serial.printf("LU=%s\n", settings.UplinkCode);

  Serial.printf("PC=%.1f\n", settings.CDA);
  Serial.printf("PW=%.2f\n", settings.PayloadWeight);
  Serial.printf("PL=%ld\n", settings.LandingAltitude);
  
  Serial.printf("RF=%.4f\n", settings.RTTYFrequency);
  Serial.printf("RB=%d\n", settings.RTTYBaudRate);
  Serial.printf("RA=%d\n", settings.RTTYAudioShift);
  Serial.printf("RC=%u\n", settings.RTTYCount);
  Serial.printf("RE=%u\n", settings.RTTYEvery);
  Serial.printf("RP=%u\n", settings.RTTYPreamble);
}

unsigned long NextLEDs=0;

// LED modes:
// ON at startup
// 4Hz till GPS lock
// 1Hz with GPS lock
// OFF above 1000m (for power save)

void ControlLED(axp_chgled_mode_t Mode)
{
  static axp_chgled_mode_t OldMode=AXP20X_LED_OFF;

  if (Mode != OldMode)
  {
    axp.setChgLEDMode(Mode);

    OldMode = Mode;
  }
}

void SetupLEDs(void)
{
  ControlLED(AXP20X_LED_LOW_LEVEL);
}

void CheckLEDs(void)
{
  if (millis() >= NextLEDs)
  {
    if (gps.Altitude > 1000)
    {
      ControlLED(AXP20X_LED_OFF);
    }
    else if (gps.Satellites >= 4)
    {
      ControlLED(AXP20X_LED_BLINK_1HZ);
    }
    else
    {
      ControlLED(AXP20X_LED_BLINK_4HZ);
    }      
    
    NextLEDs = millis() + 1000L;
  }
}