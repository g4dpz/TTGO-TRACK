#include <Arduino.h>
#include <axp20x.h>

#ifndef _COMMON_H
#define _COMMON_H

static volatile unsigned int SentenceCounter=0;

//------------------------------------------------------------------------------------------------------
//
//  Globals
#define SENTENCE_LENGTH       200        // Extend if needed
#define PAYLOAD_LENGTH         16
#define FIELDLIST_LENGTH       24
#define COMMAND_BUFFER_LENGTH  80

#define LORA_TIME_MUTLIPLER  2
#define LORA_TIME_OFFSET     1
#define LORA_PACKET_TIME    500

#define LORA_CALL_FREQ     433.650  // TODO change frequency to 868Mhz?
#define LORA_CALL_MODE     5  

// Options
// #define SERVO_PIN                15      // Cutdown via a servo motor
// #define CUTDOWN_PIN             25       // This pin made active to cut down from balloon
#define ENABLE_UPLINK                     // Enables uplink, for which you need to set TDM mode for transmission/reception
#define HAS_DFROBOT_TEN_DOF_IMU           // Has the DFRobot 10 DOF IMU

#if defined(HAS_DFROBOT_TEN_DOF_IMU)
  #define IMU_UPDATE_RATE  100
  #define IMU_UPDATE_RATE_MS  (1000/IMU_UPDATE_RATE)
  #define HAS_BMP280  // BMP280 is part of the 10 DOF IMU
  // #define HAS_Adxl345 // Adxl345 is part of the 10 DOF IMU
  // #define HAS_ITG3200 // Itg3200 is part of the 10 DOF IMU
  // #define HAS_HMC5883L // Hmc5883 is part of the 10 DOF IMU

  struct TEN_DOF
  {
    float Temperature;
    float Pressure;
    float Altitude;
    float AccelerationX;
    float AccelerationY;
    float AccelerationZ;
    float GyroX;
    float GyroY;
    float GyroZ;
    float MagneticX;
    float MagneticY;
    float MagneticZ;
  };

#endif

#if defined(SERVO_PIN) || defined(CUTDOWN_PIN)
  #define CUTDOWN
#endif

static AXP20X_Class axp;

struct TSettings
{
  // Common
  char PayloadID[PAYLOAD_LENGTH];
  char FieldList[FIELDLIST_LENGTH];

  // GPS
  unsigned int FlightModeAltitude;
  int GPSDynamicModel;
  
  // LoRa
  float LoRaFrequency;
//  unsigned char Implicit;             // 1=Implicit, 0=Explicit
//  unsigned char ErrorCoding;
//  unsigned char Bandwidth;
//  unsigned char SpreadingFactor;
//  unsigned char LowDataRateOptimize;
  int           LoRaMode;
  unsigned int  LoRaCycleTime;
  int           LoRaSlot;
  unsigned int  CallingCount;
  int           LoRaRepeatSlot1;
  int           LoRaRepeatSlot2;
  char          UseBinaryMode;
  char          BinaryNode;

  // Cutdown
  long          CutdownAltitude;
  unsigned int  CutdownPeriod;

  // Uplink
  int           EnableUplink;
  char          UplinkCode[16];

  // Prediction
  float         CDA;
  float         PayloadWeight;
  long          LandingAltitude;

  // RTTY
  float         RTTYFrequency;
  int           RTTYBaudRate;
  int           RTTYAudioShift;
  unsigned int  RTTYCount;
  unsigned int  RTTYEvery;
  unsigned int  RTTYPreamble;
};

#define EEPROM_SIZE sizeof(settings)

struct TBinaryPacket
{
	uint8_t 	PayloadIDs;
	uint16_t	Counter;
	uint16_t	BiSeconds;
	float		Latitude;
	float		Longitude;
	int32_t  	Altitude;
};  //  __attribute__ ((packed));

typedef enum {fmIdle, fmLaunched, fmDescending, fmLanding, fmLanded} TFlightMode;

struct TGPS
{
  int Hours, Minutes, Seconds;
  unsigned long SecondsInDay;					// Time in seconds since midnight
  float Longitude, Latitude;
  long Altitude, MinimumAltitude, MaximumAltitude, PreviousAltitude;
  unsigned int Satellites;
  float Speed, Direction;
  byte FixType;
  byte psm_status;
  float InternalTemperature;
  unsigned int BatteryVoltage;
  float ExternalTemperature;
  float Pressure;
  float AscentRate;
  unsigned int BoardCurrent;
  unsigned int errorstatus;
  byte GPSFlightMode;
  TFlightMode FlightMode;
  byte PowerMode;
  int CutdownStatus;
  float PredictedLatitude;
  float PredictedLongitude;
  float CDA;
  int UseHostPosition;
  int TimeTillLanding;
  float PredictedLandingSpeed;
  int LastPacketRSSI;
  int LastPacketSNR;
  int ReceivedCommandCount;
  char LastReceivedCommand[16];
  long CutdownAltitude;
  unsigned int CutdownPeriod;
  int Heading, Pitch, Roll;
  char ExtraFields[128];
  char UplinkText[33];
};

#endif // _COMMON_H