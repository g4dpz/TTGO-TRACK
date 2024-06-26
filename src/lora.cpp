/*----------------------------------------------------\
|                                                     |
| LoRa radio code, for downlink, uplink and repeating |
|                                                     |
| Messages can be timed using a GPS reference, to     |
| comply with the TDMA timing requirements.           |
|                                                     |                                                     |
\*---------------------------------------------------*/
#include <Arduino.h>
#include <SPI.h>
#include <string.h>
#include <common.h>

#define LORA_NSS           18                // Comment out to disable LoRa code
#define LORA_RESET         14                // Comment out if not connected
#define LORA_DIO0          26    
#define LORA_TIME_INDEX      2
#define LORA_OFFSET           0         // Frequency to add in kHz to make Tx frequency accurate            


// RFM98 registers
#define REG_FIFO                    0x00
#define REG_OPMODE                  0x01
#define REG_FIFO_ADDR_PTR           0x0D 
#define REG_FIFO_TX_BASE_AD         0x0E
#define REG_FIFO_RX_BASE_AD         0x0F
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_IRQ_FLAGS               0x12
#define REG_RX_NB_BYTES             0x13
#define REG_MODEM_CONFIG            0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_MODEM_CONFIG3           0x26
#define REG_PREAMBLE_MSB            0x20
#define REG_PREAMBLE_LSB            0x21
#define REG_PAYLOAD_LENGTH          0x22
#define REG_HOP_PERIOD              0x24
#define REG_FREQ_ERROR              0x28
#define REG_DETECT_OPT              0x31
#define	REG_DETECTION_THRESHOLD     0x37
#define REG_PACKET_SNR              0x19
#define REG_PACKET_RSSI             0x1A
#define REG_CURRENT_RSSI            0x1B
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41

// FSK stuff
#define REG_PREAMBLE_MSB_FSK        0x25
#define REG_PREAMBLE_LSB_FSK        0x26
#define REG_PACKET_CONFIG1          0x30
#define REG_PAYLOAD_LENGTH_FSK      0x32
#define REG_FIFO_THRESH             0x35
#define REG_FDEV_MSB                0x04
#define REG_FDEV_LSB                0x05
#define REG_FRF_MSB                 0x06
#define REG_FRF_MID                 0x07
#define REG_FRF_LSB                 0x08
#define REG_BITRATE_MSB             0x02
#define REG_BITRATE_LSB             0x03
#define REG_IRQ_FLAGS2              0x3F

// MODES
#define RF98_MODE_RX_CONTINUOUS     0x85
#define RF98_MODE_TX                0x83
#define RF98_MODE_SLEEP             0x80
#define RF98_MODE_STANDBY           0x81

// TODO Check this as it also appears in common.h to support a struct
// #define PAYLOAD_LENGTH              255

// Modem Config 1
#define EXPLICIT_MODE               0x00
#define IMPLICIT_MODE               0x01

#define ERROR_CODING_4_5            0x02
#define ERROR_CODING_4_6            0x04
#define ERROR_CODING_4_7            0x06
#define ERROR_CODING_4_8            0x08

#define BANDWIDTH_7K8               0x00
#define BANDWIDTH_10K4              0x10
#define BANDWIDTH_15K6              0x20
#define BANDWIDTH_20K8              0x30
#define BANDWIDTH_31K25             0x40
#define BANDWIDTH_41K7              0x50
#define BANDWIDTH_62K5              0x60
#define BANDWIDTH_125K              0x70
#define BANDWIDTH_250K              0x80
#define BANDWIDTH_500K              0x90

// Modem Config 2

#define SPREADING_6                 0x60
#define SPREADING_7                 0x70
#define SPREADING_8                 0x80
#define SPREADING_9                 0x90
#define SPREADING_10                0xA0
#define SPREADING_11                0xB0
#define SPREADING_12                0xC0

#define CRC_OFF                     0x00
#define CRC_ON                      0x04


// POWER AMPLIFIER CONFIG
#define REG_PA_CONFIG               0x09
#define PA_MAX_BOOST                0x8F    // 100mW (max 869.4 - 869.65)
#define PA_LOW_BOOST                0x81
#define PA_MED_BOOST                0x8A
#define PA_MAX_UK                   0x88    // 10mW (max 434)
#define PA_OFF_BOOST                0x00
#define RFO_MIN                     0x00

// 20DBm
#define REG_PA_DAC                  0x4D
#define PA_DAC_20                   0x87

// LOW NOISE AMPLIFIER
#define REG_LNA                     0x0C
#define LNA_MAX_GAIN                0x23  // 0010 0011
#define LNA_OFF_GAIN                0x00

typedef enum {lmIdle, lmListening, lmSending} tLoRaMode;

tLoRaMode LoRaMode;
uint8_t currentMode = 0x81;
int TargetID;
struct TBinaryPacket PacketToRepeat;
uint8_t SendRepeatedPacket, RepeatedPacketType=0;
int ImplicitOrExplicit;
int GroundCount;
int AirCount;
int BadCRCCount;
unsigned char Sentence[SENTENCE_LENGTH];
unsigned long LastLoRaTX=0;
unsigned long TimeToSendIfNoGPS=0;
int CallingCount=0;
int RTTYCount=0;
int InRTTYMode=0;
int SendingRTTY=0;
int RTTYIndex, RTTYMask, RTTYLength;
int FSKBitRate, FSKOverSample, RTTYBitLength;

// function prototypes
void setLoRaMode();
uint8_t readRegister(uint8_t addr);
void writeRegister(uint8_t addr, uint8_t value);
void setupRFM98(double Frequency, int Mode);
void setFrequency(double Frequency);
void select();
void unselect();
void setMode(uint8_t newMode);
int receiveMessage(unsigned char *message, int MaxLength);
void startReceiving();
void AddBytesToFSKBuffer(int MaxBytes);
char Hex(int Character);


int BuildSentence(TGPS gps, TSettings settings, char *TxLine)
{
  int Count, i, j;
  unsigned int CRC;
  char Temp[16];

  strcpy(TxLine, "$$");
  SentenceCounter++;
  
  for (i=0; settings.FieldList[i]; i++)
  {
    char Field;

    Field = settings.FieldList[i];
    *Temp = 0;
  
    if (Field == '0')
    {
      // PayloadID
      strcpy(Temp, settings.PayloadID);
    }
    else if (Field == '1')
    {
      // Counter
      sprintf(Temp, "%u", SentenceCounter);
    }
    else if (Field == '2')
    {
      // Time
      sprintf(Temp, "%02d:%02d:%02d", gps.Hours, gps.Minutes, gps.Seconds);
    }
    else if (Field == '3')
    {
      // Latitude
      dtostrf(gps.Latitude, 7, 5, Temp);
    }
    else if (Field == '4')
    {
      // Longitude
      dtostrf(gps.Longitude, 7, 5, Temp);
    }
    else if (Field == '5')
    {
      // Altitude
      sprintf(Temp, "%ld", gps.Altitude);
    }
    else if (Field == '6')
    {
      // Satellites
      sprintf(Temp, "%u", gps.Satellites);
    }
    else if (Field == '7')
    {
      // Speed
      sprintf(Temp, "%u", (int)((gps.Speed * 13) / 7));
    }
    else if (Field == '8')
    {
      // Direction
      sprintf(Temp, "%d", (int)(gps.Direction));
    }
    else if (Field == '9')
    {
      // ADC
      sprintf(Temp, "%d", gps.BatteryVoltage);
    }
    else if (Field == 'A')
    {
      sprintf(Temp, "%.1f", gps.InternalTemperature);
    }
    else if (Field == 'B')
    {
      sprintf(Temp, "%.1f", gps.ExternalTemperature);
    }
    else if (Field == 'C')
    {
      dtostrf(gps.PredictedLatitude, 7, 5, Temp);
    }
    else if (Field == 'D')
    {
      dtostrf(gps.PredictedLongitude, 7, 5, Temp);
    }
    else if (Field == 'E')
    {
      sprintf(Temp, "%d", gps.CutdownStatus);
    }
    else if (Field == 'F')
    {
      sprintf(Temp, "%d", gps.LastPacketSNR);
    }
    else if (Field == 'G')
    {
      sprintf(Temp, "%d", gps.LastPacketRSSI);
    }
    else if (Field == 'H')
    {
      sprintf(Temp, "%u", gps.ReceivedCommandCount);
    }
//    else if ((Field >= 'I') && (Field <= 'N'))
//    {
//      sprintf(Temp, "%u", gps.ExtraFields[Field-'I']);
//    }

    if (i > 0)
    {
        strcat(TxLine, ",");
    }
    strcat(TxLine, Temp);
  }

  strcat(TxLine, gps.ExtraFields);

  Count = strlen(TxLine);

  CRC = 0xffff;           // Seed
 
   for (i = 2; i < Count; i++)
   {   // For speed, repeat calculation instead of looping for each bit
      CRC ^= (((unsigned int)TxLine[i]) << 8);
      for (j=0; j<8; j++)
      {
          if (CRC & 0x8000)
              CRC = (CRC << 1) ^ 0x1021;
          else
              CRC <<= 1;
      }
   }

  TxLine[Count++] = '*';
  TxLine[Count++] = Hex((CRC >> 12) & 15);
  TxLine[Count++] = Hex((CRC >> 8) & 15);
  TxLine[Count++] = Hex((CRC >> 4) & 15);
  TxLine[Count++] = Hex(CRC & 15);
  TxLine[Count++] = '\n';  
  TxLine[Count++] = '\0';
  
  return strlen(TxLine) + 1;
}

int BuildLoRaCall(TSettings settings, unsigned char *TxLine)
{
	char Frequency[8];
	
	dtostrf(settings.LoRaFrequency, 7, 3, Frequency);

  sprintf((char *)TxLine, "^^%s,%s,%d,%d,%d,%d,%d",
    			        settings.PayloadID, Frequency,
        			    settings.LoRaMode == 1 ? 1 : 0, 
        			    settings.LoRaMode == 1 ? ERROR_CODING_4_5 : ERROR_CODING_4_8,
        			    settings.LoRaMode == 2 ? BANDWIDTH_62K5 : BANDWIDTH_20K8,
        			    settings.LoRaMode == 2 ? SPREADING_8 : (settings.LoRaMode == 1 ? SPREADING_6 : SPREADING_11),
        			    settings.LoRaMode == 0 ? 0x08 : 0);
			
	return strlen((char *)TxLine) + 1;
}

void SetupLoRa(TSettings settings)
{
  pinMode(LORA_NSS, OUTPUT);
  pinMode(LORA_DIO0, INPUT);

  // SPI.begin();
  SPI.begin(SCK,MISO,MOSI,LORA_NSS);

  setupRFM98(settings.LoRaFrequency, settings.LoRaMode);

  if (settings.RTTYBaudRate == 50)
  {
    FSKBitRate = 40000;
    FSKOverSample = 2;
    RTTYBitLength = 7;
  }
  else
  {
    // 300 baud
    FSKBitRate = 13333;
    FSKOverSample = 1;
    RTTYBitLength = 8;
  }
}

void setupRFM98(double Frequency, int Mode)
{
  int ErrorCoding;
  int Bandwidth;
  int SpreadingFactor;
  int LowDataRateOptimize;
  int PayloadLength;
   
  // LoRa mode 
  setLoRaMode();

  // Frequency
  setFrequency(Frequency + LORA_OFFSET / 1000.0);

  // LoRa settings for various modes.  We support modes 2 (repeater mode), 1 (normally used for SSDV) and 0 (normal slow telemetry mode).
  
  if (Mode == 5)
  {
    ImplicitOrExplicit = EXPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_8;
    Bandwidth = BANDWIDTH_41K7;
    SpreadingFactor = SPREADING_11;
    LowDataRateOptimize = 0;		
  }
  else if (Mode == 2)
  {
    ImplicitOrExplicit = EXPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_8;
    Bandwidth = BANDWIDTH_62K5;
    SpreadingFactor = SPREADING_8;
    LowDataRateOptimize = 0;		
  }
  else if (Mode == 1)
  {
    ImplicitOrExplicit = IMPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_5;
    Bandwidth = BANDWIDTH_20K8;
    SpreadingFactor = SPREADING_6;
    LowDataRateOptimize = 0;    
  }
  else // if (Mode == 0)
  {
    ImplicitOrExplicit = EXPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_8;
    Bandwidth = BANDWIDTH_20K8;
    SpreadingFactor = SPREADING_11;
    LowDataRateOptimize = 0x08;		
  }
  
  PayloadLength = ImplicitOrExplicit == IMPLICIT_MODE ? 255 : 0;

  writeRegister(REG_MODEM_CONFIG, ImplicitOrExplicit | ErrorCoding | Bandwidth);
  writeRegister(REG_MODEM_CONFIG2, SpreadingFactor | CRC_ON);
  writeRegister(REG_MODEM_CONFIG3, 0x04 | LowDataRateOptimize);									// 0x04: AGC sets LNA gain
  
  // writeRegister(REG_DETECT_OPT, (SpreadingFactor == SPREADING_6) ? 0x05 : 0x03);					// 0x05 For SF6; 0x03 otherwise
  writeRegister(REG_DETECT_OPT, (readRegister(REG_DETECT_OPT) & 0xF8) | ((SpreadingFactor == SPREADING_6) ? 0x05 : 0x03));  // 0x05 For SF6; 0x03 otherwise
  
  writeRegister(REG_DETECTION_THRESHOLD, (SpreadingFactor == SPREADING_6) ? 0x0C : 0x0A);		// 0x0C for SF6, 0x0A otherwise  
  
  writeRegister(REG_PAYLOAD_LENGTH, PayloadLength);
  writeRegister(REG_RX_NB_BYTES, PayloadLength);
  
  // Change the DIO mapping to 01 so we can listen for TxDone on the interrupt
  writeRegister(REG_DIO_MAPPING_1,0x40);
  writeRegister(REG_DIO_MAPPING_2,0x00);
  
  // Go to standby mode
  setMode(RF98_MODE_STANDBY);
  
  Serial.println("Setup Complete");
}

void setFrequency(double Frequency)
{
  unsigned long FrequencyValue;
    
  Serial.print("Frequency is ");
  Serial.println(Frequency);

  Frequency = Frequency * 7110656 / 434;
  FrequencyValue = (unsigned long)(Frequency);

  Serial.print("FrequencyValue is ");
  Serial.println(FrequencyValue);

  writeRegister(0x06, (FrequencyValue >> 16) & 0xFF);    // Set frequency
  writeRegister(0x07, (FrequencyValue >> 8) & 0xFF);
  writeRegister(0x08, FrequencyValue & 0xFF);
}

void setLoRaMode()
{
  Serial.println("Setting LoRa Mode");
  setMode(RF98_MODE_SLEEP);
  writeRegister(REG_OPMODE,0x80);
   
  Serial.println("LoRa Mode Set");
}

/////////////////////////////////////
//    Method:   Change the mode
//////////////////////////////////////
void setMode(uint8_t newMode)
{
  if(newMode == currentMode)
    return;  
  
  // Serial.printf("Set LoRa Mode %d\n", newMode);
  
  switch (newMode) 
  {
    case RF98_MODE_TX:
      writeRegister(REG_LNA, LNA_OFF_GAIN);  // TURN LNA OFF FOR TRANSMITT
      writeRegister(REG_PA_CONFIG, PA_MAX_UK);
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      
      break;
    case RF98_MODE_RX_CONTINUOUS:
      writeRegister(REG_PA_CONFIG, PA_OFF_BOOST);  // TURN PA OFF FOR RECIEVE??
      writeRegister(REG_LNA, LNA_MAX_GAIN);  // MAX GAIN FOR RECIEVE
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    case RF98_MODE_SLEEP:
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    case RF98_MODE_STANDBY:
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    default: return;
  } 
  
  if(newMode != RF98_MODE_SLEEP)
  {
    delay(10);
  }
}


/////////////////////////////////////
//    Method:   Read Register
//////////////////////////////////////

uint8_t readRegister(uint8_t addr)
{
  select();
  SPI.transfer(addr & 0x7F);
  uint8_t regval = SPI.transfer(0);
  unselect();

  return regval;
}

/////////////////////////////////////
//    Method:   Write Register
//////////////////////////////////////

void writeRegister(uint8_t addr, uint8_t value)
{
  select();
  SPI.transfer(addr | 0x80); // OR address with 10000000 to indicate write enable;
  SPI.transfer(value);
  unselect();
}

/////////////////////////////////////
//    Method:   Select Transceiver
//////////////////////////////////////
void select() 
{
  digitalWrite(LORA_NSS, LOW);
}

/////////////////////////////////////
//    Method:   UNSelect Transceiver
//////////////////////////////////////
void unselect() 
{
  digitalWrite(LORA_NSS, HIGH);
}

void DecryptMessage(char *Code, char *Message)
{
  int i, Len;
  
  Len = strlen(Code);
  
  if (Len > 0)
  {
    printf("Decoding ...\n");
    i = 0;
    while (*Message)
    {
      *Message = (*Message ^ Code[i]) & 0x7F;
      Message++;
      i = (i + 1) % Len;
    }
  }
}

char GetChar(char **Message)
{
  return *((*Message)++);
}

void GetString(char *Field, char **Message, char delimiter)
{
  while (**Message && (**Message != delimiter))
  {
    *Field++ = *((*Message)++);
  }
  
  *Field = 0;
  if (**Message)
  {
    (*Message)++;
  }
}

int32_t GetInteger(char **Message, char delimiter)
{
  char Temp[32];
  
  GetString(Temp, Message, delimiter);
  
  return atoi(Temp);
}

void CheckLoRaRx(TGPS gps, TSettings settings)
{
  if (LoRaMode == lmListening)
  {
    if (digitalRead(LORA_DIO0))
    {
      // unsigned char Message[32];
      int Bytes;
					
      Bytes = receiveMessage(Sentence, sizeof(Sentence));
      Serial.print("Rx "); Serial.print(Bytes); Serial.println(" bytes");
      Serial.println((char *)Sentence);
      Serial.printf("Password = '%s'\n", settings.UplinkCode);
      RepeatedPacketType = 0;
      
      // Bytes = min(Bytes, sizeof(Sentence));
					
      if (Bytes > 0)
      {
        // Get RSSI etc
        int8_t SNR;
        int RSSI;
        
        SNR = readRegister(REG_PACKET_SNR);
        SNR /= 4;
        RSSI = readRegister(REG_PACKET_RSSI) - 157;
        if (SNR < 0)
        {
          RSSI += SNR;
        }
        
        gps.LastPacketSNR = SNR;
        gps.LastPacketRSSI = RSSI;

        Serial.printf("Command byte = '%c'\n", Sentence[0]);

        if (Sentence[0] == '$')
        {
          // ASCII telemetry
          Serial.println("Rx ASCII");
          if (memcmp(Sentence+2, settings.PayloadID, strlen(settings.PayloadID)) != 0)
          {
            RepeatedPacketType = 3;
          }

          // Get timing from this message
          if ((LORA_TIME_INDEX > 0) && (LORA_TIME_MUTLIPLER > 0))
          {
            unsigned char Slot;
            long Offset;

            Slot = (Sentence[LORA_TIME_INDEX+2] - '0') * LORA_TIME_MUTLIPLER + LORA_TIME_OFFSET;
            Offset = (settings.LoRaSlot - Slot) * 1000L - LORA_PACKET_TIME;
            if (Offset < 0) Offset += settings.LoRaCycleTime * 1000L;

            Serial.print("Rx Slot = "); Serial.println(Slot);
            Serial.print(" Offset = "); Serial.println(Offset);

            TimeToSendIfNoGPS = millis() + Offset;
          }
        }
        else if (Sentence[0] == '*')
        {
          char Command, Parameter, PayloadID[32], *Message;
          
          Message = (char *)(Sentence + 1);

          Serial.println("Rx command");
          
          DecryptMessage(settings.UplinkCode, Message);
                   
          Serial.printf("Uplink: %s\n", Sentence);

          GetString(PayloadID, &Message, '/');
          
          if (strcmp(PayloadID, settings.PayloadID) == 0)
          {
            gps.ReceivedCommandCount++;

            strncpy(gps.LastReceivedCommand, Message, sizeof(gps.LastReceivedCommand));
            
            printf("Uplink message for us = '%s'\n", Message);
            
            Command = GetChar(&Message);
            
            #ifdef CUTDOWN
            if (Command == 'C')
            {
              // Cutdown
              Parameter = GetChar(&Message);
            
              if (Parameter == 'N')
              {
                int CutdownPeriod;
                
                // Cutdown Now
                CutdownPeriod = GetInteger(&Message, ',');
                      
                if (CutdownPeriod <= 0)
                {
                  CutdownPeriod = settings.CutdownPeriod;
                }
                
                Serial.printf("** MANUAL CUTDOWN FOR %d SECONDS **\n", CutdownPeriod);
                
                CutdownNow(CutdownPeriod * 1000);
                
                gps.CutdownStatus = 3;      // Manually triggered
              }
              else if (Parameter == 'A')
              {
                // Cutdown at specified altitude
                Serial.printf("Set cutdown altitude %sm\n", Message);
                gps.CutdownAltitude = GetInteger(&Message, ',');
              }
            } else 
            #endif
            if (Command == 'R')
            {
              int Pin, Period;    
              // "Run script" --> fill in string to be sent to i2c master
              
              strncpy(gps.UplinkText, Message, 32);
              
              Serial.print("** SET I2C STRING TO '");
              Serial.print(gps.UplinkText);
              Serial.println("'");
            }
          }
        }
      }
    }
  }
}

int TimeToSend(TGPS gps, TSettings settings)
{
  int CycleSeconds;
	
  SendRepeatedPacket = 0;

  if (settings.LoRaCycleTime <= 0)
  {
    // Not using time to decide when we can send
    return 1;
  }

  if ((millis() > (LastLoRaTX + settings.LoRaCycleTime*1000+2000)) && (TimeToSendIfNoGPS == 0))
  {
    // Timed out
    Serial.println("Using Timeout");
    return 1;
  }
  
  if (gps.Satellites > 0)
  {
    static int LastCycleSeconds=-1;

    // Can't Tx twice at the same time
    CycleSeconds = (gps.SecondsInDay+settings.LoRaCycleTime) % settings.LoRaCycleTime;   // Could just use GPS time, but it's nice to see the slot agree with UTC
    
    if (CycleSeconds != LastCycleSeconds)
    {
      LastCycleSeconds = CycleSeconds;
      
      if (CycleSeconds == settings.LoRaSlot)
      {
        // Serial.println("Using GPS Timing");
        SendRepeatedPacket = 0;
        return 1;
      }

      if (RepeatedPacketType && ((CycleSeconds == settings.LoRaRepeatSlot1) || (CycleSeconds == settings.LoRaRepeatSlot2)))
      {
        Serial.println("Time to repeat");
        SendRepeatedPacket = RepeatedPacketType;
        RepeatedPacketType = 0;
        return 1;
      }
    }
  }
  else if ((TimeToSendIfNoGPS > 0) && (millis() >= TimeToSendIfNoGPS))
  {
    Serial.println("Using LoRa Timing");
    SendRepeatedPacket = 0;
    return 1;
  }
    
  return 0;
}


int LoRaIsFree(TGPS gps, TSettings settings)
{
  if ((LoRaMode != lmSending) || digitalRead(LORA_DIO0))
  {
    // Either not sending, or was but now it's sent.  Clear the flag if we need to
    if (LoRaMode == lmSending)
    {
      // Clear that IRQ flag
      writeRegister( REG_IRQ_FLAGS, 0x08); 
      LoRaMode = lmIdle;
    }
				
    // Now we test to see if we're doing TDM or not
    // For TDM, if it's not a slot that we send in, then we should be in listening mode
    // Otherwise, we just send
				
    if (TimeToSend(gps, settings))
    {
      // Either sending continuously, or it's our slot to send in
      // printf("Channel %d is free\n", Channel);
					
      return 1;
    }
    
    if (settings.LoRaCycleTime > 0)
    {
      // TDM system and not time to send, so we can listen
#ifdef LORA_DONT_LISTEN
      lora_sleep();
#else
      if (LoRaMode == lmIdle)
      {
        startReceiving();
      }
#endif      
    }
  }
  
  return 0;
}

void SendLoRaPacket(TSettings settings,  unsigned char *buffer, int Length)
{
  int i;
  
  LastLoRaTX = millis();
  TimeToSendIfNoGPS = 0;

  if (InRTTYMode != 0)
  {
    setupRFM98(settings.LoRaFrequency, settings.LoRaMode);
    InRTTYMode = 0;
  }
  
  // Serial.print("Sending "); Serial.print(Length);Serial.println(" uint8_ts");
  
  setMode(RF98_MODE_STANDBY);

  writeRegister(REG_DIO_MAPPING_1, 0x40);		// 01 00 00 00 maps DIO0 to TxDone
  writeRegister(REG_FIFO_TX_BASE_AD, 0x00);  // Update the address ptr to the current tx base address
  writeRegister(REG_FIFO_ADDR_PTR, 0x00); 
  if (ImplicitOrExplicit == EXPLICIT_MODE)
  {
    writeRegister(REG_PAYLOAD_LENGTH, Length);
  }
  select();
  // tell SPI which address you want to write to
  SPI.transfer(REG_FIFO | 0x80);

  // loop over the payload and put it on the buffer 
  for (i = 0; i < Length; i++)
  {
    SPI.transfer(buffer[i]);
  }
  unselect();

  // go into transmit mode
  setMode(RF98_MODE_TX);
  
  LoRaMode = lmSending;
  SendingRTTY = 0;
}

void startReceiving(void)
{
  writeRegister(REG_DIO_MAPPING_1, 0x00);		// 00 00 00 00 maps DIO0 to RxDone
	
  writeRegister(REG_FIFO_RX_BASE_AD, 0);
  writeRegister(REG_FIFO_ADDR_PTR, 0);
	  
  // Setup Receive Continuous Mode
  setMode(RF98_MODE_RX_CONTINUOUS); 
		
  LoRaMode = lmListening;
}

void lora_sleep(void)
{
  // writeRegister(REG_DIO_MAPPING_1, 0x00);    // 00 00 00 00 maps DIO0 to RxDone
  
  // writeRegister(REG_FIFO_RX_BASE_AD, 0);
  // writeRegister(REG_FIFO_ADDR_PTR, 0);
    
  // Setup Receive Continuous Mode
  setMode(RF98_MODE_SLEEP); 
    
  LoRaMode = lmIdle;
}

int receiveMessage(unsigned char *message, int MaxLength)
{
  int i, Bytes, currentAddr, x;

  Bytes = 0;
	
  x = readRegister(REG_IRQ_FLAGS);
  
  // clear the rxDone flag
  writeRegister(REG_IRQ_FLAGS, 0x40); 
    
  // check for payload crc issues (0x20 is the bit we are looking for
  if((x & 0x20) == 0x20)
  {
    // CRC Error
    writeRegister(REG_IRQ_FLAGS, 0x20);		// reset the crc flags
    BadCRCCount++;
  }
  else
  {
    currentAddr = readRegister(REG_FIFO_RX_CURRENT_ADDR);
    Bytes = readRegister(REG_RX_NB_BYTES);
    Bytes = min(Bytes, MaxLength-1);

    writeRegister(REG_FIFO_ADDR_PTR, currentAddr);   
		
    for(i = 0; i < Bytes; i++)
    {
      message[i] = (unsigned char)readRegister(REG_FIFO);
    }
    message[Bytes] = '\0';

    // Clear all flags
    writeRegister(REG_IRQ_FLAGS, 0xFF); 
  }
  
  return Bytes;
}


int BuildLoRaPositionPacket(TGPS gps, TSettings settings,  unsigned char *TxLine)
{
  struct TBinaryPacket BinaryPacket;

  SentenceCounter++;

  BinaryPacket.PayloadIDs = 0xC0 | (settings.BinaryNode << 3) | settings.BinaryNode;
  BinaryPacket.Counter = SentenceCounter;
  BinaryPacket.BiSeconds = gps.SecondsInDay / 2L;
  BinaryPacket.Latitude = gps.Latitude;
  BinaryPacket.Longitude = gps.Longitude;
  BinaryPacket.Altitude = gps.Altitude;

  memcpy(TxLine, &BinaryPacket, sizeof(BinaryPacket));
	
  return sizeof(struct TBinaryPacket);
}

int FSKPacketSent(void)
{
  return ((readRegister(REG_IRQ_FLAGS2) & 0x48) != 0);
}

int FSKBufferLow(void)
{
  return (readRegister(REG_IRQ_FLAGS2) & 0x20) == 0;
}


void CheckFSKBuffer(void)
{
  if ((LoRaMode == lmSending) && SendingRTTY)
  {
    // Check if packet sent
    if (FSKPacketSent())
    {
      LoRaMode = lmIdle;
      SendingRTTY = 0;
    }
    else if (FSKBufferLow())
    {
      AddBytesToFSKBuffer(64 - 20);
    }
  }
}

void AddBytesToFSKBuffer(int MaxBytes)
{
  unsigned char data[65], temp;
  int i;
  uint8_t BytesWritten = 0;

  if (RTTYIndex < RTTYLength)
  {
    data[BytesWritten++] = REG_FIFO | 0x80;
    
    while((BytesWritten <= (MaxBytes-FSKOverSample+1)) &&
        (RTTYIndex < RTTYLength))
    {
      if (RTTYMask < 0)  //start bit
      {
        temp = 0xFF;
        RTTYMask++;
      }
      else if (RTTYMask == 0)  //start bit
      {
        temp = 0;
        RTTYMask = 1;
      }
      else if (RTTYMask >= (1<<RTTYBitLength))  //stop bits
      {
        RTTYMask <<= 1;
        temp = 0xFF;
        if (RTTYMask >= (1<<(RTTYBitLength+2)))
        {
          RTTYMask = 0;
          RTTYIndex++;
        }
      }
      else  //databits
      {
        if (Sentence[RTTYIndex] & RTTYMask)
        {
          temp = 0xFF;
        }
        else
        {
          temp = 0x0;
        }
        RTTYMask <<= 1;
      }

      for (i = 0; i < FSKOverSample; i++)
      {
        data[BytesWritten++] = temp;
      }
    }


    select();

    for (i = 0; i < BytesWritten; i++)
    {
      SPI.transfer(data[i]);
    }
    unselect();
  }
}

void SwitchToFSKMode(TSettings settings)
{
  unsigned long FrequencyValue;
  
  uint8_t mode = readRegister(REG_OPMODE);
  mode |= (1<<3);                           //set LF range

  if (mode & (1<<7))  //if in lora mode
  {
    writeRegister(REG_OPMODE,(mode & ~(uint8_t)7));    //set to sleep mode so fsk bit can be written
  }
  else
  {
    writeRegister(REG_OPMODE,(mode & ~(uint8_t)7) | 1);  //set to standby mode so various settings can be written  
  }

  mode = readRegister(REG_OPMODE);
  writeRegister(REG_OPMODE, mode & ~(uint8_t)(7<<5));         //set to FSK

  writeRegister(REG_LNA, LNA_OFF_GAIN);  // TURN LNA OFF FOR TRANSMIT
  writeRegister(REG_PA_CONFIG, PA_MAX_UK);
    
  // Frequency
  FrequencyValue = (unsigned long)((settings.RTTYFrequency + (LORA_OFFSET / 1000.0)) * 7110656 / 434);
  writeRegister(REG_FRF_MSB, (FrequencyValue >> 16) & 0xFF);   // Set frequency
  writeRegister(REG_FRF_MID, (FrequencyValue >> 8) & 0xFF);
  writeRegister(REG_FRF_LSB, FrequencyValue & 0xFF);
  
  //write modem config
  writeRegister(REG_BITRATE_LSB, FSKBitRate & 0xFF);
  writeRegister(REG_BITRATE_MSB, (FSKBitRate >> 8) & 0xFF);
  writeRegister(REG_FDEV_LSB, (settings.RTTYAudioShift / 122) & 0xFF);
  writeRegister(REG_FDEV_MSB, 0);
  writeRegister(REG_PREAMBLE_LSB_FSK, 0);    // Preamble
  writeRegister(REG_PREAMBLE_MSB_FSK, 0);
  
  // writeRegister(REG_PACKET_CONFIG1, 0x80);    // variable length, no DC fix, no CRC, no addressing
  writeRegister(REG_PACKET_CONFIG1, 0x00);   // fixed length, no DC fix, no CRC, no addressing
  writeRegister(REG_PAYLOAD_LENGTH_FSK, 0);
}

void SendLoRaRTTY(TSettings settings, int Length)
{
  if (InRTTYMode != 1)
  {
    // Set RTTY mode
    SwitchToFSKMode(settings);
    InRTTYMode = 1;
  }

  // Fill in RTTY buffer
  // memcpy(RTTYBuffer, buffer, Length);
  RTTYLength = Length;
  RTTYIndex = 0;
  RTTYMask = -settings.RTTYPreamble;
  
  // Set FIFO threshold
  uint8_t r = readRegister(REG_FIFO_THRESH); 
  writeRegister(REG_FIFO_THRESH,(r&(~0x3F)) | 20);     // 20 = FIFO threshold
  
  writeRegister(REG_OPMODE, 0x0B);   // Tx mode

  // Populate FIFO
  AddBytesToFSKBuffer(64);
  
  // Set channel state
  LoRaMode = lmSending;
  SendingRTTY = 1;
}

void CheckLoRa(TGPS gps, TSettings settings)
{
  CheckFSKBuffer();

  if (settings.EnableUplink)
  {
    CheckLoRaRx(gps, settings);
  }
		
  if (LoRaIsFree(gps, settings))
  {		
    if (SendRepeatedPacket == 3)
    {
      // Repeat ASCII sentence
      Sentence[0] = '%';
      SendLoRaPacket(settings, Sentence, strlen((char *)Sentence)+1);
				
      RepeatedPacketType = 0;
      SendRepeatedPacket = 0;
    }
    else if (SendRepeatedPacket == 2)
    {
      Serial.println(F("Repeating uplink packet"));
				
        // 0x80 | (LORA_ID << 3) | TargetID
      SendLoRaPacket(settings, (unsigned char *)&PacketToRepeat, sizeof(PacketToRepeat));
				
      RepeatedPacketType = 0;
      SendRepeatedPacket = 0;
    }
    else if (SendRepeatedPacket == 1)
    {
      Serial.println(F("Repeating balloon packet"));
				
        // 0x80 | (LORA_ID << 3) | TargetID
      SendLoRaPacket(settings, (unsigned char *)&PacketToRepeat, sizeof(PacketToRepeat));
				
      RepeatedPacketType = 0;
      SendRepeatedPacket = 0;
    }
    else			
    {
      int PacketLength;

      if (++RTTYCount >= (settings.RTTYCount + settings.RTTYEvery))
      {
        RTTYCount = 0;
      }
            
      if (RTTYCount < settings.RTTYCount)
      {
        // Send RTTY packet
        PacketLength = BuildSentence(gps, settings, (char *)Sentence);
        Serial.printf("RTTY=%s", Sentence);
        SendLoRaRTTY(settings, PacketLength);    
        #ifdef OLED
          ShowTxStatus("RTTY", SentenceCounter);
        #endif
      }
      else
      {
        if ((settings.CallingCount > 0) && (++CallingCount > settings.CallingCount))
  	    {
  		    CallingCount = 0;
  		    setupRFM98(LORA_CALL_FREQ, LORA_CALL_MODE);
          PacketLength = BuildLoRaCall(settings, Sentence);
  		    Serial.println(F("LoRa: Calling Mode"));
          #ifdef OLED
            ShowTxStatus("CALL MODE", 0);
          #endif
  	    }
        else
  	    {
  		    if ((settings.CallingCount > 0) && (CallingCount == 1))
  		    {
  			    setupRFM98(settings.LoRaFrequency, settings.LoRaMode);
  		    }
  		
  	      if (settings.UseBinaryMode)
          {
            // 0x80 | (LORA_ID << 3) | TargetID
            PacketLength = BuildLoRaPositionPacket(gps, settings, Sentence);
  		      Serial.println(F("LoRa: Tx Binary packet"));
            #ifdef OLED
              ShowTxStatus("Binary", SentenceCounter);
            #endif
          }
          else
          {
            PacketLength = BuildSentence(gps, settings, (char *)Sentence);
            Serial.printf("LORA=%s", Sentence);
            // Serial.print((char *)Sentence);
            #ifdef OLED
              ShowTxStatus("LoRa", SentenceCounter);  
            #endif
  		    }
        }
  							
        SendLoRaPacket(settings, Sentence, PacketLength);		
      }
    }
  }
}