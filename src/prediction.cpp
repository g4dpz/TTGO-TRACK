#include <Arduino.h>
#include <common.h>

#define DEG2RAD (3.142 / 180)
#define SLOTSIZE 100
#define SLOTS 45000 / SLOTSIZE
#define POLL_PERIOD 5

struct TPosition
{
	float LatitudeDelta;
	float LongitudeDelta;
};

struct TPosition Positions[SLOTS];		// 100m slots from 0 to 45km

float PreviousLatitude, PreviousLongitude, cd_area;
unsigned long PreviousAltitude;

int GetSlot(int32_t Altitude)
{
  int Slot;
  
  Slot = Altitude / SLOTSIZE;
  
  if (Slot < 0) Slot = 0;
  if (Slot >= SLOTS) Slot = SLOTS-1;
  
  return Slot;
}

float CalculateAirDensity(float Altitude)
{
	float Temperature, Pressure;

  if (Altitude < 11000.0)
  {
      // below 11Km - Troposphere
      Temperature = 15.04 - (0.00649 * Altitude);
      Pressure = 101.29 * pow((Temperature + 273.1) / 288.08, 5.256);
  }
  else if (Altitude < 25000.0)
  {
    // between 11Km and 25Km - lower Stratosphere
    Temperature = -56.46;
    Pressure = 22.65 * exp(1.73 - ( 0.000157 * Altitude));
  }
  else
  {
    // above 25Km - upper Stratosphere
    Temperature = -131.21 + (0.00299 * Altitude);
    Pressure = 2.488 * pow((Temperature + 273.1) / 216.6, -11.388);
  }
  
  return Pressure / (0.2869 * (Temperature + 273.1));
}

float CalculateDescentRate(float Weight, float CDTimesArea, float Altitude)
{
  float Density;
  
  Density = CalculateAirDensity(Altitude);
  
  return sqrt((Weight * 9.81)/(0.5 * Density * CDTimesArea));
}

float CalculateCDA(float Weight, float Altitude, float DescentRate)
{
  float Density;
  
  Density = CalculateAirDensity(Altitude);
  
  Serial.printf("Alt %f, Rate %f, CDA %f\n", Altitude, DescentRate, (Weight * 9.81)/(0.5 * Density * DescentRate * DescentRate));
  
  return (Weight * 9.81)/(0.5 * Density * DescentRate * DescentRate);
}

void SetupPrediction(TGPS gps, TSettings settings)
{
  PreviousLatitude = 0;
  PreviousLongitude = 0;
  PreviousAltitude = 0;
  
  gps.CDA = settings.CDA;
}

int CalculateLandingPosition(TSettings settings, float Latitude, float Longitude, int32_t Altitude, float CDA, float *PredictedLatitude, float *PredictedLongitude)
{
  float TimeTillLanding, TimeInSlot, DescentRate;
  int Slot;
  int32_t DistanceInSlot;
  
  TimeTillLanding = 0;
  
  Slot = GetSlot(Altitude);
  DistanceInSlot = Altitude + 1 - (Slot * SLOTSIZE);
  
  while (Altitude > settings.LandingAltitude)
  {
    Slot = GetSlot(Altitude);
    
    if (Slot == GetSlot(settings.LandingAltitude))
    {
      DistanceInSlot = Altitude - settings.LandingAltitude;
    }
    
    DescentRate = CalculateDescentRate(settings.PayloadWeight, CDA, Altitude);
    TimeInSlot = DistanceInSlot / DescentRate;
    
    Latitude += Positions[Slot].LatitudeDelta * TimeInSlot;
    Longitude += Positions[Slot].LongitudeDelta * TimeInSlot;
    
    // printf("SLOT %d: alt %lu, lat=%f, long=%f, rate=%f, dist=%lu, time=%f\n", Slot, Altitude, Latitude, Longitude, DescentRate, DistanceInSlot, TimeInSlot);
    
    TimeTillLanding = TimeTillLanding + TimeInSlot;
    Altitude -= DistanceInSlot;
    DistanceInSlot = SLOTSIZE;
  }
        
  *PredictedLatitude = Latitude;
  *PredictedLongitude = Longitude;
  
  return TimeTillLanding; 
}

void CheckPrediction(TGPS gps, TSettings settings)
{
  static unsigned long NextCheck=0;
  
  if ((millis() >= NextCheck) && (gps.Satellites >= 4) && (gps.Latitude >= -90) && (gps.Latitude <= 90) && (gps.Longitude >= -180) && (gps.Longitude <= 180))
  {
    int Slot;
    
    NextCheck = millis() + POLL_PERIOD * 1000;

    // Serial.printf("FLIGHT MODE = %d ***\n", gps.FlightMode);
    
    if ((gps.FlightMode >= fmLaunched) && (gps.FlightMode < fmLanded))
    {
      // Ascent or descent?
      if (gps.FlightMode == fmLaunched)
      {
        // Going up - store deltas
        Slot = GetSlot(gps.Altitude/2 + PreviousAltitude/2);
        
        // Deltas are scaled to be horizontal distance per second (i.e. speed)
        Positions[Slot].LatitudeDelta = (gps.Latitude - PreviousLatitude) / POLL_PERIOD;
        Positions[Slot].LongitudeDelta = (gps.Longitude - PreviousLongitude) / POLL_PERIOD;
        Serial.printf("Slot %d (%" PRId32 "): %f, %f\n", Slot, gps.Altitude, Positions[Slot].LatitudeDelta, Positions[Slot].LongitudeDelta);
      }
      else if ((gps.FlightMode >= fmDescending) && (gps.FlightMode <= fmLanding))
      {
        // Coming down - try and calculate how well chute is doing
        
        gps.CDA = (gps.CDA*4 + CalculateCDA(settings.PayloadWeight, gps.Altitude/2 + PreviousAltitude/2, (PreviousAltitude - gps.Altitude) / POLL_PERIOD)) / 5;
      }

      // Estimate landing position
      gps.TimeTillLanding = CalculateLandingPosition(settings, gps.Latitude, gps.Longitude, gps.Altitude, gps.CDA, &(gps.PredictedLatitude), &(gps.PredictedLongitude));
      
      gps.PredictedLandingSpeed = CalculateDescentRate(settings.PayloadWeight, gps.CDA, settings.LandingAltitude);
      
      Serial.printf("Expected Descent Rate = %4.1f (now) %3.1f (landing), time till landing %d\n", 
                    CalculateDescentRate(settings.PayloadWeight, gps.CDA, gps.Altitude),
                    gps.PredictedLandingSpeed, gps.TimeTillLanding);
      
      Serial.printf("Current    %f, %f, alt %" PRId32 "\n", gps.Latitude, gps.Longitude, gps.Altitude);
      Serial.printf("Prediction %f, %f, CDA %f\n", gps.PredictedLatitude, gps.PredictedLongitude, gps.CDA);
    }
      
    PreviousLatitude = gps.Latitude;
    PreviousLongitude = gps.Longitude;
    PreviousAltitude = gps.Altitude;
  }
}  