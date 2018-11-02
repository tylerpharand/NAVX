// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
// Heavy physical hookup comments by Phantom YoYo A.D. 2014


#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <String.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303.h>
#include <Adafruit_LSM303_U.h>

//---compass init
/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
int offset = 0;
float heading = 0;

//uncomment the string versions for fixed data + change interrupt code as needed...
//String current_lon = ""; 
//String current_lat = "";
float current_lon = 0;
float current_lat = 0;
//---

//---GPS software serial
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  true
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
//---

void setup()  
{    
  Serial.begin(115200);

  //---initialize Compass
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("No LSM303 detected ... Check wiring!");
    while(1);
  }
  //---

  //---initialize GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  
  // 1 Hz update rate
//  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // 5 Hz update rate- for 9600 baud you'll have to set the output to RMC or RMCGGA only (see above)
//  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  // 10 Hz update rate - for 9600 baud you'll have to set the output to RMC only (see above)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

  useInterrupt(true);
  //---
  
  delay(5000);
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
//  char c = GPS.read();
  char c = GPS.read();
  if (GPS.newNMEAreceived()){
    GPS.parse(GPS.lastNMEA());
//      Serial.print(GPS.longitudeDegrees, 6);
//      current_lon = "-79."+String(random(999999));
      current_lon = GPS.longitudeDegrees;
//      Serial.print(current_lon);
//      Serial.print(",");
//      Serial.println(GPS.latitudeDegrees, 6);
//      current_lat = "43."+String(random(999999));
      current_lat = GPS.latitudeDegrees;
//      Serial.print(current_lat);
//      Serial.print(",");
//      Serial.print(heading);
//      Serial.print("\n");
  //  char x = GPS.latitude;
  // if you want to debug, this is a good time to do it!
//  if (GPSECHO)
//    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time.
  }
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;

  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}


void loop()                     // run over and over again
{
    sensors_event_t event; 
    mag.getEvent(&event);
    
    float Pi = 3.14159;
    
    // Calculate the angle of the vector y,x
    heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi + offset;
    
    // Normalize to 0-360
    if (heading < 0)
    {
    heading = 360 + heading;
    }
    
      Serial.print(current_lon, 6);
//      Serial.print(current_lon);
      Serial.print(",");
      Serial.print(current_lat, 6);
//      Serial.print(current_lat);
      Serial.print(",");
      Serial.print(heading);

      //uncomment for accelerometer data. Needs to be converted to LINEAR acc to be useful.
      //Further, the y axis will only be relevant since we care about the car's forward acceleration.
//      Serial.print(",");
//      Serial.print(event.acceleration.x);
//      Serial.print(",");
//      Serial.print(event.acceleration.y);
//      Serial.print(",");
//      Serial.print(event.acceleration.z);
      Serial.print("\n");
    delay(94); //outputting at 10.6Hz so python won't have insufficient data...
}
