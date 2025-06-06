#include <TinyGPS++.h>
/*
    This sample sketch demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
    It assumes that you have a 9600-baud serial GPS device connected to the hardware serial port (e.g., Serial1).
*/
static const int GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

void setup()
{
  Serial.begin(9600); // For debugging
  Serial1.begin(GPSBaud); // Hardware serial for GPS

  Serial.println(F("DeviceExample.ino"));
  Serial.println(F("A simple demonstration of TinyGPS++ with an attached GPS module"));
  Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (Serial1.available() > 0)
     if (gps.encode(Serial1.read()))
        displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
     Serial.println(F("No GPS detected: check wiring."));
     while (true);
  }
}

void displayInfo()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
     Serial.print(gps.location.lat(), 6);
     Serial.print(F(","));
     Serial.print(gps.location.lng(), 6);
  }
  else
  {
     Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
     Serial.print(gps.date.month());
     Serial.print(F("/"));
     Serial.print(gps.date.day());
     Serial.print(F("/"));
     Serial.print(gps.date.year());
  }
  else
  {
     Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
     if (gps.time.hour() < 10) Serial.print(F("0"));
     Serial.print(gps.time.hour());
     Serial.print(F(":"));
     if (gps.time.minute() < 10) Serial.print(F("0"));
     Serial.print(gps.time.minute());
     Serial.print(F(":"));
     if (gps.time.second() < 10) Serial.print(F("0"));
     Serial.print(gps.time.second());
     Serial.print(F("."));
     if (gps.time.centisecond() < 10) Serial.print(F("0"));
     Serial.print(gps.time.centisecond());
  }
  else
  {
     Serial.print(F("INVALID"));
  }

  Serial.println();
}
