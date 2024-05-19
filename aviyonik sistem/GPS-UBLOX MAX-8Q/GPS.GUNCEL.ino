#include <SoftwareSerial.h>
#include <TinyGPS++.h>

SoftwareSerial gpsSerial(3, 4); // RX pin: 2, TX pin: 3
TinyGPSPlus gps;

void setup()
{
  Serial.begin(9600);
  gpsSerial.begin(9600);
}

void loop()
{
  while (gpsSerial.available() > 0)
  {
    if (gps.encode(gpsSerial.read()))
    {
      if (gps.location.isUpdated())
      {
        Serial.print("Uydu Sayisi: ");
        Serial.println(gps.satellites.value());
        Serial.print("Enlem: ");
        Serial.print(gps.location.lat(), 6);
        Serial.print(" Boylam: ");
        Serial.println(gps.location.lng(), 6);
      }
    }
  }
}