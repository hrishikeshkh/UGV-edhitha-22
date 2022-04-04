#include <TinyGPS++.h>
#include <SoftwareSerial.h>
int RXPin = 4;
int TXPin = 3;
int GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);
void setup()
{
  Serial.begin(9600);
  gpsSerial.begin(GPSBaud);
}

void loop(){
  while (gpsSerial.available() > 0){
    if (gps.encode(gpsSerial.read())){
      if (gps.location.isValid()){
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    }
  double dest_lat=13.031297;
  double dest_long=77.565239;
  double distancem=TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),dest_lat,dest_long);
  Serial.print(distancem);
  }  
  }
}
