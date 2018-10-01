
#include "Ubx.h"
UBLOX gps(Serial1,115200);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  gps.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(gps.readSensor()) 
  {
    String st=gps.getNMEA_RMC();
    Serial.println(st);
     st=gps.getNMEA_GSA();
    Serial.println(st);
  }
}
