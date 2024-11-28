#include <Arduino.h>
#include "ultrasonic.h"
#include "magnetometer.h"
#include <Adafruit_GPS.h>

#include <SoftwareSerial.h>

extern Magnetometer* magnetometer;

void read_angle() {
  float measurement = magnetometer->get_angle();
  if(measurement != -1){
    Serial.print("Angle: "); Serial.println(measurement);
  } else {
    Serial.println("Failed to retrieve angle");
  }
}

void setup() {
  Serial.begin(9600);
  magnetometer = new Magnetometer();
}

void loop() {
	

  delay(1000);
}
