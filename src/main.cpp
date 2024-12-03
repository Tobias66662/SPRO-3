#include <Arduino.h>
#include "ultrasonic.h"
#include "magnetometer.h"
#include "direction.h"
#include "gps.h"
#include <Adafruit_GPS.h>

#include <SoftwareSerial.h>

extern Magnetometer* magnetometer;

bool phase1_f = 0;

void phase_one(void);
void phase_two(void);

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

 phase_one();
 if (phase1_f = 1){
  phase_two();
 }

}

void phase_one(void){

for(int i=0;i<n1;i++){
  // bla bla I will finish this tomorrow
}

phase1_f = 1; // phase one done
}

void phase_two(void){

}