#include <Arduino.h>
#include "ultrasonic.h"
#include "magnetometer.h"
#include "direction.h"
#include "gps.h"
#include <Adafruit_GPS.h>

#include <SoftwareSerial.h>

extern Magnetometer* magnetometer;

bool phase_one_f = 0; // 0 for we are not done with phase one, 1 for we are done with phase one

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
 if (phase_one_f = 1){
  phase_two();
 }

}

void phase_one(void){

for(int i=0;i<n1;i++){
  // bla bla I will finish this tomorrow. It will have the gps store cooridnates here, direction check, PD control and object detection functions
}

phase_one_f = 1; // phase one done
}

void phase_two(void){

}