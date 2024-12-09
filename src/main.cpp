#include <Arduino.h>
#include "ultrasonic.h"
#include "magnetometer.h"
#include "direction.h"
#include "gps.h"
#include <Adafruit_GPS.h>

#include <SoftwareSerial.h>

#define BOUNDARY_CONSTANTS
// The four points defining our boundary (in decimal degrees): 
// Point 1 (top left)
#define LAT1 54.901667001056424
#define LONG1 9.808867693865457
// Point 2 (top right)
#define LAT2 54.90167543056495
#define LONG2 9.809096623046505
// Point 3 (bottom right)
#define LAT3 54.901612533420376
#define LONG3 9.809053769308182
// Point 4 (bottom left)
#define LAT4 54.90159826807496
#define LONG4 9.808851905646074

#define ARRAY_RESOLUTION 1 //(IN METERS) Set the resolution of which the vehicle will clean the area (e.g., 0.5 corresponds to points 0.5 meters apart) This affects the magnitude of n1 and n2.

extern Magnetometer* magnetometer;

// bool direction_f; // using function return value simply

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
  bool direc = check_direction();
  Serial.println(direc);
  phase_one();
  phase_two();

  delay(1000);
}

void phase_one(void){
  boundary_check(); // needs to happen first to initialise the GPS, store the first coordinates and finally calculate the gradients and intercepts needed for the point arrays
  check_angle();
  check_direction();

  bool phase_one_f=0;
  int i1 = n1;
  int i2 = n2;
  // WE KEEP THIS VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
  float long_diff1, lat_diff1, long_diff2, lat_diff2, lat_diff_radians; // temporary variables used for calculating the difference in longitude and latitude, which are then converted in meters
  float lat_meters, long_meters; // temporary variables used for calculating the difference in long and lat in meters, which will be used in calculating the number of points for the top and bottom line

  lat_diff1=LAT2-LAT1; // Difference in lat (in degrees)
  lat_diff_radians=(lat_diff1*PI)/180; // Difference in lat, but in radians, because the cos() function only takes radians
  long_diff1=LONG2-LONG1; // Difference in long (in degrees)
  lat_meters=lat_diff1*111000; // Difference in lat (in meters)
  long_meters=((40075*cos(lat_diff_radians)*1000)/360)*long_diff1; // Difference in long (in meters)

  n1=sqrt(lat_meters*lat_meters+long_meters*long_meters)/ARRAY_RESOLUTION; // number of array points for top line (Note: calculation returns a floating point number, but since n1 and n2 are an int, they will be rounded)

  lat_diff2=LAT3-LAT4; // Difference in lat (in degrees)
  lat_diff_radians=(lat_diff2*PI)/180; // Difference in lat (in rads)
  long_diff2=LONG3-LONG4; // Difference in long (in degrees)
  lat_meters=lat_diff2*111000; // Difference in lat (in meters)
  long_meters=((40075*cos(lat_diff_radians)*1000)/360)*long_diff2; // Difference in long (in meters)

  n2=sqrt(lat_meters*lat_meters+long_meters*long_meters)/ARRAY_RESOLUTION; // number of array points for bottom line // number of array points for the bottom line
  // WE KEEP THIS ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  bool flip_flag = 0;
  while ((i1 > 0) && (i2 > 0)){
    if ((flip_flag == 0) && (i1 > 0)){ // target point on top line
      target_point_lat= (LAT1+(lat_diff1/n1)*i1);
      target_point_long= (LONG1+(long_diff1/n1)*i1);
      if (i1 > 0){
        i1--;
      }
    }
    if ((flip_flag == 1) && (i2 > 0)){ // target point on bottom line
      target_point_lat= (LAT4+(lat_diff2/n2)*i2);
      target_point_long= (LONG4+(long_diff2/n2)*i2);
      if (i2 > 0){
        i2--;
      }
    }
    if (flip_flag == 0){
      flip_flag = 1;
    }
    else{
      flip_flag = 0;
    }

    // insert code here that makes the vehicle move to target point
  }


}

void phase_two(void){

}
