#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include "ultrasonic.h"
#include "magnetometer.h"
#include "direction.h"
#include "gps.h"
#include "motor.h"
#include "current.h"
#include "navigation.h"
#include "motor.h"
#include "tests.h"

#define PATH_RESOLUTION 1 //(IN METERS) Set the resolution of which the vehicle will clean the area (e.g., 0.5 corresponds to points 0.5 meters apart) This affects the magnitude of n1 and n2.

// Globals
float long_diff1, lat_diff1, long_diff2, lat_diff2; // temporary variables used for calculating the difference in longitude and latitude, which are then converted in meters

extern int8_t obstacle_array[100]; // Obstacle array storing the starting and ending i1/i2 values where the obstacle was detected for both the top and bottom line
// For example, if(bottom_area_blocked_f); i values are stored as obstacle[0]= (i1 start value), obstacle[1]= (i2 start value), obstacle[2]= (i1 end value), obstacle[3]= (i2 end value): all of this is for the first object. From obstacle[4] to obstacle[7], it will be info for the second object and so on...
//              if(!bottom_area_blocked_f); i values are stored as obstacle [0]= (i2 start value), obstacle[1]= (i1 start value)...
// ^^ Allows storing areas for up to 25 object blocks. ^^

void phase_one(void);
void phase_two(void);
void get_next_point(int *i1, int *i2);
void check_obstacles(int8_t *i, int i1, int i2);
void i1_i2_init(int *i1, int *i2);

void setup()
{
  GPS_setup();                   // GPS intialiser (with a 1 sec delay to give the gps some time to execute all commands)
  gradient_and_intercept_calc(); // getting gradients and intercepts for straight line equations
  // Serial.begin(9600); // this is already in GPS_setup

  Motor::initialize();
  magnetometer = new Magnetometer();
  nav = Navigation(magnetometer);

  flip_flag = 0;
  float lat_meters, long_meters, lat_diff_radians; // temporary variables used for calculating the difference in long and lat in meters, which will be used in calculating the number of points for the top and bottom line

  lat_diff1 = LAT2 - LAT1;                                                   // Difference in lat (in degrees)
  lat_diff_radians = (lat_diff1 * PI) / 180;                                 // Difference in lat, but in radians, because the cos() function only takes radians
  long_diff1 = LONG2 - LONG1;                                                // Difference in long (in degrees)
  lat_meters = lat_diff1 * 111000;                                           // Difference in lat (in meters)
  long_meters = ((40075 * cos(lat_diff_radians) * 1000) / 360) * long_diff1; // Difference in long (in meters)

  n1 = sqrt(lat_meters * lat_meters + long_meters * long_meters) / PATH_RESOLUTION; // number of array points for top line (Note: calculation returns a floating point number, but since n1 and n2 are an int, they will be rounded)

  lat_diff2 = LAT3 - LAT4;                                                   // Difference in lat (in degrees)
  lat_diff_radians = (lat_diff2 * PI) / 180;                                 // Difference in lat (in rads)
  long_diff2 = LONG3 - LONG4;                                                // Difference in long (in degrees)
  lat_meters = lat_diff2 * 111000;                                           // Difference in lat (in meters)
  long_meters = ((40075 * cos(lat_diff_radians) * 1000) / 360) * long_diff2; // Difference in long (in meters)

  n2 = sqrt(lat_meters * lat_meters + long_meters * long_meters) / PATH_RESOLUTION; // number of array points for bottom line // number of array points for the bottom line

  left_motor.toggle(true);
  right_motor.toggle(true);
}

void loop()
{
  // test_magneto();
  // test_motors();
  // test_turn();
  // test_straight();

  phase_one();
  // phase_two();
}

void check_gps()
{
  float begin = millis() + 5000;

  while (standby_flag)
  {
    store_coordinates();

    if (millis() > begin)
    {
      Serial.println("Waiting for GPS signal...");
      begin = millis() + 5000;
    }
  }
}

void phase_one(void)
{
  int i1, i2;
  check_gps();
  i1_i2_init(&i1, &i2);

  int8_t i = 0;

  while (((i1 >= 0) && (i2 >= 0)) && ((i1 <= n1) && (i1 <= n2)))
  {
    check_gps();
    Serial.print("Latitude: ");
    Serial.println(lat_gps, 10);
    Serial.print("Longitude: ");
    Serial.println(long_gps, 10);

    get_next_point(&i1, &i2); // !!THIS NEEDS TO BE CALLED BEFORE FLIPPING THE FLAGS!!

    if (flip_flag == 0) // flipping the flip_flag so we take turns between target points on the bottom line and target points on the top line
    {
      flip_flag = 1;
    }
    else
    {
      flip_flag = 0;
    }

    nav.turn(get_angle());
    nav.motor_control(&i, i1, i2, true); // remove these ugly placeholders as a temporary
  }
} // end of phase_one()

void phase_two(void)
{
  // use info from obstacle_array[] to get points

} // end of phase_two()

void get_next_point(int *i1, int *i2)
{
  nav.set_object_avoidance(true);
  switch (find_closest())
  {
  case 1:
    if ((flip_flag == 0) && (*i1 <= n1))
    { // target point on top line
      target_point_lat = (LAT1 + (lat_diff1 / n1) * *i1);
      target_point_long = (LONG1 + (long_diff1 / n1) * *i1);
      if (*i1 <= n1)
      {
        (*i1)++;
      }
    }
    if ((flip_flag == 1) && (*i2 <= n2))
    { // target point on bottom line
      target_point_lat = (LAT4 + (lat_diff2 / n2) * *i2);
      target_point_long = (LONG4 + (long_diff2 / n2) * *i2);
      if (*i2 <= 0)
      {
        (*i2)++;
      }
    }
    break;
  case 2:
    if ((flip_flag == 0) && (*i1 > 0))
    { // target point on top line
      target_point_lat = (LAT1 + (lat_diff1 / n1) * *i1);
      target_point_long = (LONG1 + (long_diff1 / n1) * *i1);
      if (*i1 > 0)
      {
        (*i1)--;
      }
    }
    if ((flip_flag == 1) && (*i2 > 0))
    { // target point on bottom line
      target_point_lat = (LAT4 + (lat_diff2 / n2) * *i2);
      target_point_long = (LONG4 + (long_diff2 / n2) * *i2);
      if (*i2 > 0)
      {
        (*i2)--;
      }
    }
    break;
  case 3:
    if ((flip_flag == 0) && (*i2 > 0))
    { // target point on bottom line
      target_point_lat = (LAT4 + (lat_diff2 / n2) * *i2);
      target_point_long = (LONG4 + (long_diff2 / n2) * *i2);
      if (*i2 > 0)
      {
        (*i2)--;
      }
    }
    if ((flip_flag == 1) && (*i1 > 0))
    { // target point on top line
      target_point_lat = (LAT1 + (lat_diff1 / n1) * *i1);
      target_point_long = (LONG1 + (long_diff1 / n1) * *i1);
      if (*i1 > 0)
      {
        (*i1)--;
      }
    }
    break;
  case 4:
    if ((flip_flag == 0) && (*i2 <= n2))
    { // target point on bottom line
      target_point_lat = (LAT4 + (lat_diff2 / n2) * *i2);
      target_point_long = (LONG4 + (long_diff2 / n2) * *i2);
      if (*i2 <= 0)
      {
        (*i2)++;
      }
    }
    if ((flip_flag == 1) && (*i1 <= n1))
    { // target point on top line
      target_point_lat = (LAT1 + (lat_diff1 / n1) * *i1);
      target_point_long = (LONG1 + (long_diff1 / n1) * *i1);
      if (*i1 <= n1)
      {
        (*i1)++;
      }
    }
    break;
  }
}

void i1_i2_init(int *i1, int *i2)
{
  if ((find_closest() == 1) || (find_closest() == 4))
  {
    *i1 = 0;
    *i2 = 0;
  }
  else
  {
    *i1 = n1;
    *i2 = n2;
  }
}