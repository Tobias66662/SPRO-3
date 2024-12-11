#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include "ultrasonic.h"
#include "magnetometer.h"
#include "direction.h"
#include "gps.h"
#include "navigation.h"
#include "motor.h"

#define PATH_RESOLUTION 1          //(IN METERS) Set the resolution of which the vehicle will clean the area (e.g., 0.5 corresponds to points 0.5 meters apart) This affects the magnitude of n1 and n2.
#define DEFAULT_OBJECT_DISTANCE 20 //(IN CENTIMETERS) Set the default distance from which the robot will detect an object in front of it.

extern Magnetometer *magnetometer;
Navigation nav;

float long_diff1, lat_diff1, long_diff2, lat_diff2; // temporary variables used for calculating the difference in longitude and latitude, which are then converted in meters

int8_t obstacle_array[100]; // Obstacle array storing the starting and ending i1/i2 values where the obstacle was detected for both the top and bottom line
// For example, i values are stored as obstacle[0]= (i1 start value), obstacle[1]= (i2 start value), obstacle[2]= (i1 end value), obstacle[3]= (i2 end value): all of this is for the first object. From obstacle[4] to obstacle[7], it will be info for the second object and so on...
// Allows storing areas for up to 25 object blocks.

bool flip_flag = 0;
bool obstacle_flag = 0;
bool bottom_area_blocked_f = 0; // will tell if the top or bottom area is left uncleaned

void phase_one(void);
void phase_two(void);
void get_next_point(int *i1, int *i2);
void check_obstacles(int8_t *i, int i1, int i2);

void setup()
{
  Serial.begin(9600);

  Motor::initialize();
  magnetometer = new Magnetometer();
  nav = Navigation(magnetometer);

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
}

void test_motor()
{
  float angle = magnetometer->get_angle();
  nav.motor_control(true, 100);
  return;
}

void test2()
{
}
void loop()
{
  // for testing the motor

  phase_one();
  phase_two();

  delay(1000);
}

void phase_one(void)
{
  boundary_check(); // needs to happen first to initialise the GPS, store the first coordinates and finally calculate the gradients and intercepts needed for the point arrays
  check_angle();
  check_direction();

  int i1 = n1;
  int i2 = n2;

  int8_t i = 0;

  while ((i1 > 0) && (i2 > 0))
  {
    store_coordinates();

    // getting the next point if it's not in initialize otherwise call find_closest
    get_next_point(&i1, &i2);

    if (flip_flag == 0)
    { // flipping the flip_flag so we take turns between target points on the bottom line and target points on the top line
      flip_flag = 1;
    }
    else
    {
      flip_flag = 0;
    }

    // DISABLED
    // check_obstacles(&i, i1, i2);
    boundary_check();
    check_angle();

    nav.turn(angle_diff);
    nav.motor_control(true);
  }
} // end of phase_one()

void phase_two(void)
{
  // use info from obstacle_array[] to get points

} // end of phase_two()

void get_next_point(int *i1, int *i2)
{
  if ((flip_flag == 0) && (i1 > 0))
  { // target point on top line
    target_point_lat = (LAT1 + (lat_diff1 / n1) * *i1);
    target_point_long = (LONG1 + (long_diff1 / n1) * *i1);
    if (*i1 > 0)
    {
      (*i1)--;
    }
  }
  if ((flip_flag == 1) && (i2 > 0))
  { // target point on bottom line
    target_point_lat = (LAT4 + (lat_diff2 / n2) * *i2);
    target_point_long = (LONG4 + (long_diff2 / n2) * *i2);
    if (*i2 > 0)
    {
      (*i2)--;
    }
  }
}

void check_obstacles(int8_t *i, int i1, int i2)
{
  if ((checkFrontSensors(DEFAULT_OBJECT_DISTANCE) == true) && (obstacle_flag == 0))
  {
    obstacle_flag = 1;
    if (flip_flag == 0)
    {
      obstacle_array[*i] = i1;
      obstacle_array[(*i)++] = i2;
      bottom_area_blocked_f = 1; // object prevents cleaning the area underneath it
    }
    else
    {
      obstacle_array[*i] = i2;
      obstacle_array[(*i)++] = i1;
      bottom_area_blocked_f = 0; // object prevents cleaning the area above it
    }

    i += 2;
  }
  if ((checkFrontSensors(DEFAULT_OBJECT_DISTANCE) == true) && (bottom_area_blocked_f == 1) && (obstacle_flag == 1))
  { // check when obstacle is no longer in the way and save those points
    if (bottomline_f == 0)
    { // here we know the vehicle is no longer being blocked by an object
      obstacle_array[*i] = i1;
      obstacle_array[(*i)++] = i2;
      obstacle_flag = 0;
      i += 2;
    }
  }
  else if ((checkFrontSensors(DEFAULT_OBJECT_DISTANCE) == true) && (bottom_area_blocked_f == 0) && (obstacle_flag == 1))
  {
    if (topline_f == 0)
    { // here we know the vehicle is no longer being blocked by an object
      obstacle_array[*i] = i2;
      obstacle_array[(*i)++] = i1;
      obstacle_flag = 0;
      i += 2;
    }
  }
}