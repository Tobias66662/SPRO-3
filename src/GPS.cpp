// Navigation Code
// Author: Ventsislav Andreev Ivanov
// Semester Project 3(SPRO3)
// Project Report: {insert link of report here}
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <Adafruit_GPS.h>   // GPS library
#include <SoftwareSerial.h> // Software Serial library that lets us create a new serial port to talk to the GPS sensor.
                            // This way we can use any digital pin to connect to the GPS's rx and tx pins.
#include "gps.h"

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Arduino RX
// Connect the GPS RX (receive) pin to Arduino TX

// you can change the pin numbers to match your wiring:
SoftwareSerial mySerial(4, 2); // mySerial object is created to pass two parameters: rx and tx respectively.
Adafruit_GPS GPS(&mySerial);   // GPS object created using the Adafruit_GPS class
//====================
// Constants
//====================
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO true
#define ARRAY_RESOLUTION 1 //(IN METERS) Set the resolution of which the vehicle will clean the area (e.g., 0.5 corresponds to points 0.5 meters apart) This affects the magnitude of n1 and n2.

//====================
// Global Variables
//====================
bool standby_flag = 1; // Standby flag tied to GPS.fix function returning 1 when the vehicles needs to be in standby mode(wait for GPS to connect to satellites) and 0 when it can continue(GPS location can be retrieved).
bool do_once_flag = 0; // Flag used for making sure the calculations in boundary_check() are done only once.

// Flags for letting us know if we are in or out of each boundary line (0 means we are outside and 1 means we are inside).
// E.g.(1) if the vehicle is above the top line, topline_f will return 0.
// E.g.(2) if the vehicle is above the left line but the left line has a negative slope, leftline_f will return 1.
// E.g.(3) if the vehicle is above the left line but the left line has a positive slope, leftline_f will return 0.
// Use this to perform checks(i.e., check if all flags are 1 to make sure vehicle is within all boundaries)
bool topline_f = 0;
bool rightline_f = 0;
bool bottomline_f = 0;
bool leftline_f = 0;

// Used in calculating the straight line equations
float m1, m2, m3, m4; // gradient
float c1, c2, c3, c4; // intercept

float target_point_lat = 0;  // target point latitude  (will be calculated in main)
float target_point_long = 0; // target point longitude (same ^^)

// Array sizes (dependent on ARRAY_RESOLUTION)
int n1 = 0; // Array size for top line
int n2 = 0; // Array size for bottom line

// variables storing the current latitude and longitude given by the gps
float lat_gps = 0;
float long_gps = 0;

point boundaries[] = {point(54.912155631362886, 9.779128545696924),
                      point(54.912202342965166, 9.779243666022944),
                      point(54.912142007135344, 9.779319848591634),
                      point(54.912116704986275, 9.779199649427701)};

//====================
// Function Definitions
//====================
void boundary_check(void);
void store_coordinates(void);
void gradient_and_intercept_calc(void);
void GPS_setup(void);

//====================
// Function Prototypes
//====================

void print_location()
{
  store_coordinates();
  Serial.print("GPS coordinates: ");
  Serial.print(lat_gps, 10);
  Serial.print(", ");
  Serial.println(long_gps, 10);
}

// checking if the target is to the left from the vehicle
bool is_target_left()
{
  store_coordinates();

  // the opposite relation gives true if the orientation is flipped
  return (target_point_long > long_gps && target_point_lat > lat_gps) || (target_point_long < long_gps && target_point_lat < lat_gps);
}

float distance_points(point p1, point p2)
{
  float dist_lon = abs(p1.lon - p2.lon);
  float dist_lat = abs(p1.lat - p2.lat);

  return sqrt(dist_lon * dist_lon + dist_lat * dist_lat);
}

uint8_t find_closest()
{
  store_coordinates();

  uint8_t closest;
  float distance = INFINITY;
  for (size_t i = 0; i < 4; i++)
  {
    if (distance < distance_points(point(lat_gps, long_gps), boundaries[i]))
    {
      distance = distance_points(point(lat_gps, long_gps), boundaries[i]);
      closest = i+1;
    }
  }

  return closest;
}

float remaining_distance()
{
  store_coordinates();
  float rem_dis_meters; // remaining distance between the two points (in meters)

  float lat_diff = target_point_lat - lat_gps;    // Difference in lat (in degrees)
  float lat_diff_radians = (lat_diff * PI) / 180; // Difference in lat (in rads)
  float long_diff = target_point_long - long_gps; // Difference in long (in degrees)

  float lat_diff_meters = lat_diff * 111000;                                           // Difference in lat (in meters)
  float long_diff_meters = ((40075 * cos(lat_diff_radians) * 1000) / 360) * long_diff; // Difference in long (in meters)

  rem_dis_meters = sqrt(lat_diff_meters * lat_diff_meters + long_diff_meters * long_diff_meters); // pythagorean theorem to calculate distance between the target point and current vehicle position
  return rem_dis_meters;
}

void GPS_setup(void)
{                     // initializer
  Serial.begin(9600); // initialising the serial monitor

  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 10 Hz update rate (can be change for up to 10Hz)

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  _delay_ms(1000); // we use a delay to give the gps a second to start up and execute all commands
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
}

void store_coordinates(void)
{

  char c = GPS.read(); // storing the characters coming through the serial bus in a 'c' char.

  if ((c) && (GPSECHO))

    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived())
    {

      if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
        return;                       // we can fail to parse a sentence in which case we should just wait for another
    }

  if (GPS.fix)
  { // GPS.fix returns the fix status as true or false
    lat_gps = GPS.latitude_fixed / 1.0E7;

    long_gps = GPS.longitude_fixed / 1.0E7;

    standby_flag = 0;
  }
  else
  { // Satellites not detected! Location data cannot be retreived!
    standby_flag = 1;
  }
}

void boundary_check(void)
{ // check if the vehicle is out of bounds
  store_coordinates();
  // TOP BOUNDARY
  if (lat_gps > (m1 * long_gps + c1))
  { // since it's our top line, our check is the same for any m, meaning the vehicle is outside if above the top line
    topline_f = 0;
  }
  else
  {
    topline_f = 1;
  }
  // RIGHT BOUNDARY
  if (m2 >= 0)
  { // for a positive slope of our right line the vehicle is outside if below the right line
    if (lat_gps > (m2 * long_gps + c2))
    { // since it's our top line, our check is the same for any m, meaning the vehicle is outside if above the top line
      rightline_f = 1;
    }
    else
    {
      rightline_f = 0;
    }
  }
  else
  { // for a negative slope of our right line, the vehicle is outside if above the right line
    if (lat_gps < (m2 * long_gps + c2))
    { // since it's our top line, our check is the same for any m, meaning the vehicle is outside if above the top line
      rightline_f = 1;
    }
    else
    {
      rightline_f = 0;
    }
  }
  // BOTTOM BOUNDARY
  if (lat_gps < (m3 * long_gps + c3))
  { // since it's our bottom line, our check is the same for any m, meaning the vehicle is outside if below the bottom line
    bottomline_f = 0;
  }
  else
  {
    bottomline_f = 1;
  }
  // LEFT BOUNDARY
  if (m4 <= 0)
  { // for a positive slope of our right line the vehicle is outside if above the left line
    if (lat_gps < (m4 * long_gps + c4))
    { // since it's our top line, our check is the same for any m, meaning the vehicle is outside if above the top line
      leftline_f = 0;
    }
    else
    {
      leftline_f = 1;
    }
  }
  else
  { // for a negative slope of our right line, the vehicle is outside if below the left line
    if (lat_gps > (m4 * long_gps + c4))
    { // since it's our top line, our check is the same for any m, meaning the vehicle is outside if above the top line
      leftline_f = 0;
    }
    else
    {
      leftline_f = 1;
    }
  }
}

void gradient_and_intercept_calc()
{ // Used in boundary_check()
  // Calculating the straight line equation "y=mx+c" for each line
  // Reminder: "m=(y2-y1)/(x2-x1)" and "c=y-mx"

  float a1, a2, a3, a4; // a for calculating the gradient (numerator) x-longitude
  float b1, b2, b3, b4; // b for calculating the gradient (denominator) y-latitude

  // Calculating the difference between longitudes(x) to get the numerator for calculating the gradient
  a1 = boundaries[0].lon - boundaries[1].lon;
  a2 = boundaries[1].lon - boundaries[2].lon;
  a3 = boundaries[2].lon - boundaries[3].lon;
  a4 = boundaries[3].lon - boundaries[0].lon;
  // Calculating the difference between latitudes(y) to get the denominator for calculating the gradient
  b1 = boundaries[0].lat - boundaries[1].lat;
  b2 = boundaries[1].lat - boundaries[2].lat;
  b3 = boundaries[2].lat - boundaries[3].lat;
  b4 = boundaries[3].lat - boundaries[0].lat;
  // Calculating the gradient m for each line
  m1 = b1 / a1; // gradient of our top line
  m2 = b2 / a2; // gradient of our right line
  m3 = b3 / a3; // gradient of our bottom line
  m4 = b4 / a4; // gradient of our left line

  // Calculating the intercept c for each line
  c1 = boundaries[0].lat - m1 * boundaries[0].lon; // intercept of our top line // NOTE: LONG AND LAT MIGHT NEED TO BE SWAPPED
  c2 = boundaries[1].lat - m2 * boundaries[1].lon; // intercept of our right line
  c3 = boundaries[2].lat - m3 * boundaries[2].lon; // intercept of our bottom line
  c4 = boundaries[3].lat - m4 * boundaries[3].lon; // intercept of our left line
}

// Notes: when referring to 'top' line and 'bottom' line, these are the top(North) and bottom(South) boundaries of the area of operation (AO).

// Sequence of functions for main.cpp:
// store_coordinates();
// boundary_check(); // contains GPS_setup(), which may not work (test it)
// All of these need to be placed in a loop and in the order mentioned.
