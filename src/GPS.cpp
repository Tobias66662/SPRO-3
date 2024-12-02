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
// Connect the GPS TX (transmit) pin to Digital 8
// Connect the GPS RX (receive) pin to Digital 7

// you can change the pin numbers to match your wiring:
SoftwareSerial mySerial(8, 7); // mySerial object is created to pass two parameters: rx and tx respectively.
Adafruit_GPS GPS(&mySerial); // GPS object created using the Adafruit_GPS class
//====================
// Constants
//====================
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true

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
//====================
// Global Variables
//====================
bool standby_flag = 0; // Standby flag tied to GPS.fix function returning 1 when the vehicles needs to be in standby mode(wait for GPS to connect to satellites) and 0 when it can continue(GPS location can be retrieved).
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

// Point arrays used to store a number of points on two opposite lines for the BF path navigation (may cause RAM overflow issues, optimize later if possible)
float array1_lat[60]; // latitude and longitude coordinate for the points on the top line
float array1_long[60];// ^^
float array2_lat[60]; // latitude and longitude coordinate for the points on the bottom line
float array2_long[60];// ^^
float target_point_lat=0;
float target_point_long=0;

// Array sizes (dependent on ARRAY_RESOLUTION)
int n1 = 0; // Array size for top line
int n2 = 0; // Array size for bottom line

// variables storing the current latitude and longitude given by the gps
float lat_gps = 0;
float long_gps = 0;

uint32_t timer = millis(); // will be removed once code is integrated

//====================
// Function Definitions
//====================
void boundary_check(void);
void store_coordinates(void);
void gradient_and_intercept_calc(void);
void create_arrays(void);
void GPS_setup(void);

//====================
// Function Prototypes
//====================
void GPS_setup(void){ // initializer
  Serial.begin(9600); // initialising the serial monitor

  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate (can be change for up to 10Hz)

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000); // we use a delay to give the gps a second to start up and execute all commands
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
}

void store_coordinates(void){

  char c = GPS.read(); // storing the characters coming through the serial bus in a 'c' char.

  if ((c) && (GPSECHO))
    //Serial.write(c);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats(remove timer when integrating to the project)
  if (millis() - timer > 2000){
    timer = millis(); // reset the timer

    if (GPS.fix){ // GPS.fix returns the fix status, where 0 means no fix and 1 means there is a fix.
      Serial.println("Latitude:"); //debugging
      lat_gps = GPS.latitude_fixed/1.0E7;
      Serial.println(lat_gps, 10); //debugging
      // Serial.print(GPS.lat); // returns N/S for North/South (uncomment if needed)
      Serial.println("Longitude:"); //debugging
      long_gps = GPS.longitude_fixed/1.0E7;
      Serial.println(long_gps, 10); //debugging
      // Serial.println(GPS.lon); // returns E/W for East/West (uncomment if needed)
      standby_flag = 0;
    }
    else{
      Serial.println("Satellites not detected! \nLocation data cannot be retreived!");
      standby_flag = 1;
    }
  }

}

void boundary_check(void){ // check if the vehicle is out of bounds

  if (do_once_flag == 0){ // only executed once to create the straight line equation variables and array points

    GPS_setup(); // GPS intialiser

    gradient_and_intercept_calc(); // getting our gradient and intercept for our straight line equations

    create_arrays();// creating 4 arrays of points for the BF path navigation, two for the top line and two for the bottom, where one will contain the latitude and the other will contain the longitude coordinates of each point

    do_once_flag=1;
  }

  // TOP BOUNDARY
  if (lat_gps>(m1*long_gps+c1)){ // since it's our top line, our check is the same for any m, meaning the vehicle is outside if above the top line
    topline_f=0;
  }
  else{
    topline_f=1;
  }
  // RIGHT BOUNDARY
  if (m2>=0){ // for a positive slope of our right line the vehicle is outside if below the right line
    if (lat_gps>(m2*long_gps+c2)){ // since it's our top line, our check is the same for any m, meaning the vehicle is outside if above the top line
      rightline_f=1;
    }
    else{
      rightline_f=0;
    }
  }
  else{ // for a negative slope of our right line, the vehicle is outside if above the right line
    if (lat_gps<(m2*long_gps+c2)){ // since it's our top line, our check is the same for any m, meaning the vehicle is outside if above the top line
      rightline_f=1;
    }
    else{
      rightline_f=0;
    }
  }
  // BOTTOM BOUNDARY
  if (lat_gps<(m3*long_gps+c3)){// since it's our bottom line, our check is the same for any m, meaning the vehicle is outside if below the bottom line
    bottomline_f=0;
  }
  else{
    bottomline_f=1;
  }
  // LEFT BOUNDARY
  if (m4<=0){ // for a positive slope of our right line the vehicle is outside if above the left line
    if (lat_gps<(m4*long_gps+c4)){ // since it's our top line, our check is the same for any m, meaning the vehicle is outside if above the top line
      leftline_f=0;
    }
    else{
      leftline_f=1;
    }
  }
  else{ // for a negative slope of our right line, the vehicle is outside if below the left line
    if (lat_gps>(m4*long_gps+c4)){ // since it's our top line, our check is the same for any m, meaning the vehicle is outside if above the top line
      leftline_f=0;
    }
    else{
      leftline_f=1;
    }
  }
}

void gradient_and_intercept_calc(){ // Used in boundary_check()
    // Calculating the straight line equation "y=mx+c" for each line
    // Reminder: "m=(y2-y1)/(x2-x1)" and "c=y-mx"

    float a1, a2, a3, a4; // a for calculating the gradient (numerator) x-longitude
    float b1, b2, b3, b4; // b for calculating the gradient (denominator) y-latitude

    // Calculating the difference between longitudes(x) to get the numerator for calculating the gradient
      a1= LONG1-LONG2;
      a2= LONG2-LONG3;
      a3= LONG3-LONG4;
      a4= LONG4-LONG1;
    // Calculating the difference between latitudes(y) to get the denominator for calculating the gradient
      b1= LAT1-LAT2;
      b2= LAT2-LAT3;
      b3= LAT3-LAT4;
      b4= LAT4-LAT1;
    // Calculating the gradient m for each line
    m1=b1/a1; // gradient of our top line       DEBUG Check: good
    m2=b2/a2; // gradient of our right line     DEBUG Check: good
    m3=b3/a3; // gradient of our bottom line    DEBUG Check: good
    m4=b4/a4; // gradient of our left line      DEBUG Check: good

    // Calculating the intercept c for each line
    c1= LAT1-m1*LONG1; // intercept of our top line // NOTE: LONG AND LAT MIGHT NEED TO BE SWAPPED
    c2= LAT2-m2*LONG2; // intercept of our right line
    c3= LAT3-m3*LONG3; // intercept of our bottom line
    c4= LAT4-m4*LONG4; // intercept of our left line
}

void create_arrays(void){ // Used in boundary_check()

  float long_diff, lat_diff, lat_diff_radians; // temporary variables used for calculating the difference in longitude and latitude, which are then converted in meters
  float lat_meters, long_meters; // temporary variables used for calculating the difference in long and lat in meters, which will be used in calculating the number of points for the top and bottom line

  lat_diff=LAT2-LAT1; // Difference in lat (in degrees)
  lat_diff_radians=(lat_diff*PI)/180; // Difference in lat, but in radians, because the cos() function only takes radians
  long_diff=LONG2-LONG1; // Difference in long (in degrees)
  lat_meters=lat_diff*111000; // Difference in lat (in meters)
  long_meters=((40075*cos(lat_diff_radians)*1000)/360)*long_diff; // Difference in long (in meters)

  n1=sqrt(lat_meters*lat_meters+long_meters*long_meters)/ARRAY_RESOLUTION; // number of array points for top line (Note: calculation returns a floating point number, but since n1 and n2 are an int, they will be rounded)

  // Now creating the points for the top line and storing their longitude and latitude in their respective arrays

  for(int i=0;i<=n1;i++){// for top line point array
    array1_lat[i]= (LAT1+(lat_diff/n1)*i);
    array1_long[i]= (LONG1+(long_diff/n1)*i);
  }

  lat_diff=LAT3-LAT4; // Difference in lat (in degrees)
  lat_diff_radians=(lat_diff*PI)/180; // Difference in lat (in rads)
  long_diff=LONG3-LONG4; // Difference in long (in degrees)
  lat_meters=lat_diff*111000; // Difference in lat (in meters)
  long_meters=((40075*cos(lat_diff_radians)*1000)/360)*long_diff; // Difference in long (in meters)

  n2=sqrt(lat_meters*lat_meters+long_meters*long_meters)/ARRAY_RESOLUTION; // number of array points for bottom line // number of array points for the bottom line

  // Now creating the points for the bottom line and storing their longitude and latitude in their respective arrays

  for(int i=0;i<=n2;i++){// for bottom line point array
    array2_lat[i]= (LAT4+(lat_diff/n2)*i);
    array2_long[i]= (LONG4+(long_diff/n2)*i);
  }
}

// TO-DO LIST: 
// 1.Remove arrays and move the calculation of the points along the top and bottom line to the main navigation sequence code. This will massively optimise the program.
// Also keep the function and only calculate the arrays size n1 and n2, which will be used for the calculation of points.

// Notes: when referring to 'top' line and 'bottom' line, these are the top(North) and bottom(South) boundaries of the area of operation (AO).

// Sequence of functions for main.cpp:
// store_coordinates();
// boundary_check(); // contains GPS_setup(), which may not work (test it)
// All of these need to be placed in a loop and in the order mentioned.

