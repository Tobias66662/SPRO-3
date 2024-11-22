#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <Adafruit_GPS.h>   // GPS library
#include <SoftwareSerial.h> // Software Serial library that lets us create a new serial port to talk to the GPS sensor.
                            // This way we can use any digital pin to connect to the GPS's rx and tx pins.

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

// The four points defining our boundary: 
// Point 1 (top left)
#define LAT1 54.90168063457623
#define LONG1 9.80910556587135
// Point 2 (top right)
#define LAT2 54.90169100935217
#define LONG2 9.809538614174416
// Point 3 (bottom right)
#define LAT3 54.90150296612318
#define LONG3 9.809475461296886
// Point 4 (bottom left)
#define LAT4 54.90153927798755
#define LONG4 9.808936406377965
//====================
// Global Variables
//====================
uint8_t standby_flag = 0; // Standby flag tied to GPS.fix function returning 1 when the vehicles needs to be in standby mode(wait for GPS to connect to satellites) and 0 when it can continue(GPS location can be retrieved).

// Flags for letting us know if we are in or out of each boundary line (0 means we are outside and 1 means we are inside).
// E.g.(1) if the vehicle is above the top line, topline_f will return 0.
// E.g.(2) if the vehicle is above the left line but the left line has a negative slope, leftline_f will return 1.
// E.g.(3) if the vehicle is above the left line but the left line has a positive slope, leftline_f will return 0.
// Use this to perform checks(i.e., check if all flags are 1 to make sure vehicle is within all boundaries)
uint8_t topline_f = 0;
uint8_t rightline_f = 0;
uint8_t bottomline_f = 0;
uint8_t leftline_f = 0;

// Used in calculating the straight line equations
float m1, m2, m3, m4; // gradient
float c1, c2, c3, c4; // intercept

// Point arrays used to store a number of points on two opposite lines for the BF path navigation
float array[10];

// variables storing the current latitude and longitude given by the gps
uint16_t lat_gps = 0;
uint16_t long_gps = 0;
//====================
// Function Definitions
//====================
void boundary_check(void);
void store_coordinates(void);
void gradient_and_intercept_calc(void);
void create_arrays(void);

void setup() // initializer
{
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

uint32_t timer = millis();

void loop(){// run over and over again
  _delay_ms(1000);
  store_coordinates();
  // boundary_check();

}
//====================
// Function Prototypes
//====================
void store_coordinates(void){

  char c = GPS.read(); // storing the characters coming through the serial bus in a 'c' char.

  if ((c) && (GPSECHO))
    //Serial.write(c);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000){
    timer = millis(); // reset the timer

    if (GPS.fix){ // GPS.fix returns the fix status, where 0 means no fix and 1 means there is a fix.
      Serial.println("Latitude:");
      Serial.print(GPS.latitude_fixed, 4);
      lat_gps = GPS.latitude_fixed;
      // Serial.print(GPS.lat); // returns N/S for North/South (uncomment if needed)
      Serial.println("Longitude:");
      Serial.print(GPS.longitude_fixed, 4); 
      long_gps = GPS.longitude_fixed;
      // Serial.println(GPS.lon); // returns E/W for East/West (uncomment if needed)
    }
    else{
      Serial.println("Satellites not detected! \nLocation data cannot be retreived!");
      standby_flag = 1;
    }
  }

}

void boundary_check(void){ // check if the vehicle is out of bounds
  uint8_t do_once_flag = 0;

  if (do_once_flag == 0){ // only executed once to create the straight line equation variables and array points

    gradient_and_intercept_calc(); // getting our gradient and intercept for our straight line equations

    create_arrays();// creating 4 arrays of points for the BF path navigation, two for the top line and two for the bottom, where one will contain the latitude and the other will contain the longitude coordinates of each point

    do_once_flag++;
  }

  // Performing the boundary check
  // TOP BOUNDARY
  if (long_gps>(m1*lat_gps+c1)){ // since it's our top line, our check is the same for any m, meaning the vehicle is outside if above the top line
    Serial.println("ABOVE TOP LINE");
    topline_f=0;
  }
  else{
    Serial.println("BELOW TOP LINE");
    topline_f=1;
  }
  // RIGHT BOUNDARY
  if (m2>=0){ // for a positive slope of our right line the vehicle is outside if below the right line
    if (long_gps>(m2*lat_gps+c2)){ // since it's our top line, our check is the same for any m, meaning the vehicle is outside if above the top line
      Serial.println("ABOVE RIGHT LINE");
      rightline_f=1;
    }
    else{
      Serial.println("BELOW RIGHT LINE");
      rightline_f=0;
    }
  }
  else{ // for a negative slope of our right line, the vehicle is outside if above the right line
    if (long_gps<(m2*lat_gps+c2)){ // since it's our top line, our check is the same for any m, meaning the vehicle is outside if above the top line
      Serial.println("BELOW RIGHT LINE");
      rightline_f=1;
    }
    else{
      Serial.println("ABOVE RIGHT LINE");
      rightline_f=0;
    }
  }
  // BOTTOM BOUNDARY
  if (long_gps<(m3*lat_gps+c3)){// since it's our bottom line, our check is the same for any m, meaning the vehicle is outside if below the bottom line
    Serial.println("BELOW BOTTOM LINE");
    bottomline_f=0;
  }
  else{
    Serial.println("ABOVE BOTTOM LINE");
    bottomline_f=1;
  }
  // LEFT BOUNDARY
  if (m4<=0){ // for a positive slope of our right line the vehicle is outside if above the left line
    if (long_gps<(m4*lat_gps+c4)){ // since it's our top line, our check is the same for any m, meaning the vehicle is outside if above the top line
      Serial.println("BELOW LEFT LINE");
      leftline_f=0;
    }
    else{
      Serial.println("ABOVE LEFT LINE");
      leftline_f=1;
    }
  }
  else{ // for a negative slope of our right line, the vehicle is outside if below the left line
    if (long_gps>(m4*lat_gps+c4)){ // since it's our top line, our check is the same for any m, meaning the vehicle is outside if above the top line
      Serial.println("ABOVE LEFT LINE");
      leftline_f=0;
    }
    else{
      Serial.println("BELOW LEFT LINE");
      leftline_f=1;
    }
  }
}

void gradient_and_intercept_calc(){ // Used in boundary_check()
    // Calculating the straight line equation "y=mx+c" for each line
    // Reminder: "m=(y2-y1)/(x2-x1)" and "c=y-mx"

    float a1, a2, a3, a4; // a for calculating the gradient (numerator) y
    float b1, b2, b3, b4; // b for calculating the gradient (denominator) x

    // Calculating the difference between longitudes(y) to get the numerator for calculating the gradient
    if (LONG1 > LONG2){
      a1= LONG1-LONG2;
    }
    else{
      a1= LONG2-LONG1;
    }
    if (LONG2 > LONG3){
      a2= (LONG2-LONG3);
    }
    else{
      a2= LONG3-LONG2;
    }
    if (LONG3 > LONG4){
      a3= LONG3-LONG4;
    }
    else{
      a3= LONG4-LONG3;
    }
      if (LONG4 > LONG1){
      a4= LONG4-LONG1;
    }
    else{
      a4= LONG1-LONG4;
    }
    // Calculating the difference between latitudes(x) to get the denominator for calculating the gradient
    if (LAT1 > LAT2){
      b1= LAT1-LAT2;
    }
    else{
      b1= LAT2-LAT1;
    }
    if (LAT2 > LAT3){
      b2= LAT2-LAT3;
    }
    else{
      b2= LAT3-LAT2;
    }
    if (LAT3 > LAT4){
      b3= LAT3-LAT4;
    }
    else{
      b3= LAT4-LAT3;
    }
      if (LAT4 > LAT1){
      b4= LAT4-LAT1;
    }
    else{
      b4= LAT1-LAT4;
    }
    // Calculating the gradient m for each line
    m1=a1-b1; // gradient of our top line
    m2=a2-b2; // gradient of our right line
    m3=a3-b3; // gradient of our bottom line
    m4=a4-b4; // gradient of our left line

    // Calculating the intercept c for each line
    c1= LONG1-m1*LAT1; // intercept of our top line
    c2= LONG2-m2*LAT2; // intercept of our right line
    c3= LONG3-m3*LAT3; // intercept of our bottom line
    c4= LONG4-m4*LAT4; // intercept of our left line
}

void create_arrays(void){ // Used in boundary_check()
  //calculate length
  float x, y;
  if(LAT1>LAT2){
    x=LAT1-LAT2;
  }
  else{
    x=LAT2-LAT1;
  }
  if(LONG1>LONG2){
    y=LONG1-LONG2;
  }
  // finish this.....


  //n1=sqrt();
  //n2=(insert bottom line resolution here);
  for(int i=0;i<n1;i++){

  }// for top line

  for(int i=0;i<n2;i++){

  }// for bottom line
}

// TO-DO BEFORE TESTING: 
// 1. See what format the new GPS data looks like.
// 2. Make sure the point coordinates format matches that of the coordinates given by the GPS.