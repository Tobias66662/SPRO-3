#include <Arduino.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "magnetometer.h"
#include "gps.h"
#include "direction.h"

bool direction_f = 0; // flag returning which direction the vehicle should turn to face a point of interest (1 represents left and 0 represents right)
int8_t dir_angle = 0; // variable returning the angle between the vector pointing  to the target point and the vector facing the same direction as the vehicle (passed to PD control)
extern bool direction_f; // flag returning which direction the vehicle should turn to face a point of interest (1 represents left and 0 represents right)

Magnetometer* magnetometer;

bool check_direction(void){
    float azimuth = magnetometer->get_angle();
    Serial.println(azimuth);

    float theta = (azimuth)*(PI/180); // getting the angle between the normal line to the vector of the vehicle direction and the horizontal axis

    float p;
    p = long_gps*cos(theta)+lat_gps*sin(theta);

    if((target_point_lat)>((p - target_point_long*cos(theta)) / (sin(theta)) )){ // if true, target point is above the line with a slope that's the same as the direction vector of the vehicle
        if((azimuth>=0)&&(azimuth<180)){ // if pointed East
            direction_f=0;
        }
        if((azimuth>=180)&&(azimuth<360)){ // if pointed West
            direction_f=1;
        }
    }
    if((target_point_lat)<( (p-target_point_long*cos(theta)) / (sin(theta)) )){ // if true, target point is below the line with a slope that's the same as the direction vector of the vehicle
        if((azimuth>=0)&&(azimuth<180)){ // if pointed East
            direction_f=1;
        }
        if((azimuth>=180)&&(azimuth<360)){ // if pointed West
            direction_f=0;
        }
    }

    return direction_f;
}

void check_angle(void){
    float m1, m2; // m1 represents the slope of the vehicle direction vector and m2 represents the vector pointing towards the target point from the vehicle's position

    float azimuth = magnetometer->get_angle();

    float theta = (azimuth)*(PI/180);

    float m1_angle = PI-theta;

    m1 = atan(m1_angle);

    m2 = (target_point_lat-lat_gps)/(target_point_long-long_gps);

    float calc1 = (m1-m2)/(1+m1*m2);
    if (calc1 < 0){ // taking the absolute value for the calculation of the acute angle
        calc1=calc1*(-1);
    }
    float acute_angle= atan(calc1); // acute angle between the two intersecting straight lines (in radians)
    dir_angle = acute_angle*(180/PI); // converting acute angle to degrees

    if ((m1<1) && (m2<1) && (azimuth < 90)){
        // do nothing, we got the correct angle
    }
    else if ((m1<1) && (m2<1) && (azimuth > 90))
}