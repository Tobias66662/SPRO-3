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


void check_direction(void);

void check_direction(void){
    float azimuth = get_angle(); // why doesn't this work?? I included the "magnetometer.h" header file
                                // there was something I had to do with the class to use get_angle, but I don't remember what
    float theta = (azimuth)*(PI/180); // getting the angle between the normal line to the vector of the vehicle direction and the horizontal axis

    float p = long_gps*cos(theta)+lat_gps*sin(theta);

    if((target_point_lat*sin(theta))>(p-target_point_long*cos(theta))){ // if true, target point is above the line with a slope that's the same as the direction vector of the vehicle
        if((azimuth>=0)&&(azimuth<90)){
            direction_f=0;
        }
        if((azimuth>=90)&&(azimuth<180)){
            direction_f=0;
        }
        if((azimuth>=180)&&(azimuth<270)){
            direction_f=1;
        }
        if((azimuth>=270)&&(azimuth<=360)){
            direction_f=1;
        }
    }
    if((target_point_lat*sin(theta))<(p-target_point_long*cos(theta))){ // if true, target point is below the line with a slope that's the same as the direction vector of the vehicle
        if((azimuth>=0)&&(azimuth<90)){
            direction_f=1;
        }
        if((azimuth>=90)&&(azimuth<180)){
            direction_f=1;
        }
        if((azimuth>=180)&&(azimuth<270)){
            direction_f=0;
        }
        if((azimuth>=270)&&(azimuth<=360)){
            direction_f=0;
        }
    }
}