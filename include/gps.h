#ifndef _GPS_H_

#define _GPS_H_

#include "point.h"

extern bool standby_flag;

extern bool topline_f;
extern bool rightline_f;
extern bool bottomline_f;
extern bool leftline_f;

extern int n1;
extern int n2;

extern float lat_gps;
extern float long_gps;

extern float target_point_lat;
extern float target_point_long;

void GPS_setup();
void gradient_and_intercept_calc();
void boundary_check();
void store_coordinates();
uint8_t find_closest();

point boundaries[] = {point(54.912155631362886, 9.779128545696924),
point(54.912202342965166, 9.779243666022944),
point(54.912142007135344, 9.779319848591634),
point(54.912116704986275, 9.779199649427701) };

#endif
