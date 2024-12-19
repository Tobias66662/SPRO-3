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
float distance_points(point, point);
float remaining_distance();
bool is_target_left();
void print_location();

extern point boundaries[];
#endif
