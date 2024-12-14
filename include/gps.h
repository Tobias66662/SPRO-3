#ifndef _GPS_H_

#define _GPS_H_

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

void boundary_check();
void store_coordinates();
void gradient_and_intercept_calc();
void GPS_setup();
uint8_t find_closest();

#define BOUNDARY_CONSTANTS
// The four points defining our boundary (in decimal degrees):
// Point 1 (top left)
#define LAT1 54.912155631362886
#define LONG1 9.779128545696924
// Point 2 (top right)
#define LAT2 54.912202342965166
#define LONG2 9.779243666022944
// Point 3 (bottom right)
#define LAT3 54.912142007135344
#define LONG3 9.779319848591634
// Point 4 (bottom left)
#define LAT4 54.912116704986275
#define LONG4 9.779199649427701

#endif
