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
uint8_t find_closest();

#define BOUNDARY_CONSTANTS
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

#endif
