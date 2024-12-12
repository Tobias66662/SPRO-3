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
#define LAT1 54.91199421399982
#define LONG1 9.780650584513328
// Point 2 (top right)
#define LAT2 54.912084969945255
#define LONG2 9.780900940563537
// Point 3 (bottom right)
#define LAT3 54.911878824002315
#define LONG3 9.781052056377629
// Point 4 (bottom left)
#define LAT4 54.91186067273662
#define LONG4 9.780819744006713

#endif
