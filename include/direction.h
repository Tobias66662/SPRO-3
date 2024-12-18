#ifndef _DIRECTION_H_

#define _DIRECTION_H_

extern int8_t angle_diff;
extern bool direction_f;    // flag returning which direction the vehicle should turn to face a point of interest (1 represents left and 0 represents right)
bool check_direction(void); // Note: Function definitions are extern on default.
int8_t get_angle(void);

#endif