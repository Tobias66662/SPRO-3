#if !defined(NAVIGATION_H)
#define NAVIGATION_H

extern bool flip_flag;
extern bool full_flag;

#include "magnetometer.h"

class Navigation
{

private:
    Magnetometer *magnetometer;
    float target;
    float get_offseted_angle() const;
    float PID_control(int *, int *, int *);

    bool object_avoidance_mode;

public:
    Navigation();
    Navigation(Magnetometer *);
    void store_target(float = 0);
    void straight(int8_t *i, int i1, int i2);
    void turn(float);
    void set_object_avoidance(bool);
    void avoid_obstacles();
};

#endif // NAVIGATION_H
