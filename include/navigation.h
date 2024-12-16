#if !defined(NAVIGATION_H)
#define NAVIGATION_H

extern bool flip_flag;

#include "magnetometer.h"

class Navigation
{

private:
    Magnetometer *magnetometer;
    bool is_clockwise;
    float target;
    float get_offseted_angle() const;

    bool object_avoidance_mode;

public:
    Navigation();
    Navigation(Magnetometer *);
    void store_target(float = 0);
    void motor_control(int8_t *i, int i1, int i2, bool);
    void align_device(bool, bool, float);
    bool heading_forward(bool = false);
    void turn(float);
    void set_object_avoidance(bool);
    void avoid_obsticales();
};

#endif // NAVIGATION_H
