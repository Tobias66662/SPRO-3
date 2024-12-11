#if !defined(NAVIGATION_H)
#define NAVIGATION_H

extern bool flip_flag;

#include "magnetometer.h"

class Navigation
{

private:
    Magnetometer *magnetometer;
    bool is_clockwise;
    float offset;
    float get_offseted_angle() const;
    void store_offset(float = 0);

    bool object_avoidance_mode;

public:
    Navigation();
    Navigation(Magnetometer *);
    void motor_control(int8_t *i, int i1, int i2, bool, unsigned char = 255);
    void align_device(bool, bool, float);
    bool heading_forward(bool = false);
    void turn(float);
    void set_object_avoidance(bool);
};

#endif // NAVIGATION_H
