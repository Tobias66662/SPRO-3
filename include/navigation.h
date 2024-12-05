#if !defined(NAVIGATION_H)
#define NAVIGATION_H

#include "magnetometer.h"

class Navigation {
    
    private:
    Magnetometer* magnetometer;
    bool is_clockwise;
    float offset;
    void motor_control(int);
    
    public:
    Navigation(Magnetometer*);
    void align_device(bool, bool, float);
    bool heading_forward(bool);
    void target_angle(float, float);
    void store_offset();
    float get_offset() const;
    void turn(float, bool); // static?
};


#endif // NAVIGATION_H
