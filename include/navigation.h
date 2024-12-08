#if !defined(NAVIGATION_H)
#define NAVIGATION_H

#include "magnetometer.h"

class Navigation {
    
    private:
        Magnetometer* magnetometer;
        bool is_clockwise;
        float offset;
        void motor_control(bool, unsigned char = 255);
        float get_offseted_angle() const;
        void store_offset(float = 0);
    
    public:
        Navigation();
        Navigation(Magnetometer*);
        void align_device(bool, bool, float);
        bool heading_forward(bool = false);
        void target_angle(float, float);
        void turn(float);
};


#endif // NAVIGATION_H
