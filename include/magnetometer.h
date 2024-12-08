#if !defined(_MAGNETOMETER_H)
#define _MAGNETOMETER_H

#include "i2cmaster.h"

class Magnetometer {
    public:
    Magnetometer();
    float* get_measurement();
    float get_angle();
    bool is_north();


    private:
    void magnetometer_init();
    bool write_register(unsigned char address, unsigned char data);
    uint16_t* read_values();
    
    float offset[3];
    float result[3];
};

#endif // _MAGNETOMETER_H