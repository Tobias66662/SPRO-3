#ifndef _MAGNETOMETER_H_

#define _MAGNETOMETER_H_

#include "i2cmaster.h"

#define I2C_ADDRESS 0x30
#define CTRLR0 0x07 // address 0x07 corresponds to Control Register 0 
#define STR0 0x06 // address 0x06 corresponds to Status Register 0 
#define SLA_W (0x30<<1) // device ID 0x30 with a write (0) bit
#define SLA_R (0x30<<1) | I2C_READ // device ID 0x30 with a read (1) bit

#define SIZE 3 // three coordinates
#define CONVERSION 0.48828125
#define DECLINATION 5.11f // magnetic declination in DK

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

#endif