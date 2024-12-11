#if !defined(_MAGNETOMETER_H)
#define _MAGNETOMETER_H

#include "i2cmaster.h"
#include "Wire.h"

class Magnetometer
{
public:
    Magnetometer();
    float *get_measurement();
    float get_angle();
    bool is_north();

private:
    bool magnetometer_init();
    bool write_register(unsigned char address, unsigned char data);
    bool read_values();
    bool readData();
    uint8_t writeRegister(uint8_t reg, uint8_t val);

    float offset[3];
    float result[3];
    uint16_t raw[3];

    TwoWire *_wire;
};

#endif // _MAGNETOMETER_H