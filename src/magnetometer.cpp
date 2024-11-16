#include <Arduino.h>
#include "magnetometer.h"

Magnetometer:: Magnetometer() {
    magnetometer_init();
}

void Magnetometer::magnetometer_init() {
    i2c_init();
    offset[0] = offset[1] = offset[2] = 0;
    bool success = true;
    
    success &= write_register(CTRLR0, 0x80); delay(50); // refills capacitor
    success &= write_register(CTRLR0, 0x20); delay(50); // starts the SET action
    unsigned int* measure = read_values();

    success &= write_register(CTRLR0, 0x80); delay(50); // refills capacitor
    success &= write_register(CTRLR0, 0x40); delay(50); // starts the SET action
    unsigned int* measure2 = read_values();

    if (success && measure != nullptr && measure2 != nullptr){
            offset[0] = (float)(measure[0] + measure2[0]) * 0.5;
            offset[1] = (float)(measure[1] + measure2[1]) * 0.5;
            offset[2] = (float)(measure[2] + measure2[2]) * 0.5;    
    }

    delete measure; delete measure2;
}

bool Magnetometer::write_register(unsigned char address, unsigned char data = 0) {
    bool error = false;
    error |= i2c_rep_start(SLA_W);
    error |= i2c_write(address);
    
    if (data) {
        error |= i2c_write(data);
    }
    i2c_stop();

    return !error;
}

unsigned int* Magnetometer::read_values() {
    if (!write_register(CTRLR0, 0x01) 
        || !write_register(STR0)
        || i2c_start(SLA_R)) // initiates a data acquisition from status register and switches to read
    {
        return nullptr;
    }

    unsigned long int deadline = millis() + 1000; 
    while (!(i2c_readAck() & 1)) // waiting for status bit 0 to indicate ready
    {
        if (millis() > deadline) {
            return nullptr;
        }
    }

    if (i2c_readNak() && (!write_register(0x00) || !i2c_start(SLA_R))) // setting pointer to the first data bit and switches to read
    {
        unsigned int* result = new unsigned int[SIZE];
        result[0] = i2c_readAck() | ((unsigned int)i2c_readAck() << 8);
        result[1] = i2c_readAck() | ((unsigned int)i2c_readAck() << 8);
        result[2] = i2c_readAck() | ((unsigned int)i2c_readNak() << 8);

        return result;
    } else {
        return nullptr;
    }
}

float* Magnetometer::get_measurement() {
    if (!offset[0]) {
        return nullptr;
    }

    unsigned int* measure = read_values();
    if (read_values() == nullptr) {
        return nullptr;
    }
        result[0] = (float)(measure[0] - offset[0]) * CONVERSION;
        result[1] = (float)(measure[1] - offset[1]) * CONVERSION;
        result[2] = (float)(measure[2] - offset[2]) * CONVERSION;

    delete measure;
    return result;
}

// returns -1 if measurement is unsuccessful
float Magnetometer::get_angle() {
    if (get_measurement() == nullptr) {
        return -1;
    }

    float temp0 = 0;
    float temp1 = 0;
    float deg = 0;

    if(result[0] < 0) {
        if (result[1] > 0) {
          //Quadrant 1
          temp0 = result[1];
          temp1 = -result[0];
          deg = 90 - atan(temp0 / temp1) * (180 / PI);
        } else {
          //Quadrant 2
          temp0 = -result[1];
          temp1 = -result[0];
          deg = 90 + atan(temp0 / temp1) * (180 / PI);
        }
    } else {
        if (result[1] < 0) {
          //Quadrant 3
          temp0 = -result[1];
          temp1 = result[0];
          deg = 270 - atan(temp0 / temp1) * (180 / PI);
        } else {
          //Quadrant 4
          temp0 = result[1];
          temp1 = result[0];
          deg = 270 + atan(temp0 / temp1) * (180 / PI);
        }
    }

    deg += DECLINATION;

    if (deg > 360) {
        deg -= 360;
    }

    return deg;
}