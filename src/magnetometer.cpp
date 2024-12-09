#include <Arduino.h>
#include "magnetometer.h"

#define I2C_ADDRESS 0x30
#define CTRLR0 0x07 // address 0x07 corresponds to Control Register 0 
#define STR0 0x06 // address 0x06 corresponds to Status Register 0 
#define SLA_W (0x30<<1) // device ID 0x30 with a write (0) bit
#define SLA_R (0x30<<1) | I2C_READ // device ID 0x30 with a read (1) bit

#define SIZE 3 // three coordinates
#define CONVERSION 0.48828125
#define DECLINATION 5.11f // magnetic declination in DK

Magnetometer:: Magnetometer() {
    magnetometer_init();
}

void Magnetometer::magnetometer_init() {
    i2c_init();
    offset[0] = offset[1] = offset[2] = 0;
    bool success = true;
    
    success &= write_register(CTRLR0, 0x80); delay(60); // refills capacitor
    success &= write_register(CTRLR0, 0x20); delay(10); // starts the SET action
    uint16_t* measure = read_values();

    success &= write_register(CTRLR0, 0x80); delay(60); // refills capacitor
    success &= write_register(CTRLR0, 0x40); delay(10); // starts the SET action
    uint16_t* measure2 = read_values();

    if (success && measure != nullptr && measure2 != nullptr){
            offset[0] = ((float)measure[0] + (float)measure2[0]) * CONVERSION * 0.5;
            offset[1] = ((float)measure[1] + (float)measure2[1]) * CONVERSION * 0.5;
            offset[2] = ((float)measure[2] + (float)measure2[2]) * CONVERSION * 0.5;    

            // Serial.print(offset[0]); Serial.print(" "); Serial.print(offset[1]); Serial.print(" "); Serial.print(offset[2]); Serial.print(" ");
            // Serial.println("Object offset");
    }

    delete measure; delete measure2;

    success &= write_register(CTRLR0, 0x40); delay(10); // starts the SET action
    success &= write_register(0x08, 0x00); delay(10); // starts the SET action

    if (!success) Serial.println("Failed to calibrate the magnetometer");
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

/**
 * @returns dinamically allocated array of 16 bits, make sure to delete it!
 */
uint16_t* Magnetometer::read_values() {
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
        uint16_t* measure = new uint16_t[SIZE];
        measure[0] = i2c_readAck() | ((uint16_t)i2c_readAck() << 8);
        measure[1] = i2c_readAck() | ((uint16_t)i2c_readAck() << 8);
        measure[2] = i2c_readAck() | ((uint16_t)i2c_readNak() << 8);

        // Serial.print(measure[0]); Serial.print(" "); Serial.print(measure[1]); Serial.print(" "); Serial.print(measure[2]); Serial.print(" ");
        // Serial.println("Object raw");
        return measure;
    } else {
        return nullptr;
    }
}

float* Magnetometer::get_measurement() {
    if (!offset[0]) {
        return nullptr;
    }

    uint16_t* measure = read_values();
    if (measure == nullptr) {
        return nullptr;
    }

    result[0] =  CONVERSION * (float)measure[0] - 16060.30;
    result[1] =  CONVERSION * (float)measure[1] - 16000.73;
    result[2] =  CONVERSION * (float)measure[2] - 16066.65;

    delete measure;
    return result;
}

// returns -1 if measurement is unsuccessful
float Magnetometer::get_angle() {
    if (get_measurement() == nullptr) {
        return -1;
    }

    // Serial.print(result[0]); Serial.print(" "); Serial.print(result[1]); Serial.print(" "); Serial.print(result[2]); Serial.print(" ");
    // Serial.println("Object readings");

    float temp0 = 0;
    float temp1 = 0;
    float deg = 0;

    if(result[0] < 0) {
        if (result[1] > 0) {
          // NW
          temp0 = result[1];
          temp1 = -result[0];
          deg = 90 - atan(temp0 / temp1) * (180 / PI);
        } else {
          // SW
          temp0 = -result[1];
          temp1 = -result[0];
          deg = 90 + atan(temp0 / temp1) * (180 / PI);
        }
    } else {
        if (result[1] < 0) {
          // SE
          temp0 = -result[1];
          temp1 = result[0];
          deg = 270 - atan(temp0 / temp1) * (180 / PI);
        } else {
          // NE
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

bool Magnetometer::is_north() {
    get_measurement();
    return (result[0] < 0 && result[1] > 0) || (result[0] >= 0 && result[1] <= 0);
}