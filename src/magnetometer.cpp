#include <Arduino.h>
#include "magnetometer.h"
#include "Wire.h"

#define I2C_ADDRESS 0x30
#define CTRLR0 0x07 // address 0x07 corresponds to Control Register 0 
#define STR0 0x06 // address 0x06 corresponds to Status Register 0 
#define SLA_W (0x30<<1) // device ID 0x30 with a write (0) bit
#define SLA_R (0x30<<1) | I2C_READ // device ID 0x30 with a read (1) bit

#define SIZE 3 // three coordinates
#define CONVERSION 0.48828125
#define DECLINATION 5.11f // magnetic declination in DK

uint8_t Magnetometer::writeRegister(uint8_t reg, uint8_t val){
  _wire->beginTransmission(I2C_ADDRESS);
  _wire->write(reg);
  _wire->write(val);
  return _wire->endTransmission();
}

bool Magnetometer::readData(){
    byte ack = writeRegister(0x07, 0x01);
    if (ack != 0) return false;
    bool flag = false;
    uint8_t temporal = 0;
    unsigned long _t = millis() + 1000;
    while (!flag) {
        _wire->beginTransmission(I2C_ADDRESS);
        _wire->write(byte(0x06));
        _wire->endTransmission();
        _wire->requestFrom(I2C_ADDRESS,1);
        if(_wire->available()){
            temporal = _wire->read();
        }
        temporal &= 1;
        if (temporal != 0) {
          flag = true;
        }
        if (millis() > _t) return false;
    }

    byte tmp[6] = {0, 0, 0, 0, 0, 0};
    _wire->beginTransmission(I2C_ADDRESS);
    _wire->write(byte(0x00));
    _wire->endTransmission();
    _wire->requestFrom(I2C_ADDRESS,6);
    if(_wire->available()){
        for (int i = 0; i < 6; i++) {
            tmp[i] = Wire.read(); //save it
        }
    } else {
        return false;
    }

    raw[0] = tmp[1] << 8 | tmp[0]; //x
    raw[1] = tmp[3] << 8 | tmp[2]; //y
    raw[2] = tmp[5] << 8 | tmp[4]; //z
    // Serial.print(raw[0]); Serial.print(" "); Serial.print(raw[1]); Serial.print(" "); Serial.print(raw[2]); Serial.print(" ");
    // Serial.println("raw readings");
    return true;

    return true;
}

Magnetometer:: Magnetometer() {
    Wire.begin();
    _wire = &Wire;
    magnetometer_init();
}

bool Magnetometer::magnetometer_init() {
    float out1[3];
    float out2[3];
    byte ack = writeRegister(0x07, 0x80);
    if (ack != 0) return false;
    delay(60);
    ack = writeRegister(0x07, 0x20);
    if (ack != 0) return false;
    delay(10);
    if (!readData()) return false;
    for (int i=0; i<3; i++) {
        out1[i] = 0.48828125 * (float)raw[i];
    }
    ack = writeRegister(0x07, 0x80);
    if (ack != 0) return false;
    delay(60);
    ack = writeRegister(0x07, 0x40);
    if (ack != 0) return false;
    delay(10);
    if (!readData()) return false;
    for (int i=0; i<3; i++) {
        out2[i] = 0.48828125 * (float)raw[i];
    }
    for (int i=0; i<3; i++) {
        offset[i] = (out1[i]+out2[i])*0.5;
    }
    ack = writeRegister(0x07, 0x40);
    if (ack != 0) return false;
    delay(10);
    ack = writeRegister(0x08, 0x00);
    if (ack != 0) return false;
    delay(10);
    // Serial.print(offset[0]); Serial.print(" "); Serial.print(offset[1]); Serial.print(" "); Serial.print(offset[2]); Serial.print(" ");
    // Serial.println("object offset");
    return true;
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
bool Magnetometer::read_values() {
    if (!write_register(CTRLR0, 0x01) 
        || !write_register(STR0)
        || i2c_start(SLA_R)) // initiates a data acquisition from status register and switches to read
    {
        return false;
    }

    unsigned long int deadline = millis() + 1000; 
    while (!(i2c_readAck() & 1)) // waiting for status bit 0 to indicate ready
    {
        if (millis() > deadline) {
            return false;
        }
    }

    if (i2c_readNak() && (!write_register(0x00) || !i2c_start(SLA_R))) // setting pointer to the first data bit and switches to read
    {
        raw[0] = i2c_readAck() | ((uint16_t)i2c_readAck() << 8);
        raw[1] = i2c_readAck() | ((uint16_t)i2c_readAck() << 8);
        raw[2] = i2c_readAck() | ((uint16_t)i2c_readNak() << 8);

        // Serial.print(raw[0]); Serial.print(" "); Serial.print(raw[1]); Serial.print(" "); Serial.print(raw[2]); Serial.print(" ");
        // Serial.println("raw readings");

        return true;
    } else {
        return false;
    }
}

float* Magnetometer::get_measurement() {
    if (!offset[0]) {
        return nullptr;
    }

    if (!read_values()) {
        return nullptr;
    }

    for (int i=0; i<3; i++) {
        result[i] = CONVERSION * (float)raw[i] - offset[i];
    }

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