#include "navigation.h"
#include <Arduino.h>

Navigation:: Navigation(Magnetometer* magnetometer) {
    this->magnetometer = magnetometer;
}

// state of the vehicle so it knows which direction it should initiate turning when section is over
bool Navigation::heading_forward(bool turn = false) {
    if (is_clockwise) {
        is_clockwise = !turn;
    } else {
        is_clockwise = turn;
    }

    return is_clockwise;
}

// difference between absolute north and device's relative
void Navigation::store_offset() {
    float angle = magnetometer->get_angle();
    if (angle < 90) {
        this->offset = angle;
    } else if (angle < 180) {
        this->offset = 180 - angle;
    } else if (angle < 270) {
        this->offset = angle - 180;
    } else {
        this->offset = 360 - angle;
    }
}

// offseted angle pointing to the target
void Navigation::target_angle(float previous, float atan) {
    if (previous/atan > 0  || abs(previous) > 180 || abs(atan) > 180) {
        Serial.print("1: "); Serial.print(previous);
        Serial.print(" 2: "); Serial.print(atan);
        Serial.println(" - Invalid input!");
        return;
    }
    
    float difference = previous + atan;
    turn(difference, is_clockwise);
    is_clockwise = !is_clockwise;

    store_offset();
}

// robust function to handle either degrees up to 360 or plusminus 180
void Navigation::turn(float _degrees, bool clockwise = true) {
    if (_degrees < 0) clockwise = false;
    // calculate the shorter angular displacement for the desired position
    float new_degrees = abs(_degrees) > 180 ? 360 - abs(_degrees) : abs(_degrees);
    bool new_clockwise =  abs(_degrees) > 180 ? !clockwise : clockwise;
}

// aligns to north then targets the end of the section.
// abs(target) < 180 and clockwise determines the first turn to be taken.
void Navigation::align_device(bool is_north, bool is_clockwise, float target) {
    this->is_clockwise = is_clockwise;
    unsigned short max_misorientation = is_north == magnetometer->is_north() ? 180 : 360;
    float angle = magnetometer->get_angle();
    turn(max_misorientation - angle);

    // after being aligned with the map offset should not be >180
    target_angle(0, target);
}

float Navigation::get_offset() const {
    return offset;
}

void motor_control(int target) {
    int prevT = 0;
    int eintegral = 0;
    int eprev = 0;

    int target = 0;

    float kp = 1;
    float kd = 0.025;
    float ki = 0;

    while (eprev > 1) {
        long currT = micros();
        float deltaT = ((float) (currT - prevT))/( 1.0e6 );
        prevT = currT;
        float pos = 0; // magnetometer->get_angle();

        // error
        int e = pos - target;

        // derivative
        float dedt = (e-eprev)/(deltaT);

        // integral
        eintegral = eintegral + e*deltaT;

        // control signal
        float u = kp*e + kd*dedt + ki*eintegral;

        // store previous error
        eprev = e;
    }
}