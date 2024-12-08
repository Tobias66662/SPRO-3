#include "navigation.h"
#include <Arduino.h>

#define CTRL_SIG_LIMIT 100

Navigation:: Navigation() {} // not used but necessary for compiler
Navigation:: Navigation(Magnetometer* magnetometer) {
    this->magnetometer = magnetometer;
}

// state of the vehicle so it knows which direction it should initiate turning when section is over
bool Navigation::heading_forward(bool turn) {
    if (is_clockwise) {
        is_clockwise = !turn;
    } else {
        is_clockwise = turn;
    }

    return is_clockwise;
}

/**
 * converts @param angle smaller than 360 into signed angle with maximal 180 absolute value
 */
float convert(float angle) {
    // Serial.print("Convert: "); Serial.println(angle);
    if (abs(angle) <= 180) {
        return angle;
    } else if (angle > 180) {
        return -1 * (360 - angle);
    } else {
        return 360 + angle;
    }
}

/**
 * stores existing offset increased by @param offset resulting between 0 to 359
 */
void Navigation::store_offset(float offset) {
    if (offset > 359 || 0 > offset) 
        offset = (int)floor(abs(offset)) % 360;

    this->offset = magnetometer->get_angle() + offset;

    if (this->offset > 359) this->offset -= 360;
    else if (this->offset < 0) this->offset += 360;
    Serial.print("Offset stored: "); Serial.println(this->offset);
}

/**
 * @returns signed angle relative to the offset with maximal 180 absolute value
 */
float Navigation::get_offseted_angle() const {
    float res = convert(magnetometer->get_angle() - this->offset);
    Serial.print("Offset: "); Serial.println(res);
    return res;
}

void Navigation::turn(float angle) {
    store_offset(angle);
    motor_control(false);
    // motors off
}

/**
 * aligns to @param is_north then targets the end of the section.
 * @param target has max absolute value 180 and @param is_clockwise determines the first turn to be taken.
 */
void Navigation::align_device(bool is_north, bool is_clockwise, float target) {
    this->is_clockwise = is_clockwise;
    unsigned short max_misorientation = is_north == magnetometer->is_north() ? 180 : 360;
    float angle = magnetometer->get_angle();
    turn(max_misorientation - angle + target);
}

/** 
 * @param peak is optional to give maximum power a motor should have when the other is off (ratio 100 : 0)
 * @param peak is 255 by default 
 * proportional mode for straight vs linear for turn?
 */
void Navigation::motor_control(bool is_straight, unsigned char peak) {
    if (peak < 1 || peak > 255)
        peak = 255;

    int prevT = 0;
    int eintegral = 0;
    int eprev = 0;

    float kp = 1;
    float kd = 0.025;
    float ki = 0.025;

    do {
        long currT = micros();
        float deltaT = ((float) (currT - prevT))/( 1.0e6 );
        prevT = currT;
        // float pos = magnetometer->get_angle();

        // error
        int e = get_offseted_angle(); //pos - target;
        bool left = e >= 0;
        e = abs(e);

        // derivative
        float dedt = (e-eprev)/(deltaT);

        // integral
        eintegral = eintegral + e*deltaT;

        // control signal
        float u = kp*e + kd*dedt + ki*eintegral;

        float power = abs(u);
        if( power > CTRL_SIG_LIMIT ){
            power = CTRL_SIG_LIMIT;
        }

        Serial.print("Applied power: "); Serial.println(power);

        int powerLeft, powerRight = peak;

        if (is_straight) {
            if (left) {
                powerRight -= peak * (power/CTRL_SIG_LIMIT); // if error 0 then decrease by zero
            } else powerLeft -= peak * (power/CTRL_SIG_LIMIT);
        } else {
            if (left) {
                powerLeft = peak * (power/CTRL_SIG_LIMIT);
                powerRight = -1 * powerLeft;
            } else {
                powerRight = peak * (power/CTRL_SIG_LIMIT);
                powerLeft = -1 * powerRight;
            }
        }
        
        Serial.print(powerLeft); Serial.print(" - "); Serial.println(powerRight);

        // store previous error
        eprev = e;
        delay(500);
    } while (is_straight || eprev > 0); // TODO third condition
}

// offseted angle pointing to the target
// void Navigation::target_angle(float previous, float atan) {
//     if (previous/atan > 0  || abs(previous) > 180 || abs(atan) > 180) {
//         Serial.print("1: "); Serial.print(previous);
//         Serial.print(" 2: "); Serial.print(atan);
//         Serial.println(" - Invalid input!");
//         return;
//     }
    
//     float difference = previous + atan;
//     turn(difference, is_clockwise);
//     is_clockwise = !is_clockwise;

//     store_offset();
// }

/**
 * @param in handles either degrees up to 360 or plusminus 180
 * @returns signed values with maximal 180 absolute value
 */
// float optimal_arc(float in, bool clockwise) {
//     // calculate the shorter angular displacement for the desired position
//     char new_clockwise =  abs(in) > 180 ? !clockwise : clockwise;
//     float new_degrees = abs(in) > 180 ? 360 - abs(in) : abs(in);

//     float result = (new_clockwise == clockwise ? 1 : -1) * new_degrees;
//     // Serial.print("Opt. result: "); Serial.println(result);
//     return result;
// }