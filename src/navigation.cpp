#include "navigation.h"
#include <Arduino.h>
#include "gps.h"
#include "motor.h"
#include "ultrasonic.h"

#define CTRL_SIG_LIMIT 100

// declared in main
int8_t obstacle_array[100];

// global
bool flip_flag;

bool obstacle_flag = 0;
bool bottom_area_blocked_f = 0; // will tell if the top or bottom area is left uncleaned

Navigation::Navigation() {} // not used but necessary for compiler
Navigation::Navigation(Magnetometer *magnetometer)
{
    this->magnetometer = magnetometer;
}

void Navigation::set_object_avoidance(bool object_avoidance_mode)
{
    this->object_avoidance_mode = object_avoidance_mode;
}

// state of the vehicle so it knows which direction it should initiate turning when section is over
bool Navigation::heading_forward(bool turn)
{
    if (is_clockwise)
    {
        is_clockwise = !turn;
    }
    else
    {
        is_clockwise = turn;
    }

    return is_clockwise;
}

/**
 * converts @param angle smaller than 360 into signed angle with maximal 180 absolute value
 */
float convert(float angle)
{
    // Serial.print("Convert: "); Serial.println(angle);
    if (abs(angle) <= 180)
    {
        return angle;
    }
    else if (angle > 180)
    {
        return -1 * (360 - angle);
    }
    else
    {
        return 360 + angle;
    }
}

/**
 * stores existing offset increased by @param offset resulting between 0 to 360
 */
void Navigation::store_offset(float offset)
{
    if (abs(offset) > 360)
        offset = (int)floor(abs(offset)) % 360;

    this->offset = magnetometer->get_angle() + offset;

    if (this->offset > 360)
        this->offset -= 360;
    else if (this->offset < 0)
        this->offset += 360;
    // Serial.print("Offset stored: "); Serial.println(this->offset);
}

/**
 * @returns signed angle relative to the offset with maximal 180 absolute value
 */
float Navigation::get_offseted_angle() const
{
    float res = convert(magnetometer->get_angle() - this->offset);
    // Serial.print("Offset: "); Serial.println(res);
    return res;
}

void Navigation::turn(float angle)
{
    store_offset(angle);
    motor_control(0, 0, 0, false);
}

/**
 * aligns to @param is_north then targets the end of the section.
 * @param target has max absolute value 180 and @param is_clockwise determines the first turn to be taken.
 */
void Navigation::align_device(bool is_north, bool is_clockwise, float target)
{
    this->is_clockwise = is_clockwise;
    unsigned short max_misorientation = is_north == magnetometer->is_north() ? 180 : 360;
    float angle = magnetometer->get_angle();
    turn(max_misorientation - angle + target);
}

bool is_border_hit()
{
    boundary_check();
    return topline_f || bottomline_f;
}

void check_obstacles(int8_t *i, int i1, int i2)
{
    if (checkFrontSensors(DEFAULT_OBJECT_DISTANCE) && (obstacle_flag == 0))
    {
        obstacle_flag = 1;
        if (flip_flag == 0)
        {
            obstacle_array[*i] = i1;
            obstacle_array[(*i)++] = i2;
            bottom_area_blocked_f = 1; // object prevents cleaning the area underneath it
        }
        else
        {
            obstacle_array[*i] = i2;
            obstacle_array[(*i)++] = i1;
            bottom_area_blocked_f = 0; // object prevents cleaning the area above it
        }

        i += 2;
    }
    else if (checkFrontSensors(DEFAULT_OBJECT_DISTANCE) && (bottom_area_blocked_f == 1) && (obstacle_flag == 1))
    { // check when obstacle is no longer in the way and save those points
        if (bottomline_f == 0)
        { // here we know the vehicle is no longer being blocked by an object
            obstacle_array[*i] = i1;
            obstacle_array[(*i)++] = i2;
            obstacle_flag = 0;
            i += 2;
        }
    }
    else if (checkFrontSensors(DEFAULT_OBJECT_DISTANCE) && (bottom_area_blocked_f == 0) && (obstacle_flag == 1))
    {
        if (topline_f == 0)
        { // here we know the vehicle is no longer being blocked by an object
            obstacle_array[*i] = i2;
            obstacle_array[(*i)++] = i1;
            obstacle_flag = 0;
            i += 2;
        }
    }
}

void avoid_obsticales()
{
    while (!checkFrontSensors(5))
    {
    }
}

// totally disgusting sin against humanity
/**
 * @param is_straight gives if it compensates straight line
 * @param peak is optional to give maximum power a motor should have when the other is off (ratio 100 : 0)
 * @param peak is 255 by default
 * proportional mode for straight vs linear for turn?
 */
void Navigation::motor_control(int8_t *i, int i1, int i2, bool is_straight, unsigned char peak)
{
    if (peak < 1 || peak > 255)
        peak = 255;

    int prevT = 0;
    int eintegral = 0;
    int eprev = 0;

    float kp = 1;
    float kd = 0.025;
    float ki = 0;

    do
    {
        long currT = micros();
        float deltaT = ((float)(currT - prevT)) / (1.0e6);
        prevT = currT;
        // float pos = magnetometer->get_angle();

        // error
        int e = get_offseted_angle(); // pos - target;
        bool left = e >= 0;
        e = abs(e);

        // derivative
        float dedt = (e - eprev) / (deltaT);

        // integral
        eintegral = eintegral + e * deltaT;

        // control signal
        float u = kp * e + kd * dedt + ki * eintegral;

        float power = abs(u);
        if (power > CTRL_SIG_LIMIT)
        {
            power = CTRL_SIG_LIMIT;
        }

        // Serial.print("Applied power: "); Serial.println(power);

        uint8_t powerLeft, powerRight = peak;

        if (is_straight)
        {
            if (left)
            {
                powerRight -= peak * (power / CTRL_SIG_LIMIT); // if error 0 then decrease by zero
            }
            else
                powerLeft -= peak * (power / CTRL_SIG_LIMIT);

            left_motor.set_speed(powerLeft);
            right_motor.set_speed(powerRight);
        }
        else
        {
            if (left)
            {
                powerLeft = peak * (power / CTRL_SIG_LIMIT);

                left_motor.set_speed(powerLeft);
                right_motor.set_speed(powerLeft);
                right_motor.set_direction(0);
            }
            else
            {
                powerRight = peak * (power / CTRL_SIG_LIMIT);

                right_motor.set_speed(powerRight);
                left_motor.set_speed(powerRight);
                left_motor.set_direction(0);
            }
        }
        // Serial.print(powerLeft); Serial.print(" - "); Serial.println(powerRight);

        // store previous error
        eprev = e;

        if (is_straight)
        {
            if (object_avoidance_mode)
            {
                avoid_obsticales();
            }
            else
            {
                check_obstacles(i, i1, i2);
                break;
            }
        }

        // delay(100);
    } while ((is_straight && !is_border_hit()) || (!is_straight && eprev > 0));

    // reset them if turning mode was used
    left_motor.set_direction(1);
    left_motor.set_direction(1);
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