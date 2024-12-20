#include "navigation.h"
#include <Arduino.h>
#include "gps.h"
#include "motor.h"
#include "ultrasonic.h"
#include "direction.h"
#include "current.h"

#define CTRL_SIG_MAX 255   // max of value of 8 bits register
#define CTRL_SIG_MIN 220   // min observed value for not stalling motors
#define ATTEMPT_ANGLE 90   // attempt for avoid obstacle
#define BIN_TRIGGERED 4    // measured value
#define DISTANCE_REACHED 2 // meters of closeness when we consider the target hit
#define ANGLE_REACHED 5    // degrees of closeness
#define COOL_DOWN 750      // cool down for the control loop, time out for the adjustments to make effect
#define TURNING_CIRCLE 20  // space necessary for turning around
#define FRONT_COLLISION 10 // check distance in advance

// PID Control loop variables
#define KP 1
#define KD 0
#define KI 0

// declared in main
int8_t obstacle_array[100]; // stores the i1 and i2 values when the obstacle_mode_f switches to 1 and stores them again to the next array elements when it switches back to 0

// global
bool flip_flag; // flag that flips to switch between targeting a point on the top line and the bottom line (check main to see when it's flipped)
bool full_flag; // flip when trash is full

bool obstacle_mode_f = 0;       // tells if vehicle is in obstacle mode; obstacle mode starts when the vehicle sees an object for the first time and ends when it can again reach from top line to bottom line.
bool bottom_area_blocked_f = 0; // tells if the top or bottom area is left uncleaned

Navigation::Navigation() {} // not used but necessary for compiler
Navigation::Navigation(Magnetometer *magnetometer)
{
    this->magnetometer = magnetometer;
    this->object_avoidance_mode = true;
}

void Navigation::set_object_avoidance(bool object_avoidance_mode)
{
    this->object_avoidance_mode = object_avoidance_mode;
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
void Navigation::store_target(float offset)
{
    float current_angle = magnetometer->get_angle();
    Serial.print("Devices current angle: ");
    Serial.println(current_angle);
    if (abs(offset) > 360)
        offset = (int)floor(abs(offset)) % 360;

    this->target = current_angle + offset;

    if (this->target > 360)
        this->target -= 360;
    else if (this->target < 0)
        this->target += 360;
    Serial.print("Target stored: ");
    Serial.println(this->target);
    Serial.println("################");
}

/**
 * @returns signed angle relative to the offset with maximal 180 absolute value
 */
float Navigation::get_offseted_angle() const
{
    float angle = magnetometer->get_angle();
    return convert(angle - this->target);
}

bool is_border_hit()
{
    boundary_check();

    if (!flip_flag)
    {
        return topline_f;
    }
    else
        return bottomline_f;
}

bool check_obstacles(int8_t *i, int i1, int i2)
{
    if (!checkFrontSensors(DEFAULT_OBJECT_DISTANCE))
    {
        return false;
    }

    Serial.println("Object detected, aborting...");
    if (obstacle_mode_f == 0)
    {
        obstacle_mode_f = 1;
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

        return true;
    }
    else if ((bottom_area_blocked_f == 1) && (obstacle_mode_f == 1))
    { // check when obstacle is no longer in the way and save those points
        if (bottomline_f == 0)
        { // here we know the vehicle is no longer being blocked by an object
            obstacle_array[*i] = i1;
            obstacle_array[(*i)++] = i2;
            obstacle_mode_f = 0;
            i += 2;
        }

        return true;
    }
    else if ((bottom_area_blocked_f == 0) && (obstacle_mode_f == 1))
    {
        if (topline_f == 0)
        { // here we know the vehicle is no longer being blocked by an object
            obstacle_array[*i] = i2;
            obstacle_array[(*i)++] = i1;
            obstacle_mode_f = 0;
            i += 2;
        }

        return true;
    }
}

void Navigation::avoid_obstacles()
{
    if (!checkFrontSensors(FRONT_COLLISION))
        return;

    Serial.println("Object detected, avoiding...");

    char relevant_sensor = !is_target_left() ? LEFT_SENSOR : RIGHT_SENSOR;
    if (checkForObstacle((relevant_sensor == LEFT_SENSOR ? RIGHT_SENSOR : LEFT_SENSOR), TURNING_CIRCLE))
    {
        Serial.println("Vehicle stuck");
        return;
    }

    int8_t attempt = ATTEMPT_ANGLE * (relevant_sensor == LEFT_SENSOR ? 1 : -1);
    turn(attempt);

    // go forward, if detained continue recursion otherwise undo
    while (!checkFrontSensors(FRONT_COLLISION) && checkForObstacle(relevant_sensor, TURNING_CIRCLE))
    {
        left_motor.set_direction(1);
        right_motor.set_direction(1);

        left_motor.set_speed(CTRL_SIG_MIN);
        right_motor.set_speed(CTRL_SIG_MIN);
    }
    left_motor.set_speed(0);
    right_motor.set_speed(0);

    if (checkFrontSensors(FRONT_COLLISION))
    {
        avoid_obstacles();
    }

    turn(attempt * -1);
}

float Navigation::PID_control(int *prevT, int *eintegral, int *eprev)
{
    long currT = micros();
    float deltaT = ((float)(currT - *prevT)) / (1.0e6);
    *prevT = currT;
    // float pos = magnetometer->get_angle();

    // error
    int e = abs(get_offseted_angle()); // pos - target;

    // derivative
    float dedt = (e - *eprev) / (deltaT);

    // integral
    *eintegral = *eintegral + e * deltaT;

    // control signal
    float u = KP * e + KD * dedt + KI * *eintegral;

    // store previous error
    *eprev = e;

    return u;
}
// totally disgusting sin against humanity
/**
 * @param is_straight gives if it compensates straight line
 * @param peak is optional to give maximum power a motor should have when the other is off (ratio 100 : 0)
 * @param peak is 255 by default
 * proportional mode for straight vs linear for turn?
 */
void Navigation::straight(int8_t *i, int i1, int i2)
{
    store_target();
    int prevT = 0;
    int eintegral = 0;
    int eprev = 0;
    do
    {
        print_location();
        float u = PID_control(&prevT, &eintegral, &eprev);

        float power = abs(u);
        if (power > CTRL_SIG_MAX)
        {
            power = CTRL_SIG_MAX;
        }
        else if (UINT8_MAX - power < CTRL_SIG_MIN)
        {
            power = UINT8_MAX;
        }

        uint8_t powerLeft, powerRight;
        powerLeft = powerRight = UINT8_MAX;

        float angle = get_offseted_angle();
        Serial.print("Offset: ");
        Serial.println(angle);
        if (angle < 0)
        {
            powerRight -= power; // if error 0 then decrease by zero
        }
        else
            powerLeft -= power;

        left_motor.set_direction(1);
        right_motor.set_direction(1);

        left_motor.set_speed(powerLeft);
        right_motor.set_speed(powerRight);

        Serial.print(powerLeft);
        Serial.print(" - ");
        Serial.println(powerRight);

        if (object_avoidance_mode)
        {
            brush_motor.toggle(0);
            avoid_obstacles();
            turn(get_angle());
        }
        else
        {
            unsigned long motor_demo = millis();
            brush_motor.toggle(1);

            if (millis() > motor_demo + 1000)
            {
                brush_motor.set_speed(0);
            }

            // this case the obstacle will not be avoided, rather getting new target
            if (check_obstacles(i, i1, i2))
                break;

            if (voltage_reached(BIN_TRIGGERED)) // todo needs to be adjusted
            {
                Serial.println("Gate blocked");
                full_flag = true;
                break;
            }
        }

        _delay_ms(COOL_DOWN);
    } while ((object_avoidance_mode && remaining_distance() > DISTANCE_REACHED) || (!object_avoidance_mode && !is_border_hit()));

    left_motor.set_speed(0);
    right_motor.set_speed(0);
}

void Navigation::turn(float angle) // todo ensure not getting stuck and lower cool_down
{
    store_target(angle);

    int prevT = 0;
    int eintegral = 0;
    int eprev = 0;
    do
    {
        float u = PID_control(&prevT, &eintegral, &eprev);

        float power = abs(u);
        if (power > CTRL_SIG_MAX)
        {
            power = CTRL_SIG_MAX;
        }
        // avoid stalling the motors
        else if (power < CTRL_SIG_MIN)
        {
            power = CTRL_SIG_MIN;
        }

        float angle = get_offseted_angle();
        Serial.print("Offset: ");
        Serial.println(angle);
        if (angle > 0)
        {
            left_motor.set_direction(0);
            right_motor.set_direction(1);
            Serial.print('L');
        }
        else
        {
            left_motor.set_direction(1);
            right_motor.set_direction(0);
            Serial.print('R');
        }

        left_motor.set_speed(power);
        right_motor.set_speed(power);

        Serial.print(" - ");
        Serial.println(power);

        _delay_ms(COOL_DOWN);
    } while (eprev > ANGLE_REACHED);

    left_motor.set_speed(0);
    right_motor.set_speed(0);
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

//     store_target();
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

// state of the vehicle so it knows which direction it should initiate turning when section is over
// bool Navigation::heading_forward(bool turn)
// {
//     if (is_clockwise)
//     {
//         is_clockwise = !turn;
//     }
//     else
//     {
//         is_clockwise = turn;
//     }

//     return is_clockwise;
// }

/**
 * aligns to @param is_north then targets the end of the section.
 * @param target has max absolute value 180 and @param is_clockwise determines the first turn to be taken.
 */
// void Navigation::align_device(bool is_north, bool is_clockwise, float target)
// {
//     this->is_clockwise = is_clockwise;
//     unsigned short max_misorientation = is_north == magnetometer->is_north() ? 180 : 360;
//     float angle = magnetometer->get_angle();
//     turn(max_misorientation - angle + target);
// }