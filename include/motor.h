#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <stdint.h> // For uint8_t type

class Motor
{
public:
  Motor(uint8_t motor_id) : motor_id(motor_id) {} // Constructor initializer
  static void initialize();
  void set_speed(uint8_t duty_cycle);
  void toggle(bool state);
  void set_direction(uint8_t direction);
  void operate_servo(bool direction); // 1 open - 0 close

private:
  const uint8_t motor_id;
  bool motor_state; // Track motor state (on/off)
};

// Declare motor objects
extern Motor left_motor;  // Left track motor
extern Motor right_motor; // Right track motor
extern Motor brush_motor; // Brush motor
extern Motor servo_motor; // Servo motor

#endif