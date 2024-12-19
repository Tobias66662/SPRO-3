#include "tests.h"

Navigation nav;

bool test_straight() // TEST ========== IGNORE
{
  nav.straight(0, 0, 0);
  return true;
}

bool test_motors()
{
  left_motor.set_speed(255);
  right_motor.set_speed(255);
  left_motor.set_direction(1);
  right_motor.set_direction(1);
  _delay_ms(1000);
  left_motor.set_direction(0);
  right_motor.set_direction(0);
  _delay_ms(1000);
  left_motor.set_speed(0);
  right_motor.set_speed(0);

  brush_motor.set_speed(255);
  servo_motor.operate_servo(1);

  _delay_ms(1000);
  brush_motor.set_speed(0);
  servo_motor.operate_servo(0);

  return true;
}

bool test_turn() // TEST ========== IGNORE
{
  float angle = magnetometer->get_angle();
  for (size_t i = 0; i < 3; i++)
  {
    nav.turn(240);
  }

  return angle == magnetometer->get_angle();
}

bool test_magneto() // TEST ========== IGNORE
{
  Serial.print("Angle: ");
  Serial.println(magnetometer->get_angle());
  return true;
}

void test_gate()
{
  Serial.print("Gate voltage: ");
  Serial.println(get_voltage_gate());
}

void test_ultrasonics()
{
  for (int i = 0; i < 8; i++)
  {
    Serial.print("Object detected by sensor ");
    Serial.println(i);
    Serial.print("At distance ");
    Serial.println(getDistance(i));
    Serial.print("At port ");
    Serial.println(PORTB);

    _delay_ms(100);
  }
  _delay_ms(1000);
}