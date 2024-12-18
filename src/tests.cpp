#include "tests.h"
#include "MMC34160PJ.h"

Navigation nav;

bool test_straight() // TEST ========== IGNORE
{
  static char i = 0;
  if (i == 0)
  {
    nav.store_target();
    i++;
  }
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
  _delay_ms(100);
  left_motor.toggle(0);
  right_motor.toggle(0);

  brush_motor.set_speed(255);
  servo_motor.operate_servo(1);

  _delay_ms(1000);
  brush_motor.toggle(0);
  servo_motor.operate_servo(0);

  return true;
}

bool test_turn() // TEST ========== IGNORE
{
  float angle = magnetometer->get_angle();
  for (size_t i = 0; i < 2; i++)
  {
    nav.turn(180);
  }

  left_motor.toggle(0);
  right_motor.toggle(0);
  return angle == magnetometer->get_angle();
}

bool test_magneto() // TEST ========== IGNORE
{
  Serial.println(magnetometer->get_angle());
  return true;
}

void test_gate()
{
  Serial.print("Gate voltage: ");
  Serial.println(get_voltage_gate());
}