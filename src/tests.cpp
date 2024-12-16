#include "tests.h"

Navigation nav;

bool test_straight() // TEST ========== IGNORE
{
  static char i = 0;
  if (i == 0)
  {
    nav.store_target();
    i++;
  }
  nav.motor_control(0, 0, 0, true);
  return true;
}

bool test_motors()
{
  for(int i=200;i<255;i+=10){
    Serial.print("Duty cycle:");
    Serial.println(i);
    left_motor.set_speed(i);
    right_motor.set_speed(i);
    left_motor.set_direction(1);
    right_motor.set_direction(1);
    _delay_ms(1000);
  }
  return true;
}

bool test_turn()  // TEST ========== IGNORE
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