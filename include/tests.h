#if !defined(TESTS)
#define TESTS
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include "ultrasonic.h"
#include "magnetometer.h"
#include "direction.h"
#include "gps.h"
#include "motor.h"
#include "current.h"
#include "navigation.h"
#include "motor.h"
#include "tests.h"

extern Magnetometer *magnetometer;
extern Navigation nav;

bool test_straight();
bool test_motors();
bool test_turn();
bool test_magneto();

#endif // TESTS
