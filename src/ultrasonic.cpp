#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <Arduino.h>

#include "ultrasonic.h"

//---------CONSTANTS------------
#define Echo_Trig PB0 // Reflection and Trigger pin combined
#define speed 34      // speed of sound in cm/ms
#define A_control PB1
#define B_control PB2
#define C_control PB4

// #define Echo PB0 // Reflection pin
// #define Trig PB3 // Trigger pin

// ################################________GLOBAL_VARIABLES________############################################

volatile uint16_t pulse_width = 0;
volatile uint8_t edge_rising = 1;
volatile uint16_t k = 0;

// ################################________FUNCTION_PRECALLS________############################################

uint16_t calculateDistance();
void ultrasonicInit();
void TriggerSensor(uint8_t sensor);
uint16_t getDistance(uint8_t sensor);
bool checkForObstacle(uint8_t sensor, uint8_t distance_cm);
void MUXState(uint8_t sensor);
bool checkFrontSensors(uint8_t distance_cm);

// ################################___________INTERRUPTS_____________############################################

ISR(TIMER1_CAPT_vect) // Interrupt on PB0
{
  if (edge_rising)
  {
    TCNT1 = 0;               // Reset Timer1 counter
    TCCR1B &= ~(1 << ICES1); // Change input capture to falling edge
    TIFR1 |= (1 << ICF1);    // Clear ICF1 flag
    edge_rising = 0;
  }
  else
  {
    pulse_width = ICR1;     // Capture the pulse width
    TCCR1B |= (1 << ICES1); // Change input capture to rising edge
    TIFR1 |= (1 << ICF1);   // Clear ICF1 flag
    edge_rising = 1;
  }
  // k++;
}

// ################################___________FUNCTIONS_____________############################################

// Checks for objects within a specified distance of the sensor. Returns true if a object is detected
bool checkForObstacle(uint8_t sensor, uint8_t distance_cm)
{
  TriggerSensor(sensor);
  if (calculateDistance() <= distance_cm)
    return true;
  else
    return false;
}

// Returns distance from sensor to
uint16_t getDistance(uint8_t sensor)
{
  TriggerSensor(sensor); // Trigger the ultrasonic sensor
  Serial.println(pulse_width);
  // Serial.println(k);
  return calculateDistance();
}

void TriggerSensor(uint8_t sensor)
{
  // MUXState(sensor);

  // Send the trigger pusle
  DDRB |= (1 << Echo_Trig);   // Output
  PORTB &= ~(1 << Echo_Trig); // makes sure the trigger pin is low
  _delay_us(2);
  PORTB |= (1 << Echo_Trig);
  _delay_us(10); // 10 microsecond delay to send out a 8 cycle ultrasonic burst
  PORTB &= ~(1 << Echo_Trig);
  _delay_us(20);
  DDRB &= ~(1 << Echo_Trig); // input

  // PORTB &= ~(1 << Trig); // makes sure the trigger pin is low
  // _delay_us(2);
  // PORTB |= (1 << Trig);
  // _delay_us(10); // 10 microsecond delay to send out a 8 cycle ultrasonic burst
  // PORTB &= ~(1 << Trig);

  _delay_ms(40); // Wait for echo pulse to complete (sensor delay) (max high period = 38ms)
}

uint16_t calculateDistance()
{
  // Calculate and return distance in cm
  return (speed * (pulse_width * 0.0005)) / 2;
}

void ultrasonicInit()
{
  // MUX control pins
  DDRB |= (1 << A_control) | (1 << B_control) | (1 << C_control);
  PORTB &= ~((1 << A_control) | (1 << B_control) | (1 << C_control));
  // PORTB |= (1 << A_control) | (1 << B_control) | (1 << C_control);

  // DDRB &= ~(1 << Echo); //  Configures the Echo pin to be an input
  // DDRB |= (1 << Trig); //  Configures the Trig pin to be an output
  DDRB |= (1 << Echo_Trig); //  Configures the Trig pin to be an output

  // Timer 1 (input capture unit)
  TCCR1A = 0;                            // Normal mode
  TCCR1B = 0;                            // Reset in case arduino.h fucked with something
  TIMSK1 = 0;                            // Reset in case arduino.h fucked with something
  TCCR1C = 0;                            // Reset in case arduino.h fucked with something
  TCCR1B |= (1 << ICES1) | (1 << ICNC1); // Set input capture to rising edge And enables input noise cancelation
  TIMSK1 |= (1 << ICIE1);                // Inable input capture interrupt
  TCCR1B |= (1 << CS11);                 // Start timer with prescaler of 8
  sei();

  // PORTB |= (1 << A_control) | (1 << B_control) | (1 << C_control);
}

void MUXState(uint8_t sensor)
{
  switch (sensor)
  {
  case 0:
    PORTB &= ~((1 << A_control) | (1 << B_control) | (1 << C_control));
    break;
  case 1:
    PORTB = (PORTB & ~((1 << B_control) | (1 << C_control))) | (1 << A_control);
    break;
  case 2:
    PORTB = (PORTB & ~((1 << A_control) | (1 << C_control))) | (1 << B_control);
    break;
  case 3:
    PORTB = (PORTB & ~(1 << C_control)) | (1 << A_control) | (1 << B_control);
    break;
  case 4:
    PORTB = (PORTB & ~((1 << A_control) | (1 << B_control))) | (1 << C_control);
    break;
  case 5:
    PORTB = (PORTB & ~(1 << B_control)) | (1 << A_control) | (1 << C_control);
    break;
  case 6:
    PORTB = (PORTB & ~(1 << A_control)) | (1 << B_control) | (1 << C_control);
    break;
  case 7:
    PORTB |= (1 << A_control) | (1 << B_control) | (1 << C_control);
    break;
  }
  _delay_ms(10);
}

bool checkFrontSensors(uint8_t distance_cm)
{
  for (int i = 0; i <= 5; i++)
  {
    if (checkForObstacle(i, distance_cm))
      return true; // Object detected
  }
  return false; // No object detected
}

// case 1: PORTB = (1 << A) | ~(1 << B) | ~(1 << C);  break;
//   case 2: PORTB = ~(1 << A) | (1 << B) | ~(1 << C);  break;
//   case 3: PORTB = (1 << A) | (1 << B) | ~(1 << C);  break;
//   case 4: PORTB = ~(1 << A) | ~(1 << B) | (1 << C);  break;
//   case 5: PORTB = (1 << A) | ~(1 << B) | (1 << C);  break;
//   case 6: PORTB = (1 << A) | (1 << B) | ~(1 << C);  break;
//   case 7: PORTB = (1 << A) | (1 << B) | (1 << C);  break;
