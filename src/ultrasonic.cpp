#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <Arduino.h>
#include <math.h>

#include "current.h"
#include "ultrasonic.h"

//---------CONSTANTS------------
#define Echo_Trig PB0                  // Reflection and Trigger pin combined
#define speed 34                       // speed of sound in cm/ms
#define MAX_PULSE_WIDTH 47059          // ca 400cm or 4m
#define MAX_PULSE_WIDTH_OVERFLOW 60000 // (510 cm) Value given if pulse exceeded max value
#define A_control PB1
#define B_control PB2
#define C_control PB4
#define B 4791.842
#define A_1 0.00335
#define B_1 0.000257
#define C_1 0.00000262
#define D_1 0.0000000638
#define R_25 10000   // NTC resistance at 25 celcius
#define R_air 287.05 // Gas constant (ideal gas)
#define K_air 1.4    // Specific heat ratio

#define Echo PB0 // Reflection pin
#define Trig PD7 // Trigger pin

// ################################________GLOBAL_VARIABLES________############################################

volatile uint16_t pulse_width = 0;
volatile uint8_t edge_rising = 1;
volatile uint8_t overflow = 0; // timer overflow counter
float velocity_sound = 0;

// ################################________FUNCTION_PRECALLS________############################################

uint16_t calculateDistance();
void ultrasonicInit();
void triggerSensor(uint8_t sensor);
uint16_t getDistance(uint8_t sensor);
bool checkForObstacle(uint8_t sensor, uint8_t distance_cm);
void MUXState(uint8_t sensor);
bool checkFrontSensors(uint8_t distance_cm);
void CaluclateSpeedOfSound();

// ################################___________INTERRUPTS_____________############################################

ISR(TIMER1_CAPT_vect) // Interrupt on PB0
{
  if (edge_rising)
  {
    TCNT1 = 0;               // Reset Timer1 counter
    TCCR1B &= ~(1 << ICES1); // Change input capture to falling edge
    TIFR1 |= (1 << ICF1);    // Clear ICF1 flag
    edge_rising = 0;
    overflow = 0;
  }
  else
  {
    if (!overflow && (TCNT1 <= MAX_PULSE_WIDTH))
      pulse_width = ICR1; // Capture the pulse width (47059 =~ 400 cm)
    else
      pulse_width = MAX_PULSE_WIDTH_OVERFLOW; // Max distance of 255 cm exceeded (60000 = 510 cm) (value is temporary to help debug)
    TCCR1B |= (1 << ICES1);                   // Change input capture to rising edge
    TIFR1 |= (1 << ICF1);                     // Clear ICF1 flag
    edge_rising = 1;
  }
}

ISR(TIMER1_OVF_vect)
{
  if (!edge_rising)
    overflow++;
  if (overflow > 100)
    overflow = 0;
}

// ################################___________FUNCTIONS_____________############################################

// Checks for objects within a specified distance of the sensor. Returns true if a object is detected
bool checkForObstacle(uint8_t sensor, uint8_t distance_cm)
{
  triggerSensor(sensor);
  if (calculateDistance() <= distance_cm)
    return true;
  else
    return false;
}

// Returns distance from sensor to
uint16_t getDistance(uint8_t sensor)
{
  triggerSensor(sensor); // Trigger the ultrasonic sensor
  return calculateDistance();
}

void triggerSensor(uint8_t sensor)
{
  MUXState(sensor);

  TIMSK1 |= (1 << TOIE1); // Enable timer overflow interupt

  // Send the trigger pusle
  PORTD &= ~(1 << Trig); // makes sure the trigger pin is low
  _delay_us(2);
  PORTD |= (1 << Trig);
  _delay_us(10); // 10 microsecond delay to send out a 8 cycle ultrasonic burst
  PORTD &= ~(1 << Trig);
  _delay_ms(70); // Wait for echo pulse to complete (sensor delay) (max high period = 70ms)

  TIMSK1 &= ~(1 << TOIE1); // Disable timer overflow interupt
}

uint16_t calculateDistance()
{
  // Calculate and return distance in cm
  // return (velocity_sound * (pulse_width * 0.0005)) / 2;
  return (speed * (pulse_width * 0.0005)) / 2;
}

void ultrasonicInit()
{
  // MUX control pins
  DDRB |= (1 << A_control) | (1 << B_control) | (1 << C_control);
  PORTB &= ~((1 << A_control) | (1 << B_control) | (1 << C_control));
  DDRB &= ~(1 << Echo); //  Configures the Echo pin to be an input
  DDRD |= (1 << Trig);  //  Configures the Trig pin to be an output

  // Timer 1 (input capture unit)
  TCCR1A = 0;                            // Normal mode
  TCCR1B = 0;                            // Reset in case arduino.h fucked with something
  TIMSK1 = 0;                            // Reset in case arduino.h fucked with something
  TCCR1C = 0;                            // Reset in case arduino.h fucked with something
  TCCR1B |= (1 << ICES1) | (1 << ICNC1); // Set input capture to rising edge And enables input noise cancelation
  TIMSK1 |= (1 << ICIE1);                // Enable input capture interrupt
  TCCR1B |= (1 << CS11);                 // Start timer with prescaler of 8
  sei();

  // CaluclateSpeedOfSound();
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
    // Serial.print(" - Case 6 - ");
    break;
  case 7:
    PORTB |= (1 << A_control) | (1 << B_control) | (1 << C_control);
    // Serial.print(" - Case 7 - ");
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

void CaluclateSpeedOfSound()
{
  float R_NTC = (107113.54 / (readADC(ADC0) + 2.4187847)) - 10833.151; // Resistance of reading of NTC thermistor

  float log_value = R_NTC = log(R_NTC / (float)R_25);
  float T_kelvin = pow((double)(A1 + B1 * log_value + C_1 * pow(log_value, 2) + D_1 * pow(log_value, 3)), -1); // Temperature in kelvin

  velocity_sound = sqrt(K_air * R_air * T_kelvin) / 10; // Calculatd the veleocity of sound in air
  Serial.print("Temperature (K): ");
  Serial.println(T_kelvin);
  Serial.print("Speed of sound (cm/ms): ");
  Serial.println(velocity_sound);
}