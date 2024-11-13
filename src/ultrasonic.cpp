#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <Arduino.h>

#include "ultrasonic.h"

//---------CONSTANTS------------
#define Echo PB0 // Reflection pin
#define Trig PB1 // Trigger pin 
#define Echo_Trig PB0 // Reflection and Trigger pin combined
#define speed 34 // speed of sound in cm/ms

//################################________GLOBAL_VARIABLES________############################################

volatile uint16_t pulse_width = 0;
volatile uint8_t edge_rising = 1;

//################################________FUNCTION_PRECALLS________############################################

uint16_t calculateDistance();
void ultrasonicInit();
void TriggerSensor(uint8_t sensor);
uint16_t getDistance(uint8_t sensor);
bool checkForObstacle(uint8_t sensor, uint8_t distance_cm);

//################################___________INTERRUPTS_____________############################################

ISR(TIMER1_CAPT_vect) // Interrupt on PB0
{
  if(edge_rising)
  {
    TCNT1 = 0; // Reset Timer1 counter
    TCCR1B &= ~(1 << ICES1); //Change input capture to falling edge
    TIFR1 |= (1 << ICF1); // Clear ICF1 flag
    edge_rising = 0; 
  }
  else
  {
    pulse_width = ICR1; // Capture the pulse width
    TCCR1B |= (1 << ICES1); // Change input capture to rising edge 
    TIFR1 |= (1 << ICF1); // Clear ICF1 flag
    edge_rising = 1;
  }
  
}

//################################___________FUNCTIONS_____________############################################

// Checks for objects within a specified distance of the sensor. Returns true if a object is detected
bool checkForObstacle(uint8_t sensor, uint8_t distance_cm)
{
  TriggerSensor(sensor);
  if(calculateDistance() <= distance_cm) return true; 
  else return false;
}

// Returns distance from sensor to 
uint16_t getDistance(uint8_t sensor)
{
  TriggerSensor(sensor); // Trigger the ultrasonic sensor
  return calculateDistance();
}

void TriggerSensor(uint8_t sensor)
{
  // Send the trigger pusle 
  DDRB |= (1 << sensor); // Output
  PORTB &= ~(1 << sensor); // makes sure the trigger pin is low
  _delay_us(2); 
  PORTB |= (1 << sensor); 
  _delay_us(10); // 10 microsecond delay to send out a 8 cycle ultrasonic burst
  PORTB &= ~(1 << sensor); 
  DDRB &= ~(1 << sensor); // input

  _delay_ms(40); // Wait for echo pulse to complete (sensor delay) (max high period = 38ms)
}

uint16_t calculateDistance()
{
  // Calculate and return distance in cm
  return (speed * (pulse_width * 0.0005))/2;
}

void ultrasonicInit()
{
  // Timer 1 (input capture unit)
  TCCR1A = 0; // Normal mode
  TCCR1B = 0; // Reset in case arduino.h fucked with something
  TIMSK1 = 0; // Reset in case arduino.h fucked with something
  TCCR1C = 0; // Reset in case arduino.h fucked with something
  TCCR1B |= (1 << ICES1) | (1 << ICNC1); // Set input capture to rising edge And enables input noise cancelation
  TIMSK1 |= (1 << ICIE1); // Inable input capture interrupt
  TCCR1B |= (1 << CS11); // Start timer with prescaler of 8
  sei();
  
}
