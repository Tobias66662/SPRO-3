
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "ultrasonic.h"

//---------CONSTANTS------------
#define Echo PB0 // Reflection pin
#define Trig PB1 // Trigger pin 
#define speed 34 // speed of sound in cm/ms

typedef struct 
{
  uint16_t front_distance;
  uint16_t left_distance;
  uint16_t right_distance;
  uint16_t rear_distance;
} distance_t;

//################################________GLOBAL_VARIABLES________############################################

volatile uint16_t pulse_width = 0;
volatile uint8_t edge_rising = 1;

//################################________FUNCTION_PRECALLS________############################################

uint16_t calculateDistance();
void ultrasonicInit();
void TriggerSensor();
uint16_t getDistance();

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

uint16_t getDistance()
{
  TriggerSensor(); // Trigger the ultrasonic sensor
  _delay_ms(60); // Wait for echo pulse to complete (sensor delay)

  return calculateDistance();
}

void TriggerSensor()
{
  // Send the trigger pusle 
  PORTB &= ~(1 << Trig); // makes sure the trigger pin is low
  _delay_us(2); 
  PORTB |= (1 << Trig); 
  _delay_us(10); // 10 microsecond delay to send out a 8 cycle ultrasonic burst
  PORTB &= ~(1 << Trig);
}

uint16_t calculateDistance()
{
  // Calculate and return distance in cm
  return (speed * (pulse_width * 0.0005))/2;
}

void ultrasonicInit()
{
  DDRB &= ~(1 << Echo); //  Configures the Echo pin to be an input
  DDRB |= (1 << Trig); //  Configures the Trig pin to be an output
  // PORTB |= (1 << Echo); // pull-up 

  // Timer 1 (input capture unit)
  TCCR1A = 0; // Normal mode
  TCCR1B |= (1 << ICES1) | (1 << ICNC1); // Set input capture to rising edge And enables input noise cancelation
  TIMSK1 |= (1 << ICIE1); // Inable input capture interrupt
  TCCR1B |= (1 << CS11); // Start timer with prescaler of 8
  sei();
}
