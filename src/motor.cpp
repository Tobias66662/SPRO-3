#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#include "motor.h"

// Create motor objects
Motor left_motor(1);  // Left track motor
Motor right_motor(2); // Right track motor
Motor brush_motor(3); // Brush motor
Motor servo_motor(4); // Servo motor

// Initialize Timer0 and Timer2 for PWM
void Motor::initialize()
{
  // Timer0 //
  DDRD |= (1 << PD6); // Set PD6 (OC0A) as an output pin
  DDRD |= (1 << PD5); // Set PD5 (OC0B) as an output pin

  // Configure Timer0 for Fast PWM mode, non-inverting
  TCCR0A = 0x00;                         // Reset in case arduino.h fucked with something
  TCCR0B = 0x00;                         // Reset in case arduino.h fucked with something
  TCCR0A |= (1 << WGM00) | (1 << WGM01); // Fast PWM mode
  TCCR0B |= (1 << CS00) | (1 << CS01);   // 64 bit prescaler

  // L298N Dual H-bridge driver board
  DDRB |= (1 << PB5); // IN1 pin
  DDRC |= (1 << PC3); // IN2 pin
  DDRC |= (1 << PC1); // IN3 pin
  DDRC |= (1 << PC2); // IN4 pin

  // Timer2 //
  DDRB |= (1 << PB3); // Set PB3 (OC2A) as an output pin
  DDRD |= (1 << PD3); // Set PD3 (OC2B) as an output pin

  // Configure Timer2 for Fast PWM mode, non-inverting
  TCCR2A = 0x00;                         // Reset in case arduino.h fucked with something
  TCCR2B = 0x00;                         // Reset in case arduino.h fucked with something
  TCCR2A |= (1 << WGM20) | (1 << WGM21); // Fast PWM mode
  TCCR2B |= (1 << CS22);                 // 16 bit prescaler
}

// Set PWM duty cycle
void Motor::set_speed(uint8_t duty_cycle)
{
  switch (motor_id) // Set the output compare register
  {
  case 1:
    OCR0A = duty_cycle;
    break; // Left track motor
  case 2:
    OCR0B = duty_cycle;
    break; // Right track motor
  case 3:
    OCR2A = duty_cycle;
    break; // Brush motor
  case 4:
    OCR2B = duty_cycle;
    break; // Servo motor
  }
}

// Toggle motor state (1 == ON , 0 == OFF)
void Motor::toggle(bool state)
{
  motor_state = state;

  if (state)
  {
    switch (motor_id)
    {
    case 1:
      TCCR0A |= (1 << COM0A1);
      break; // Enable OC0A (left motor)
    case 2:
      TCCR0A |= (1 << COM0B1);
      break; // Enable OC0B (right motor)
    case 3:
      TCCR2A |= (1 << COM2A1);
      break; // Enable OC2A (brush motor)
    case 4:
      TCCR2A |= (1 << COM2B1);
      break; // Enable OC2B (servo motor)
    }
  }
  else
  {
    switch (motor_id)
    {
    case 1:
      TCCR0A &= ~(1 << COM0A1);
      break; // Disable OC0A (left motor)
    case 2:
      TCCR0A &= ~(1 << COM0B1);
      break; // Disable OC0B (right motor)
    case 3:
      TCCR2A &= ~(1 << COM2A1);
      break; // Disable OC2A (brush motor)
    case 4:
      TCCR2A &= ~(1 << COM2B1);
      break; // Disable OC2B (servo motor)
    }
  }
}

// Set motor direction (1 = Forward, 0 = Reverse)
void Motor::set_direction(uint8_t direction)
{
  switch (motor_id)
  {
  case 1:               // Left track motor
    if (direction == 1) // Forward
    {
      PORTB |= (1 << PB5);
      PORTC &= ~(1 << PC3);
    }
    else if (direction == 0) // Reverse
    {
      PORTB &= ~(1 << PB5);
      PORTC |= (1 << PC3);
    }
    break;
  case 2:               // Right track motor
    if (direction == 1) // Forward
    {
      PORTC &= ~(1 << PC1);
      PORTC |= (1 << PC2);
    }
    else if (direction == 0) // Reverse
    {
      PORTC |= (1 << PC1);
      PORTC &= ~(1 << PC2);
    }
    break;
  }
}