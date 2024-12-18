#include <stdio.h>
#include <avr/io.h>

#include "current.h"

#define ADC_REFERENCE_VOLTAGE 5.0 // Refference voltage in Volts
#define ADC_RESOLUTION 1024.0     // 10-bit ADC (2^10 = 1024)
#define ACS712_SENSITIVITY 0.185  // Sensitivity in V/A (x05B varient) (Typical value at T = 25 degrees celsius)
#define ACS712_OFFSET_Voltage 2.5 // Voltage at 0A (x05B varient)

void ADC_Init();
uint16_t readADC(uint8_t ADC_channel);
float get_current();

uint16_t readADC(uint8_t ADC_channel)
{
  ADMUX &= 0xF0;        // Clears the input channel selections, but keep voltage selection reference
  ADMUX |= ADC_channel; // Sets what pin the voltage is going to be read from

  // Starts a conversion
  ADCSRA |= (1 << ADSC);

  // Waits for the convertion to complete, since ADSC will be set to 0 when the conversion is complete
  while ((ADCSRA & (1 << ADSC)))
    ;

  // Retuns the voltage with a range from 0 to 1024, where 5V == 1024
  return ADC; // 16 bit unsigned integer
}

void ADC_Init()
{
  ADMUX |= (1 << REFS0) | (1 << MUX0) | (1 << MUX1) | (1 << MUX2);    // Voltage Reference Selection: AVCC with external capacitor at AREF pin. (ACD7)
  ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC, ADC Prescaler Select Bits: 128 Division factor (Slow input clock)
}

float get_current()
{
  float ADC_voltage = ((float)readADC(ADC7) * ADC_REFERENCE_VOLTAGE) / ADC_RESOLUTION;
  float current = (ADC_voltage - ACS712_OFFSET_Voltage) / ACS712_SENSITIVITY;

  return current;
}

float get_voltage_gate()
{
  return ((float)readADC(ADC6) * ADC_REFERENCE_VOLTAGE) / ADC_RESOLUTION;
}

bool voltage_reached(float voltage)
{
  float ADC_voltage = ((float)readADC(ADC6) * ADC_REFERENCE_VOLTAGE) / ADC_RESOLUTION;
  return ADC_voltage >= voltage;
}
