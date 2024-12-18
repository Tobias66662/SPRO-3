#ifndef _CURRENT_H_ 
#define _CURRENT_H_ 

#define ADC0 0 // NTC thermistor
#define ADC6 6 // Gate sensor
#define ADC7 7 // Current reading (hall-effect sensor)

void ADC_Init();
float get_current();
uint16_t readADC(uint8_t ADC_channel);

#endif