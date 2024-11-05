#ifndef ULTRASONIC_H_INCLUDED // checks whether NEXTION_H_INCLUDED is not declared.
#define ULTRASONIC_H_INCLUDED // Will declare NEXTION_H_INCLUDED once #ifndef generates true.

//function headers 
uint16_t calculateDistance();
void ultrasonicInit();
void TriggerSensor();
uint16_t getDistance();

#endif