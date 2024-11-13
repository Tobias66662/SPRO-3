// #ifndef ULTRASONIC_H_INCLUDED // checks whether NEXTION_H_INCLUDED is not declared.
// #define ULTRASONIC_H_INCLUDED // Will declare NEXTION_H_INCLUDED once #ifndef generates true.
#ifndef _ULTRASONIC_H_ // checks whether NEXTION_H_INCLUDED is not declared.
#define _ULTRASONIC_H_ // Will declare NEXTION_H_INCLUDED once #ifndef generates true.

//function headers 
uint16_t calculateDistance();
void ultrasonicInit();
void TriggerSensor(uint8_t sensor);
uint16_t getDistance(uint8_t sensor);
bool checkForObstacle(uint8_t sensor, uint8_t distance_cm);

#endif