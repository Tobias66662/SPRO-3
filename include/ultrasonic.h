#ifndef _ULTRASONIC_H_ // checks whether NEXTION_H_INCLUDED is not declared.
#define _ULTRASONIC_H_ // Will declare NEXTION_H_INCLUDED once #ifndef generates true.

//function headers 
uint16_t calculateDistance();
void ultrasonicInit();
void TriggerSensor(uint8_t sensor);
uint16_t getDistance(uint8_t sensor);
bool checkForObstacle(uint8_t sensor, uint8_t distance_cm);
void MUXState(uint8_t sensor);
bool checkFrontSensors(uint8_t distance_cm);

#endif