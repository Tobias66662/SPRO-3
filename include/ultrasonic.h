#ifndef _ULTRASONIC_H_ // checks whether NEXTION_H_INCLUDED is not declared.
#define _ULTRASONIC_H_ // Will declare NEXTION_H_INCLUDED once #ifndef generates true.

#define DEFAULT_OBJECT_DISTANCE 20 //(IN CENTIMETERS) Set the default distance from which the robot will detect an object in front of it.

#define LEFT_SENSOR 6
#define RIGHT_SENSOR 7

// function headers
uint16_t calculateDistance();
void ultrasonicInit();
void triggerSensor(uint8_t sensor);
uint16_t getDistance(uint8_t sensor);
bool checkForObstacle(uint8_t sensor, uint8_t distance_cm);
void MUXState(uint8_t sensor);
bool checkFrontSensors(uint8_t distance_cm);
bool checkSideSensors(uint8_t distance_cm);
void CaluclateSpeedOfSound();

extern bool is_triggered;

#endif