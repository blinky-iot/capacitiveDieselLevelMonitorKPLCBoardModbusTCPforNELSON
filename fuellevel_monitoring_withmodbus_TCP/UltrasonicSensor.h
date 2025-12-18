// UltrasonicSensor.h (add this header if you want to separate)
#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>
#include <HardwareSerial.h>

// Hardware pins
#define A02YYUW_TX 25
#define A02YYUW_RX 26

// Error codes
#define ULTRASONIC_OK 1
#define ULTRASONIC_TOO_CLOSE 0
#define ULTRASONIC_CHECKSUM_ERROR -1
#define ULTRASONIC_TIMEOUT_ERROR -2
#define ULTRASONIC_INVALID_DATA -3

// Sensor constants
#define ULTRASONIC_BAUD_RATE 9600
#define ULTRASONIC_MIN_RANGE_MM 30
#define ULTRASONIC_MAX_RANGE_MM 4500
#define ULTRASONIC_READ_TIMEOUT_MS 1000
#define ULTRASONIC_RETRY_COUNT 10
#define ULTRASONIC_RETRY_DELAY_MS 50

// Function declarations
void setupUltrasonic();
float readUltrasonicCM();
float readUltrasonicCMWithTimeout();
void updateUltrasonicDistance();
void resetUltrasonicSensor();
void debugUltrasonicSensor();
float getFilteredDistance();
int getUltrasonicStatus();
bool isUltrasonicDataValid();

#endif