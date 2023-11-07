/*
  Project: GY-US42V2 Ultrasonic Distance Sensor Interface
  Description: This code interfaces with the GY-US42V2 ultrasonic distance sensor using the I2C protocol,
               reads the distance, and prints it to the serial monitor.
  Author: Diego Salazar
  Date: 2023-10-24
*/

#include "GY_US42V2.h"  // Include the GY_US42V2 class

GY_US42V2 sensor;  // Create an instance of the GY_US42V2 class

void setup() {
    Serial.begin(9600);  // Initialize serial communication for debugging
}

void loop() {
    uint16_t distance = sensor.readDistance();  // Read distance from sensor
    if (distance != 0xFFFF) {  // Check for valid data
        Serial.println(distance);  // Print the distance to the Serial Monitor
    } else {
        Serial.println("Error: Could not read distance");  // Print error message if data is not valid
    }
    delay(100);  // Delay to prevent flooding the serial output
}
