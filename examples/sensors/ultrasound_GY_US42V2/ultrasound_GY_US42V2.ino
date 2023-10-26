/*
  Project: GY-US42V2 Ultrasonic Distance Sensor Interface
  Description: This code interfaces with the GY-US42V2 ultrasonic distance sensor using the I2C protocol,
               reads the distance, and prints it to the serial monitor.
  Author: Diego Salazar
  Date: 2023-10-24
*/

#include <Wire.h>

#define GY_US42V2_ADDRESS 0x70  // The I2C address of the GY-US42V2 sensor
#define RANGE_COMMAND 0x51  // Command to start a ranging cycle in cm
#define READ_BYTES 2  // Number of bytes to read from the sensor

// Function prototypes
uint16_t readDistance();

void setup() {
  Wire.begin();  // Initialize the I2C bus
  Serial.begin(9600);  // Initialize serial communication for debugging
}

void loop() {
  uint16_t distance = readDistance();
  if (distance != 0xFFFF) {  // Check for valid data
    Serial.println(distance);  // Print the distance to the Serial Monitor
  } else {
    Serial.println("Error: Could not read distance");  // Error message
  }
  delay(100);  // Delay to prevent flooding the serial output
}

uint16_t readDistance() {
  Wire.beginTransmission(GY_US42V2_ADDRESS);
  Wire.write(RANGE_COMMAND);  // Send command to start a ranging cycle in cm
  Wire.endTransmission();
  delay(100);  // Wait for ranging to complete

  Wire.requestFrom(GY_US42V2_ADDRESS, READ_BYTES);  // Request 2 bytes of data
  if (Wire.available() == READ_BYTES) {
    uint16_t distance = Wire.read() << 8 | Wire.read();  // Combine the two bytes into a 16-bit number
    return distance;
  } else {
    return 0xFFFF;  // Return 0xFFFF if data is not available to indicate an error
  }
}
