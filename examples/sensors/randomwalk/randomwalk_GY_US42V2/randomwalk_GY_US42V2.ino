/*
  Project: GY-US42V2 Ultrasonic Distance Sensor Interface
  Description: This code interfaces with the GY-US42V2 Ultrasonic distance sensor using the I2C protocol,
               reads the distance, and prints it to the serial monitor, Also sends to a ground station
               in the variable values[0]
  Author: Diego Salazar
  Date: 2023-10-25
*/

// Include necessary libraries
#include <Arduino.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>

// Define constants

#define GY_US42V2_ADDRESS 0x70  // The I2C address of the GY-US42V2 sensor
#define RANGE_COMMAND 0x51  // Command to start a ranging cycle in cm
#define READ_BYTES 2  // Number of bytes to read from the sensor

#define DEV_I2C Wire
#define LED_BUILTIN_PIN 13
#define NUM_CONTROL_PARAMS 13 

// Function prototypes
uint16_t readDistance();

// Global Variables
uint8_t broadcastAddress1[] = {0x34, 0x85, 0x18, 0xAB, 0xED, 0xC0};


// Define custom data structures

typedef struct {
    int flag;
    float values[6];
} ReceivedData;

// Instantiate data structures
ReceivedData espSendData;

esp_now_peer_info_t peerInfo;

// Callback function for ESP-NOW data sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // This function can be used to log the status of sent data
}

void setup() {
  // Set up the built-in LED pin as output
  pinMode(LED_BUILTIN_PIN, OUTPUT);

  // Initialize Serial communication
  Serial.begin(115200);
  Serial.println("Starting...");

  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  Serial.print("ESP Board MAC Address:  ");
  Serial.print(WiFi.macAddress());
    
  esp_now_register_send_cb(OnDataSent);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Sensor
  Wire.begin();  // Initialize the I2C bus
  
  Serial.println("Ready to Go!");
}

void loop() {
  
  float dist = 0;

  uint16_t distance = readDistance();

  if (distance != 0xFFFF) {  // Check for valid data
    Serial.println(distance);  // Print the distance to the Serial Monitor
  } else {
    Serial.println("Error: Could not read distance");  // Error message
  }
  delay(100);  // Delay to prevent flooding the serial output

  // Update data structure for ESP-NOW transmission
  espSendData.flag = 1;
  espSendData.values[0] = distance;

  // Transmit data via ESP-NOW
  esp_now_send(broadcastAddress1, (uint8_t *)&espSendData, sizeof(espSendData));

  // Log distance measurement to Serial
  // Serial.println(dist);
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