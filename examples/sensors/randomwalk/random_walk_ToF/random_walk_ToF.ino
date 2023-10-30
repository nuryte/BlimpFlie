/*
  Project: ToF Distance Sensor Interface
  Description: This code interfaces with the ToF distance sensor using the I2C protocol,
               reads the distance, and prints it to the serial monitor, Also sends to a ground station
               in the variable values[0]
  Author: Diego Salazar
  Date: 2023-10-25
*/

// Include necessary libraries
#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>
#include <esp_now.h>
#include <WiFi.h>

// Define constants
#define DEV_I2C Wire
#define LED_BUILTIN_PIN 13
#define NUM_CONTROL_PARAMS 13 

// Global Variables
uint8_t broadcastAddress1[] = {0x34, 0x85, 0x18, 0xAB, 0xED, 0xC0};

// Create instances of components
VL53L4CX sensor_vl53l4cx_sat(&DEV_I2C, A1);

// Define custom data structures
typedef struct {
  float params[NUM_CONTROL_PARAMS];
  int channel;
} ControlInput;

typedef struct {
    int flag;
    float values[6];
} ReceivedData;

// Instantiate data structures
ControlInput controlParams;
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
  esp_now_register_send_cb(OnDataSent);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Initialize I2C and sensor
  DEV_I2C.begin();
  // sensor_vl53l4cx_sat.VL53L4CX_SetDeviceAddress(0x12);
  sensor_vl53l4cx_sat.begin();
  sensor_vl53l4cx_sat.VL53L4CX_Off();

  int status = sensor_vl53l4cx_sat.InitSensor(0x52);
  // sensor_vl53l4cx_sat.VL53L4CX_DataInit();
  // sensor_vl53l4cx_sat.VL53L4CX_SetCalibrationData();
  sensor_vl53l4cx_sat.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_LONG);
  if (status) {
    Serial.println("Init sensor_vl53l4cx_sat1 failed...");
  }
  sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();

  Serial.println("Ready to Go!");
}

void loop() {
  VL53L4CX_MultiRangingData_t MultiRangingData;
  uint8_t NewDataReady = 0;
  int status = 0;
  float dist = 0;

  // Wait for measurement data to be ready
  do {
    status = sensor_vl53l4cx_sat.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
  } while (!NewDataReady);

  // Process measurement data if ready and no errors occurred
  if ((NewDataReady != 0) && (!status)) {
    status = sensor_vl53l4cx_sat.VL53L4CX_GetMultiRangingData(&MultiRangingData);
    int no_of_objects = MultiRangingData.NumberOfObjectsFound;
    dist = (no_of_objects > 0) ? MultiRangingData.RangeData[0].RangeMilliMeter : 6000;
    
    if (status == 0) {
      status = sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
    }
  }

  // Update data structure for ESP-NOW transmission
  espSendData.flag = 1;
  espSendData.values[0] = dist;

  // Transmit data via ESP-NOW
  esp_now_send(broadcastAddress1, (uint8_t *)&espSendData, sizeof(espSendData));

  // Log distance measurement to Serial
  Serial.println(dist);
}
