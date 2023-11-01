/*
  Project: Walk follower
  Description: This code interfaces with the two ToF distance sensor using the I2C protocol,
               reads the distance, and prints it to the serial monitor. The algorithm uses two measures
               to compute the angle from the wall and the distance. The ESP32 sends back to ground station.
               the address needs to be changed.
  Author: Diego Salazar
  Date: 2023-10-25
*/

#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>
#include <esp_now.h>
#include <WiFi.h>

// Constants
#define DEV_I2C Wire
#define Serial Serial
#define LED_BUILTIN 13
#define LedPin LED_BUILTIN
#define NUM_PARAMS 15        
#define NUM_CONTROL_PARAMS 13 

// Global Variables
uint8_t broadcastAddress1[] = {0x34, 0x85, 0x18, 0xAB, 0xED, 0xC0}; //34:85:18:8D:86:70 //  34:85:18:AB:ED:C0 // 34:85:18:AB:ED:C0
float dist_1 = 0, dist_2 = 0;
float cos_mbeta, sin_mbeta, cos_pbeta, sin_pbeta;
double px1, px2, py1, py2;

// Components
VL53L4CX sensor_vl53l4cx_sat(&DEV_I2C, A1);
// VL53L4CX sensor_vl53l4cx_sat2(&DEV_I2C, A3);

typedef struct ControlInput {
  float params[NUM_CONTROL_PARAMS];
  int channel;
} ControlInput;


typedef struct ReceivedData {
    int flag;
    float values[6];
} ReceivedData;


ControlInput controlParams;
ReceivedData espSendData;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // Serial.printf("Packet to: %s Status: %s\n", macStr, status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  pinMode(LedPin, OUTPUT);
  Serial.begin(115200);
  Serial.println("Starting...");


  // ESP-NOW Initialization
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


  // I2C initialization
  DEV_I2C.begin();
  sensor_vl53l4cx_sat.VL53L4CX_SetDeviceAddress(0x12);
  // sensor_vl53l4cx_sat2.VL53L4CX_SetDeviceAddress(0x30);
  sensor_vl53l4cx_sat.begin();
  // sensor_vl53l4cx_sat2.begin();
  sensor_vl53l4cx_sat.VL53L4CX_Off();
  // sensor_vl53l4cx_sat2.VL53L4CX_Off();

  sensor_vl53l4cx_sat.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_LONG);
  // sensor_vl53l4cx_sat2.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_LONG);
  // sensor_vl53l4cx_sat.VL53L4CX_get_preset_mode_timing_cfg(VL53L4CX_DEVICEPRESETMODE_HISTOGRAM_LONG_RANGE);
  // sensor_vl53l4cx_sat2.VL53L4CX_get_preset_mode_timing_cfg(VL53L4CX_DEVICEPRESETMODE_HISTOGRAM_LONG_RANGE);


  // int status = sensor_vl53l4cx_sat.InitSensor(0x12);
  // if (status) {
  //   Serial.println("Init sensor_vl53l4cx_sat1 failed...");
  // }
  // status = sensor_vl53l4cx_sat2.InitSensor(0x30);
  // if (status) {
  //   Serial.println("Init sensor_vl53l4cx_sat2 failed...");
  // }
  sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();
  // sensor_vl53l4cx_sat2.VL53L4CX_StartMeasurement();

  // Constants for projections
  float beta = 20 * PI / 180;  // Convert to radians
  cos_mbeta = cos(PI / 2 - beta);
  sin_mbeta = sin(PI / 2 - beta);
  cos_pbeta = cos(PI / 2 + beta);
  sin_pbeta = sin(PI / 2 + beta);
  Serial.println("Ready to Go!");

  
}

void loop() {
  processSensorData(sensor_vl53l4cx_sat, dist_1, cos_mbeta, sin_mbeta);
  // processSensorData(sensor_vl53l4cx_sat2, dist_2, cos_pbeta, sin_pbeta);
  
  px1 = dist_1 * cos_mbeta;
  py1 = dist_1 * sin_mbeta;
  px2 = dist_2 * cos_pbeta;
  py2 = dist_2 * sin_pbeta;

  float a = (py1 - py2) / (px1 - px2);
  float b = py2 - a * px2;
  float ang = atan2(py1 - py2, px1 - px2);

  Serial.printf("%f , %f , %f , %f\n", dist_1, dist_2, b, ang);
  
  espSendData.flag = 1;
  espSendData.values[0] = b;
  espSendData.values[1] = ang*100;
  espSendData.values[2] = 0;
  espSendData.values[3] = 0;
  espSendData.values[4] = 0;
  espSendData.values[5] = 0;

  
  unsigned long startTime = micros();
  esp_now_send(broadcastAddress1, (uint8_t *)&espSendData, sizeof(espSendData));
  unsigned long elapsedTime = micros() - startTime;
  Serial.print("Elapsed time: ");
  Serial.println(elapsedTime);// digitalWrite(LedPin, LOW);
  // sleep(0.5);
  // delay(50);

}

void processSensorData(VL53L4CX& sensor, float& dist) {
  VL53L4CX_MultiRangingData_t MultiRangingData;
  uint8_t NewDataReady = 0;
  int status = 0;
  // int counter  = 0;
  //Serial.println("starting sensor measurement...");
  //do {
  status = sensor.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
    // counter += 1;
  //} while (!NewDataReady);
  // if (counter >= 10){
  //   sensor.VL53L4CX_ClearInterruptAndStartMeasurement();
  //   return;
  // }
  //Serial.println("recieved sensor measurement!");

  //digitalWrite(LedPin, HIGH);
  
  if ((NewDataReady != 0) && (!status)) {
    status = sensor.VL53L4CX_GetMultiRangingData(&MultiRangingData);
    int no_of_objects = MultiRangingData.NumberOfObjectsFound;
    if (no_of_objects > 0) {
      dist = MultiRangingData.RangeData[0].RangeMilliMeter;
    }else{
      dist = 6000;
    }
    
    if (status == 0) {
      status = sensor.VL53L4CX_ClearInterruptAndStartMeasurement();
    }
  }
}
