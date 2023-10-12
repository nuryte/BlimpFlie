/*
 * @Author       : Hanqing Qi
 * @Date         : 2023-08-01 15:54:11
 * @LastEditors  : Hanqing Qi
 * @LastEditTime : 2023-08-01 16:47:12
 * @FilePath     : /sensfusion_10DOF-main/ESPNOW.h
 * @Description  : Header file for ESPNOW.cpp
 */

#include "WiFi.h"
#include <esp_now.h> // This is the arduino library for ESP-NOW
#include <data_types.h>
void ESPNOW_DataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);

// Receiver side data structures
const int NUM_CONTROL_PARAMS = 13; // Number of parameters used for control

typedef struct ControlInput
{
  float params[NUM_CONTROL_PARAMS];
  int channel; // The channel to broadcast on
} ControlInput;

class ESPNOW
{
private:
  int delayMS = 1000;
  int CHANNEL = 1;

public:
  ESPNOW();
  void init();
  void getControllerInputs(controller_t *controls);
  void getControllerRaws(raw_t *raws);
};