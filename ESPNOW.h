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
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len);



class ESPNOW
{
private:
  int delayMS = 1000;

public:
  ESPNOW();
  void init();
  void setChannel(int set_channel);
  void getControllerInputs(controller_t *controls);
  void getControllerRaws(raw_t *raws);
};