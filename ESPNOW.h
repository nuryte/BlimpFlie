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
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);



class ESPNOW
{
private:
  int delayMS = 1000;
  bool isPeerAlreadyAdded(const uint8_t *mac_addr);

public:
  ESPNOW();
  void init();
  void setChannel(int set_channel);
  void getControllerInputs(controller_t *controls);
  void getControllerRaws(raw_t *raws);
  esp_err_t attemptToAddPeer(uint8_t mac_addr[6]);
  void sendResponse(uint8_t mac_addr[6], ReceivedData *responseData);
  void getSensorRaws(ReceivedData *sensorData);
};