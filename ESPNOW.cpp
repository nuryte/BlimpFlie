/*
 * @Author       : Hanqing Qi
 * @Date         : 2023-08-01 15:54:19
 * @LastEditors  : Hanqing Qi
 * @LastEditTime : 2023-08-01 17:35:04
 * @FilePath     : /undefined/Users/hanqingqi/Desktop/sensfusion_10DOF/ESPNOW.cpp
 * @Description  : This is the file for the ESPNOW class
 */

#include <ESPNOW.h>

volatile bool esp_ready;
volatile unsigned long esp_time_now;
Control_Input ESPNOW_Input;

typedef struct ControlInput
{
  float params[NUM_CONTROL_PARAMS];
  int channel; // The channel to broadcast on
} ControlInput;

// Callback when data is received
void ESPNOW::OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  
  ControlInput *incomingData = (ControlInput *)data; // Cast data to our structure type
  
  if (incomingData->channel == -1) // Check if the data is P2P
  {
    Serial.print("Packet from: ");
    Serial.println(macStr);
    Serial.print("Control params: ");
    for (int i = 0; i < NUM_CONTROL_PARAMS; i++)
    {
      Serial.print(incomingData->params[i]);
      if (i < NUM_CONTROL_PARAMS - 1)
      {
        Serial.print(", ");
      }
    }
    esp_time_now = millis();
    esp_ready = false;
    memcpy(&ESPNOW_Input, incomingData, sizeof(ESPNOW_Input));
    esp_ready = true;
    Serial.println("\tListening from P2P");
  }else if (incomingData->channel == CHANNEL){ // Check if the data is broadcast
    Serial.print("Packet from: ");
    Serial.println(macStr);
    Serial.print("Control params: ");
    for (int i = 0; i < NUM_CONTROL_PARAMS; i++)
    {
      Serial.print(incomingData->params[i]);
      if (i < NUM_CONTROL_PARAMS - 1)
      {
        Serial.print(", ");
      }
    }
    esp_time_now = millis();
    esp_ready = false;
    memcpy(&ESPNOW_Input, incomingData, sizeof(ESPNOW_Input));
    esp_ready = true;
    Serial.print("\tListening on channel: ");
    Serial.println(incomingData->channel);
  }
  else
  {
    Serial.println("Data received on an unexpected channel. Ignoring.");
  }
}

void ESPNOW::init()
{
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    Serial.print("ESP Board MAC Address:  ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }else{
        Serial.println("ESP-NOW initialized");
    }

    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(OnDataRecv);
    esp_time_now = millis();
}

/**
 * @description: This function gets the controller inputs from the ESPNOW
 * @param       {controller_t} *controls:
 * @return      {*}
 */
void ESPNOW::getControllerInputs(controller_t *controls)
{

    // Serial.print(esp_ready);
    // Serial.print(',');
    // Serial.println(millis() - esp_time_now);
    if (esp_ready && millis() - esp_time_now < delayMS)
    {
        controls->flag = (int)ESPNOW_Input.params[0];
        controls->fx = ESPNOW_Input.params[1];
        controls->fy = ESPNOW_Input.params[2];
        controls->fz = ESPNOW_Input.params[3];
        controls->tx = ESPNOW_Input.params[4];
        controls->ty = ESPNOW_Input.params[5];
        controls->tz = ESPNOW_Input.params[6];
        controls->absz = ESPNOW_Input.params[7];
        controls->ready = (bool)((int)(ESPNOW_Input.params[8]) != 0);
        controls->snapshot = (int)ESPNOW_Input.params[9];
    }
    else
    {
        controls->ready = false;
    }
    return;
}

/**
 * @description: This function gets the controller inputs from the ESPNOW
 * @param       {raw_t} *controls:
 * @return      {*}
 */
void ESPNOW::getControllerRaws(raw_t *raws)
{

    // Serial.print(esp_ready);
    // Serial.print(',');
    // Serial.println(millis() - esp_time_now);
    if (esp_ready && millis() - esp_time_now < delayMS)
    {
        raws->flag = (int)ESPNOW_Input.params[0];
        raws->ready = (bool)((int)ESPNOW_Input.params[1] == 1);
        for (int x = 0; x < 11; x++) {
            raws->data[0] = ESPNOW_Input.params[x+2];
        }
        
    }
    else
    {
        raws->ready = false;
    }
    return;
}