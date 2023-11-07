
#include <ESPNOW.h>

// Receiver side data structures
const int NUM_CONTROL_PARAMS = 13; // Number of parameters used for control
volatile int CHANNEL = 1;
volatile bool esp_ready;
volatile bool esp_sensor_ready;
volatile bool verbose = false;
volatile unsigned long esp_time_now;


ControlInput ESPNOW_Input;
ReceivedData ESPNOW_ReceivedData;

// Callback when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  char macStr[18];
  if (verbose) {
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  }
  if (data_len == sizeof(ControlInput))
  {
    ControlInput *incomingData = (ControlInput *)data;
      
    if (incomingData->channel == -1) // Check if the data is P2P
    {
      if (verbose) {
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
        Serial.println("\tListening from P2P");
      }
      esp_time_now = millis();
      esp_ready = false;
      memcpy(&ESPNOW_Input, incomingData, sizeof(ESPNOW_Input));
      esp_ready = true;
    }else if (incomingData->channel == CHANNEL){ // Check if the data is broadcast
      if (verbose) {
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
        Serial.print("\tListening on channel: ");
        Serial.println(incomingData->channel);
      }
      esp_time_now = millis();
      esp_ready = false;
      memcpy(&ESPNOW_Input, incomingData, sizeof(ESPNOW_Input));
      esp_ready = true;
    }
    else
    {
      Serial.println("Data received on an unexpected channel. Ignoring.");
    }
  }
  else if (data_len == sizeof(ReceivedData))
  {
    ReceivedData *incomingSensorData = (ReceivedData *)data;
    // Process the ReceivedData as needed
    memcpy(&ESPNOW_ReceivedData, incomingSensorData, sizeof(ESPNOW_ReceivedData));
    esp_sensor_ready = true;
  }
  else {
    Serial.println("Received data of unexpected size. Ignoring.");
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (verbose)
  {
    Serial.print(" Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  }
}

ESPNOW::ESPNOW(){
    for (int x = 0; x < 13; x++) {
        ESPNOW_Input.params[x] = 0;
    }
    esp_ready = false;
}

void ESPNOW::setChannel(int set_channel)
{
    CHANNEL = set_channel;
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
  //esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  
  esp_time_now = millis();
}

void ESPNOW::getSensorRaws(ReceivedData *sensorData)
{
  if (esp_sensor_ready){
    memcpy(sensorData, &ESPNOW_ReceivedData, sizeof(ESPNOW_ReceivedData));
  }
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
            raws->data[x] = ESPNOW_Input.params[x+2];
        }
        
    }
    else
    {
        raws->ready = false;
    }
    return;
}


void ESPNOW::sendResponse(uint8_t mac_addr[6], ReceivedData *responseData) {
    // Serial.print(mac_addr[0]);
    // Serial.print(":");
    // Serial.print(mac_addr[1]);
    // Serial.print(":");
    // Serial.print(mac_addr[2]);
    // Serial.print(":");
    // Serial.print(mac_addr[3]);
    // Serial.print(":");
    // Serial.print(mac_addr[4]);
    // Serial.print(":");
    // Serial.print(mac_addr[5]);


    
    // ReceivedData localData;
    // memset(&localData, 0, sizeof(ReceivedData)); 
    // memcpy(&localData, responseData, sizeof(ReceivedData));
    
    esp_err_t result = esp_now_send(mac_addr, (uint8_t *)responseData, sizeof(ReceivedData));
    // if (result == ESP_OK) {
    //     Serial.println(" Sent response successfully");
    // } else {
    //     Serial.println(" Error sending response");
    // }
}

bool ESPNOW::isPeerAlreadyAdded(const uint8_t *mac_addr) {
    // esp_now_peer_info_t *peer_list = NULL;
    // uint8_t peer_count = 0;

    // if (esp_now_get_peer_num(&peer_count) != ESP_OK) {
    //     return false; // Error getting peer count
    // }

    // if (peer_count == 0) {
    //     return false; // No peers exist
    // }

    // peer_list = (esp_now_peer_info_t *)malloc(sizeof(esp_now_peer_info_t) * peer_count);
    // if (peer_list == NULL) {
    //     return false; // Memory allocation failed
    // }

    // if (esp_now_get_peer_list(peer_list, &peer_count) != ESP_OK) {
    //     free(peer_list);
    //     return false; // Error getting peer list
    // }

    // for (int i = 0; i < peer_count; i++) {
    //     if (memcmp(peer_list[i].peer_addr, mac_addr, 6) == 0) {
    //         free(peer_list);
    //         return true; // Found a matching MAC address
    //     }
    // }

    // free(peer_list);
    // return false; // No matching MAC address found
    return false;
}

esp_err_t ESPNOW::attemptToAddPeer(uint8_t mac_addr[6]) {
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo)); // Initialize peerInfo structure to zero
    
    // Set default properties for peerInfo here. For example:
    peerInfo.channel = 0;  // Default to Wi-Fi channel 0
    peerInfo.encrypt = false; // No encryption by default

    if (!isPeerAlreadyAdded(mac_addr)) {
        memcpy(peerInfo.peer_addr, mac_addr, 6); 
        return esp_now_add_peer(&peerInfo);
    } else {
        return ESP_FAIL; // Indicate that the peer already exists
    }
}