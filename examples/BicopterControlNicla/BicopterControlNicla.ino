#include "modBlimp.h"
#include "BNO85.h"
#include "baro390.h"


ModBlimp blimp;
BNO85 bno;
baro390 baro;

//IBusBM IBus; 

/*
flags to be used in the init 
-bool verbose: allows some debug print statments
-bool sensors: enables or disables the sensorsuite package: if false all values will be 0, and sensorReady =false in the sensor 
-bool UDP: starts up the UDP connection such that other UDP functions will be enabled
-int motor_type: determines if you are using brushless or brushed motors: 0 = brushless, 1 = brushed;
-int mode: sets which controller to listen to: 0 = UDP, 1 = IBUS,2 = espnow, -1 = None;
-int control: sets which type of controller to use: 0 = bicopter, 1 = spinning(TODO),2 = s-blimp, -1 = None;
*/
init_flags_t init_flags = {
  .verbose = false,
  .sensors = false,
  .escarm = false,
  .UDP = false,
  .Ibus = true,
  .ESPNOW = true,
  .PORT = 1345,
  .motor_type = 0,
  .mode = 2,
  .control = 0,
};


/*
sensor values that control the sensors - if you want to turn off sensors use init_flags (sensors = false)
- float Kacc: kp value in implementation for accelerometer
- float Kgyro: kp value in implementation for gyroscope
- float Kmag: kp value in implementation for magnetometer
- bool baro: enables or disables barometer
- float eulerGamma: is the weight of the weighted average on euler angles (0 means that it always uses the newest value)
- float rateGamma: is the weight of the weighted average on gyroscope rates (0 means that it always uses the newest value)
- float zGamma: is the weight of the weighted average on estimatedZ (0 means that it always uses the newest value)
*/
init_sensors_t init_sensors = {
  .Kacc = 5,
  .Kgyro = -1,
  .Kmag = 0,
  .baro = true,
  .eulerGamma = 0,
  .rateGamma = 0.9f,
  .zGamma = 0.9f,
};

/*
sensor values that control the sensors - if you want to turn off sensors use init_flags (sensors = false)
- float Kacc: kp value in implementation for accelerometer
- float Kgyro: kp value in implementation for gyroscope
- float Kmag: kp value in implementation for magnetometer
- bool baro: enables or disables barometer
- float eulerGamma: is the weight of the weighted average on euler angles (0 means that it always uses the newest value)
- float rateGamma: is the weight of the weighted average on gyroscope rates (0 means that it always uses the newest value)
- float zGamma: is the weight of the weighted average on estimatedZ (0 means that it always uses the newest value)
*/
sensor_weights_t weights = {
  .eulerGamma = 0.0f,
  .rollRateGamma = 0.7f,
  .yawRateGamma = 0.7f,
  .pitchRateGamma = 0.7f,
  .zGamma = 0.9f,
  .vzGamma = 0.9f,
};


/*
PD terms for use in the feedback controller 
- bool roll, pitch, yaw, x, y, z, rotation: 
          enables that type of feedback (true means feedback is on for that state variable)
- float Croll, Cpitch, Cyaw, Cx, Cy, Cz, Cabsz: 
          KP term applied to the controller input
- float kproll, kdroll, kppitch, kdpitch, kpyaw, kdyaw: 
- float kpx, kdx, kpy, kdy, kpz, kdz;
          Kp and kd terms applied to each feedback mechanism using the sensors 
          (some do not have sensor availibility like x and y)
- float lx;
          a control variable used as the arm between the center of mass and the propellers
*/
feedback_t feedbackPD = {
  .roll = false,
  .pitch = false, 
  .yaw = false,
  .x = false,
  .y = false,
  .z = false,
  .rotation = false,

  .Croll = 1,
  .Cpitch = 0, 
  .Cyaw = 1,
  .Cx = 1,
  .Cy = 0,
  .Cz = 1,
  .Cabsz = 1,

  .kproll = 0,
  .kdroll = 0.0f,
  .kppitch = 0,
  .kdpitch = 0,
  .kpyaw = 3.0f,
  .kdyaw = -150.0f,//-70//5f,

  .kpx = 0,
  .kdx = 0,
  .kpy = 0,
  .kdy = 0,
  .kpz = 0.1f,//.4f
  .kdz = -0.5f,

  .lx = .15,
};
// EXTRA TERMS
RollPitchAdjustments rollPitchAdjust = {
  .rollPitchSwitch = false,
  .pitchSign = 1,
  .pitchOffset = 0,
  .rollSign = 1,
  .rollOffset = 0,
};

//active feedback terms

float z_integral = 0;

float yawrate_integral = 0;
float yaw_integral = 0;

//z feedback terms
float kiz = 0;
float z_int_low = 0;
float z_int_high = 50;

//yaw feedback terms
float kiyaw = 0;
float kiyawrate = 0;
float yawRateIntegralRange = 10;

//motor tuning terms
float kf1 = 1;
float kf2 = 1;



//passive term
float servo1offset = 0;
float servo2offset = 0;

//yawrate stuff
float fxyawScale = 0;
float maxRadsYaw = .07; 
bool yawScaleEnable = true;

//sender feedback 
bool transceiverEnabled = false;
uint8_t transceiverAddress[6];
ReceivedData espSendData1;
ReceivedData espSendData2;
float battery_level = 0;

feedback_t * PDterms = &feedbackPD;
//storage variables
sensors_t sensors;
controller_t controls;
raw_t raws;
actuation_t outputs;



int dt; //microseconds
//HardwareSerial MySerial0(0);
void setup() {
  
    
  //initializes systems based on flags and saves flags into the system
  blimp.init(&init_flags, &init_sensors, &feedbackPD);
    
  delay(100);
  // baro.init();
  //bno.init();


  getLatestSensorData(&sensors);
  sensors.groundZ = baro.getEstimatedZ();


}
float absoluteyawave = 0;
bool snapon = 0;
// float resyncPitch = 0.09;
// float resyncPitchTemp = 0;
// float resyncTimer = 0;
unsigned long timed = micros();
int lastflag = 0;
bool nicla_state = false;
int counter2 = 0;
int countSwitch = 0;
void loop() {

  

  /*
  //    attempts to get the lastest information about the CONTROLLER and places them into the 
  //    raw_t data structure
  //    contains: flag, ready, data[11]
  */
  blimp.getControllerRaws(&raws);



  /*
  if flag = 0: normal control logic
  if flag = 1 or 2: use old magnetometer calibration
  if flag = 10, 11 or 12: do flag changes // flag changes should turn off or on sensor feedback as well as PID controls
  if flag = 20: do low level control
  if flag = 21: do high level control (same as 0)
  if flag = 22: do nicla low level control
  if flag = 23: do nicla high level control
  */
  
  int flag = raws.flag;
  getLatestSensorData(&sensors);
  blimp.IBus.loop();
  

  if ((int)(flag/10) == 0){// flag == 0, 1, 2uses control of what used to be the correct way
    outputs.ready = false;
    battery_level = blimp.executeOutputs(&outputs);
    return; //changes outputs using the old format
  } else if ((int)(flag/10) == 1){ //flag == 10, 11, 12
    //set FLAGS for other stuff
    setPDflags(&init_flags, PDterms,&weights, &raws, &rollPitchAdjust);
    outputs.m1 = 0;
    outputs.m2 = 0;
    outputs.s1 = 0;
    outputs.s2 = 0;
    outputs.ready = false;
    z_integral = 0;
    yawrate_integral = 0;
    yaw_integral = 0;
    
  } else if (flag == 20 or flag == 22){ // low level control
    if (flag == 20){
      outputs.ready = raws.ready;
      outputs.m1 = raws.data[0];
      outputs.m2 = raws.data[1];
      outputs.s1 = raws.data[2];
      outputs.s2 = raws.data[3];
    } else { //nicla control
      blimp.IBus.loop();
      outputs.ready = raws.ready;
      outputs.m1 = (float)blimp.IBus.readChannel(0)/1000.0f;
      outputs.m2 = (float)blimp.IBus.readChannel(1)/1000.0f;
      outputs.s1 = (float)blimp.IBus.readChannel(2)/1000.0f;
      outputs.s2 = (float)blimp.IBus.readChannel(3)/1000.0f;

    }

  }
  else if (flag == 21 or flag == 23){ // high level control
    if (flag == 21){
      controls.ready = raws.ready;
      controls.fx = raws.data[0];
      controls.fy = raws.data[1];
      controls.fz = raws.data[2];
      controls.tx = raws.data[3];
      controls.ty = raws.data[4];
      controls.tz = raws.data[5];
      controls.absz = raws.data[6];
      nicla_state = (bool)raws.data[7];
      
    } else { //nicla control
      blimp.IBus.loop();
      controls.ready = raws.ready;
      controls.fx = (float)blimp.IBus.readChannel(0)/1000.0f;
      controls.fy = (float)blimp.IBus.readChannel(1)/1000.0f;
      controls.fz = (float)blimp.IBus.readChannel(2)/1000.0f;
      controls.tx = (float)blimp.IBus.readChannel(3)/1000.0f;
      controls.ty = (float)blimp.IBus.readChannel(4)/1000.0f;
      controls.tz = (float)blimp.IBus.readChannel(5)/1000.0f;
      controls.absz = (float)blimp.IBus.readChannel(6)/1000.0f;
    } 
    if (nicla_state){
      addNiclaControl(&controls, &sensors, &blimp);
    }
    addFeedback(&controls, &sensors); //this function is implemented here for you to customize
    getOutputs(&controls, &sensors, &outputs);

  }
  else if (flag == 98 && lastflag != flag){
    Serial.print("Set flags: ");
    Serial.println(flag);
    baro.init();
    getLatestSensorData(&sensors);
    delay(30);
    sensors.groundZ = baro.getEstimatedZ();
    delay(30);
    getLatestSensorData(&sensors);
    int tempcount = 0;
    while (abs(sensors.groundZ - baro.getEstimatedZ()) > .4 || sensors.groundZ == baro.getEstimatedZ()){
      Serial.print("Ground Cal...");
      Serial.println(sensors.groundZ - baro.getEstimatedZ());
      if (tempcount >10){
        break;
      }
      tempcount += 1;
      sensors.groundZ = baro.getEstimatedZ();
      delay(100);

      getLatestSensorData(&sensors);
    }
    Serial.print("Ground Calibrated to: ");
    Serial.println(sensors.groundZ);
  }
  else if (flag == 97 && lastflag != flag){
    Serial.print("Set flags: ");
    Serial.println(flag);
    bno.init();
    
    //getLatestSensorData(&sensors);
  } 
  battery_level = blimp.executeOutputs(&outputs);
  dt = (int)(micros()-timed);
  while (4000 - dt > 0){
    dt = (int)(micros()-timed);
  }
  timed = micros();
  counter2 += 1;
  
  
  if (counter2 >= 25){
    Serial.print(dt);
    Serial.print(',');
    Serial.print((bool)controls.ready);
    Serial.print(',');
    Serial.print(sensors.estimatedZ - sensors.groundZ);
    Serial.print(',');
    Serial.print(sensors.yaw);
    Serial.print(',');
    Serial.print(battery_level);
    Serial.print(',');
    Serial.print(espSendData2.values[2]);
    Serial.print(',');
    Serial.print(espSendData2.values[3]);
    Serial.print(',');
    Serial.print(espSendData2.values[0]);
    Serial.print(',');
    Serial.println(espSendData2.values[1]);
    counter2 = 0;
    if (transceiverEnabled){
      
      espSendData1.flag = 1;
      espSendData1.values[0] = sensors.estimatedZ - sensors.groundZ;
      espSendData1.values[1] = sensors.yaw;
      espSendData1.values[2] = (float)blimp.IBus.readChannel(0)/1000.0f;
      espSendData1.values[3] = (float)blimp.IBus.readChannel(1)/1000.0f;
      espSendData1.values[4] = (float)blimp.IBus.readChannel(2)/1000.0f;
      espSendData1.values[5] = (float)blimp.IBus.readChannel(3)/1000.0f;      
      blimp.send_esp_feedback(transceiverAddress, &espSendData1);
      espSendData2.flag = 2;
      espSendData2.values[0] = outputs.m1;
      espSendData2.values[1] = outputs.m2;
      espSendData2.values[2] = outputs.s1;
      espSendData2.values[3] = outputs.s2;
      espSendData2.values[4] = controls.tz;
      espSendData2.values[5] = battery_level;      
      blimp.send_esp_feedback(transceiverAddress, &espSendData2);

    }
  }
  lastflag = flag;


}


void getLatestSensorData(sensors_t *sensors) {
  
  bno.updateSensors(sensors, &weights, &rollPitchAdjust);
  if (baro.updateBarometer())
  {
    sensors->estimatedZ = sensors->estimatedZ * weights.zGamma  + baro.getEstimatedZ()* (1 - weights.zGamma);
    sensors->velocityZ = sensors->velocityZ * weights.vzGamma + baro.getVelocityZ()*(1 - weights.zGamma);
  }
}



float clamp(float in, float min, float max){
  if (in< min){
    return min;
  } else if (in > max){
    return max;
  } else {
    return in;
  }
}




