#include "modBlimp.h"

#include "BNO85.h"
#include "baro390.h"
#include "GY_US42V2.h"  // Include the header file

// #include "BNO55.h"
// #include "baro280.h"

ModBlimp blimp;
BNO85 bno;
baro390 baro;
GY_US42V2 sonar_sensor;  // Create an instance of the GY_US42V2 class


IBusBM IBus;


robot_specs_s robot_specs = {
        .min_thrust = 1000,
        .max_thrust = 2000,
};

/*
flags to be used in the init
-bool verbose: allows some debug print statments
-bool sensors: enables or disables the sensorsuite package: if false all values will be 0, and sensorReady =false in the sensor
-bool UDP: starts up the UDP connection such that other UDP functions will be enabled
-bool servo: switching between 180 and 270 degree: false will be 180 degree and true will be 270
-int motor_type: determines if you are using brushless or brushed motors: 0 = brushless, 1 = brushed;
-int mode: sets which controller to listen to: 0 = UDP, 1 = IBUS,2 = espnow, -1 = None;
-int control: sets which type of controller to use: 0 = bicopter, 1 = spinning(TODO),2 = s-blimp, -1 = None;
*/
init_flags_t init_flags = {
        .verbose = false,
        .sensors = false,
        .escarm = true,
        .calibrate_esc = false,
        .UDP = false,
        .Ibus = false,
        .ESPNOW = true,
        .servo = false,
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

float servo1offset = 0;
float servo2offset = 0;
//sender feedback
bool transceiverEnabled = false;
uint8_t transceiverAddress[6];
ReceivedData espSendData1;
ReceivedData espSendData2;
float battery_level = 0;
ReceivedData sensorData;

feedback_t * PDterms = &feedbackPD;
//storage variables
sensors_t sensors;
controller_t controls;
raw_t raws;
actuation_t outputs;

int actionFlag = 0;

int dt; //microseconds
// Spinning Blimp Variables
float sf1, sf2, bf1, bf2 = 0;
float st1, st2, bt1, bt2 = 0;
float f1, f2, t1, t2 = 0;
float sigmoi, s_yaw, tau, ss = 0;
float alpha = 1;
float lastState = -1; // Initialized to a value that is not 0 or 1 to ensure the initial check works
unsigned long stateChangeTime = 0; // Time at which controls->ss changes state
// Time of flight sensor values
float wall = 0;
const int ANGLE_INCREMENT = 10;
const int TOTAL_ANGLES = 360;
const int ARRAY_SIZE = TOTAL_ANGLES / ANGLE_INCREMENT;

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

int counter2 = 0;
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
  blimp.getSensorRaws(&sensorData); //reading from Ultrasound wireless
  


  int sonar_sensor_enabled = 1;  //FIXME make this a flag

  if(sonar_sensor_enabled){
    sensorData.values[0] = sonar_sensor.readDistance();  // Read distance from sensor
  }


  if ((int)(flag/10) == 0){// flag == 0, 1, 2 uses control of what used to be the correct way
    zero(init_flags.servo, &outputs); // call zeroing function for servo
    battery_level = blimp.executeOutputs(&outputs, &robot_specs);
    return; //changes outputs using the old format
  } else if ((int)(flag/10) == 1 or (int)(flag/10) == 9){ //flag in 10, or 90
    //set FLAGS for other stuff
    setPDflags(&init_flags, PDterms,&weights, &raws, &rollPitchAdjust);
    outputs.m1 = 0;
    outputs.m2 = 0;
    outputs.s1 = 0;
    outputs.s2 = 0;
    outputs.ready = false;
    z_integral = 0;

  } else if (flag == 20){ //motor servo level control
          outputs.ready = raws.ready;
          outputs.m1 = raws.data[0];
          outputs.m2 = raws.data[1];
          outputs.s1 = raws.data[2];
          outputs.s2 = raws.data[3];
      

  }
  else if (flag == 21){ // A-matrix high level control
  
    controls.ready = raws.ready;
    controls.fx = raws.data[0];
    controls.fy = raws.data[1];
    controls.fz = raws.data[2];
    controls.tx = raws.data[3];
    controls.ty = raws.data[4];
    controls.tz = raws.data[5];
    controls.absz = raws.data[6];
    actionFlag = (int)raws.data[7]; // for the spinning blimp switch states

    if (actionFlag == 1){// nicla controller
      addNiclaControl(&controls, &sensors, &blimp);
    } else if (actionFlag == 2) { //random walk
      actionFlag = 0;// put random walk here
    } else if (actionFlag == 3) { // full control flow
      actionFlag = 0; // put control flow function to do state control here. 
    } //else just use joystick controls
    addFeedback(&controls, &sensors); //this function is implemented here for you to customize

    // Init flags to select which getOutput function is selected
    if (init_flags.servo == 0){
        // 180 degree servo getOutputs
        getOutputs(&controls, &sensors, &outputs);
    } else {
        // 270 degree servo getOutputs
        getOutputs270(&controls, &sensors, &outputs);
    }

  }
  
  battery_level = blimp.executeOutputs(&outputs, &robot_specs);
  int dt = (int)(micros()-timed);
  while (4000 - dt > 0){
      dt = (int)(micros()-timed);
  }
  lastflag = flag;
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
      espSendData2.values[2] = outputs.m1;
      espSendData2.values[3] = outputs.m2;
      espSendData2.values[4] = outputs.s1;
      espSendData2.values[5] = outputs.s2;
      blimp.send_esp_feedback(transceiverAddress, &espSendData1);
      // espSendData2.flag = 2;
      // espSendData1.values[2] = (float)blimp.IBus.readChannel(0)/1000.0f;
      // espSendData1.values[3] = (float)blimp.IBus.readChannel(1)/1000.0f;
      // espSendData1.values[4] = (float)blimp.IBus.readChannel(2)/1000.0f;
      // espSendData1.values[5] = (float)blimp.IBus.readChannel(3)/1000.0f;     
      // espSendData2.values[0] = outputs.m1;
      // espSendData2.values[1] = outputs.m2;
      // espSendData2.values[2] = outputs.s1;
      // espSendData2.values[3] = outputs.s2;
      // espSendData2.values[4] = controls.tz;
      // espSendData2.values[5] = battery_level;      
      // blimp.send_esp_feedback(transceiverAddress, &espSendData2);
    }
  }


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

// Function for initialising the servos whether they're 180 or 270 degree variants
void zero(bool servo, actuation_t *out){
    if (servo == 0){
        // 180 servo
        out->s1 = .5f;
        out->s2 = .5f;
        out->m1 = 0;
        out->m2 = 0;
        out->ready = false;
    } else {
        // 270 servo
        out->s1 = 0.33f;
        out->s2 = 0.33f;
        out->m1 = 0;
        out->m2 = 0;
        out->ready = false;
    }
}

