#include "modBlimp.h"


ModBlimp blimp;


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
  .calibrate_esc = true,
  .UDP = false,
  .Ibus = false,
  .ESPNOW = true,
  .PORT = 1345,
  .motor_type = 0,
  .mode = 2,
  .control = 0,
};

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

init_sensors_t init_sensors = {
  .Kacc = 5,
  .Kgyro = -1,
  .Kmag = 0,
  .baro = true,
  .eulerGamma = 0,
  .rateGamma = 0.9f,
  .zGamma = 0.9f,
};

void setup() {
  //initializes systems based on flags and saves flags into the system
  blimp.init(&init_flags, &init_sensors, &feedbackPD);

}


void loop() {
}



