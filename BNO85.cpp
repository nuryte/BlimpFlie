#include "BNO85.h"

BNO85::BNO85(){
    
  //pass
    
    
}

void BNO85::init(){
  if (bnoOn){
    return;
  }
  Wire.begin();
  /* Initialise the sensor */
  int tempcount = 0;
  while (myIMU.begin(0x4A, Wire) == false)
  {
    tempcount += 1;
    delay(50);
    if (tempcount > 20){
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO085 detected ... Check your wiring or I2C ADDR!");
      return;
    }
    
  }
  
  Serial.println("BNO started!");
  bnoOn = true;
  Wire.setClock(400000);
    
  setReports();
  
  

}

// Here is where you define the sensor outputs you want to receive
void BNO85::setReports(void) {
  Serial.println("Setting desired reports");
  if (myIMU.enableRotationVector() == true) {
    Serial.println(F("Rotation vector enabled"));
    Serial.println(F("Output in form roll, pitch, yaw"));
  } else {
    Serial.println("Could not enable rotation vector");
  }
  if (myIMU.enableGyro() == true) {
    Serial.println(F("Gyro enabled"));
    Serial.println(F("Output in form x, y, z, in radians per second"));
  } else {
    Serial.println("Could not enable gyro");
  }
}

void BNO85::updateSensors(sensors_t *sensors, sensor_weights_t *weights){
    // Serial.println("BNO Update!");
  //bnoOn = false;
  if (bnoOn){
    if (myIMU.wasReset()) {
      Serial.println("sensor was reset ");
      setReports();
    }
    while (myIMU.getSensorEvent() == true) {
    
      // is it the correct sensor data we want?
      if (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {

      sensors->roll = (myIMU.getRoll()) ; // In radians
      sensors->pitch = (myIMU.getPitch()); // In radians
      sensors->yaw = (myIMU.getYaw()) ; // In radians
      
      if (sensors->yaw > 3.1416f){
        sensors->yaw -= 3.1416f*2;
      }
      
      }
      // is it the correct sensor data we want?
      if (myIMU.getSensorEventID() == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {

      
      
        sensors->yawrate = sensors->yawrate *weights->yawRateGamma + myIMU.getGyroZ()* (1- weights->yawRateGamma);
        sensors->rollrate = sensors->rollrate *weights->rollRateGamma+ myIMU.getGyroY()* (1- weights->rollRateGamma);
        sensors->pitchrate = sensors->pitchrate *weights->pitchRateGamma+  myIMU.getGyroX()* (1- weights->pitchRateGamma);
      }
    
    }
  } else {
    sensors->yaw = 0;
    sensors->roll = 0;
    sensors->pitch = 0;
    
    sensors->yawrate = 0;
    sensors->rollrate = 0;
    sensors->pitchrate = 0;
  }
}
