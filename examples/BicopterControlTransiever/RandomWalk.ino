


void Sonar_sensor(controller_t *controls, int sonar_sensor_enabled, int randomWalk_enabled){

    if(sonar_sensor_enabled){
      sensorData.values[0] = sonar_sensor.readDistance();  // Read distance from sensor
      if(randomWalk_enabled){
        randomWalk.execute(sensorData.values[0], randomW_force, randomW_z, randomW_yaw);
        
        // Serial.print("sonar_sensor: ");
        // Serial.print(sensorData.values[0]);
        

        controls->fx = -randomW_force;
        // controls->fz = randomW_z;
       controls->tz += randomW_yaw;

        // Serial.print(" randomW_force: ");
        // Serial.print(controls->fx);
        // Serial.print(" randomW_z: ");
        // Serial.print(controls->fz);
        // Serial.print(" randomW_z: ");
        // Serial.print(controls->tz);
        // Serial.println("Excute");

      }else{
        // Serial.print(" randomW_force: ");
        // Serial.print(controls->fx);
        // Serial.print(" randomW_z: ");
        // Serial.print(controls->fz);
        // Serial.print(" randomW_z: ");
        // Serial.print(controls->tz);
        // Serial.println("No Excute");
        }
    }

}