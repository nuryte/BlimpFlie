// creates the output values used for actuation from the control values
void BlendedgetOutputs(controller_t *controls, sensors_t *sensors, actuation_t *out)
{
  // Bicopter control
  
  // set up output

  // set output to default if controls not ready
  if (controls->ready == false)
  {
    out->s1 = 0.5f; // angle  is based on 0 > PI and 0.5 is based on the normalized value.
    out->s2 = 0.5f; // this portion means that 0.5 should be centered
    // out->s1 = PI * 0.35;
    // out->s2 = PI * 0.35;
    out->m1 = 0;
    out->m2 = 0;
    out->ready = false;
    return;
  }
  // Serial.print("I'm in the spinning section");
  // Serial.println(ss);
  // Serial.println(sensors->yaw);

  out->ready = true;
  // inputs to the A-Matrix
  float l = PDterms->lx; //.

  float fx = clamp(controls->fx, -1, 1);                  // setpoint->bicopter.fx;
  float fz = clamp(controls->fz, 0.1, 2);                 // setpoint->bicopter.fz;
  //float maxRadsYaw = .07; //.1f                                 //.175;
  //float magxz = max(fz * tan(maxRadsYaw), fx * l * 0.17f); // limits the yaw based on the magnitude of the force
  float taux = clamp(controls->tx, -l + (float)0.01, l - (float)0.01);
  float tauz = clamp(controls->tz, -1, 1) ;//* magxz; // limit should be .25 setpoint->bicopter.tauz; //- stateAttitudeRateYaw

  // inverse A-Matrix calculations
  float term1 = l * l * fx * fx + l * l * fz * fz + taux * taux + tauz * tauz;
  float term2 = 2 * fz * l * taux - 2 * fx * l * tauz;
  float term3 = sqrt(term1 + term2);
  float term4 = sqrt(term1 - term2);
  // bf and bt are variables associated with the spinning blimp
  bf1 = term3 / (2 * l); // in unknown units
  bf2 = term4 / (2 * l);
  bt1 = atan2((fz * l - taux) / term3, (fx * l + tauz) / term3) - sensors->pitch; // in radians
  bt2 = atan2((fz * l + taux) / term4, (fx * l - tauz) / term4) - sensors->pitch;
  // bt1 = PI * 0.35 + bt1;
  // bt2 = PI * 0.35 + bt2;

  // checking for full rotations
  while (bt1 < -PI / 2)
  {
    bt1 = bt1 + 2 * PI;
  }
  while (bt1 > 3 * PI / 2)
  {
    bt1 = bt1 - 2 * PI;
  }
  while (bt2 < -PI / 2)
  {
    bt2 = bt2 + 2 * PI;
  }
  while (bt2 > 3 * PI / 2)
  {
    bt2 = bt2 - 2 * PI;
  }

  // Spinning Blimp
  s_yaw = sensors->yaw;
  float yaw_calibrate;
  float joytheta = atan2(controls->fy,-controls->fx);  
  float joymag  = sqrt(pow(controls->fx,2) + pow(controls->fy,2));

  if ((s_yaw + joytheta) > PI + yaw_calibrate){
  joytheta = joytheta - 2*PI;
  } 
  else
    {if ((s_yaw + joytheta) < -1.0*PI + yaw_calibrate){
          joytheta = joytheta + 2*PI;
        }
    }

  // if (0 <= ((sensors->yaw + walk_heading)) && ((sensors->yaw + walk_heading)) < PI){
  if (0 <= ((s_yaw + joytheta)) && ((s_yaw + joytheta)) < PI){
    tau = joymag;
  } else{
    tau = -joymag;
  }
  // sf and st are variables associated with the spinning blimp
  sf1 = controls->fz + (tau); // LHS motor
  sf2 = controls->fz - (tau); // RHS motor
  st1 = PI * (0.16);
  st2 = PI * (1 - 0.16); 

  // Sigmoid Function
  // f(x) = 1 / ( 1+exp(-x*alpha)) (below is faster c++ version)
  float currentTime = millis() / 1000.0f; // Convert milliseconds to seconds
  sigmoi = updateFunction(currentTime);

  // Combination of the two states
  f1 = (1 - sigmoi) * bf1 + sigmoi * sf1;
  f2 = (1 - sigmoi) * bf2 + sigmoi * sf2;
  t1 = (1 - sigmoi) * bt1 + sigmoi * st1;
  t2 = (1 - sigmoi) * bt2 + sigmoi * st2;
  // Serial.print(ss);
  // Serial.print(" ,");
  // Serial.println(sigmoi);

  // converting values to a more stable form

  out->s1 = clamp(t1 + servo1offset, 0, PI) / (PI); // cant handle values between PI and 2PI
  out->s2 = clamp(t2 + servo2offset, 0, PI) / (PI);
  // out->s1 = clamp(t1 + servo1offset, 0, PI); // cant handle values between PI and 2PI
  // out->s2 = clamp(t2 + servo2offset, 0, PI);
  out->m1 = clamp(f1, 0, 1);
  out->m2 = clamp(f2, 0, 1);
  if (out->m1 < 0.02f)
  {
    out->s1 = 0.5f;
  }
  if (out->m2 < 0.02f)
  {
    out->s2 = 0.5f;
  }
  return;


// Example usage for time of flight:
// float* distArray = GetDistances(45, 123.4);
// for (int i = 0; i < ARRAY_SIZE; i++) {
//     Serial.println(distArray[i]);
// }

}

float updateFunction(float currentTime) {

    // Check if the state of ss has changed
    if (ss != lastState) {
        stateChangeTime = currentTime; // Update the starting time
        lastState = ss; // Update the last state
    }

    float x = currentTime - stateChangeTime;

    // If ss is 1, plot the sigmoid for up to 3 seconds
    if (ss == 1) {
        if (x >= 0 && x <= 5) { // Check if x is within the 0 to 3-second range
            sigmoi = 0.5 * (x * alpha / (1 + abs(x*alpha))) + 0.5;
        }
    }
    // If ss is 0, decrease the sigmoid value over time
    else if (ss == 0) {
        if (x >= 0 && x <= 5) { // Check if x is within the 0 to 3-second range
            sigmoi = 1.0 - (0.5 * (x * alpha / (1 + abs(x*alpha))) + 0.5);
        }
    }

    return sigmoi;  // Return the sigmoid value
}
