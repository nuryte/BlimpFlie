// creates the output values used for actuation from the control values
void SpinniggetOutputs(controller_t *controls, sensors_t *sensors, actuation_t *out)
{
  out->ready = true;

  // Spinning Blimp

  float yaw_calibrate = 0;
  float joytheta = raws.data[5]; 
  float joymag   = raws.data[0];

  if (0 <= ((s_yaw + joytheta)) && ((s_yaw + joytheta)) < PI){
    tau = joymag;
  } else{
    tau = -joymag;
  }

  // sf1 and sf2 variables associated with the spinning blimp motor values
  // Bang Bang motor configuration

  // // For CW spinning blimp
  // sf1 = controls->fz + (tau); // LHS motor
  // sf2 = controls->fz - (tau); // RHS motor

  // For CCW spinning blimp
  sf1 = controls->fz - (tau); // LHS motor
  sf2 = controls->fz + (tau); // RHS motor

  // Motor outputs
  out->m1 = clamp(sf1, 0, 1);
  out->m2 = clamp(sf2, 0, 1);
}
