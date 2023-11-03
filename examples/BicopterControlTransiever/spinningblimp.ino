// creates the output values used for actuation from the control values
void SpinniggetOutputs(controller_t *controls, sensors_t *sensors, actuation_t *out)
{
  // Spinning Blimp
  s_yaw = sensors->yaw;
  float yaw_calibrate;
  // float joytheta = atan2(controls->fy,-controls->fx); 
  float joytheta = controls->tz; 
  float joymag  = controls->fx;

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
  st1 = 0;
  st2 = 0;

  out->s1 = 0;
  out->s2 = 0;
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
}
