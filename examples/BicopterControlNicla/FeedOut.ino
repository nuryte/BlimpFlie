

//adds sensor feedback into the control values
void addFeedback(controller_t *controls, sensors_t *sensors) {
    //controller weights
    controls->fx *= PDterms->Cx;
    controls->fy *= PDterms->Cy;
    controls->fz *= PDterms->Cz;
    controls->tx *= PDterms->Croll;
    controls->ty *= PDterms->Cpitch;
    controls->tz *= PDterms->Cyaw;
    controls->absz *= PDterms->Cabsz;

    //z feedback 
    if (PDterms->z) {
      if (controls->ready){
        z_integral += (controls->fz + controls->absz - (sensors->estimatedZ-sensors->groundZ)) * ((float)dt)/1000000.0f * kiz;
        z_integral = clamp(z_integral, z_int_low, z_int_high);
        //Serial.println(z_integral);
      } 
      controls->fz = (controls->fz + controls->absz - (sensors->estimatedZ-sensors->groundZ))*PDterms->kpz 
                      - (sensors->velocityZ)*PDterms->kdz + (z_integral);
      
      // fzave = fzave * .9 + controls->fz * .1;
      // controls->fz = fzave;
    }
    
    //yaw feedback
    if (PDterms->yaw) { 

      // Computing error between angles
      float e_yaw = controls->tz - sensors->yaw;
      e_yaw = atan2(sin(e_yaw), cos(e_yaw));
      e_yaw = clamp(e_yaw, -PI/3, PI/3);

      if (false){
        yaw_integral += e_yaw *((float)dt)/1000000.0f* kiyaw;
        
        yaw_integral = clamp(yaw_integral, -PI/4, PI/4);
      }

      float yaw_desired_rate = (e_yaw * PDterms->kpyaw+ yaw_integral);
      
      float e_yawrate = yaw_desired_rate - sensors->yawrate;
      if (controls->fx != 0){
        yawrate_integral += e_yawrate *((float)dt)/1000000.0f * kiyawrate;
        yawrate_integral = clamp(yawrate_integral, -yawRateIntegralRange, yawRateIntegralRange);
        controls->tz = e_yawrate*PDterms->kdyaw + yawrate_integral;

      } else{
      // PD for yaw control
        controls->tz = e_yawrate*PDterms->kdyaw;
      }
    }
    
    //roll feedback
    if (PDterms->roll) { 
      controls->tx = controls->tx - sensors->roll* PDterms->kproll - sensors->rollrate * PDterms->kdroll;
    }

    //roll and pitch rotation state feedback
    if (PDterms->rotation) { 
      float cosp = (float) cos(sensors->pitch);
      float sinp = (float) sin(sensors->pitch);
      float cosr = (float) cos(sensors->roll);
      float ifx = controls->fx;
      controls->fx = ifx*cosp + controls->fz*sinp;
      controls->fz = (-1*ifx*sinp + controls->fz* cosp)/cosr;
    }
}

//creates the output values used for actuation from the control values
void getOutputs270(controller_t *controls, sensors_t *sensors, actuation_t *out)
{

  // set up output

  // set output to default if controls not ready
  if (controls->ready == false)
  {
    out->s1 = clamp(PI/2 + servo1offset, 0, 3*PI/2) / (3*PI/2); // cant handle values between PI and 2PI
    out->s2 = clamp(PI/2 + servo2offset, 0, 3*PI/2) / (3*PI/2);

    out->m1 = 0;
    out->m2 = 0;
    out->ready = false;
    return;
  }

  out->ready = true;
  // inputs to the A-Matrix
  float l = PDterms->lx; //.3

  float fx = clamp(controls->fx, -1, 1);                  // setpoint->bicopter.fx;
  float fz = clamp(controls->fz, -2, 2);                 // setpoint->bicopter.fz;
  //float maxRadsYaw = .07; //.1f                                 //.175;
  //float magxz = max(fz * tan(maxRadsYaw), fx * l * 0.17f); // limits the yaw based on the magnitude of the force
  float taux = clamp(controls->tx, -l + (float)0.01, l - (float)0.01);
  float tauz = clamp(controls->tz, -1, 1) ;//* magxz; // limit should be .25 setpoint->bicopter.tauz; //- stateAttitudeRateYaw

  // inverse A-Matrix calculations
  float term1 = l * l * fx * fx + l * l * fz * fz + taux * taux + tauz * tauz;
  float term2 = 2 * fz * l * taux - 2 * fx * l * tauz;
  float term3 = sqrt(term1 + term2);
  float term4 = sqrt(term1 - term2);
  float f1 = term3 / (2 * l); // in unknown units
  float f2 = term4 / (2 * l);
  float t1 = atan2((fz * l - taux) / term3, (fx * l + tauz) / term3) - sensors->pitch; // in radians
  float t2 = atan2((fz * l + taux) / term4, (fx * l - tauz) / term4) - sensors->pitch;

  // checking for full rotations
  while (t1 < -PI / 4)
  {
    t1 = t1 + 2 * PI;
  }
  while (t1 > 7 * PI / 4)
  {
    t1 = t1 - 2 * PI;
  }
  while (t2 < -PI / 4)
  {
    t2 = t2 + 2 * PI;
  }
  while (t2 > 7 * PI / 4)
  {
    t2 = t2 - 2 * PI;
  }

  // converting values to a more stable form
  
  out->m1 = clamp(f1, 0, 1);
  out->m2 = clamp(f2, 0, 1);
  if (out->m1 < 0.02f)
  {
    t1 = PI/2;
  }
  if (out->m2 < 0.02f)
  {
    t2 = PI/2;
  }
  
  out->s1 = clamp(t1 + servo1offset, 0, 3*PI/2) / (3*PI/2); // cant handle values between PI and 2PI
  out->s2 = clamp(t2 + servo2offset, 0, 3*PI/2) / (3*PI/2);
  return;
}

float magxz =1;
//creates the output values used for actuation from the control values
void getOutputs(controller_t *controls, sensors_t *sensors, actuation_t *out)
{

  // set up output

  // set output to default if controls not ready
  if (controls->ready == false)
  {
    out->s1 = .5f;
    out->s2 = .5f;
    out->m1 = 0;
    out->m2 = 0;
    out->ready = false;
    return;
  }

  out->ready = true;
  // inputs to the A-Matrix
  float l = PDterms->lx; //.3

  float fx = clamp(controls->fx, -1, 1);                  // setpoint->bicopter.fx;
  float fz = clamp(controls->fz, 0.001, 2);                 // setpoint->bicopter.fz;
  /*
  HACK WARNING
  yawScaleEnable allows yaw to scale with the fz in order to prevent large changes in yaw with small z 
  however when servos are not pointed straight up, this scaling is less accurate. 
  */
  if (yawScaleEnable){
    magxz = max(fz * tan(maxRadsYaw), fx * l * fxyawScale); // limits the yaw based on the magnitude of the force
  } else {
    magxz = 1;
  }
  float taux = clamp(controls->tx, -l + (float)0.01, l - (float)0.01);
  float tauz = clamp(controls->tz, -1, 1) * magxz; // limit should be .25 setpoint->bicopter.tauz; //- stateAttitudeRateYaw

  // inverse A-Matrix calculations
  float term1 = l * l * fx * fx + l * l * fz * fz + taux * taux + tauz * tauz;
  float term2 = 2 * fz * l * taux - 2 * fx * l * tauz;
  float term3 = sqrt(term1 + term2);
  float term4 = sqrt(term1 - term2);
  float f1 = kf1 * sqrt(term3 / (2 * l)); // in unknown units
  float f2 = kf2 * sqrt(term4 / (2 * l));
  float t1 = atan2((fz * l - taux) / term3, (fx * l + tauz) / term3); // in radians
  float t2 = atan2((fz * l + taux) / term4, (fx * l - tauz) / term4);
  if (feedbackPD.pitch){
    t1 -= sensors->pitch;
    t2 -= sensors->pitch;
  }

  // checking for full rotations
  while (t1 < -PI / 2)
  {
    t1 = t1 + 2 * PI;
  }
  while (t1 > 3 * PI / 2)
  {
    t1 = t1 - 2 * PI;
  }
  while (t2 < -PI / 2)
  {
    t2 = t2 + 2 * PI;
  }
  while (t2 > 3 * PI / 2)
  {
    t2 = t2 - 2 * PI;
  }

  // converting values to a more stable form
  
  out->s1 = clamp((t1 + servo1offset)/PI, 0.05, .95) ; // cant handle values between PI and 2PI
  out->s2 = clamp((t2 + servo2offset)/PI, 0.05, .95) ;
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