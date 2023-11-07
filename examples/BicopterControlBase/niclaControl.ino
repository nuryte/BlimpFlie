


float x_cal;
float max_x = 240;
float max_y = 160;
float _x, _y, _w, _h;
float des_yaw, _yaw, Ky, _height;
float x = 0;
float y = 0;
float w = 0;
float h = 0;
float control_yaw = 0;
float control_height = 0;
float des_height, gamma2;

void addNiclaControl(controller_t *controls, sensors_t *sensors, ModBlimp *blimp, nicla_tuning_s *nicla_tuning){
  
  blimp->IBus.loop();
  
  _x = (float)blimp->IBus.readChannel(1);
  _y = (float)blimp->IBus.readChannel(2);
  _w = (float)blimp->IBus.readChannel(7);
  _h = (float)blimp->IBus.readChannel(8);
  _yaw = sensors->yaw;
  _height = sensors->estimatedZ;
  x_cal = (x / max_x); //- x_min_cal) / (x_max_cal - x_min_cal);
  if (_x == 0 && _y == 0 && _w == 0 && _h == 0) {
    control_yaw = control_yaw  + 0.3f * (float)dt/1000000.0f;
    controls->fx = -0.05;
    //detected = false;
  } else if (x != _x || y != _y || w != _w || h != _h) {
    //detected = true;
    
    des_yaw = ((x_cal - 0.5) * PI / 2);
    float robot_to_goal = _yaw - des_yaw; //* Ky; // Removed * .2 + control_yaw* .8 as it seemed like a leftover code
    float relative_to_goal = nicla_tuning->goal_theta_back - robot_to_goal;
    relative_to_goal = atan2(sin(relative_to_goal), cos(relative_to_goal));
    float goal_act = nicla_tuning->goal_theta_back;
    
    if (abs(relative_to_goal) > 3*PI/4) { // seeing front goal
      //relative_to_goal = atan2(sin(relative_to_goal + PI), cos(relative_to_goal + PI));
      goal_act = goal_act + PI;
      control_yaw = robot_to_goal * nicla_tuning->goal_ratio + goal_act * (1 - nicla_tuning->goal_ratio);
      control_yaw = atan2(sin(control_yaw), cos(control_yaw));
      // des_height = des_height * gamma2 + ((y - max_y / 2) / max_y) * (1 - gamma2);
      // control_height = _height;
      if (abs(control_yaw - _yaw) < nicla_tuning->yaw_move_threshold){
        controls->fx = nicla_tuning->max_move_x;
      } else {
        controls->fx = 0;
      }
    }
    else if (abs(relative_to_goal) > PI/2){ //seeing noise
      control_yaw = control_yaw  + 0.3f * (float)dt/1000000.0f;
      controls->fx = -0.05;
      
    } else{ // seeing back goal
      control_yaw = robot_to_goal * nicla_tuning->goal_ratio + goal_act * (1 - nicla_tuning->goal_ratio);
      control_yaw = atan2(sin(control_yaw), cos(control_yaw));
      // des_height = des_height * gamma2 + ((y - max_y / 2) / max_y) * (1 - gamma2);
      // control_height = _height;
      if (abs(control_yaw - _yaw) < nicla_tuning->yaw_move_threshold){
        controls->fx = nicla_tuning->max_move_x;
      } else {
        controls->fx = 0;
      } 
    }
  } else {// nicla has seen something, but 
    if (abs(control_yaw - _yaw) < nicla_tuning->yaw_move_threshold){
      controls->fx = nicla_tuning->max_move_x;
    } else {
      controls->fx = 0;
    }
  }
  
  
  controls->tz = control_yaw;
  x = _x;
  y = _y;
  w = _w;
  h = _h;
}


