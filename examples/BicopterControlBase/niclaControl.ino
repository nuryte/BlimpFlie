


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

void addNiclaControl(controller_t *controls, sensors_t *sensors, ModBlimp *blimp){
  
  blimp->IBus.loop();
  
  _x = (float)blimp->IBus.readChannel(0);
  _y = (float)blimp->IBus.readChannel(1);
  _w = (float)blimp->IBus.readChannel(2);
  _h = (float)blimp->IBus.readChannel(3);
  _yaw = sensors->yaw;
  _height = sensors->estimatedZ;
  x_cal = (x / max_x); //- x_min_cal) / (x_max_cal - x_min_cal);
  if (_x == 0 && _y == 0 && _w == 0 && _h == 0) {
    //detected = false;
  } else if (x != _x || y != _y || w != _w || h != _h) {
    //detected = true;
    
    des_yaw = ((x_cal - 0.5) * PI / 2);
    control_yaw = _yaw - des_yaw; //* Ky; // Removed * .2 + control_yaw* .8 as it seemed like a leftover code
    // des_height = des_height * gamma2 + ((y - max_y / 2) / max_y) * (1 - gamma2);
    // control_height = _height;
  }
  if (abs(control_yaw - _yaw) < .1){
    controls->fx = 0.3;
  } else {
    controls->fx = 0;
  } 
  controls->tz = control_yaw;
  x = _x;
  y = _y;
  w = _w;
  h = _h;
}