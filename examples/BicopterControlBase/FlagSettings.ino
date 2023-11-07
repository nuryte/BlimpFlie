
void setPDflags(init_flags_t *init_flags,feedback_t *PDterms, sensor_weights_t *weights, raw_t *raws, RollPitchAdjustments *rollPitchAdjust, nicla_tuning_s *nicla_tuning){
  if (lastflag == raws->flag)
  {
    return;
  }
  Serial.print("Set flags: ");
  Serial.println(raws->flag);
  if (raws->flag == 10){// enables or disables feedback in these terms
    PDterms->roll = raws->data[0] == 1.0f;
    PDterms->pitch = raws->data[1] == 1.0f;
    PDterms->yaw = raws->data[2] == 1.0f;
    PDterms->x = raws->data[3] == 1.0f;
    PDterms->y = raws->data[4] == 1.0f;
    PDterms->z = raws->data[5] == 1.0f;
    PDterms->rotation = raws->data[6] == 1.0f;
  }
  else if (raws->flag == 11){
    PDterms->Croll = raws->data[0];
    PDterms->Cpitch = raws->data[1];
    PDterms->Cyaw = raws->data[2];
    PDterms->Cx = raws->data[3];
    PDterms->Cy = raws->data[4];
    PDterms->Cz = raws->data[5];
    PDterms->Cabsz = raws->data[6];
  }
  else if (raws->flag == 12){
    PDterms->kproll = raws->data[0];
    PDterms->kdroll = raws->data[1];
    PDterms->kppitch = raws->data[2];
    PDterms->kdpitch = raws->data[3];
    PDterms->kpyaw = raws->data[4];
    PDterms->kdyaw = raws->data[5];
    kiyaw = raws->data[6];
    kiyawrate = raws->data[7];
    yawRateIntegralRange = raws->data[8];
    errorYawrateRange = raws->data[9];
    errorYawRange = raws->data[10];
  }
  else if (raws->flag == 13){
    PDterms->kpx = raws->data[0];
    PDterms->kdx = raws->data[1];
    PDterms->kpy = raws->data[2];
    PDterms->kdy = raws->data[3];
    PDterms->kpz = raws->data[4];
    PDterms->kdz = raws->data[5];
    PDterms->lx = raws->data[6];
    rollPitchAdjust->pitchSign = raws->data[7];
    rollPitchAdjust->pitchOffset = raws->data[8];
    rollPitchAdjust->rollSign = raws->data[9];
    rollPitchAdjust->rollOffset = raws->data[10];

  }
  else if (raws->flag == 14){
    weights->eulerGamma = raws->data[0];
    weights->rollRateGamma = raws->data[1];
    weights->pitchRateGamma = raws->data[2];
    weights->yawRateGamma = raws->data[3];
    weights->zGamma = raws->data[4];
    weights->vzGamma = raws->data[5];
    // yawScaleEnable = (bool)(raws->data[6] == 1);
  }
  else if (raws->flag == 15){
    kiz = raws->data[0];
    // integral_dt = raws->data[1];
    z_int_low = raws->data[2];
    z_int_high = raws->data[3];
    servo1offset = raws->data[4];
    servo2offset = raws->data[5];
    rollPitchAdjust->rollPitchSwitch = raws->data[6] == 1.0f;
    kf1 = raws->data[7];
    kf2 = raws->data[8];
    maxRadsYaw = raws->data[9]; //.1f                                 //.175;
    fxyawScale = raws->data[10];
    
  }

  else if (raws->flag == 16){
    init_flags->verbose = raws->data[0] == 1.0f,
    init_flags->sensors = raws->data[1] == 1.0f,
    init_flags->escarm = raws->data[2] == 1.0f,
    init_flags->UDP = raws->data[3] == 1.0f,
    init_flags->Ibus = raws->data[4] == 1.0f,
    init_flags->ESPNOW = raws->data[5] == 1.0f,
    init_flags->PORT = raws->data[6],
    init_flags->motor_type = raws->data[7],
    init_flags->mode = raws->data[8],
    init_flags->control = raws->data[9],
    Serial.println("REINIT!");
    blimp.init(init_flags, &init_sensors, PDterms);
    
  } else if (raws->flag == 17){
    if (transceiverEnabled == false){
      //uint8_t transceiverAddress[6];
      transceiverEnabled = true;
       espSendData1.flag = (int) raws->data[6];
      for (int i = 0; i < 6; i++) {
        // print((uint8_t)raws->data[i]);
        // print(":");
        transceiverAddress[i] = (uint8_t)raws->data[i];
      }
      if (blimp.attemptToAddPeer(transceiverAddress) != ESP_OK) {
        Serial.println(" Failed to add peer or peer already exists!");
    } else {
        Serial.println(" Peer added successfully!");
    }
    }
  } else if (raws->flag == 98 ){
    
      baro.init();
      getLatestSensorData(&sensors);
      delay(30);
      sensors.groundZ = baro.getEstimatedZ();
      delay(30);
      getLatestSensorData(&sensors);
      while (abs(sensors.groundZ - baro.getEstimatedZ()) > .4 || sensors.groundZ == baro.getEstimatedZ()){
          sensors.groundZ = baro.getEstimatedZ();
          delay(100);
          getLatestSensorData(&sensors);
      }
  }
  else if (raws->flag == 97){ //IMU initialization
  
      bno.init();

      //getLatestSensorData(&sensors);
  }
  else if (raws->flag == 96 ){// min_max thrust
      // Extract data from raw message
      robot_specs.min_thrust = (int) raws->data[0];
      robot_specs.max_thrust = (int) raws->data[1];
      Serial.print("Min thrust: ");
      Serial.println(robot_specs.min_thrust);
      Serial.print("Max thrust: ");
      Serial.println(robot_specs.max_thrust);
  } else if (raws->flag == 95){//nicla specs
    nicla_tuning->goal_theta_back = raws->data[0];
    nicla_tuning->goal_theta_front = raws->data[1];
    nicla_tuning->goal_dist_thresh = raws->data[2];
    nicla_tuning->max_move_x = raws->data[3];
    nicla_tuning->goal_ratio = raws->data[4];
    nicla_tuning->yaw_move_threshold = raws->data[5];
  }

}