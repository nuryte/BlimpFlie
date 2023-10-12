import time
feedbackPD = { "roll" : 0,
  "pitch" : 0,
  "yaw" : 0,
  "x" : 0,
  "y" : 0,
  "z" : 0,
  "rotation" : 0,

  "Croll" : 1,
  "Cpitch" : 0, 
  "Cyaw" : 1,
  "Cx" : 1,
  "Cy" : 0,
  "Cz" : 1,
  "Cabsz" : 1,

  "kproll" : 0,
  "kdroll" : 0 ,
  "kppitch" : 0,
  "kdpitch" : 0,
  "kpyaw" : 3.0,
  "kdyaw" : -120,

  "kpx" : 0,
  "kdx" : 0,
  "kpy" : 0,
  "kdy" : 0,
  "kpz" : 0,#.2,#.5
  "kdz" : 0,#-3
  "kiz" : 0,

  "integral_dt" : 0,#.0001,
  "z_int_low" : 0,
  "z_int_high" : 200,

  "lx" : .15,
  "pitchSign" : 1,
  "pitchOffset" : -3.2816,

  "servo1offset" : -0.1,
  "servo2offset" : .0
}
weights = { "eulerGamma" : 0,
  "rollRateGamma" : 0.7,
  "yawRateGamma" : 0.975,
  "pitchRateGamma" : 0.7,
  "zGamma" : 0.5,
  "vzGamma" : 0.975
}


def sendAllFlags(esp_now, BRODCAST_CHANNEL, SLAVE_INDEX):
    print("send all flags!")
    esp_now_input = [
        10, 0,
        feedbackPD["roll"], 
        feedbackPD["pitch"], 
        feedbackPD["yaw"],  
        feedbackPD["x"], 
        feedbackPD["y"], 
        feedbackPD["z"], 
        feedbackPD["rotation"], 
        0, 0, 0, 0
    ]
    esp_now.send(esp_now_input, BRODCAST_CHANNEL, SLAVE_INDEX)
    time.sleep(0.05) 
    
    esp_now_input = [
        11, 0,
        feedbackPD["Croll"], 
        feedbackPD["Cpitch"], 
        feedbackPD["Cyaw"],  
        feedbackPD["Cx"], 
        feedbackPD["Cy"], 
        feedbackPD["Cz"], 
        feedbackPD["Cabsz"],
        0, 0, 0, 0
    ]
    
    esp_now.send(esp_now_input, BRODCAST_CHANNEL, SLAVE_INDEX)
    time.sleep(0.05) 
    
    esp_now_input = [
        12, 0,
        feedbackPD["kproll"], 
        feedbackPD["kdroll"], 
        feedbackPD["kppitch"],  
        feedbackPD["kdpitch"], 
        feedbackPD["kpyaw"], 
        feedbackPD["kdyaw"], 
        0, 0, 0, 0, 0
    ]
    
    esp_now.send(esp_now_input, BRODCAST_CHANNEL, SLAVE_INDEX)
    time.sleep(0.05) 
    
    esp_now_input = [
        13, 0,
        feedbackPD["kpx"], 
        feedbackPD["kdx"], 
        feedbackPD["kpy"],  
        feedbackPD["kdy"], 
        feedbackPD["kpz"], 
        feedbackPD["kdz"],  
        feedbackPD["lx"], 
        feedbackPD["pitchSign"],  
        feedbackPD["pitchOffset"], 
        0,0
    ]
    
    esp_now.send(esp_now_input, BRODCAST_CHANNEL, SLAVE_INDEX)
    time.sleep(0.05) 
    
    esp_now_input = [
        14, 0,
        weights["eulerGamma"], 
        weights["rollRateGamma"], 
        weights["pitchRateGamma"],  
        weights["yawRateGamma"], 
        weights["zGamma"], 
        weights["vzGamma"], 
        0, 0, 0, 0, 0
    ]
    
    esp_now.send(esp_now_input, BRODCAST_CHANNEL, SLAVE_INDEX)
    time.sleep(0.05) 
    
    esp_now_input = [
        15, 0,
        feedbackPD["kiz"], 
        feedbackPD["integral_dt"],  
        feedbackPD["z_int_low"], 
        feedbackPD["z_int_high"],  
        feedbackPD["servo1offset"],  
        feedbackPD["servo2offset"], 
        0,0,0,0,0
    ]
    
    esp_now.send(esp_now_input, BRODCAST_CHANNEL, SLAVE_INDEX)
    time.sleep(0.05) 

    print("All Flags Sent!")
