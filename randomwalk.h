#ifndef RandomWalk_h  // Include guard to prevent multiple declarations
#define RandomWalk_h

#include "Arduino.h"  // Include Arduino library

class RandomWalk {
  public:
    // Constructor with default values for member variables
    RandomWalk(float forward_force = 0.3, int min_distance = 400, int des_z = 5);

    // Method to initialize the timer
    void begin();

    // Method to execute actions based on feedback
    void execute(int distance_sensor, float &force, int &z, float &yaw);
    void setForwardForce(float forward_force);
    void setMinDistance(int min_distance);
    void setDesZ(int des_z);

  private:
    float _forward_force;  // Forward force value
    int _min_distance;  // Minimum distance threshold
    int _des_z;  // Desired z-axis value
    int _time_backward;  // Time duration for moving backward
    int _time_rotate;  // Time duration for rotation
    float _des_yaw;  // Desired yaw value
    int _current_action;  // Current action index
    unsigned long _time_elapse;  // Timer value to calculate elapsed time

    // Method to choose the next action based on feedback
    void choose_action(int distance_sensor);

    // Method to reset the timer
    void restart_timer();

    // Method to get the elapsed time since the timer was reset
    unsigned long time_elapsed();

    // Action methods to be executed based on feedback
    void action_move_forward(float &force, int &z, float &yaw);
    void action_move_backward(float &force, int &z, float &yaw);
    void action_rotate(float &force, int &z, float &yaw);
    void action_wait(float &force, int &z, float &yaw);
};

#endif
