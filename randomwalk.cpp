#include "RandomWalk.h"

// Constructor to initialize member variables
RandomWalk::RandomWalk(float forward_force, int min_distance, int des_z):
    _forward_force(forward_force), _min_distance(min_distance), _des_z(des_z),
    _time_backward(1000), _time_rotate(2000), _des_yaw(0), _current_action(0)
{}

// Method to reset the timer
void RandomWalk::begin() {
    restart_timer();
}

// Method to execute actions based on feedback
void RandomWalk::execute(int distance_sensor, float &force, int &z, float &yaw) {
    choose_action(distance_sensor);  // Choose the next action
    // Execute the chosen action
    switch (_current_action) {
        case 0:
            action_move_forward(force, z, yaw);
            break;
        case 1:
            action_move_backward(force, z, yaw);
            break;
        case 2:
            action_rotate(force, z, yaw);
            break;
        case 3:
            action_wait(force, z, yaw);
            break;
    }
}

// Method to choose the next action based on feedback
void RandomWalk::choose_action(int distance_sensor) {
    int distance = distance_sensor;  // Get distance from feedback
    unsigned long elapsed_time  = time_elapsed();  // Get elapsed time

    // Choose the next action based on current action, distance, and elapsed time
    if (_current_action == 0 && distance < _min_distance) {
        _current_action = 1;
        restart_timer();
    } else if (_current_action == 1 && elapsed_time > _time_backward) {
        _current_action = 2;
        restart_timer();
    } else if (_current_action == 2) {
        _current_action = 3;
    } else if (_current_action == 3 && elapsed_time > _time_rotate) {
        _current_action = 0;
    }
}

// Method to reset the timer
void RandomWalk::restart_timer() {
    _time_elapse = millis();  // Get the current time in milliseconds
}

// Method to get the elapsed time since the timer was reset
unsigned long RandomWalk::time_elapsed() {
    return millis() - _time_elapse;  // Calculate and return elapsed time
}

// Action method to move forward
void RandomWalk::action_move_forward(float &force, int &z, float &yaw) {
    force = _forward_force;
    z = _des_z;
    yaw = _des_yaw;
}

// Action method to move backward
void RandomWalk::action_move_backward(float &force, int &z, float &yaw) {
    force = -_forward_force;
    z = _des_z;
    yaw = _des_yaw;
}

// Action method to rotate
void RandomWalk::action_rotate(float &force, int &z, float &yaw) {
    _des_yaw += random(0, 314) / 100.0 + PI / 2;  // Generate a random yaw value
    force = _forward_force;
    z = _des_z;
    yaw = _des_yaw;
}

// Action method to wait
void RandomWalk::action_wait(float &force, int &z, float &yaw) {
    force = 0;
    z = _des_z;
    yaw = _des_yaw;
}


// Setter method for forward_force
void RandomWalk::setForwardForce(float forward_force) {
    _forward_force = forward_force;
}

// Setter method for min_distance
void RandomWalk::setMinDistance(int min_distance) {
    _min_distance = min_distance;
}

// Setter method for des_z
void RandomWalk::setDesZ(int des_z) {
    _des_z = des_z;
}


