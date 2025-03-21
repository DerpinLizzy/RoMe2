/*
 * TaskMoveTo.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include <cmath>
#include "TaskMoveTo.h"

using namespace std;

const float TaskMoveTo::DEFAULT_VELOCITY = 0.3f;    // default velocity value, given in [m/s]
const float TaskMoveTo::DEFAULT_ZONE = 0.05f;       // default zone value, given in [m]
const float TaskMoveTo::M_PI = 3.14159265f;         // the mathematical constant PI
const float TaskMoveTo::K1 = 2.0f;                  // position controller gain parameter
const float TaskMoveTo::K2 = 2.0f;                  // position controller gain parameter
const float TaskMoveTo::K3 = 1.0f;                  // position controller gain parameter

/**
 * Creates a task object that moves the robot to a given pose.
 * @param conroller a reference to the controller object of the robot.
 * @param x the x coordinate of the target position, given in [m].
 * @param y the y coordinate of the target position, given in [m].
 * @param alpha the target orientation, given in [rad].
 */
TaskMoveTo::TaskMoveTo(Controller& controller, float x, float y, float alpha) : controller(controller) {
    
    this->x = x;
    this->y = y;
    this->alpha = alpha;
    this->velocity = DEFAULT_VELOCITY;
    this->zone = DEFAULT_ZONE;
}

/**
 * Creates a task object that moves the robot to a given pose.
 * @param conroller a reference to the controller object of the robot.
 * @param x the x coordinate of the target position, given in [m].
 * @param y the y coordinate of the target position, given in [m].
 * @param alpha the target orientation, given in [rad].
 * @param velocity the maximum translational velocity, given in [m/s].
 */
TaskMoveTo::TaskMoveTo(Controller& controller, float x, float y, float alpha, float velocity) : controller(controller) {
    
    this->x = x;
    this->y = y;
    this->alpha = alpha;
    this->velocity = velocity;
    this->zone = DEFAULT_ZONE;
}

/**
 * Creates a task object that moves the robot to a given pose.
 * @param conroller a reference to the controller object of the robot.
 * @param x the x coordinate of the target position, given in [m].
 * @param y the y coordinate of the target position, given in [m].
 * @param alpha the target orientation, given in [rad].
 * @param velocity the maximum translational velocity, given in [m/s].
 * @param zone the zone threshold around the target position, given in [m].
 */
TaskMoveTo::TaskMoveTo(Controller& controller, float x, float y, float alpha, float velocity, float zone) : controller(controller) {
    
    this->x = x;
    this->y = y;
    this->alpha = alpha;
    this->velocity = velocity;
    this->zone = zone;
}

/**
 * Deletes the task object.
 */
TaskMoveTo::~TaskMoveTo() {}

/**
 * This method is called periodically by a task sequencer.
 * @param period the period of the task sequencer, given in [s].
 * @return the status of this task, i.e. RUNNING or DONE.
 */
int TaskMoveTo::run(float period) {
    
    // bitte implementieren!
    
    return RUNNING;
}
