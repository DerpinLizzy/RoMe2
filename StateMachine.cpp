/*
 * StateMachine.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include <cmath>
#include "StateMachine.h"

using namespace std;

const float StateMachine::PERIOD = 0.01f;                   // period of task, given in [s]
const float StateMachine::DISTANCE_THRESHOLD = 0.2f;        // minimum allowed distance to obstacle in [m]
const float StateMachine::TRANSLATIONAL_VELOCITY = 0.3f;    // translational velocity in [m/s]
const float StateMachine::ROTATIONAL_VELOCITY = 1.0f;       // rotational velocity in [rad/s]
const float StateMachine::VELOCITY_THRESHOLD = 0.01;        // velocity threshold before switching off, in [m/s] and [rad/s]

/**
 * Creates and initializes a state machine object.
 */
StateMachine::StateMachine(Controller& controller, DigitalOut& enableMotorDriver, DigitalOut& led0, DigitalOut& led1, DigitalOut& led2, DigitalOut& led3, DigitalOut& led4, DigitalOut& led5, DigitalIn& button, IRSensor& irSensor0, IRSensor& irSensor1, IRSensor& irSensor2, IRSensor& irSensor3, IRSensor& irSensor4, IRSensor& irSensor5) : controller(controller), enableMotorDriver(enableMotorDriver), led0(led0), led1(led1), led2(led2), led3(led3), led4(led4), led5(led5), button(button), irSensor0(irSensor0), irSensor1(irSensor1), irSensor2(irSensor2), irSensor3(irSensor3), irSensor4(irSensor4), irSensor5(irSensor5), thread(osPriorityAboveNormal, STACK_SIZE) {
    
    enableMotorDriver = 0;
    state = ROBOT_OFF;
    buttonNow = button;
    buttonBefore = buttonNow;
    
    // start thread and timer interrupt
    
    thread.start(callback(this, &StateMachine::run));
    ticker.attach(callback(this, &StateMachine::sendThreadFlag), PERIOD);
}

/**
 * Deletes the state machine object and releases all allocated resources.
 */
StateMachine::~StateMachine() {
    
    ticker.detach();
}

/**
 * Gets the actual state of this state machine.
 * @return the actual state as an int constant.
 */
int StateMachine::getState() {
    
    return state;
}

/**
 * This method is called by the ticker timer interrupt service routine.
 * It sends a flag to the thread to make it run again.
 */
void StateMachine::sendThreadFlag() {
    
    thread.flags_set(threadFlag);
}

/**
 * This is an internal method of the state machine that is running periodically.
 */
void StateMachine::run() {
    
    while (true) {
        
        // wait for the periodic thread flag
        
        ThisThread::flags_wait_any(threadFlag);
        
        // set the leds based on distance measurements
        
        led0 = irSensor0 < DISTANCE_THRESHOLD;
        led1 = irSensor1 < DISTANCE_THRESHOLD;
        led2 = irSensor2 < DISTANCE_THRESHOLD;
        led3 = irSensor3 < DISTANCE_THRESHOLD;
        led4 = irSensor4 < DISTANCE_THRESHOLD;
        led5 = irSensor5 < DISTANCE_THRESHOLD;
        
        // implementation of the state machine
        
        switch (state) {
            
            case ROBOT_OFF:
                
                buttonNow = button;
                
                if (buttonNow && !buttonBefore) {   // detect button rising edge
                    
                    enableMotorDriver = 1;
                    
                    controller.setTranslationalVelocity(TRANSLATIONAL_VELOCITY);
                    controller.setRotationalVelocity(0.0f);
                    
                    state = MOVE_FORWARD;
                }
                
                buttonBefore = buttonNow;
                
                break;
                
            case MOVE_FORWARD:
                
                
                break;
                
            case TURN_LEFT:
                
                
                break;
                
            case TURN_RIGHT:
                
                
                break;
                
            case SLOWING_DOWN:
                
                
                break;
                
            default:
                
                state = ROBOT_OFF;
        }
    }
}
