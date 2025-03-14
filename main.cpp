/*
 * Main.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include <stdio.h>
#include <mbed.h>
#include "IRSensor.h"
#include "EncoderCounter.h"
#include "Controller.h"

int main() {
    
    // create miscellaneous periphery objects
    
    DigitalOut led(LED1);
    
    DigitalOut led0(PD_4);
    DigitalOut led1(PD_3);
    DigitalOut led2(PD_6);
    DigitalOut led3(PD_2);
    DigitalOut led4(PD_7);
    DigitalOut led5(PD_5);
    
    // create IR sensor objects
    
    AnalogIn distance(PA_0);
    DigitalOut enable(PG_1);
    DigitalOut bit0(PF_0);
    DigitalOut bit1(PF_1);
    DigitalOut bit2(PF_2);
    
    IRSensor irSensor0(distance, bit0, bit1, bit2, 0);
    IRSensor irSensor1(distance, bit0, bit1, bit2, 1);
    IRSensor irSensor2(distance, bit0, bit1, bit2, 2);
    IRSensor irSensor3(distance, bit0, bit1, bit2, 3);
    IRSensor irSensor4(distance, bit0, bit1, bit2, 4);
    IRSensor irSensor5(distance, bit0, bit1, bit2, 5);
    
    enable = 1;
    
    // create motor control objects
    
    DigitalOut enableMotorDriver(PG_0); 
    DigitalIn motorDriverFault(PD_1);
    DigitalIn motorDriverWarning(PD_0);
    
    PwmOut pwmLeft(PF_9);
    PwmOut pwmRight(PF_8);
    
    EncoderCounter counterLeft(PD_12, PD_13);
    EncoderCounter counterRight(PB_4, PC_7);
    
    enableMotorDriver = 1;
    
    // create robot controller object
    
    Controller controller(pwmLeft, pwmRight, counterLeft, counterRight);
    
    controller.setTranslationalVelocity(0.2f);
    controller.setRotationalVelocity(0.5f);
    
    while (true) {
        
        led = !led;
        
        led0 = irSensor0 < 0.2f;
        led1 = irSensor1 < 0.2f;
        led2 = irSensor2 < 0.2f;
        led3 = irSensor3 < 0.2f;
        led4 = irSensor4 < 0.2f;
        led5 = irSensor5 < 0.2f;
        
        printf("actual velocity: %.3f [m/s] / %.3f [rad/s]\r\n", controller.getActualTranslationalVelocity(), controller.getActualRotationalVelocity());
        
        ThisThread::sleep_for(100ms);
    }
}
