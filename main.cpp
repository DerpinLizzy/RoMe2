/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "IRSensor.h"
#include <InterruptIn.h>
#include <cmath>
#include <cstdio>
#include "EncoderCounter.h"
#include "Controller.h"

// Blinking rate in milliseconds
#define BLINKING_RATE     500ms

bool run = false;
const float speed = 250.0;

InterruptIn user_button(BUTTON1);
DigitalOut enableMotorDriver(PG_0);

void user_button_pressed(){
    run = !run;
    enableMotorDriver = !enableMotorDriver;

}

int main(){

    user_button.fall(&user_button_pressed);
    
    //LEDS
    DigitalOut ledg(LED1);
    DigitalOut ledb(LED2);
    DigitalOut ledr(LED3);
    DigitalOut led0(PD_4);
    DigitalOut led1(PD_3);
    DigitalOut led2(PD_6);
    DigitalOut led3(PD_2);
    DigitalOut led4(PD_7);
    DigitalOut led5(PD_5);

    //IR Sense
    AnalogIn distance(PA_0);
    DigitalOut enable(PG_1);
    DigitalOut bit0(PF_0);
    DigitalOut bit1(PF_1);
    DigitalOut bit2(PF_2);

    enable = 1; // schaltet die Sensoren ein

    IRSensor irSensor0(distance, bit0, bit1, bit2, 0);
    IRSensor irSensor1(distance, bit0, bit1, bit2, 1);
    IRSensor irSensor2(distance, bit0, bit1, bit2, 2);
    IRSensor irSensor3(distance, bit0, bit1, bit2, 3);
    IRSensor irSensor4(distance, bit0, bit1, bit2, 4);
    IRSensor irSensor5(distance, bit0, bit1, bit2, 5);

    //Motors
    DigitalIn motorDriverFault(PD_1);
    DigitalIn motorDriverWarning(PD_0);
    PwmOut pwmLeft(PF_9);
    PwmOut pwmRight(PF_8);

    pwmLeft.period(0.00005); // Setzt die Periode auf 50 μs
    pwmRight.period(0.00005);

    enableMotorDriver = 0;

    //controller and Encoder
    EncoderCounter counterLeft(PD_12, PD_13);
    EncoderCounter counterRight(PB_4, PC_7);
    Controller controller(pwmLeft, pwmRight, counterLeft, counterRight);
    controller.setDesiredSpeedLeft(speed); // Drehzahl in [rpm]
    controller.setDesiredSpeedRight((-1) * speed);

    
    while (true) {
        ledg = !ledg;
        if(run){
            ledb = !ledb;
            ledr = !ledr;

            led0 = irSensor0 < 0.2f;
            led1 = irSensor1 < 0.2f;
            led2 = irSensor2 < 0.2f;
            led3 = irSensor3 < 0.2f;
            led4 = irSensor4 < 0.2f;
            led5 = irSensor5 < 0.2f;

            // double d = irSensor0.read();
            // printf("hinten distance = %d [mm]\r\n", (int)(1000.0f * d));
            // d = irSensor1.read();
            // printf("hinten-links distance = %d [mm]\r\n", (int)(1000.0f * d));
            // d = irSensor2.read();
            // printf("vorne-links distance = %d [mm]\r\n", (int)(1000.0f * d));
            // d = irSensor3.read();
            // printf("vorne distance = %d [mm]\r\n", (int)(1000.0f * d));
            // d = irSensor4.read();
            // printf("vorne-rechts distance = %d [mm]\r\n", (int)(1000.0f * d));
            // d = irSensor5.read();
            // printf("hinten-rechts distance = %d [mm]\r\n\n--------------------------------------\n\n", (int)(1000.0f * d));

            printf("actual speed (left/right): %.3f / %.3f [rpm]\r\n", controller.getActualSpeedLeft(), controller.getActualSpeedRight());
        }

        ThisThread::sleep_for(BLINKING_RATE);
    }
}