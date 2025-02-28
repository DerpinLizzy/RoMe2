/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "IRSensor.h"


// Blinking rate in milliseconds
#define BLINKING_RATE     500ms


int main()
{
    DigitalOut ledg(LED1);
    DigitalOut ledb(LED2);
    DigitalOut ledr(LED3);
    DigitalOut led0(PD_4);
    DigitalOut led1(PD_3);
    DigitalOut led2(PD_6);
    DigitalOut led3(PD_2);
    DigitalOut led4(PD_7);
    DigitalOut led5(PD_5);

    AnalogIn distance(PA_0);
    DigitalOut enable(PG_1);
    DigitalOut bit0(PF_0);
    DigitalOut bit1(PF_1);
    DigitalOut bit2(PF_2);
    enable = 1; // schaltet die Sensoren ein
    IRSensor IR0(distance, bit0, bit1, bit2, 0);

    
    while (true) {
        ledg = !ledg;
        ledb = !ledb;
        ledr = !ledr;
        led0 = !led0;
        led1 = !led1;
        led2 = !led2;
        led3 = !led3;
        led4 = !led4;
        led5 = !led5;

        double d = IR0.read();
        printf("distance = %d [mm]\r\n", (int)(1000.0f * d));

        ThisThread::sleep_for(BLINKING_RATE);
    }
}
