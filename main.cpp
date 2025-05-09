/*
 * Main.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include <stdio.h>
#include <mbed.h>
#include "IRSensor.h"
#include "EncoderCounter.h"
#include "IMU.h"
#include "LIDAR.h"
#include "Controller.h"
#include "StateMachine.h"
#include "HTTPServer.h"
#include "HTTPScriptLIDAR.h"

int main() {
    
    // create miscellaneous periphery objects
    
    DigitalIn button(BUTTON1);
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
    
    // create inertial measurement unit object
    
    SPI spi(PC_12, PC_11, PC_10);
    DigitalOut csAG(PC_8);
    DigitalOut csM(PC_9);
    
    IMU imu(spi, csAG, csM);

    // create LIDAR device driver
    
    PwmOut pwm(PE_9);
    pwm.period(0.00005f);
    pwm.write(0.5f);
    
    ThisThread::sleep_for(500ms);
    
    UnbufferedSerial* serial = new UnbufferedSerial(PG_14, PG_9);
    LIDAR* lidar = new LIDAR(*serial);
    
    // create robot controller objects
    
    Controller controller(pwmLeft, pwmRight, counterLeft, counterRight);
    StateMachine stateMachine(controller, enableMotorDriver, led0, led1, led2, led3, led4, led5, button, irSensor0, irSensor1, irSensor2, irSensor3, irSensor4, irSensor5);
    
    // create ethernet interface and webserver
    
    DigitalOut enableRouter(PB_15);
    enableRouter = 1;
    
    EthernetInterface* ethernet = new EthernetInterface();
    ethernet->set_network("192.168.0.10", "255.255.255.0", "192.168.0.1"); // configure IP address, netmask and gateway address
    ethernet->connect();
    
    HTTPServer* httpServer = new HTTPServer(*ethernet);
    httpServer->add("lidar", new HTTPScriptLIDAR(*lidar));

    // Variables
    float y_a = 0.5;
    float x_a_1 = 0.0;
    float x_a_2 = 2.0;
    float x_a_3 = 4.0;
    float x_a_4 = 6.0;

    Point beacon_1;
    beacon_1.x = x_a_1;
    beacon_1.y = y_a;

    Point beacon_2;
    beacon_2.x = x_a_3;
    beacon_2.y = y_a;

    Point beacon_3;
    beacon_3.x = x_a_3;
    beacon_3.y = y_a;

    Point beacon_4;
    beacon_4.x = x_a_4;
    beacon_4.y = y_a;


    while (true) {
        
        led = !led;
        
        float x = controller.getX();
        float y = controller.getY();
        float alpha = controller.getAlpha();
        
        deque<Point> beacons = lidar->getBeacons();
        for(int i = 0; i<beacons.size();i+=1){
            float x_l_m = beacons.at(i).x;
            float y_l_m = beacons.at(i).y;

            Point beacon_g_m;
            beacon_g_m.x = cos(alpha) * x_l_m - sin(alpha) * y_l_m + x;
            beacon_g_m.y = sin(alpha) *x_l_m + cos(alpha) * y_l_m + y;

            if(beacon_1.distance(beacon_g_m) < 0.4f){
                controller.correctPoseWithBeacon(beacon_1, beacon_g_m);

            } else if(beacon_2.distance(beacon_g_m) < 0.4f){
                controller.correctPoseWithBeacon(beacon_2, beacon_g_m);

            } else if(beacon_3.distance(beacon_g_m) < 0.4f){
                controller.correctPoseWithBeacon(beacon_3, beacon_g_m);

            } else if(beacon_4.distance(beacon_g_m) < 0.4f){
                controller.correctPoseWithBeacon(beacon_4, beacon_g_m);

            }else{
                printf("You Suck!!");
            }

        }

        ThisThread::sleep_for(100ms);  
        
    }
}
