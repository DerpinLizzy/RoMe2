/*
 * Main.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include <stdio.h>
#include <mbed.h>
#include "IMU.h"
#include "SensorFusion.h"
#include "HTTPServer.h"
#include "HTTPScriptSensorFusion.h"
#include <iostream>
#include <fstream>
using namespace std;

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
    
    // create inertial measurement unit object
    
    SPI spi(PC_12, PC_11, PC_10);
    DigitalOut csAG(PC_8);
    DigitalOut csM(PC_9);
    
    IMU imu(spi, csAG, csM);
    
    SensorFusion sensorFusion(imu);
    
    // create ethernet interface and webserver
    
    DigitalOut enableRouter(PB_15);
    enableRouter = 1;
    
    EthernetInterface* ethernet = new EthernetInterface();
    ethernet->set_network("192.168.0.10", "255.255.255.0", "192.168.0.1"); // configure IP address, netmask and gateway address
    ethernet->connect();
    
    HTTPServer* httpServer = new HTTPServer(*ethernet);
    httpServer->add("sensorfusion", new HTTPScriptSensorFusion(sensorFusion));
    
    // making funny outputs
    char line;
    fstream file;
    file.open("According to all known laws of avia.txt", ios::in);
    int i = 0;
    while (true) {
        
        led = !led;
        if(i == 1){ 
            printf("workin fine boss\n");
            i = 0;
        }
        i += 0.002;
        if(file){

            file >> line;
            if(file.eof()){
                file.close();
                file.open("According to all known laws of avia.txt", ios::in);
                printf("You open the door . . .\n");
            }else{
                printf("workin well ennuff Boss!!\n");
                printf("%c",line);
                cout << line;
            }
        }

        ThisThread::sleep_for(250ms);
    }
}
