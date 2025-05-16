/*
 * LIDAR.h
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#ifndef LIDAR_H_
#define LIDAR_H_

#include <cstdlib>
#include <deque>
#include <mbed.h>
#include "Controller.h"
#include "Point.h"

/**
 * This is a device driver class for the Slamtec RP LIDAR A1.
 */
class LIDAR {
    
    public:
        
                        LIDAR(UnbufferedSerial& serial, Controller& controller);
        virtual         ~LIDAR();
        deque<Point>    getScan();
        deque<Point>    getBeacons();
        
    private:
        
        static const unsigned int   STACK_SIZE = 4096;      // stack size of thread, given in [bytes]
        static const unsigned short BUFFER_SIZE = 500;      // size of buffer for measurements
        static const char           QUALITY_THRESHOLD = 10; // quality threshold used for accepting measurements
        static const float          MIN_DISTANCE_THRESHOLD; // minimum threshold for measured distance, given in [m]
        static const float          MAX_DISTANCE_THRESHOLD; // maximum threshold for measured distance, given in [m]
        static const float          M_PI;                   // the mathematical constant PI
        
        static const unsigned short HEADER_SIZE = 7;
        static const unsigned short DATA_SIZE = 5;
        
        static const char   START_FLAG = 0xA5;
        static const char   SCAN = 0x20;
        static const char   STOP = 0x25;
        static const char   RESET = 0x40;
        
        UnbufferedSerial&   serial;             // reference to serial interface for communication
        Controller&         controller;         // reference to a controller to get actual pose values
        char                byte;               // single byte to read in serial interrupt service routine
        char                headerCounter;
        char                dataCounter;
        char                data[DATA_SIZE];
        int                 bufferCounter;
        float               x[BUFFER_SIZE];     // measured x-positions in global system, given in [m]
        float               y[BUFFER_SIZE];     // measured y-positions in global system, given in [m]
        Mutex               mutex;
        ThreadFlag          threadFlag;
        Thread              thread;

        void    receive();
        void    processMeasurements();
};

#endif /* LIDAR_H_ */
