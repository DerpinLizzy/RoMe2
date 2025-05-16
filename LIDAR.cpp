/*
 * LIDAR.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include <cmath>
#include "LIDAR.h"

using namespace std;

const float LIDAR::MIN_DISTANCE_THRESHOLD = 0.01f;  // minimum threshold for measured distance, given in [m]
const float LIDAR::MAX_DISTANCE_THRESHOLD = 4.00f;  // maximum threshold for measured distance, given in [m]
const float LIDAR::M_PI = 3.1415926535897932f;      // the mathematical constant PI

/**
 * Creates a LIDAR object.
 * @param serial a reference to a serial interface to communicate with the laser scanner.
 */
LIDAR::LIDAR(UnbufferedSerial& serial, Controller& controller) : serial(serial), controller(controller), thread(osPriorityRealtime, STACK_SIZE) {
    
    // initialize serial interface
    
    serial.baud(115200);
    serial.format(8, SerialBase::None, 1);
    
    // initialize local values
    
    byte = 0;
    headerCounter = 0;
    dataCounter = 0;
    bufferCounter = 0;
    
    for (unsigned short i = 0; i < BUFFER_SIZE; i++) {

        x[i] = 0.0f;
        y[i] = 0.0f;
    }

    // start thread and serial interrupt
    
    thread.start(callback(this, &LIDAR::processMeasurements));
    serial.attach(callback(this, &LIDAR::receive), UnbufferedSerial::RxIrq);
    
    // start the continuous operation of the LIDAR
    
    char bytes[] = {START_FLAG, SCAN};
    serial.write(&bytes, 2);
}

/**
 * Stops the lidar and deletes this object.
 */
LIDAR::~LIDAR() {
    
    // stop the LIDAR
    
    char bytes[] = {START_FLAG, STOP};
    serial.write(&bytes, 2);
}

/**
 * Get a list of points of a scan.
 * @return a deque vector of point objects.
 */
deque<Point> LIDAR::getScan() {
    
    deque<Point> scan;
    
    for (unsigned short i = 0; i < BUFFER_SIZE; i++) {
        
        mutex.lock();
        scan.push_back(Point(x[i], y[i]));
        mutex.unlock();
    }
    
    return scan;
}

/**
 * Get a list of points which are part of beacons.
 * @return a deque vector of points that are beacons.
 */
deque<Point> LIDAR::getBeacons() {
    
    // get a list of all points of a scan
    
    deque<Point> scan = getScan();
    
    // create a list of points of beacons
    
    deque<Point> beacons;
    
    // check the points of a scan for beacons
    
    for (unsigned short i = 0; i < scan.size(); i++) {
        
        bool beacon = true; // flag to check if this point is possibly a beacon
        int counter = 0;
        
        // check distance to other points
        
        for (unsigned short j = 0; beacon && (j < scan.size()); j++) {
            
            float distance = scan[i].manhattanDistance(scan[j]);
            if (distance < 0.1f) {
                
                counter++; // another point which may be part of this beacon
                
            } else if (distance < 0.5f) {
                
                beacon = false; // this point cannot be part of a beacon
            }
        }
        
        if (beacon && (counter > 1)) beacons.push_back(scan[i]);
    }
    
    return beacons;
}

/**
 * This method is called by the serial interrupt service routine.
 * It sends a flag to the thread to make it run again.
 */
void LIDAR::receive() {

    if (serial.readable()) {
        
        serial.read(&byte, 1);
        thread.flags_set(threadFlag);
    }
}

/**
 * This method handles the reception of measurements from the LIDAR.
 */
void LIDAR::processMeasurements() {
    
    while (true) {
        
        // wait for the thread flag
        
        ThisThread::flags_wait_any(threadFlag);

        // add the received byte to the header or to the data buffer
        
        if (headerCounter < HEADER_SIZE) {

            headerCounter++;

        } else {

            if (dataCounter < DATA_SIZE) {
                
                data[dataCounter] = byte;
                dataCounter++;
            }

            if (dataCounter >= DATA_SIZE) {
                
                // data buffer is full, process measurement
                
                char quality = data[0] >> 2;
                float angle = -(float)(((unsigned short)data[1] | ((unsigned short)data[2] << 8)) >> 1)/64.0f*M_PI/180.0f;
                float distance = (float)((unsigned short)data[3] | ((unsigned short)data[4] << 8))/4000.0f;
                
                if ((quality < QUALITY_THRESHOLD) || (distance < MIN_DISTANCE_THRESHOLD) || (distance > MAX_DISTANCE_THRESHOLD)) {
                    
                    // ignore measurement
                
                } else {
                    
                    // calculate and store global position of measured point
                    
                    float xRobot = controller.getX();
                    float yRobot = controller.getY();
                    float alphaRobot = controller.getAlpha();
                    
                    float xLocal = distance*cos(angle);
                    float yLocal = distance*sin(angle);
                    
                    float xGlobal = cos(alphaRobot)*xLocal-sin(alphaRobot)*yLocal+xRobot;
                    float yGlobal = sin(alphaRobot)*xLocal+cos(alphaRobot)*yLocal+yRobot;
                    
                    mutex.lock();
                    
                    x[bufferCounter] = xGlobal;
                    y[bufferCounter] = yGlobal;
                    
                    mutex.unlock();
                    
                    bufferCounter++;
                    if (bufferCounter >= BUFFER_SIZE) bufferCounter = 0;
                }

                // reset data counter
                
                dataCounter = 0;
            }
        }
    }
}
