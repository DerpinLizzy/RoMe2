/*
 * IMU.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include "IMU.h"

using namespace std;

const float IMU::PERIOD = 0.002f;               // period of task, given in [s]
const float IMU::LOWPASS_FILTER_FREQUENCY = 3.14f;
const float IMU::M_PI = 3.14159265f;            // the mathematical constant PI

/**
 * Creates an IMU object.
 * @param spi a reference to an spi controller to use.
 * @param csAG the chip select output for the accelerometer and the gyro sensor.
 * @param csM the chip select output for the magnetometer.
 */
IMU::IMU(SPI& spi, DigitalOut& csAG, DigitalOut& csM) : spi(spi), csAG(csAG), csM(csM), thread(osPriorityHigh, STACK_SIZE) {
    
    // initialize SPI interface
    float hertz = 10e6;

    spi.format(8, 3);
    spi.frequency(hertz);
    
    // reset chip select lines to logical high
    
    csAG = 1;
    csM = 1;
    
    // initialize accelerometer and gyro
    
    writeRegister(csAG, CTRL_REG1_G, 0xC3);     // ODR 952 Hz, full scale 245 deg/s
    writeRegister(csAG, CTRL_REG2_G, 0x00);     // disable interrupt generation
    writeRegister(csAG, CTRL_REG3_G, 0x00);     // disable low power mode, disable high pass filter, high pass cutoff frequency 57 Hz
    writeRegister(csAG, CTRL_REG4, 0x38);       // enable gyro in all 3 axis
    writeRegister(csAG, CTRL_REG5_XL, 0x38);    // no decimation, enable accelerometer in all 3 axis
    writeRegister(csAG, CTRL_REG6_XL, 0xC0);    // ODR 952 Hz, full scale 2g
    writeRegister(csAG, CTRL_REG7_XL, 0x00);    // high res mode disabled, filter bypassed
    writeRegister(csAG, CTRL_REG8, 0x00);       // 4-wire SPI interface, LSB at lower address
    writeRegister(csAG, CTRL_REG9, 0x04);       // disable gyro sleep mode, disable I2C interface, disable FIFO
    writeRegister(csAG, CTRL_REG10, 0x00);      // self test disabled
    
    // initialize magnetometer
    
    writeRegister(csM, CTRL_REG1_M, 0x10);      // temperature not compensated, low power mode for x & y axis, data rate 10 Hz
    writeRegister(csM, CTRL_REG2_M, 0x00);      // full scale 4 gauss
    writeRegister(csM, CTRL_REG3_M, 0x80);      // disable I2C interface, low power mode, SPI write only, continuous conversion mode
    writeRegister(csM, CTRL_REG4_M, 0x00);      // low power mode for z axis, LSB at lower address
    writeRegister(csM, CTRL_REG5_M, 0x00);      // fast read disabled

    // init filters
    x_lpf.setPeriod(PERIOD);
    x_lpf.setFrequency(LOWPASS_FILTER_FREQUENCY);
    x_lpf.reset();

    y_lpf.setPeriod(PERIOD);
    y_lpf.setFrequency(LOWPASS_FILTER_FREQUENCY);
    y_lpf.reset();

    // initialize local variables

    x_val = 0.0f;
    x_min = -1000.0f;
    x_max = 1000.0f;
    y_val = 0.0f;
    y_min = -1000.0f;
    y_max = 1000.0f;
    
    heading = 0.0f;
    incr = 0;
    
    // start thread and timer interrupt
    
    thread.start(callback(this, &IMU::run));
    ticker.attach(callback(this, &IMU::sendThreadFlag), PERIOD);
}

/**
 * Deletes the IMU object.
 */
IMU::~IMU() {
    
    ticker.detach();
}

/**
 * This private method allows to write a register value.
 * @param cs the chip select output to use, either csAG or csM.
 * @param address the 7 bit address of the register.
 * @param value the value to write into the register.
 */
void IMU::writeRegister(DigitalOut& cs, char address, char value) {
    
    cs = 0;
    
    spi.write(0x7F & address);
    spi.write(value & 0xFF);
    
    cs = 1;
}

/**
 * This private method allows to read a register value.
 * @param cs the chip select output to use, either csAG or csM.
 * @param address the 7 bit address of the register.
 * @return the value read from the register.
 */
char IMU::readRegister(DigitalOut& cs, char address) {
    
    cs = 0;
    
    spi.write(0x80 | address);
    int value = spi.write(0xFF);
    
    cs = 1;
    
    return (char)(value & 0xFF);
}

/**
 * Reads the acceleration in x-direction.
 * @return the acceleration in x-direction, given in [m/s2].
 */
float IMU::readAccelerationX() {
    
    mutex.lock();
    
    char low = readRegister(csAG, OUT_X_L_XL);
    char high = readRegister(csAG, OUT_X_H_XL);
    
    short value = (short)(((unsigned short)high << 8) | (unsigned short)low);
    
    mutex.unlock();
    
    return (float)value/32768.0f*2.0f*9.81f;
}

/**
 * Reads the acceleration in y-direction.
 * @return the acceleration in y-direction, given in [m/s2].
 */
float IMU::readAccelerationY() {
    
    mutex.lock();
    
    char low = readRegister(csAG, OUT_Y_L_XL);
    char high = readRegister(csAG, OUT_Y_H_XL);
    
    short value = (short)(((unsigned short)high << 8) | (unsigned short)low);
    
    mutex.unlock();
    
    return (float)value/32768.0f*2.0f*9.81f;
}

/**
 * Reads the acceleration in z-direction.
 * @return the acceleration in z-direction, given in [m/s2].
 */
float IMU::readAccelerationZ() {
    
    mutex.lock();
    
    char low = readRegister(csAG, OUT_Z_L_XL);
    char high = readRegister(csAG, OUT_Z_H_XL);
    
    short value = (short)(((unsigned short)high << 8) | (unsigned short)low);
    
    mutex.unlock();
    
    return (float)value/32768.0f*2.0f*9.81f;
}

/**
 * Reads the gyroscope about the x-axis.
 * @return the rotational speed about the x-axis given in [rad/s].
 */
float IMU::readGyroX() {
    
    mutex.lock();
    
    char low = readRegister(csAG, OUT_X_L_G);
    char high = readRegister(csAG, OUT_X_H_G);
    
    short value = (short)(((unsigned short)high << 8) | (unsigned short)low);
    
    mutex.unlock();
    
    return (float)value/32768.0f*245.0f*M_PI/180.0f;
}

/**
 * Reads the gyroscope about the y-axis.
 * @return the rotational speed about the y-axis given in [rad/s].
 */
float IMU::readGyroY() {
    
    mutex.lock();
    
    char low = readRegister(csAG, OUT_Y_L_G);
    char high = readRegister(csAG, OUT_Y_H_G);
    
    short value = (short)(((unsigned short)high << 8) | (unsigned short)low);
    
    mutex.unlock();
    
    return (float)value/32768.0f*245.0f*M_PI/180.0f;
}

/**
 * Reads the gyroscope about the z-axis.
 * @return the rotational speed about the z-axis given in [rad/s].
 */
float IMU::readGyroZ() {
    
    mutex.lock();
    
    char low = readRegister(csAG, OUT_Z_L_G);
    char high = readRegister(csAG, OUT_Z_H_G);
    
    short value = (short)(((unsigned short)high << 8) | (unsigned short)low);
    
    mutex.unlock();
    
    return (float)value/32768.0f*245.0f*M_PI/180.0f;
}

/**
 * Reads the magnetic field in x-direction.
 * @return the magnetic field in x-direction, given in [Gauss].
 */
float IMU::readMagnetometerX() {

    mutex.lock();

    char low = readRegister(csM, OUT_X_L_M);
    char high = readRegister(csM, OUT_X_H_M);
    
    short value = (short)(((unsigned short)high << 8) | (unsigned short)low);
    
    mutex.unlock();

    return (float)value/32768.0f*4.0f;
}

/**
 * Reads the magnetic field in y-direction.
 * @return the magnetic field in y-direction, given in [Gauss].
 */
float IMU::readMagnetometerY() {
    
    mutex.lock();

    char low = readRegister(csM, OUT_Y_L_M);
    char high = readRegister(csM, OUT_Y_H_M);
    
    short value = (short)(((unsigned short)high << 8) | (unsigned short)low);
    
    mutex.unlock();
    
    return (float)value/32768.0f*4.0f;
}

/**
 * Reads the magnetic field in z-direction.
 * @return the magnetic field in z-direction, given in [Gauss].
 */
float IMU::readMagnetometerZ() {
    
    mutex.lock();

    char low = readRegister(csM, OUT_Z_L_M);
    char high = readRegister(csM, OUT_Z_H_M);
    
    short value = (short)(((unsigned short)high << 8) | (unsigned short)low);
    
    mutex.unlock();
    
    return (float)value/32768.0f*4.0f;
}

/**
 * Reads the compass heading about the z-axis.
 * @return the compass heading in the range -PI to +PI, given in [rad].
 */
float IMU::readHeading() {
    
    return heading;
}

/**
 * This method is called by the ticker timer interrupt service routine.
 * It sends a flag to the thread to make it run again.
 */
void IMU::sendThreadFlag() {
    
    thread.flags_set(threadFlag);
}

/**
 * This <code>run()</code> method contains an infinite loop with the run logic.
 */
void IMU::run() {
    
    while (true) {
        
        // wait for the periodic thread flag
        ThisThread::flags_wait_any(threadFlag);
        
        // filter and process sensor data...
        x_val = x_lpf.filter(readMagnetometerX());
        y_val = y_lpf.filter(readMagnetometerY());
        if(incr == 1){
            incr = 0;
            printf("\r\n\n============================="
                    "\r\nDEBUG"
                    "\r\n============================="
                    "\r\nx_val readout rohwert: %f"
                    "\r\ny_val readout rohwert: %f"
                    "\r\nx_val gefiltert: %f"
                    "\r\ny_val gefiltert: %f\r\n", readMagnetometerX(), readMagnetometerY(), x_val, y_val);
        }
        incr += PERIOD;

        if(x_max < x_val) x_max = x_val;
        if(x_min > x_val) x_min = x_val;
        if(y_max < y_val) y_max = y_val;
        if(y_min > y_val) y_min = y_val;

        if(y_min < y_max) x_val = 2 * (x_val - x_min)/(x_max - x_min) - 1;
        if(y_min < y_max) y_val = 2 * (y_val - y_min)/(y_max - y_min) - 1;
        
        // calculate heading
        heading = (atan2(-y_val,x_val) >= 0) ? atan2(-y_val, x_val) : atan2(-y_val, x_val) + 2 * M_PI;
    }
}
