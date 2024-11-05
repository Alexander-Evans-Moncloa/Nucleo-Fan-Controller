/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include <cstdio>

// Blinking rate in milliseconds
#define BLINKING_RATE     500ms

    // Initialise the digital pins
#ifndef PINS
#define PINS
    // Digital Outputs
    DigitalOut led(LED1);
    DigitalOut extensionBoardLed(PC_0);
    DigitalOut biDirLedA(PB_7);
    DigitalOut biDirLedB(PA_15);
    
    // Digital Inputs
    DigitalIn rotaryA(PA_1);
    DigitalIn rotaryB(PA_4);

    // PWM Signals
    PwmOut fanPWM(PB_0);

    // I2C Signals
    I2C tempSensorI2C(PB_9, PB_8);

    // Interrupts
    InterruptIn button(BUTTON1);
#else
    bool led;
    bool extensionBoardLed;
#endif

// Define the Serial USB Output
BufferedSerial mypc(USBTX, USBRX);

// These are GLOBAL Variables
short int rotaryEncoderStage = 0;
const int tempSensAddr = 0x9A; // I2C Address for temperature sensor

FILE* mypcFile1 = fdopen(&mypc, "r+"); // Set up the Serial USB Ports

// =============================== INTERRUPTS ===============================

void flip() 
{ // Flips extension board on call
    extensionBoardLed = !extensionBoardLed;
} 

// =============================== OTHER FUNCTIONS ===============================

void blinky()
{ // Blinky Example Code (Don't use)
    led = !led;
    ThisThread::sleep_for(BLINKING_RATE);
}

void rotaryEncoderDirectionLED()
{ // Check OneNote for details
    // Clockwise
    if ((rotaryEncoderStage == 0) && ((!rotaryA) && (rotaryB))) rotaryEncoderStage++;
    if ((rotaryEncoderStage == 1) && ((!rotaryA) && (!rotaryB))) rotaryEncoderStage++;
    if ((rotaryEncoderStage == 2) && ((rotaryA) && (!rotaryB))) rotaryEncoderStage++;

    if ((rotaryEncoderStage == 3) && ((rotaryA) && (rotaryB))) {
        rotaryEncoderStage = 0;
        biDirLedA = 1;
        biDirLedB = 0;
    }

    // Counter-Clockwise
    if ((rotaryEncoderStage == 0) && ((rotaryA) && (!rotaryB))) rotaryEncoderStage--;
    if ((rotaryEncoderStage == -1) && ((!rotaryA) && (!rotaryB))) rotaryEncoderStage--;
    if ((rotaryEncoderStage == -2) && ((!rotaryA) && (rotaryB))) rotaryEncoderStage--;

    if ((rotaryEncoderStage == -3) && ((rotaryA) && (rotaryB))) {
        rotaryEncoderStage = 0;
        biDirLedA = 0;
        biDirLedB = 1;
    }

    // For cases where the encoder is spun too quickly
    if ((rotaryEncoderStage != 0) && ((rotaryA) && (rotaryB))) rotaryEncoderStage = 0;
}



int openLoopControl()
{ // Check OneNote for details
    int counter = 0;

    // Clockwise
    if ((rotaryEncoderStage == 0) && ((!rotaryA) && (rotaryB))) rotaryEncoderStage++;
    if ((rotaryEncoderStage == 1) && ((!rotaryA) && (!rotaryB))) rotaryEncoderStage++;
    if ((rotaryEncoderStage == 2) && ((rotaryA) && (!rotaryB))) rotaryEncoderStage++;

    if ((rotaryEncoderStage == 3) && ((rotaryA) && (rotaryB))) {
        rotaryEncoderStage = 0;
        biDirLedA = 1;
        biDirLedB = 0;

        counter = 1;
        //fanPWM.write(incrementalCounter);
    }

    // Counter-Clockwise
    if ((rotaryEncoderStage == 0) && ((rotaryA) && (!rotaryB))) rotaryEncoderStage--;
    if ((rotaryEncoderStage == -1) && ((!rotaryA) && (!rotaryB))) rotaryEncoderStage--;
    if ((rotaryEncoderStage == -2) && ((!rotaryA) && (rotaryB))) rotaryEncoderStage--;

    if ((rotaryEncoderStage == -3) && ((rotaryA) && (rotaryB))) {
        rotaryEncoderStage = 0;
        biDirLedA = 0;
        biDirLedB = 1;

        counter = -1;
        //fanPWM.write(incrementalCounter)
    }

    // For cases where the encoder is spun too quickly
    if ((rotaryEncoderStage != 0) && ((rotaryA) && (rotaryB))) rotaryEncoderStage = 0;

    return counter;
}

// I2C Interface Functions (Temperature Sensor)

void findI2CDevices() 
{
    //Check whether the sensor is connected
    fprintf(mypcFile1,"\033[0m\033[2J\033[HI2C Searching!\n\n\n");

    int count = 0;
    fprintf(mypcFile1,"Starting....\n\n");

    for (int address=0; address<256; address+=2) {
        if (!tempSensorI2C.write(address, NULL, 0)) { // 0 returned is ok
            fprintf(mypcFile1,"I2C address 0x%02X\n", address);
            count++;
        }
    }

    fprintf(mypcFile1,"\n\n%d devices found\n", count);

    wait_us(20000);
}

char getTemperatureReading()
{
    char tempSensData[2];
    
    // Display the temperature start sentence
    // Write to the tempSensAddress by modifying the last bit of the tempSensAddress 
    // and using a write command and write a "read temperature" command
    tempSensorI2C.write(tempSensAddr, 0x00, 1); 

    //Read the Data from the device by changing the last bit of the tempSensAddress to a read command
    tempSensorI2C.read(tempSensAddr, tempSensData, 1); 
    wait_us(10);


    //-----------------Print out section ----------------------
    //Display device Address and informations
    //fprintf(mypcFile1,"Device with tempSensAddress 0x%x with\r\n", tempSensAddr); 

    //Prints out the result of Method 1

    fprintf(mypcFile1,"Method 1: %d \t Method 2: %d\n\r", tempSensData[0], tempSensData[1]);
    wait_us(10); 

    return tempSensData[0];
}

// =============================== MAIN CODE ===============================

int main() 
{
    led = 0;
    extensionBoardLed = 0;

    button.mode(PullUp);
    button.rise(&flip);

    // Temp PWM Stuff
    fanPWM.period(0.1f);
    fanPWM.write(1.0f);

    // Local Variables
    char temperatureData;
    int incrementalCounter = 0;

    while (true) {
        // blinky();
        //rotaryEncoderDirectionLED();

        //temperatureData = getTemperatureReading();

        incrementalCounter += openLoopControl();
        printf("Counter Value: %d\n", incrementalCounter);

    }
}

// Write the fan speed stuff
// 