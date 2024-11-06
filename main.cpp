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

    //AnalogIn fanTACO(PA_0);

    // PWM Signals
    PwmOut fanPWM(PB_0);

    // I2C Signals
    I2C tempSensorI2C(PB_9, PB_8);

    // Interrupts
    InterruptIn button(BUTTON1);
    InterruptIn fanTACO(PA_0);
#else
    bool led;
    bool extensionBoardLed;
#endif

// Define the Serial USB Output
BufferedSerial mypc(USBTX, USBRX);

// =============================== CONSTANTS ===============================
const int TEMP_SENS_ADDR = 0x9A; // I2C Address for temperature sensor

// Microsecond conversions
const int MILLISECOND = 1000;
const int HALF_SECOND = 500000;

// =============================== GLOBAL VARS ===============================
short int rotaryEncoderStage = 0;
int fanTACOCounter = 0;

FILE* mypcFile1 = fdopen(&mypc, "r+"); // Set up the Serial USB Ports

// =============================== INTERRUPTS ===============================

void flip() 
{ // Flips extension board on call
    extensionBoardLed = !extensionBoardLed;
} 

void incrementTACO()
{
    fanTACOCounter++;
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

void readFanSpeed(int timeDelta) 
{ // RPM = (TACO Ticks/2) / (Time converted from microseconds to minutes)

    float fanRPM = ((float)fanTACOCounter/2) / ((float)timeDelta/(60000000));
    int tempfanRPM = (int)round(fanRPM);
    printf("Time Elapsed: %d\tFan RPM: %d\n", timeDelta, tempfanRPM);

    fanTACOCounter = 0;
}

int openLoopControl()
{ // Check OneNote for details

    // Clockwise
    if ((rotaryEncoderStage == 0) && ((!rotaryA) && (rotaryB))) rotaryEncoderStage++;
    if ((rotaryEncoderStage == 1) && ((!rotaryA) && (!rotaryB))) rotaryEncoderStage++;
    if ((rotaryEncoderStage == 2) && ((rotaryA) && (!rotaryB))) rotaryEncoderStage++;

    if ((rotaryEncoderStage == 3) && ((rotaryA) && (rotaryB))) {
        rotaryEncoderStage = 0;
        biDirLedA = 1;
        biDirLedB = 0;

        return 1;
    }

    // Counter-Clockwise
    if ((rotaryEncoderStage == 0) && ((rotaryA) && (!rotaryB))) rotaryEncoderStage--;
    if ((rotaryEncoderStage == -1) && ((!rotaryA) && (!rotaryB))) rotaryEncoderStage--;
    if ((rotaryEncoderStage == -2) && ((!rotaryA) && (rotaryB))) rotaryEncoderStage--;

    if ((rotaryEncoderStage == -3) && ((rotaryA) && (rotaryB))) {
        rotaryEncoderStage = 0;
        biDirLedA = 0;
        biDirLedB = 1;

        return -1;
    }

    // For cases where the encoder is spun too quickly
    if ((rotaryEncoderStage != 0) && ((rotaryA) && (rotaryB))) rotaryEncoderStage = 0;

    return 0;
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
    tempSensorI2C.write(TEMP_SENS_ADDR, 0x00, 1); 

    //Read the Data from the device by changing the last bit of the tempSensAddress to a read command
    tempSensorI2C.read(TEMP_SENS_ADDR, tempSensData, 1); 
    wait_us(10);


    //-----------------Print out section ----------------------
    //Display device Address and informations
    //fprintf(mypcFile1,"Device with tempSensAddress 0x%x with\r\n", TEMP_SENS_ADDR); 

    //Prints out the result of Method 1

    fprintf(mypcFile1,"Method 1: %d \t Method 2: %d\n\r", tempSensData[0], tempSensData[1]);
    wait_us(10); 

    return tempSensData[0];
}

// =============================== MAIN CODE ===============================

int main() 
{
    // Local Variables
    char temperatureData;
    float speedChangeValue = 0;

    float fanSpeedPWM = 1.0f;

    int tempFanSpeed = 0;
    float fanSpeedRPM = 0.5f;
    short int mainLoopCounter = 0;

    Timer mainLoopTimer;

    // Set initial states
    led = 0;
    extensionBoardLed = 0;

    button.mode(PullUp);
    button.rise(&flip);

    fanTACO.mode(PullUp);
    fanTACO.rise(&incrementTACO);

    fanPWM.period(0.01f);
    fanPWM.write(fanSpeedPWM);

    // Start the timer and the main loop
    mainLoopTimer.start();
    while (true) {
        // blinky();
        //rotaryEncoderDirectionLED();

        //temperatureData = getTemperatureReading();
        
        speedChangeValue = openLoopControl();
        speedChangeValue /= 100;

        if (fanSpeedPWM + speedChangeValue > 1) fanSpeedPWM = 1.0f;
        else if (fanSpeedPWM + speedChangeValue < 0) fanSpeedPWM = 0.0f;
        else fanSpeedPWM += speedChangeValue;

        tempFanSpeed = fanSpeedPWM * 100;
        //printf("Counter Value: %d\n", tempFanSpeed);

        fanPWM.write(fanSpeedPWM);
        //fanSpeedRPM = readFanSpeed();

        mainLoopCounter++;

        if (mainLoopTimer.elapsed_time().count() >= HALF_SECOND) {
            mainLoopTimer.stop();
            printf("fanTACO: %d \t Fan PWM: %d\t", fanTACOCounter, tempFanSpeed);
            readFanSpeed(mainLoopTimer.elapsed_time().count());

            mainLoopTimer.reset();
            mainLoopTimer.start();
        }
    }
}

// Write the fan speed stuff
// Fan Period could be altered to allow for low speed control by decreasing the period and duty cycle