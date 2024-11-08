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
const int MILLISECOND    = 1000;
const int TENTH_SECOND   = 100000;
const int QUARTER_SECOND = 250000;
const int HALF_SECOND    = 500000;

const int FAN_PWM_PERIOD = MILLISECOND*10;
const int PULSE_STRETCH_PERIOD = QUARTER_SECOND; // Stretch the PWM signal for 10ms to measure TACO

// =============================== GLOBAL VARS ===============================
short int rotaryEncoderStage = 0;
int fanTACOCounter = 0;
float maxFanRPM = 0.0f;

// Tachometer Reading & Pulse Stretching
Timer globalTimer;
bool fanTachometerReading = 0;

FILE* mypcFile1 = fdopen(&mypc, "r+"); // Set up the Serial USB Ports

// =============================== INTERRUPTS ===============================

void flip() 
{ // Flips extension board on call
    extensionBoardLed = !extensionBoardLed;
} 

void incrementTACO()
{
    if (fanTachometerReading) fanTACOCounter++;
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
    if (fanRPM > maxFanRPM) maxFanRPM = fanRPM;

    int fanSpeedPercentage = (int)((fanRPM/maxFanRPM)*100);
    int tempFanRPM = (int)round(fanRPM);
    int tempMaxFanRPM = (int)round(maxFanRPM);
    printf("Fan RPM: %d\t Max Fan Speed: %d\tFan Speed Percentage: %d\n", tempFanRPM, tempMaxFanRPM, fanSpeedPercentage);

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

    float fanPeriod = 0.00005;
    float fanSpeedPWM = 1.0f;

    int tempFanSpeed = 0;
    float fanSpeedRPM = 0.5f;

    Timer mainLoopTimer;

    // Set initial states
    led = 0;
    extensionBoardLed = 0;

    button.mode(PullUp);
    button.rise(&flip);

    //fanTACO.mode(PullUp);
    fanTACO.fall(&incrementTACO);

    //fanPWM.period(fanPeriod);
    fanPWM.period_us(FAN_PWM_PERIOD);
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

        if (!fanTachometerReading) fanPWM.write(fanSpeedPWM);
        //fanSpeedRPM = readFanSpeed();

        // 
        if (mainLoopTimer.elapsed_time().count() >= HALF_SECOND*4) {
            if (!fanTachometerReading) {
                globalTimer.start();
                fanTachometerReading = 1;
                fanPWM.write(1.0f);

            }

            mainLoopTimer.stop();
            mainLoopTimer.reset();
            mainLoopTimer.start();
        }

        // Check if the pulse stretch period has been exceeded
        if (globalTimer.elapsed_time().count() >= PULSE_STRETCH_PERIOD) {
            fanTachometerReading = 0;

            fanPWM.write(fanSpeedPWM);

            // Pulse Stretch the PWM signal, measure the Tachometer readings
            printf("Fan PWM: %d\t", tempFanSpeed);

            globalTimer.stop();
            readFanSpeed(globalTimer.elapsed_time().count());

            globalTimer.reset();
            globalTimer.start();
        }
    }
}

// Write the fan speed stuff
// 