/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include <cstdio>

// ====================================================================
// =============================== PINS ===============================
// ==================================================================== 

// Digital Outputs
DigitalOut led(LED1);
DigitalOut extensionBoardLed(PC_0);
DigitalOut biDirLedA(PB_7);
DigitalOut biDirLedB(PA_15);

BusOut sevenSegPWR( PC_8,  // Left Display
                    PC_6); // Right Display

// This will be multiplexed with both displays. MSB will be PB_1
BusOut sevenSegDisplay_L( PC_5,  // g
                        PA_12, // f
                        PA_11, // e 
                        PB_12, // d
                        PB_11, // c
                        PB_2,  // b
                        PB_1); // a
    
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


// Define the Serial USB Output
BufferedSerial mypc(USBTX, USBRX);

// =========================================================================
// =============================== CONSTANTS ===============================
// =========================================================================

const int TEMP_SENS_ADDR = 0x9A; // I2C Address for temperature sensor

// =============================== TIMING-BASED ===============================

// Microsecond Timings
const int MILLISECOND    = 1000;
const int TENTH_SECOND   = 100000;
const int QUARTER_SECOND = 250000;
const int HALF_SECOND    = 500000;

// Millisecond Timings
#define BLINKING_RATE     500ms
#define MULTIPLEX_BLINKING_RATE 5ms

// Periods
const int FAN_PWM_PERIOD = MILLISECOND*10;
const int PULSE_STRETCH_PERIOD = MILLISECOND*150; // Stretch the PWM signal to measure TACO

// ===========================================================================
// =============================== GLOBAL VARS ===============================
// ===========================================================================

short int rotaryEncoderStage = 0;
int fanTACOCounter = 0;
int maxFanRPM = 0;

// Tachometer Reading & Pulse Stretching
Timer globalTimer;
bool fanTachometerReading = 0;

FILE* mypcFile1 = fdopen(&mypc, "r+"); // Set up the Serial USB Ports

// ==========================================================================
// =============================== INTERRUPTS ===============================
// ==========================================================================

void flip() 
{ // Flips extension board on call
    extensionBoardLed = !extensionBoardLed;
} 

void incrementTACO()
{
    if (fanTachometerReading) {fanTACOCounter++; led = !led;}
    led = 0;
}

// ===============================================================================
// =============================== OTHER FUNCTIONS ===============================
// ===============================================================================

// =============================== MISC FUNCTIONS ===============================

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

// =============================== ANALYTICS ===============================

void sevenSegPrintDigit(int value)
{
    // Order of bits: a,b,c,d,e,f,g
    switch (value) {
        case 0:
            sevenSegDisplay_L = 0b0000001; // 0b1111110;
            break;
        case 1:
            sevenSegDisplay_L = 0b1001111; // 0b00110000;
            break;
        case 2:
            sevenSegDisplay_L = 0b0010010; // 0b1101101;
            break;
        case 3:
            sevenSegDisplay_L = 0b0000110; // 0b1111001;
            break;
        case 4:
            sevenSegDisplay_L = 0b1001100; // 0b0110011;
            break;
        case 5:
            sevenSegDisplay_L = 0b0100100; // 0b1011011;
            break;
        case 6:
            sevenSegDisplay_L = 0b0100000; // 0b1011111;
            break;
        case 7:
            sevenSegDisplay_L = 0b0001111; // 0b1110000;
            break;
        case 8:
            sevenSegDisplay_L = 0b0000000; // 0b1111111;
            break;
        case 9:
            sevenSegDisplay_L = 0b0000100; // 0b1111011;
            break;
        default:
            sevenSegDisplay_L = 0b1111111; // 0b00000000;
            break;
    }
}

void sevenSegPrint(int valueToDisplay)
{
    int tens = valueToDisplay / 10;
    int units = valueToDisplay % 10;

    ThisThread::sleep_for(MULTIPLEX_BLINKING_RATE);

    sevenSegPWR = 0b01;
    sevenSegPrintDigit(tens);
    
    ThisThread::sleep_for(MULTIPLEX_BLINKING_RATE);

    sevenSegPWR = 0b10;
    sevenSegPrintDigit(units);
    
}

void readFanSpeed(int timeDelta) 
{ 
    // Remove random time variations in the 10us range
    timeDelta -= (timeDelta % 100);

    // RPM = (TACO Ticks/2) / (Time converted from microseconds to minutes)
    int fanRPM = (fanTACOCounter*30000000) / timeDelta;
    if (fanRPM > maxFanRPM) maxFanRPM = fanRPM;

    // Calculate Debug Info
    int fanSpeedPercentage = (int)(((float)fanRPM/maxFanRPM)*100);
    int tempFanRPM = (int)round(fanRPM);
    int tempMaxFanRPM = (int)round(maxFanRPM);

    // Print Debug Info
    printf("Fan RPM: %d\t Max Fan Speed: %d\tFan Speed Percentage: %d\tFan TACO: %d\tTime Delta: %d\n", 
            tempFanRPM, tempMaxFanRPM, fanSpeedPercentage, fanTACOCounter, timeDelta);
     
    fanTACOCounter = 0;
}

// =============================== CONTROL ===============================

int openLoopControl()
{   // Check OneNote for details
    // Returns +1 to increase speed, Returns -1 to decrease speed, Returns 0 to keep it constant

    // Clockwise Stages
    if ((rotaryEncoderStage == 0) && ((!rotaryA) && (rotaryB))) rotaryEncoderStage++;
    if ((rotaryEncoderStage == 1) && ((!rotaryA) && (!rotaryB))) rotaryEncoderStage++;
    if ((rotaryEncoderStage == 2) && ((rotaryA) && (!rotaryB))) rotaryEncoderStage++;

    if ((rotaryEncoderStage == 3) && ((rotaryA) && (rotaryB))) {
        rotaryEncoderStage = 0;
        biDirLedA = 1;
        biDirLedB = 0;

        return 1;
    }

    // Counter-Clockwise Stages
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

// =============================== I2C ===============================

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

// =========================================================================
// =============================== MAIN CODE ===============================
// =========================================================================

int main() 
{
    // Local Variables
    char temperatureData;
    float speedChangeValue = 0;

    float fanSpeedPWM = 1.0f;

    int tempFanSpeed = 0;

    Timer mainLoopTimer;

    // Set initial states
    led = 0;
    extensionBoardLed = 0;

    button.mode(PullUp);
    button.rise(&flip);

    fanTACO.fall(&incrementTACO);

    fanPWM.period_us(FAN_PWM_PERIOD);
    fanPWM.write(fanSpeedPWM);

    // Start the timer and the main loop
    mainLoopTimer.start();
    while (true) {
        // Get the Temperature Data
        //temperatureData = getTemperatureReading();
        
        // Retrieve the value to change
        speedChangeValue = openLoopControl();
        speedChangeValue /= 100;

        if (fanSpeedPWM + speedChangeValue > 1) fanSpeedPWM = 1.0f;
        else if (fanSpeedPWM + speedChangeValue < 0) fanSpeedPWM = 0.0f;
        else fanSpeedPWM += speedChangeValue;

        tempFanSpeed = fanSpeedPWM * 100;

        // Change the fan speed
        if (!fanTachometerReading) fanPWM.write(fanSpeedPWM);

        // Start Pulse Stretching
        if (mainLoopTimer.elapsed_time().count() >= HALF_SECOND*2) {
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
        if (globalTimer.elapsed_time().count() >= PULSE_STRETCH_PERIOD && fanTachometerReading) {
            fanTachometerReading = 0;
            globalTimer.stop();

            printf("Fan PWM: %d\t", tempFanSpeed);

            readFanSpeed(globalTimer.elapsed_time().count());
            globalTimer.reset();

            fanPWM.write(fanSpeedPWM);
        }

        // Display the Information
        sevenSegPrint(tempFanSpeed);
    }
}