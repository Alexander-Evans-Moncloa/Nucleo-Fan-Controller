/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "LCD_ST7066U.h"
#include <cstdio>
#include <cstring>
#include <string>

// ====================================================================
// =============================== PINS ===============================
// ==================================================================== 

// Digital Outputs
BusOut boardLeds(LED1,  // Nucleo Board LED
                 PC_0); // Extension Board LED

BusOut biDirLeds(PB_7,   // Bi-directional LED Side A
                 PA_15); // Bi-directional LED Side B

BusOut sevenSegPWR( PC_8,  // Left Display
                    PC_6); // Right Display

DigitalIn testSignal(PC_4);

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

// LCD
LCD LCDScreen(PA_5,  // RS
              PA_6,  // E
              PA_7,  // DB0
              PB_6,  // DB1
              PC_7,  // DB2
              PA_9,  // DB3
              PA_8,  // DB4
              PB_10, // DB5
              PB_4,  // DB6
              PB_5); // DB7


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
const int FAN_PWM_PERIOD = MILLISECOND*10; // Prev Val: MILLISECOND*10
const int FAN_SPEED_UPDATE_PERIOD = HALF_SECOND;

const int MIN_TACO_PERIOD = MILLISECOND*20; // Calculation based off 3000RPM
const int MIN_TACO_POSITIVE_PULSE_WIDTH = MILLISECOND*10;
const int MIN_TACO_ALLOWANCE_PERIOD = MILLISECOND*5;

// =============================== PID CONTROL ===============================
const float fanKp = 0.5;  // Proportional gain
const float fanKi = 0.1;  // Integral gain
const float fanKd = 0.05; // Derivative gain

// ===========================================================================
// =============================== GLOBAL VARS ===============================
// ===========================================================================

short int rotaryEncoderStage = 0;
int fanTACOCounter = 0;
int maxFanRPM = 0;
int tacoAllowancePeriod = MILLISECOND*5;
float fanSpeedPWM = 1.0f;

// Enumerator for the modes
enum Mode {
    OPEN_LOOP,
    CLOSED_LOOP_FAN,
    CLOSED_LOOP_TEMP
};
enum Mode boardMode = OPEN_LOOP;

// Override the Post Increment Value so that it loops across the enumerator
Mode operator++(Mode& mode, int) {
    if (mode == Mode::CLOSED_LOOP_TEMP) {
        mode = Mode::OPEN_LOOP; // Wrap around to the beginning if needed
    } else {
        mode = static_cast<Mode>(static_cast<int>(mode) + 1);
    }
    return mode;
}

// Tachometer Reading & Pulse Stretching
Timer mainLoopTimer;
Timer tacoPeriodTimer;

// Set up the Serial USB Ports
FILE* mypcFile1 = fdopen(&mypc, "r+"); 

// ==========================================================================
// =============================== INTERRUPTS ===============================
// ==========================================================================

void changeMode() 
{ // Changes the mode of the board and reset timer
    mainLoopTimer.stop();
    boardMode++;

    mainLoopTimer.reset();
    mainLoopTimer.start();
    fanTACOCounter = 0;
} 

void incrementTACO()
{
    tacoPeriodTimer.start();
}

void checkTACOPulseWidth()
{
    tacoPeriodTimer.stop();
    int tacoPeriod = tacoPeriodTimer.elapsed_time().count();   
    if (tacoPeriod > tacoAllowancePeriod) fanTACOCounter++;
    
    tacoPeriodTimer.reset();
}

// ===============================================================================
// =============================== OTHER FUNCTIONS ===============================
// ===============================================================================

// =============================== MISC FUNCTIONS ===============================
void rotaryEncoderDirectionLED()
{ // Check OneNote for details
    // Clockwise
    if ((rotaryEncoderStage == 0) && ((!rotaryA) && (rotaryB))) rotaryEncoderStage++;
    if ((rotaryEncoderStage == 1) && ((!rotaryA) && (!rotaryB))) rotaryEncoderStage++;
    if ((rotaryEncoderStage == 2) && ((rotaryA) && (!rotaryB))) rotaryEncoderStage++;

    if ((rotaryEncoderStage == 3) && ((rotaryA) && (rotaryB))) {
        rotaryEncoderStage = 0;
        biDirLeds = 0b01;
    }

    // Counter-Clockwise
    if ((rotaryEncoderStage == 0) && ((rotaryA) && (!rotaryB))) rotaryEncoderStage--;
    if ((rotaryEncoderStage == -1) && ((!rotaryA) && (!rotaryB))) rotaryEncoderStage--;
    if ((rotaryEncoderStage == -2) && ((!rotaryA) && (rotaryB))) rotaryEncoderStage--;

    if ((rotaryEncoderStage == -3) && ((rotaryA) && (rotaryB))) {
        rotaryEncoderStage = 0;
        biDirLeds = 0b10;
    }

    // For cases where the encoder is spun too quickly
    if ((rotaryEncoderStage != 0) && ((rotaryA) && (rotaryB))) rotaryEncoderStage = 0;
}

void intToChar(int num, char *result) {
    int temp = num;
    int len = 0;
 
    while (temp > 0) {
        len++;
        temp /= 10;
    }
 
    for (int i = len - 1; i >= 0; i--) {
        result[i] = num % 10 + '0';
        num /= 10;
    }
 
    result[len] = '\0';
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

int readFanSpeed() 
{ 
    //int timeDelta = mainLoopTimer.elapsed_time().count();
    if (mainLoopTimer.elapsed_time().count() >= FAN_SPEED_UPDATE_PERIOD) {
        mainLoopTimer.stop();
        // Remove random time variations in the 10us range
        //timeDelta -= (timeDelta % 1000);

        if (fanTACOCounter == 1) fanTACOCounter = 2;
        // RPM = (TACO Ticks/2) / (Time converted from microseconds to minutes)
        //int fanRPM = ((float)fanTACOCounter/timeDelta) * 30000000;
        int fanRPM = fanTACOCounter*60; // Assume 0.5s time delta
        if (fanRPM > maxFanRPM) maxFanRPM = fanRPM;

        // Calculate Debug Info
        int fanSpeedPercentage = (int)(((float)fanRPM/maxFanRPM)*100);

        // Print Debug Info
        printf("Fan RPM: %d\t Max Fan Speed: %d\tFan Speed Percentage: %d\tFan TACO: %d\n", 
                fanRPM, maxFanRPM, fanSpeedPercentage, fanTACOCounter);
        
        fanTACOCounter = 0;

        // Display the RPM
        char currentRpmChar[16];
        sprintf(currentRpmChar, "RPM: %d", fanRPM);

        char maxRpmChar[16];
        sprintf(maxRpmChar, "Max RPM: %d", maxFanRPM);

        LCDScreen.clear();
        LCDScreen.writeLine(maxRpmChar, 0);
        LCDScreen.writeLine(currentRpmChar, 1);

        // Reset the Timers
        mainLoopTimer.reset();
        mainLoopTimer.start();

        return fanRPM;
    }
    return 0;
}

// =============================== I2C ===============================
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

    //fprintf(mypcFile1,"Method 1: %d \t Method 2: %d\n\r", tempSensData[0], tempSensData[1]);
    wait_us(10); 

    return tempSensData[0];
}

// =============================== CONTROL ===============================

int changeFanSpeed()
{   // Check OneNote for details
    // Returns +1 to increase speed, Returns -1 to decrease speed, Returns 0 to keep it constant

    // Clockwise Stages
    if ((rotaryEncoderStage == 0) && ((!rotaryA) && (rotaryB))) rotaryEncoderStage++;
    if ((rotaryEncoderStage == 1) && ((!rotaryA) && (!rotaryB))) rotaryEncoderStage++;
    if ((rotaryEncoderStage == 2) && ((rotaryA) && (!rotaryB))) rotaryEncoderStage++;

    if ((rotaryEncoderStage == 3) && ((rotaryA) && (rotaryB))) {
        rotaryEncoderStage = 0;
        biDirLeds = 0b01;

        return 1;
    }

    // Counter-Clockwise Stages
    if ((rotaryEncoderStage == 0) && ((rotaryA) && (!rotaryB))) rotaryEncoderStage--;
    if ((rotaryEncoderStage == -1) && ((!rotaryA) && (!rotaryB))) rotaryEncoderStage--;
    if ((rotaryEncoderStage == -2) && ((!rotaryA) && (rotaryB))) rotaryEncoderStage--;

    if ((rotaryEncoderStage == -3) && ((rotaryA) && (rotaryB))) {
        rotaryEncoderStage = 0;
        biDirLeds = 0b10;

        return -1;
    }

    // For cases where the encoder is spun too quickly
    if ((rotaryEncoderStage != 0) && ((rotaryA) && (rotaryB))) rotaryEncoderStage = 0;

    return 0;
}

void openLoopControl()
{
    float speedChangeValue = 0.0;
    while (boardMode == OPEN_LOOP) {
        // Retrieve the value to change
        speedChangeValue = changeFanSpeed();
        speedChangeValue /= 100;

        // Apply Speed Change
        if      (fanSpeedPWM + speedChangeValue > 1) fanSpeedPWM = 1.0f;
        else if (fanSpeedPWM + speedChangeValue < 0.05) fanSpeedPWM = 0.05f;
        else    fanSpeedPWM += speedChangeValue;

        // Set the Allowance Period for Reading Tachometer
        if      (fanSpeedPWM <= 0.1) tacoAllowancePeriod = MILLISECOND*35; // Previous Multiplier: 40
        else if (fanSpeedPWM <= 0.2) tacoAllowancePeriod = MILLISECOND*20;
        else if (fanSpeedPWM <= 0.3) tacoAllowancePeriod = MILLISECOND*12;
        else if (fanSpeedPWM <= 0.4) tacoAllowancePeriod = MILLISECOND*10;
        else                         tacoAllowancePeriod = MILLISECOND*5;

        // Change the fan speed
        fanPWM.write(fanSpeedPWM);

        int tempPWM = fanSpeedPWM*100;

        // Update the Fan Speed
        int currentFanSpeed = readFanSpeed();
        sevenSegPrint(tempPWM);
    }
}

void closedLoopControlFan()
{
    // Desired speed (setpoint in RPM)
    int setpoint = 1500; // Adjust as needed

    // PID state variables
    float integral = 0.0f;
    float previousError = 0.0f;
    
    while (boardMode == CLOSED_LOOP_FAN) {
        LCDScreen.clear();
        LCDScreen.writeLine("ABCDEFGH",0);
        LCDScreen.writeLine("01234567",0);
    }
}

void closedLoopControlTemp() // Limit to 32 characters, 16 per row.
{
    int temperatureData;
    while (boardMode == CLOSED_LOOP_TEMP) {
        // Get the Temperature Data
        temperatureData = getTemperatureReading();
        sevenSegPrint(temperatureData);
    }
}

// =========================================================================
// =============================== MAIN CODE ===============================
// =========================================================================

int main() 
{
    // Set initial states
    boardLeds = 0b00;

    button.mode(PullUp);
    button.rise(&changeMode);

    fanTACO.rise(&incrementTACO);
    fanTACO.fall(&checkTACOPulseWidth);

    fanPWM.period_us(FAN_PWM_PERIOD);
    fanPWM.write(fanSpeedPWM);

    // Start the timer and the main loop
    mainLoopTimer.start();
    while(1) {

        boardLeds = 0b01;
        openLoopControl();

        boardLeds = 0b10;
        closedLoopControlFan();

        boardLeds = 0b11;
        closedLoopControlTemp();
    }
}