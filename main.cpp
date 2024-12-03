/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "LCD_ST7066U.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <math.h>

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

// =============================== MICROSECOND ===============================
const int MILLISECOND    = 1000;
const int TENTH_SECOND   = 100000;
const int QUARTER_SECOND = 250000;
const int HALF_SECOND    = 500000;
const int SECOND         = 1000000;
const int HALF_MINUTE    = 30000000;
const int MINUTE         = 60000000;

// =============================== FAN DETAILS ===============================
const int FAN_PWM_PERIOD = MILLISECOND*10;
const int FAN_SPEED_UPDATE_PERIOD = SECOND; // Prev: SECOND
const int PID_UPDATE_PERIOD = QUARTER_SECOND;

const int MIN_TACO_PERIOD = MILLISECOND*20; // Calculation based off 3000RPM
const int MIN_TACO_POSITIVE_PULSE_WIDTH = MILLISECOND*10;
const int MIN_TACO_ALLOWANCE_PERIOD = MILLISECOND*5;

const int MAX_FAN_SPEED = 2600;
const int MIN_FAN_SPEED = 60;
const int MID_FAN_SPEED = 1440;

// =============================== OTHER ===============================
// RPM = Revs/Min
// RPM = (tacoCount/2)/time_min
// RPM = (tacoCount/2)/(time_us/(1,000,000*60))
// RPM = (tacoCount/2)*(60,000,000/time_us)
// RPM = (tacoCount*30,000,000)/time_us
// Vary this value
const char REVOLUTIONS_TO_DO = 2;
const char REVOLUTIONS_TO_TACO_COUNTER = (2*REVOLUTIONS_TO_DO)+1;
const int RPM_CALCULATION_PROPORTION = (REVOLUTIONS_TO_DO)*MINUTE;

const char POS_EDGE_ARRAY_LENGTH = 7;
const char POS_NEG_PULSES = REVOLUTIONS_TO_DO*4;

const float FAN_FILTER_ALPHA = 0.3; // Smoothing Factor

// ===========================================================================
// =============================== GLOBAL VARS ===============================
// ===========================================================================

short int rotaryEncoderStage = 0;

int fanTACOCounter = 0;
int previousTACOCounter = 0;
int fanPulseDelta = 0;
int posEdgeTime = 0;
int fanPulseDeltas[POS_EDGE_ARRAY_LENGTH];

volatile bool isPosGlitch = false;
volatile bool isNegGlitch = false;
volatile bool isMeasuring = false;
volatile bool isRiseInterrupt = false;
volatile bool isFallInterrupt = true;

int posPulseDelta = 0;
int negPulseDelta = 0;
int fanPulseGlitchDelta = 0;

int testPos = 0;
int testNeg = 0;
Timer testTimer;

int measuredFanSpeed = 0;
int currentFanSpeed = 0;

int tacoAllowancePeriod = MILLISECOND*5;
bool tacoReading = 0;
float fanSpeedPWM = 1.0;

// Enumerator for the modes
enum Mode {
    OPEN_LOOP,
    CLOSED_LOOP_FAN,
    CLOSED_LOOP_TEMP,
    CLOSED_LOOP_FAN_DEMO
};
enum Mode boardMode = OPEN_LOOP;

// Override the Post Increment Value so that it loops across the enumerator
Mode operator++(Mode& mode, int) {
    if (mode == Mode::CLOSED_LOOP_FAN_DEMO) {
        mode = Mode::OPEN_LOOP; // Wrap around to the beginning if needed
    } else {
        mode = static_cast<Mode>(static_cast<int>(mode) + 1);
    }
    return mode;
}

// =============================== PID CONTROL ===============================
const float fanHighSpeedKp = 0.0001;  // Proportional gain
const float fanHighSpeedKi = 0.00000000004;  // Integral gain
const float fanHighSpeedKd = 50; // Derivative gain

const float fanLowSpeedKp  = 0.000075;  // Proportional gain
const float fanLowSpeedKi  = 0.000000000005;  // Integral gain
const float fanLowSpeedKd  = 20; // Derivative gain

const float tempKp = -0.01;  // Proportional gain
const float tempKi = 0.0;  // Integral gain
const float tempKd = 0; // Derivative gain

// Tachometer Reading
Timer mainLoopTimer;    // Used for global timings
Timer posPulseTimer;    // Used for the filtering
Timer negPulseTimer;    // Used for the filtering
Timer pulseTimer;       // Used to calculate the RPM

volatile bool isProcessing = false;

// Set up the Serial USB Ports
FILE* mypcFile1 = fdopen(&mypc, "r+"); 

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

double round_to(double value, double precision = 1.0)
{
    return std::round(value / precision) * precision;
}

void updatePosTimes(int newValue)
{
    for (int i = POS_EDGE_ARRAY_LENGTH-1; i > -1; i--) {
        fanPulseDeltas[i] = fanPulseDeltas[i-1];
    }
    fanPulseDeltas[0] = newValue;
}

int calculateAveragePosTime()
{
    int sum = 0;
    for (int i = 0; i < POS_EDGE_ARRAY_LENGTH; i++) {
        sum += fanPulseDeltas[i];
    }
    return sum/POS_EDGE_ARRAY_LENGTH;
}

void fillPosTimes()
{
    for (int i = 0; i < POS_EDGE_ARRAY_LENGTH; i++) {
        fanPulseDeltas[i] = 2147483647;
    }
}

void printFanDetails() 
{
    if (boardMode == OPEN_LOOP) printf("\n");
    printf("Fan PWM: %d\tCur Fan RPM: %d\tFan TACO: %d\tPrev TACO: %d\tTime Delta: %d\tPos Edge: %d\tNeg Edge: %d", 
        (int)(fanSpeedPWM*100),currentFanSpeed, fanTACOCounter, previousTACOCounter, fanPulseDelta, testPos, testNeg);
}

void checkZeroFanSpeed()
{
    if ((pulseTimer.elapsed_time().count() >= HALF_SECOND)) {
        pulseTimer.stop();

        fanPulseDelta = pulseTimer.elapsed_time().count();
        fillPosTimes();

        // Calculate the speed and set the counter to zero
        currentFanSpeed = 0;
        previousTACOCounter = 0;
        fanTACOCounter = 0;
        pulseTimer.reset();
        pulseTimer.start();
    }
}

void updateTacoAllowance() 
{
    tacoAllowancePeriod = 10250 - 9800*fanSpeedPWM; // Derived From Excel sheet
    if (tacoAllowancePeriod <= MILLISECOND) tacoAllowancePeriod = MILLISECOND;
}

void updateFanSpeed() 
{
    currentFanSpeed = HALF_MINUTE/calculateAveragePosTime();
}

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
    //if (!firstRun) firstRun = true;
    if (!isFallInterrupt) return;
    isFallInterrupt = false;

    posPulseTimer.reset();
    posPulseTimer.start();

    isRiseInterrupt = true;
}

void checkTACOPulseWidth()
{
    //if (!firstRun) return;
    if (!isRiseInterrupt) return;
    isRiseInterrupt = false;

    posPulseTimer.stop();

    posPulseDelta = posPulseTimer.elapsed_time().count();   

    // Check for positive pulse glitches
    if ((posPulseDelta < tacoAllowancePeriod)) {
        isFallInterrupt = true;
        return;
    }

    pulseTimer.stop();

    fanPulseDelta = pulseTimer.elapsed_time().count();

    updatePosTimes(fanPulseDelta);
    pulseTimer.reset();
    pulseTimer.start();

    // When a glitch occurs add the previously stored pulse to the current pulse.
    //updateFanSpeed();
    isFallInterrupt = true;
}

// ===============================================================================
// =============================== OTHER FUNCTIONS ===============================
// ===============================================================================

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

    wait_us(MILLISECOND*5);

    sevenSegPWR = 0b01;
    sevenSegPrintDigit(tens);
    
    wait_us(MILLISECOND*5);

    sevenSegPWR = 0b10;
    sevenSegPrintDigit(units);
    
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

    //printf("Method 1: %d \t Method 2: %d\n\r", tempSensData[0], tempSensData[1]);
    wait_us(10); 

    return tempSensData[0];
}

void checkFanStability(int target)
{
    // Check RPM range
    if (currentFanSpeed < target*1.1 && currentFanSpeed > target*0.9) biDirLeds = 0b01;
    else biDirLeds = 0b10;
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
        //biDirLeds = 0b01;

        return 1;
    }

    // Counter-Clockwise Stages
    if ((rotaryEncoderStage == 0) && ((rotaryA) && (!rotaryB))) rotaryEncoderStage--;
    if ((rotaryEncoderStage == -1) && ((!rotaryA) && (!rotaryB))) rotaryEncoderStage--;
    if ((rotaryEncoderStage == -2) && ((!rotaryA) && (rotaryB))) rotaryEncoderStage--;

    if ((rotaryEncoderStage == -3) && ((rotaryA) && (rotaryB))) {
        rotaryEncoderStage = 0;
        //biDirLeds = 0b10;

        return -1;
    }

    // For cases where the encoder is spun too quickly
    if ((rotaryEncoderStage != 0) && ((rotaryA) && (rotaryB))) rotaryEncoderStage = 0;

    return 0;
}

float computeFanPID(int setpoint, int currentSpeed, float &integral, float &previousError, float dt) {
    int error = setpoint - currentSpeed;

    // Proportional term
    float P = fanHighSpeedKp * error;

    // Integral term
    integral += error * dt;
    float I = fanHighSpeedKi * integral;

    // Derivative term
    float derivative = (error - previousError) / dt;
    float D = fanHighSpeedKd * derivative;

    // Retune the values when the speed is low
    if (setpoint <= 1000 && currentSpeed <= 1000) {
        P = fanLowSpeedKp * error;
        I = fanLowSpeedKi * integral;
        D = fanLowSpeedKd * derivative;
    }

    // Update for next iteration
    previousError = error;

    // Compute the PID output
    float output = P + I + D;

    //output = floorf(output*10000) / 100000;

    printf("P: %d\tI: %d\tD: %d\tOutput: %d\n", (int)(P*10000), (int)(I*10000), (int)(D*10000), (int)(output*10000));

    // Clamp output to valid range
    if (output >= 1) output = 1;
    else if (output <= -1) output = -1;

    return output;
}

float computeTempPID(int setpoint, int currentSpeed, float &integral, float &previousError, float dt) {
    int error = setpoint - currentSpeed;

    // Proportional term
    float P = tempKp * error;

    // Integral term
    integral += error * dt;
    float I = tempKi * integral;

    // Derivative term
    float derivative = (error - previousError) / dt;
    float D = tempKd * derivative;

    // Update for next iteration
    previousError = error;

    // Compute the PID output
    float output = P + I + D;

    //output = floorf(output*10000) / 100000;

    printf("P: %d\tI: %d\tD: %d\tOutput: %d\n", (int)(P*10000), (int)(I*10000), (int)(D*10000), (int)(output*10000));

    // Clamp output to valid range
    if (output >= 1) output = 1;
    else if (output <= -1) output = -1;

    return output;
}

// =============================== MODES ===============================

void openLoopControl()
{
    int calculatedSpeed = 0;
    float speedChangeValue = 0.0;
    bool useCalc = false;
    fanSpeedPWM = 0.5;

    tacoReading = 1;

    // CONTROL LOOP START
    while (boardMode == OPEN_LOOP) {
        updateTacoAllowance();

        calculatedSpeed = 2493.5*pow(fanSpeedPWM,3)-6304.2*pow(fanSpeedPWM,2) + 6610.3*fanSpeedPWM - 175.11;
        //useCalc = ((currentFanSpeed < calculatedSpeed-50) || (currentFanSpeed > calculatedSpeed+50));
        checkZeroFanSpeed();
        // Retrieve the value to change
        speedChangeValue = changeFanSpeed();
        speedChangeValue /= 100;

        // Apply Speed Change
        if      (fanSpeedPWM + speedChangeValue > 1) fanSpeedPWM = 1.0f;
        else if (fanSpeedPWM + speedChangeValue < 0.05) fanSpeedPWM = 0.05f;
        else    fanSpeedPWM += speedChangeValue;

        // Change the fan speed
        fanPWM.write(fanSpeedPWM);

        int tempPWM = fanSpeedPWM*100;

        // Display PWM
        sevenSegPrint(tempPWM);

        updateFanSpeed();
        if (mainLoopTimer.elapsed_time().count() >= FAN_SPEED_UPDATE_PERIOD) {
            mainLoopTimer.stop();
            // Update the Fan Speed
            //currentFanSpeed = readFanSpeed();

            // Display the RPM
            char currentRpmChar[16];
            if (!useCalc) sprintf(currentRpmChar, "RPM: %d", currentFanSpeed);
            else sprintf(currentRpmChar, "RPM: %d", calculatedSpeed);

            char maxRpmChar[16];
            sprintf(maxRpmChar, "Delta: %d", fanPulseDelta);

            LCDScreen.clear();
            LCDScreen.writeLine(maxRpmChar, 0);
            LCDScreen.writeLine(currentRpmChar, 1);
            //printf("\n"); // Used to print a new line 

            printFanDetails();

            // Reset the Timers
            mainLoopTimer.reset();
            mainLoopTimer.start();
        }
    }
    tacoReading = 0;
}

void closedLoopControlFan()
{
    // Desired speed (setpoint in RPM)
    int setpoint = 1500; // Adjust as needed
    int speedChangeValue = 0;

    // PID state variables
    float integral = 0.0f;
    float previousError = 0.0f;
    float dt = PID_UPDATE_PERIOD;
    float pwmFeedForward = 0.04;
    
    // For Display
    char rpmChar[16];
    char targetRpmChar[16];
    
    tacoReading = 1;

    // CONTROL LOOP START
    while (boardMode == CLOSED_LOOP_FAN) {
        updateTacoAllowance();
        checkZeroFanSpeed();
        // Calculate time step

        speedChangeValue = changeFanSpeed();
        speedChangeValue *= 60;

        // Apply Speed Change
        if      (setpoint + speedChangeValue > MAX_FAN_SPEED) setpoint = MAX_FAN_SPEED;
        else if (setpoint + speedChangeValue < MIN_FAN_SPEED) setpoint = MIN_FAN_SPEED;
        else    setpoint += speedChangeValue;

        pwmFeedForward = 0.0000001*(setpoint*setpoint) + 0.00002*(setpoint) + 0.049;

        // Compute new PWM value using PID. Update Every 5 Seconds
        updateFanSpeed();
        if (mainLoopTimer.elapsed_time().count() >= PID_UPDATE_PERIOD) {
            mainLoopTimer.stop();
            // Measure current speed
            //currentFanSpeed = readFanSpeed();
            printFanDetails();

            // Set the PWM Change Value
            float pwm = computeFanPID(setpoint, currentFanSpeed, integral, previousError, dt);

            fanSpeedPWM = pwmFeedForward + pwm;

            // Clamp the PWM Value
            if (fanSpeedPWM >= 1) fanSpeedPWM = 1;
            if (fanSpeedPWM <= 0.05) fanSpeedPWM = 0.05;

            // Apply new PWM value
            fanPWM.write(fanSpeedPWM);
            //setFanSpeed(pwm);
            //printf("Fan PWM: %d\n", (int)(fanSpeedPWM*100));

            sprintf(targetRpmChar, "Target RPM:%d", setpoint);

            // Display the RPM
            LCDScreen.clear();
            LCDScreen.writeLine(targetRpmChar, 0);

            sprintf(rpmChar, "RPM:%d", currentFanSpeed);
            LCDScreen.writeLine(rpmChar, 1);

            // Check the fan stability
            checkFanStability(setpoint);

            // Reset the Timers
            mainLoopTimer.reset();
            mainLoopTimer.start();
        }
        sevenSegPrint((int)(fanSpeedPWM*100));
    }
    tacoReading = 0;
}

void closedLoopControlTemp() // Limit to 32 characters, 16 per row.
{
    // Desired Temp (setpoint in Celsius)
    int setpoint = 25; // Adjust as needed
    int tempChangeValue = 0;

    // PID state variables
    float integral = 0.0f;
    float previousError = 0.0f;
    float dt = PID_UPDATE_PERIOD;
    
    // For Display
    char rpmChar[16];
    char targetTempChar[16];

    int temperatureData;

    tacoReading = 1;

    // CONTROL LOOP START
    while (boardMode == CLOSED_LOOP_TEMP) {
        updateTacoAllowance();
        checkZeroFanSpeed();
        // Get the Temperature Data
        temperatureData = getTemperatureReading();

        updateFanSpeed();
        if (mainLoopTimer.elapsed_time().count() >= PID_UPDATE_PERIOD) {
            mainLoopTimer.stop();
            // Measure current speed
            //currentFanSpeed = readFanSpeed();

            // Set the PWM Change Value
            float pwm = computeTempPID(setpoint, temperatureData, integral, previousError, dt);

            fanSpeedPWM += pwm; // Speed up the fan when the temperature is high

            // Clamp the PWM Value
            if (fanSpeedPWM >= 1) fanSpeedPWM = 1;
            if (fanSpeedPWM <= 0.05) fanSpeedPWM = 0.05;

            // Apply new PWM value
            fanPWM.write(fanSpeedPWM);
            //setFanSpeed(pwm);
            //printf("Fan PWM: %d\n", (int)(fanSpeedPWM*100));

            sprintf(targetTempChar, "Target Temp:%d", setpoint);

            // Display the Temp
            LCDScreen.clear();
            LCDScreen.writeLine(targetTempChar, 0);

            sprintf(rpmChar, "RPM:%d", currentFanSpeed);
            LCDScreen.writeLine(rpmChar, 1);

            // Check the fan stability
            checkFanStability(setpoint);

            // Reset the Timers
            mainLoopTimer.reset();
            mainLoopTimer.start();
        }

        sevenSegPrint(temperatureData);
    }
    tacoReading = 0;
}

// Closed Loop Control for Fan Speed Setpoint changes from min and max every 45 seconds
void closedLoopFanDemo()
{
    // Desired speed (setpoint in RPM)
    int setpoint[4] = {MAX_FAN_SPEED, MID_FAN_SPEED, MIN_FAN_SPEED, MID_FAN_SPEED};

    // PID state variables
    float integral = 0.0f;
    float previousError = 0.0f;
    float dt = PID_UPDATE_PERIOD;
    
    // For Display
    char rpmChar[16];
    char targetRpmChar[16];
    
    // Timer
    Timer switchTimer;
    char index = 0;

    tacoReading = 1;
    switchTimer.start();

    // CONTROL LOOP START
    while (boardMode == CLOSED_LOOP_FAN_DEMO) {
        updateTacoAllowance();
        checkZeroFanSpeed();
        // Compute new PWM value using PID. Update Every 5 Seconds

        updateFanSpeed();
        if (mainLoopTimer.elapsed_time().count() >= PID_UPDATE_PERIOD) {
            mainLoopTimer.stop();
            // Measure current speed
            //currentFanSpeed = readFanSpeed();

            // Set the PWM Change Value
            float pwm = computeFanPID(setpoint[index], currentFanSpeed, integral, previousError, dt);

            fanSpeedPWM += pwm;

            // Clamp the PWM Value
            if (fanSpeedPWM >= 1) fanSpeedPWM = 1;
            if (fanSpeedPWM <= 0.05) fanSpeedPWM = 0.05;

            // Apply new PWM value
            fanPWM.write(fanSpeedPWM);
            //setFanSpeed(pwm);
            //printf("Fan PWM: %d\n", (int)(fanSpeedPWM*100));

            sprintf(targetRpmChar, "Target RPM:%d", setpoint[index]);

            // Display the RPM
            LCDScreen.clear();
            LCDScreen.writeLine(targetRpmChar, 0);

            sprintf(rpmChar, "RPM:%d", currentFanSpeed);
            LCDScreen.writeLine(rpmChar, 1);

            // Check the fan stability
            checkFanStability(setpoint[index]);

            // Reset the Timers
            mainLoopTimer.reset();
            mainLoopTimer.start();
        }

        sevenSegPrint(switchTimer.elapsed_time().count() / SECOND);
        
        if (switchTimer.elapsed_time().count() >= SECOND*15) {
            switchTimer.stop();
            index++;
            if (index > 3) index = 0;
            switchTimer.reset();
            switchTimer.start();
        }
        
    }
    tacoReading = 0;
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

    //fanTACO.mode(PullUp);
    fanTACO.rise(&incrementTACO);
    fanTACO.fall(&checkTACOPulseWidth);
    NVIC_SetPriority(EXTI0_1_IRQn, 1);

    fanPWM.period_us(FAN_PWM_PERIOD);
    fanPWM.write(fanSpeedPWM);

    // Start the timer and the main loop
    LCDScreen.clear();
    mainLoopTimer.start();

    // MAIN LOOP
    while(1) {

        boardLeds = 0b01;
        openLoopControl();
        LCDScreen.clear();

        boardLeds = 0b10;
        closedLoopControlFan();
        LCDScreen.clear();

        boardLeds = 0b11;
        closedLoopControlTemp();
        LCDScreen.clear();

        boardLeds = 0b00;
        closedLoopFanDemo();
        LCDScreen.clear();
    }
}