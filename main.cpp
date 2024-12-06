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
//BufferedSerial mypc(USBTX, USBRX);

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
const int FAN_SPEED_UPDATE_PERIOD = HALF_SECOND; // Prev: SECOND
const int PID_UPDATE_PERIOD = QUARTER_SECOND;

const int MIN_TACO_PERIOD = MILLISECOND*20; // Calculation based off 3000RPM
const int MIN_TACO_POSITIVE_PULSE_WIDTH = MILLISECOND*10;
const int MIN_TACO_ALLOWANCE_PERIOD = MILLISECOND*5;

const int MAX_FAN_SPEED = 2600;
const int MIN_FAN_SPEED = 60;
const int MID_FAN_SPEED = 1440;

// =============================== PID CONTROL ===============================
const float fanHighSpeedKp     = 0.0001;  // Proportional gain
const float fanHighSpeedKi     = 0.00000000008;  // Integral gain
const float fanHighSpeedKd     = 80; // Derivative gain

const float fanLowSpeedKp      = 0.00006;  // Proportional gain
const float fanLowSpeedKi      = 0.00000000001;  // Integral gain
const float fanLowSpeedKd      = 5; // Derivative gain

const float fanUltraLowSpeedKp = 0.00002;
const float fanUltraLowSpeedKi = 0.0000000000007;
const float fanUltraLowSpeedKd = 7;

const float tempKp = -0.1;  // Proportional gain
const float tempKi = 0.0;  // Integral gain
const float tempKd = 0; // Derivative gain

const int LOW_SPEED_THRESHOLD = 1000;
const int ULTRA_LOW_SPEED_THRESHOLD = 600;

// =============================== OTHER ===============================
// RPM = Revs/Min
// RPM = (tacoCount/2)/time_min
// RPM = (tacoCount/2)/(time_us/(1,000,000*60))
// RPM = (tacoCount/2)*(60,000,000/time_us)
// RPM = (tacoCount*30,000,000)/time_us
// Vary this value
//const char REVOLUTIONS_TO_DO = 2;
//const int RPM_CALCULATION_PROPORTION = (REVOLUTIONS_TO_DO)*MINUTE;

const char POS_EDGE_ARRAY_LENGTH = 15; // 7: Not Good for >50% but good for <30%

// ===========================================================================
// =============================== GLOBAL VARS ===============================
// ===========================================================================

short int rotaryEncoderStage = 0;

int fanPulseDelta = 0;
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
    CLOSED_LOOP_TEMP
};

enum Mode boardMode = CLOSED_LOOP_FAN;

// Override the Post Increment Value so that it loops across the enumerator
Mode operator++(Mode& mode, int) {
    if (mode == Mode::CLOSED_LOOP_TEMP) {
        mode = Mode::OPEN_LOOP; // Wrap around to the beginning if needed
    } else {
        mode = static_cast<Mode>(static_cast<int>(mode) + 1);
    }
    return mode;
}

// Tachometer Reading
Timer mainLoopTimer;    // Used for global timings
Timer posPulseTimer;    // Used for the filtering
Timer pulseTimer;       // Used to calculate the RPM

// Set up the Serial USB Ports
//FILE* mypcFile1 = fdopen(&mypc, "r+"); 

// =============================== MISC FUNCTIONS ===============================
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

    // Speeds lower than 30% work good with 7 Length
    for (int i = 0; i < POS_EDGE_ARRAY_LENGTH; i++) sum += fanPulseDeltas[i];
    return sum/POS_EDGE_ARRAY_LENGTH;
}

void fillPosTimes()
{
    for (int i = 0; i < POS_EDGE_ARRAY_LENGTH; i++) {
        fanPulseDeltas[i] = 90000000;
    }
}

void printFanDetails() 
{
    if (boardMode == OPEN_LOOP) printf("\n");
    printf("Fan PWM: %d\tCur Fan RPM: %d\tTime Delta: %d\t", 
        (int)(fanSpeedPWM*100),currentFanSpeed, fanPulseDelta);
}

void checkZeroFanSpeed()
{
    if ((pulseTimer.elapsed_time().count() >= HALF_SECOND+TENTH_SECOND)) {
        pulseTimer.stop();

        fanPulseDelta = pulseTimer.elapsed_time().count();
        fillPosTimes();

        //currentFanSpeed = 0;
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

volatile bool isChangingMode = false;
void enterModeMessage()
{
    isChangingMode = true;
    LCDScreen.clear();

    switch (boardMode) {
        case OPEN_LOOP:
            LCDScreen.write("OPEN LOOP MODE");
            break;
        case CLOSED_LOOP_FAN:
            LCDScreen.write("CLOSED FAN MODE");
            break;
        case CLOSED_LOOP_TEMP:
            LCDScreen.write("CLOSED TEMP MODE");
            break;
    }

    wait_us(SECOND);
    isChangingMode = false;
}

// ==========================================================================
// =============================== INTERRUPTS ===============================
// ==========================================================================

void changeMode() 
{ // Changes the mode of the board and reset timer
    if (isChangingMode) return;
    mainLoopTimer.stop();
    boardMode++;

    mainLoopTimer.reset();
    mainLoopTimer.start();
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
// =============================== MAIN FUNCTIONS ================================
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

volatile bool isUltraLowSpeed = false;
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
    if (setpoint <= ULTRA_LOW_SPEED_THRESHOLD && currentSpeed <= ULTRA_LOW_SPEED_THRESHOLD) {
        if (!isUltraLowSpeed) {isUltraLowSpeed = true; integral = 0;}
        P = fanUltraLowSpeedKp * error;
        I = fanUltraLowSpeedKi * integral;
        D = fanUltraLowSpeedKd * derivative;

        if (I >= 0.001) integral = 0.001/fanUltraLowSpeedKi;
        if (I <= -0.001) integral = -(0.001/fanUltraLowSpeedKi);
        I = fanUltraLowSpeedKi * integral;
    }
    else if (setpoint <= LOW_SPEED_THRESHOLD && currentSpeed <= LOW_SPEED_THRESHOLD) {
        isUltraLowSpeed = false;
        P = fanLowSpeedKp * error;
        I = fanLowSpeedKi * integral;
        D = fanLowSpeedKd * derivative;

        if (I >= 0.02) integral = 0.02/fanLowSpeedKi;
        if (I <= -0.02) integral = -(0.02/fanLowSpeedKi);
        I = fanLowSpeedKi * integral;
    }
    else {
        // Clamp the integral so that it doesn't get saturated
        isUltraLowSpeed = false;
        if (I >= 0.02) integral = 0.02/fanHighSpeedKi;
        if (I <= -0.02) integral = -(0.02/fanHighSpeedKi);
        I = fanHighSpeedKi * integral;
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
    fanSpeedPWM = 1.0;

    tacoReading = 1;

    // CONTROL LOOP START
    while (boardMode == OPEN_LOOP) {
        updateTacoAllowance();

        calculatedSpeed = 2493.5*pow(fanSpeedPWM,3)-6304.2*pow(fanSpeedPWM,2) + 6610.3*fanSpeedPWM - 175.11;
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
            sprintf(currentRpmChar, "RPM: %d", currentFanSpeed);

            char maxRpmChar[16];
            sprintf(maxRpmChar, "Delta: %d", calculateAveragePosTime());

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
    float speedDelta = 0;

    // PID state variables
    float integral = 0.0f;
    float previousError = 0.0f;
    float dt = PID_UPDATE_PERIOD;
    //float pwmFeedForward = 0.04;
    
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

        //pwmFeedForward = 0.0000001*(setpoint*setpoint) + 0.00002*(setpoint) + 0.049;

        // Compute new PWM value using PID. Update Every 5 Seconds
        updateFanSpeed();
        if (mainLoopTimer.elapsed_time().count() >= PID_UPDATE_PERIOD) {
            mainLoopTimer.stop();
            // Measure current speed
            //currentFanSpeed = readFanSpeed();
            printFanDetails();

            // Set the PWM Change Value
            float pwm = computeFanPID(setpoint, currentFanSpeed, integral, previousError, dt);

            fanSpeedPWM += pwm;

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

            speedDelta = abs((float)(setpoint - currentFanSpeed)/setpoint);

            // Reset the Timers
            mainLoopTimer.reset();
            mainLoopTimer.start();
        }
        
        sevenSegPrint((int)(speedDelta*100));
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

            fanSpeedPWM = pwm; // Speed up the fan when the temperature is high

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

    fillPosTimes();

    // MAIN LOOP
    while(1) {

        boardLeds = 0b01;
        enterModeMessage();
        openLoopControl();
        LCDScreen.clear();

        boardLeds = 0b10;
        enterModeMessage();
        closedLoopControlFan();
        LCDScreen.clear();

        boardLeds = 0b11;
        enterModeMessage();
        closedLoopControlTemp();
        LCDScreen.clear();
    }
}