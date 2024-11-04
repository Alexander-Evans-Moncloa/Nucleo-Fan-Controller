/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"

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

    // Interrupts
    InterruptIn button(BUTTON1);
#else
    bool led;
    bool extensionBoardLed;
#endif

// This is a test message
short int rotaryEncoderStage = 0;

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

// =============================== MAIN CODE ===============================

int main() 
{
    led = 0;
    extensionBoardLed = 0;

    button.mode(PullUp);
    button.rise(&flip);

    // Temp PWM Stuff
    fanPWM.period(1.0f);
    fanPWM.write(1.0f);

    while (true) {
        // blinky();
        rotaryEncoderDirectionLED();

    }
}