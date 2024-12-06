# Fan Controller Board

This project features a fan controller board designed using the NUCLEO-F070RB development board. The board offers three modes of operation, allowing users to control and monitor the fan's behavior through various user interfaces.

---

## Table of Contents

- [Introduction](#introduction)  
- [Features](#features)  
- [Modes of Operation](#modes-of-operation)  
- [Interfaces](#interfaces)  
- [Display Functionality](#display-functionality)  
- [Setup](#setup)  
- [Usage](#usage)  
- [License](#license)  

---

## Introduction  

The fan controller board provides robust control over fan operation, supporting open loop and closed loop control schemes. It integrates intuitive interfaces for input and output, making it a versatile tool for a range of applications.  

---

## Features  

- **Modes of Operation**:  
  - Open Loop Control  
  - Closed Loop Control with User-Variable Fan Speed  
  - Closed Loop Control with Temperature Measurements  

- **User Interfaces**:  
  - **Inputs**: Rotary encoder (extension board), button  
  - **Visual Outputs**:  
    - 2-digit 7-segment display (HDSP-521E)  
    - LCD screen (ST7066U)  
    - LEDs (board and extension board)  
    - Temperature sensor (TC74)  

- **Real-Time Feedback** for fan control and monitoring.  

---

## Modes of Operation  

1. **Open Loop Control**:  
   - The fan runs at a user-set duty cycle.  
   - Duty cycle is adjustable using the rotary encoder.  

2. **Closed Loop Control with Variable Fan Speed**:  
   - The fan speed is controlled to match a user-defined target RPM.  
   - Rotary encoder is used to set the target RPM.  

3. **Closed Loop Control with Temperature Measurements**:  
   - The fan speed adjusts automatically based on the measured temperature.  
   - No rotary encoder input is needed in this mode.  

---

## Interfaces  

### User Inputs  
- **Rotary Encoder**:  
  - Adjusts duty cycle in Open Loop Mode.  
  - Sets target RPM in Closed Loop Fan Speed Mode.  
  - Unused in Closed Loop Temperature Mode.  

- **Button**:  
  - Toggles between the three modes.  

### Visual Outputs  
- **2-Digit 7-Segment Display**:  
  - Displays:  
    - Current duty cycle (%) in Open Loop Mode.  
    - Percentage error between target RPM and current RPM in Closed Loop Fan Speed Mode.  
    - Current temperature from the TC74 sensor in Closed Loop Temperature Mode.  

- **LCD Screen**:  
  - Displays:  
    - **Second Line**: Current fan speed in all modes.  
    - **First Line**:  
      - Time delta for speed readings in Open Loop Mode.  
      - Target RPM in Closed Loop Fan Speed Mode.  
      - Target temperature in Closed Loop Temperature Mode.  

- **LEDs**: Indicate various states and operational statuses.  

---

## Display Functionality  

| **Mode**                     | **7-Segment Display**                    | **LCD Screen (Line 1)**       | **LCD Screen (Line 2)**     |  
|------------------------------|------------------------------------------|--------------------------------|-----------------------------|  
| Open Loop Control            | Duty cycle (%)                          | Time delta                    | Current fan speed           |  
| Closed Loop Fan Speed Control| Percentage error (target vs. current)   | Target RPM                    | Current fan speed           |  
| Closed Loop Temperature Control | Temperature (°C)                      | Target temperature            | Current fan speed           |  

---

## Setup  

**Fill in this section with setup instructions, including hardware connections, software installation, and required libraries.**  

---

## Usage  

1. **Power the board** and ensure all components are connected.  
2. Use the **button** to switch between modes.  
3. Adjust fan parameters as described:  
   - Use the rotary encoder for duty cycle or target RPM adjustments (depending on the mode).  
4. Monitor the **7-segment display** and **LCD screen** for real-time feedback.  

---

## License  

This project is licensed under the [MIT License](LICENSE).  

--- 
