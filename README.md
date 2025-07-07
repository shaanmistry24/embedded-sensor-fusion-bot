# Sensor Fusion Line-Following Robot

This project implements a real-time autonomous line-following robot using a TI-RSLK (Texas Instruments Robotics System Learning Kit). The robot uses infrared sensor fusion and PID control to navigate a complex track with turns, splits, and arches.

*Completed for EC ENGR 3 — UCLA Electrical Engineering Course, Spring 2025 <br/>

## Features

- 8 infrared bottom-facing sensors for real-time line detection
- Weighted sensor fusion to calculate path error
- Tuned PID controller for smooth and accurate steering
- Handles sharp turns, 180° reversals, and path splits
- Adaptive behavior using phantom streak detection
- Embedded C++ code for MSP432 microcontroller

## Hardware

- **Platform**: TI-RSLK
- **Sensors**: 8 IR sensors
- **Motors**: Dual DC motors with PWM and DIR pin control
- **Power**: Battery-powered mobile platform

## Algorithm Overview

1. **Sensor Fusion**:
   - Normalize 8 IR sensor values using `sensorMin[]` and `sensorMax[]`.
   - Compute a weighted sum to calculate the lateral position error.

2. **PID Control**:
   - Apply proportional and derivative control to calculate steering correction:
     \[
     \text{steering\_corr} = K_p \cdot \text{error} + K_d \cdot \Delta \text{error}
     \]

3. **Motor Logic**:
   - Adjust left/right motor PWM based on `steering_corr`
   - Reverse direction if needed for tight turns
   - Use special logic for 180° turn and track completion detection

## Tuning Results

- Final Kp: `0.0355`, Kd: `0.071`
- Base motor speed: `25`
- 97% track completion with tuned gains
- Performance graphs included in final report

## How to Run

1. Upload `main.cpp` to the MSP432 via Energia.
2. Power on the TI-RSLK robot and place it on the starting line.
3. The robot will begin line-following with built-in logic for 180° turnaround and stopping.
