
# MAX30102 Pulse Oximeter – STM32F405 (FreeRTOS)

## Overview

This project demonstrates interfacing the MAX30102 pulse oximeter sensor with an STM32F405 microcontroller using I2C2. It measures heart rate (HR) and oxygen saturation (SpO₂) in real time, displays readings on an LCD, and sends them via UART. The system runs under FreeRTOS for efficient task scheduling.

## Features

* **Sensor Reading**: Uses I2C2 (PB10 = SCL, PB11 = SDA) to read red/IR data from MAX30102.
* **Data Processing**:

  * Mean and RMS calculation for AC/DC components.
  * SpO₂ calculation using the ratio of red/IR signals.
  * Heart rate detection via peak counting.
  * Finger presence detection.
  * Normalization and clamping of HR/SpO₂ values.
* **Display**:

  * LCD shows live HR and SpO₂ values.
  * "Place Finger" prompt when no valid signal detected.
* **Communication**:

  * Sends HR and SpO₂ values to PC over UART3.
* **Real-Time Operation**:

  * Implemented as a FreeRTOS task (`PulseOximeterTask`).
  * 50 Hz sampling rate.

## Hardware Requirements

* STM32F405 board
* MAX30102 pulse oximeter sensor
* LCD display (character-based)
* UART connection to PC (for debugging/logging)
* Pull-up resistors on SDA/SCL lines

## Pin Configuration

| Peripheral | Pin  | Function   |
| ---------- | ---- | ---------- |
| I2C2 SCL   | PB10 | Clock      |
| I2C2 SDA   | PB11 | Data       |
| UART3 TX   | PD8  | TX to PC   |
| UART3 RX   | PD9  | RX from PC |

## Software Requirements

* STM32CubeIDE
* HAL drivers for I2C, UART, GPIO
* FreeRTOS (enabled in CubeMX)

## How It Works

1. **Initialization**:

   * Configure I2C2, UART3, LCD, and GPIO.
   * Initialize MAX30102 sensor.
2. **Measurement**:

   * Collect 100 samples of red & IR readings.
   * Calculate mean & RMS values.
   * Detect peaks for HR.
   * Compute SpO₂ from red/IR ratio.
3. **Output**:

   * Show HR and SpO₂ on LCD.
   * Send values to PC via UART.
4. **Error Handling**:

   * "Read Error" message if I2C fails.
   * Finger absence detection.

## Usage

1. Connect hardware as per pin configuration.
2. Flash the firmware to STM32F405.
3. Open a UART terminal at the configured baud rate to view readings.
4. Place finger on MAX30102 sensor to start measurement.


