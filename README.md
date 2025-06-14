# STM32 Battery Monitor & Protection System

This project implements a battery monitoring and protection system using an STM32 microcontroller and three INA219 current/voltage/power sensors over I2C. It is designed to measure lithium-ion cell parameters and trigger load disconnection in unsafe conditions.

## Features

- Monitors 3 individual battery cells (via INA219 sensors)
- Measures voltage (V), current (mA) and power (W)
- Tracks min/max values in real time
- Protection mechanisms:
  - Overvoltage (> 4.2V)
  - Undervoltage (< 3.0V)
  - Overcurrent (> 2000mA)
- Automatic load disconnection/reconnection
- UART logging via printf (115200 baud)

## Hardware

- STM32F4 microcontroller (e.g., Nucleo-F401RE)
- 3x INA219 sensors via I2C
- Load control via MOSFET or relay (GPIO-controlled)
- UART via USB for data output

## Getting Started

1. Clone the repo:

  git clone https://github.com/saturnesu/STM32_BMS_Simulation

2. Open in STM32CubeIDE.
3. Connect the hardware and flash the firmware.
4. Open a serial monitor at 115200 baud to view output.

## Serial Output Example

Cell 1: V=3.985 V (Min: 3.913, Max: 3.991) | I=317.516 mA (Min: 299.993, Max: 327.168) | P=1.269 W
Cell 2: V=4.211 V (Min: 4.193, Max: 4.218) | I=315.798 mA (Min: 298.738, Max: 325.440) | P=1.329 W
→ Cell 2: Overvoltage!
Cell 3: INA219 read failed.

## Why I Built This

This project demonstrates embedded system design skills, including I2C communication, real-time monitoring and hardware safety logic. It’s intended as a portfolio piece to present in job interviews.

**Author**: Vaggelis Koutouloulis
**Date**: June 2025
