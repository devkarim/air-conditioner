# Air Conditioner - TM4C123GH6PM

## Overview

An air conditioning system built using the TM4C123GH6PM microcontroller. The project simulates an air conditioner with heating and cooling modes based on the ambient temperature compared to a target temperature.

- University: Benha University
- College: Shoubra Faculty of Engineering
- Department: Computer Engineering
- Instructor: Mahmoud Nawar

You can check the demo video from [here](https://www.youtube.com/watch?v=N25PDt8RJDY).

## Features

1. Temperature Sensor: Reads ambient temperature using the LM35 sensor (analog input).

2. LED Indicators:
   Blue LED: Indicates cooling mode (ambient temperature < target temperature).
   Red LED: Indicates heating mode (ambient temperature > target temperature).

3. Target Temperature Adjustment:
   Two buttons to edit the target temperature.

4. Display Output:
   The Nokia 5110 display shows:
   Current system status: "Heater" or "Cooler"
   Target temperature.

5. UART Communication:
   Ambient temperature is periodically sent to a computer for real-time monitoring.

6. Timer-Based Check:
   If ambient and target temperatures are equal, the system waits for a few seconds (using a timer interrupt) before rechecking.

## Components

### Hardware

- Microcontroller: TM4C123GH6PM.
- Temperature Sensor: LM35 (analog output).
- LEDs: Blue and red for displaying the current mode.
- Buttons: Two push buttons for editing the target temperature.
- Display: Nokia 5110 LCD for showing the current mode and target temperature.
- UART: Interface for transmitting ambient temperature to a computer.

### Software

- IDE: Keil Microvision.
- Programming Language: C.
