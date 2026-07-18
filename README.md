# ESP32 Bluetooth Mouse

DIY Bluetooth Mouse using ESP32 and MX8650 optical sensor and switches form salvaged mouse.

## Features
- BLE HID
- Left / Right click
- Middle Click
- Thumb Forward / Back Click
- Scroll Wheel
- Horizontal Scroll
- DPI Mode 800, 1000, 1200 , 1600
- Battery Service
- Sensor Sleep
- Power Management

## Hardware
- ESP32 DevKit V1
- MX8650 optical sensor 
- Salvaged mouse swithes
- Logitech donor pcb (AC Pan support)

## Schematic
![Schematic](images/schematic.png)

# Prototype

![Prototype](images/1.jpg)

## Prototype Assembly
Current prototype is built using salvaged components from multiple mice. front button assembly and horizontal scroll mechanism are reused from a Logictech old mouse, and the optical sensor assembly from a generic old Bluetooth mouse.

## Current Status
Working properly.

Implemented: 
- NimBLE HID
- Cursor movement
- Left / Right click
- Middle click
- Side buttons
- Vertical scroll
- Horizontal scroll
- Adjustable DPI with state machine
- Adaptive polling

## Tested On

- ESP-IDF v5.5.x
- ESP32-WROOM-32 DevKit V1
- Arch Linux (BlueZ)
- Android 16
