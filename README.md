# ESP32 Bluetooth Mouse

DIY Bluetooth Mouse using ESP32 and MX8650 optical sensor and switches form salvaged mouse.

## Features
- Left / Right click
- Middle click
- Thumb forward / back click
- Scroll wheel
- Horizontal scroll
- Bluetooth HID

## Hardware
- ESP32 DevKit V1
- MX8650 optical sensor 
- Salvaged mouse swithes
- Logitech donor pcb (AC Pan support)

# Prototype

![Prototype](images/1.jpg)

## Prototype Assembly
Current prototype is built using salvaged components from multiple mice. front button assembly and horizontal scroll mechanism are reused from a Logictech old mouse, and the optical sensor assembly from a generic old Bluetooth mouse.

## Current Status
Working prototype.

Implemented: 
- Cursor movement
- Left / Right click
- Middle click
- Side buttons
- Vertical scroll
- Horizontal scroll
- Adjustable DPI

In Progress:
- Sleep & SWKINT wakeup

Finding in this version:
- SWKINT correct
- ESP32 wakes correctly
- HID repports resume after wakeup but BT Classic connection eventually times out

Conclusion:
BT Classic HID is not suitable for maintaining connection across light sleep.

## Future Work
- Battery monitoring
- Deep Sleep
- BLE migration
