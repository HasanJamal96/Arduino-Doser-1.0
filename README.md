# Arduino-Doser-1.0

Project aims to control following and configure then using BLE
1.  4 DC Motors using L293D Driver
2.  7 Stepper Motors using A4998 Driver
3.  I2C 16x2 LCD
4.  RTC
5.  DHT22
6.  LEDs using 74HC595 Shift Register

Stepper and BLE are controlled using ESP32 controller  
Everything else is controlled by Arduino Mega 2560  
BLE is used to configure RTC, Schedules to run Stepper and DC Motors.
