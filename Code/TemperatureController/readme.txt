Temperature Controller Module
=============================

Purpose: Event-driven PID temperature control for medical cargo protection

Files:
- TemperatureController.h: Class declaration
- TemperatureController.cpp: Implementation
- test_temperature.cpp: Unit test program
- readme.txt: This file

Hardware Requirements:
- DS18B20 digital temperature sensor
- Heating element (PWM controlled)
- Cooling fan/Peltier (PWM controlled)
- GPIO connections:
  * Temperature sensor: GPIO 12 (1-Wire)
  * Heater PWM: GPIO 13
  * Cooler PWM: GPIO 14
  * Sensor power: GPIO 15

Compilation:
g++ -std=c++17 test_temperature.cpp TemperatureController.cpp -o temp_test -lwiringPi -lpthread

Run Test:
sudo ./temp_test

Features Tested:
- DS18B20 sensor initialization and reading
- Event-driven temperature monitoring (epoll)
- PID control algorithm implementation
- Manual heating/cooling control
- Temperature target changes
- Safety range monitoring
- Temperature stability detection
- Performance statistics

Event-Driven Architecture:
- epoll-based non-blocking sensor reading
- File descriptor monitoring for temperature updates
- Automatic PID control loop
- Real-time temperature response

PID Control Features:
- Proportional-Integral-Derivative control
- Anti-windup protection
- Configurable PID parameters
- Smooth temperature transitions

Safety Features:
- Temperature range monitoring (5°C - 50°C)
- Emergency shutdown on overheating
- Gradual power control (0-80% max)
- Sensor validation and error handling

Medical Application:
- Precise temperature control (±0.5°C)
- Cargo protection during transport
- Real-time monitoring and alerts
- Professional medical device standards

All in all：
- The module is based on Raspberry Pi and WiringPi's **TemperatureController** implementation, which uses an event-driven mechanism to read the DS18B20 sensor temperature in real time and intelligently adjusts the PWM outputs of the heater and cooler based on the PID control algorithm to maintain the target temperature. The module supports GPIO initialization, custom PID parameters, safe temperature limit, breakpoint recovery, self-test function and sensor file system monitoring, which is suitable for the project that has the demand for precise temperature regulation.
