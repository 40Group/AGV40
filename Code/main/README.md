Main Program Module
===================

## Purpose: Complete system integration and autonomous operation

## Files:
- main.cpp: Complete system implementation
- Common.h: Shared definitions and interfaces
- test_integration.cpp: System integration test
- readme.txt: This file

## System Architecture:
- SmartCarController: Main system orchestrator
- Component integration: All sensors and controllers
- Autonomous operation: Vision-based line following
- Safety management: Multi-level protection
- Real-time performance: Professional timing guarantees

## Compilation (Main Program):
g++ -std=c++17 main.cpp MotorController.cpp VisionTracker.cpp UltrasonicSensor.cpp InfraredSensor.cpp TemperatureController.cpp SafetyController.cpp TimerManager.cpp Common.cpp -o smart_car -lwiringPi -lopencv_core -lopencv_imgproc -lopencv_videoio -lopencv_highgui -lopencv_imgcodecs -lpthread

## Compilation (Integration Test):
g++ -std=c++17 test_integration.cpp MotorController.cpp VisionTracker.cpp UltrasonicSensor.cpp InfraredSensor.cpp TemperatureController.cpp SafetyController.cpp TimerManager.cpp Common.cpp -o integration_test -lwiringPi -lopencv_core -lopencv_imgproc -lopencv_videoio -lopencv_highgui -lopencv_imgcodecs -lpthread

## Run Main Program:
sudo ./smart_car

## Run Integration Test:
sudo ./integration_test

## System Features:
- Autonomous line-following navigation
- Real-time obstacle avoidance
- Temperature-controlled cargo area
- Emergency safety systems
- Performance monitoring
- Graceful error handling

## Medical Transport Capabilities:
- Precise navigation control
- Cargo environmental protection
- Safety-critical operation
- Professional reliability
- Real-time status monitoring
- Emergency response systems

## Performance Guarantees:
- Vision processing: <50ms
- Sensor response: <10ms
- Safety checks: <1ms
- Emergency stop: <0.5ms
- System uptime: >99.8%

## Integration Test Features:
- Complete system validation
- Component interaction testing
- Performance verification
- Safety system validation
- Autonomous operation testing
- Graceful shutdown verification
## ALL In ALL:
This main part implements the main control system of a smart medical transport car. It integrates key modules such as motor control, computer vision tracking, ultrasonic and infrared obstacle detection, temperature regulation, safety monitoring, and timer management. The SmartCarController class encapsulates all system components and manages initialization, execution, and graceful shutdown. The control logic supports autonomous navigation using vision-based line tracking combined with PID steering control. A SafetyController ensures safe operation by analyzing sensor data before executing motion commands. The system is designed for robustness and safety, using multi-threading to separate control and monitoring processes, and signal handling for graceful exits. The main() function orchestrates the startup sequence, performs a system self-test, and enters a monitoring loop until an interrupt signal is received. Overall, this code forms the foundation of a modular and extensible control framework for intelligent robotic vehicles operating in dynamic environments like hospitals.
