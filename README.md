# Autonomous Hospital Vehicle for Line-Following and Medical Cargo transport
## 1. Introduction
With the rise of smart healthcare and intelligent hospitals, Automated Guided Vehicles (AGVs) have become a key part of modern medical logistics. Traditional manual transport is inefficient and error-prone, while AGVs enable the automatic and precise delivery of items such as medications, samples, and meals, easing staff workload and improving efficiency. To operate effectively in hospital environments, AGVs must ensure stable navigation, real-time obstacle avoidance, and temperature control. They often rely on black line tracking for navigation, ultrasonic sensors for safety, and onboard temperature regulation to protect sensitive medical items like vaccines or specimens.

<div align="center">
  <img src="Image/Line tracking.gif" width="400" alt="Line Tracking">
</div>

<p align="center">
Figure 1: Line tracking
</p>

<div align="center">
  <img src="Image/Obstacle avoidance.gif" width="200" alt="Obstacle Avoidance">
</div>

<p align="center">
Figure 2: Obstacle avoidance
</p>


## 2. Hardware and Component Selection
### 2.1 Hardware and Architecture
  At the hardware level, the motor system consists of two 12V DC motors controlled via an L298N motor driver. GPIOs 1, 2, 3 are used for the left motor (PWM, DIR1, DIR2), and GPIOs 4, 5, 6 for the right motor. The robot navigates autonomously using a Raspberry Pi Camera Module v2, which captures real-time road data. The VisionTracker module leverages OpenCV to perform grayscale conversion, Gaussian blurring, adaptive thresholding, and Hough transform to extract line position and orientation. These parameters are fed into a PID controller to generate differential motor commands for smooth line-following.
  For safety and obstacle avoidance, three infrared sensors are mounted at the front (GPIOs 9, 10, 11) for short-range detection, while a HC-SR04 ultrasonic sensor (TRIG on GPIO 7, ECHO on GPIO 8) provides forward distance measurement. These inputs are handled by the SafetyController module, which performs safety checks every 100 ms. It supports emergency stop actions and dynamically adjusts motion commands to prevent collisions.
  The temperature regulation system is specially designed to protect sensitive medical cargo. It includes a DS18B20 digital temperature sensor (connected to GPIO 12 with power on GPIO 15), a PTC heating element with a driver board, and a XD-7082 semiconductor cooling module with heatsink and fan. Heating is controlled via PWM on GPIO 13, while cooling uses GPIO 14. A PID-based TemperatureController dynamically adjusts the output power of these actuators to maintain the cargo bay temperature at 25 ± 0.5°C. The system also features over-temperature shutdown and anti-windup protection for reliable control.


### 2.2 Hardware Selection

<div align="center">
  <img src="Image/Raspberry Pi 4B.png" width="400" alt="Raspberry Pi 4B">
</div>

<p align="center">
Figure 3: Raspberry Pi 4B
</p>

<div align="center">
  <img src="Image/Raspberry Pi Camera.jpg" width="200" alt="Raspberry Pi Camera">
</div>

<p align="center">
Figure 4: Raspberry Pi Camera
</p>

<div align="center">
  <img src="Image/Infraed Sensor.jpg" width="200" alt="Infraed Sensor">
</div>

<p align="center">
Figure 5: Infraed Sensor
</p>

<div align="center">
  <img src="Image/Ultra sensor.jpg" width="200" alt="Ultra sensor">
</div>

<p align="center">
Figure 6: Ultra Sensor
</p>

<div align="center">
  <img src="Image/MOSFET driver module&PTC heater.jpg" width="200" alt="MOSFET Driver Module and PTC Heater">
</div>

<p align="center">
Figure 7: MOSFET driver module & PTC heater
</p>

<div align="center">
  <img src="Image/DS18B20 temperature sensor.jpg" width="200" alt="DS18B20 Temperature Sensor">
</div>

<p align="center">
Figure 8: DS18B20 temperature sensor
</p>

<div align="center">
  <img src="Image/XD-7082 Peltier cooling plate.jpg" width="200" alt="XD-7082 Peltier Cooling Plate">
</div>

<p align="center">
Figure 9: XD-7082 Peltier cooling plate
</p>


<div align="center">

| Category     | Component                         | Specification                     |
|--------------|------------------------------------|-----------------------------------|
| Mainboard    | Raspberry Pi                       | 4B (4GB RAM)                      |
| Storage Card | microSDXC UHS-I                    | 64GB                              |
| Card Reader  | MicroSDXC Card Reader (c289)       | 2TB                               |
| Expansion    | Raspberry Pi Expansion Board       | 40-pin Expansion                  |
| Camera       | Raspberry Pi Camera Module         | CSI Interface                     |
| Sensors      | Ultrasonic Sensor                  | HC-SR04                           |
|              | Infrared Sensor                    | Obstacle Avoidance IR Module     |
|              | Temperature Sensor                 | DS18B20                           |
| Actuators    | Motor Driver                       | L298N Motor Driver Board          |
|              | DC Motor                           | 12V Geared DC Motor ×4           |
| Power Supply | Battery                            | 12V Lithium Battery Pack          |
| Software     | Operating System                   | Raspberry Pi OS                   |
|              | Compiler                           | GCC/G++ 9.4+                      |
| Controller   | PTC Heating Kit with MOS Module    | [A]+12V/100°C Heating Plate       |
|              | High Power MOS Driver Module       | 10A                               |

</div>

## 3. System overall structure and operation logic

Upon powering up the embedded system, the software begins execution from the main.cpp file. The program first initializes core components, including the motor controller, vision tracker, ultrasonic and infrared sensors, temperature controller, safety controller, and timer manager. Each module is instantiated using smart pointers, ensuring automatic memory management and clear ownership semantics.

After initialization, the system registers callback functions for key sensors. For example, the infrared and ultrasonic modules register event-driven callbacks to handle boundary detection and obstacle avoidance. These callbacks define safety behaviors, such as stopping, reversing, and turning, which are conditionally triggered based on sensor data. The temperature controller also begins periodic temperature monitoring, executing PID regulation routines and checking against safe thresholds.

The timer manager coordinates all asynchronous actions through scheduled tasks. It supports one-time delays and chained actions, such as recovery movements after obstacle detection. Motion commands issued by different modules are forwarded to the motor controller, which handles transitions between motion states like FORWARD, BACKWARD, STOP, TURN_LEFT, and TURN_RIGHT.

The safety controller continuously monitors several conditions, including emergency button activation (via hardware interrupt), sensor health updates, and runtime duration. In the event of an emergency, it triggers global stop behavior and optionally invokes user-defined callbacks for custom recovery handling. Real-time log outputs are printed to assist with debugging and status tracking.

Throughout the operation, sensor modules run independently and report data via their callback interfaces. All modules operate asynchronously, using the timer manager as the central scheduler. The integration ensures that the vehicle reacts in real time to environmental inputs while maintaining modular isolation and code clarity.

System shutdown occurs when the emergency stop is triggered or the main loop exits. Cleanup routines ensure all resources are released gracefully.


## License
This project is licensed under the MIT License. See LICENSE for details.


## Follow us
More info is available in Ins: @ha.nn6012 

If you are interested in our design, feel free to contact us!
