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
### 2.1Hardware and Architecture
  At the hardware level, the motor system consists of two 12V DC motors controlled via an L298N motor driver. GPIOs 1, 2, 3 are used for the left motor (PWM, DIR1, DIR2), and GPIOs 4, 5, 6 for the right motor. The robot navigates autonomously using a Raspberry Pi Camera Module v2, which captures real-time road data. The VisionTracker module leverages OpenCV to perform grayscale conversion, Gaussian blurring, adaptive thresholding, and Hough transform to extract line position and orientation. These parameters are fed into a PID controller to generate differential motor commands for smooth line-following.
  For safety and obstacle avoidance, three infrared sensors are mounted at the front (GPIOs 9, 10, 11) for short-range detection, while a HC-SR04 ultrasonic sensor (TRIG on GPIO 7, ECHO on GPIO 8) provides forward distance measurement. These inputs are handled by the SafetyController module, which performs safety checks every 100 ms. It supports emergency stop actions and dynamically adjusts motion commands to prevent collisions.
  The temperature regulation system is specially designed to protect sensitive medical cargo. It includes a DS18B20 digital temperature sensor (connected to GPIO 12 with power on GPIO 15), a PTC heating element with a driver board, and a XD-7082 semiconductor cooling module with heatsink and fan. Heating is controlled via PWM on GPIO 13, while cooling uses GPIO 14. A PID-based TemperatureController dynamically adjusts the output power of these actuators to maintain the cargo bay temperature at 25 ± 0.5°C. The system also features over-temperature shutdown and anti-windup protection for reliable control.




<div align="center">
  <img src="Image/Raspberry Pi 4B.png" width="200" alt="Raspberry Pi 4B">
</div>

<p align="center">
Figure 3: Raspberry Pi 4B
</p>


<div align="center">
  <img src="Image/MOSFET driver module&PTC heater.jpg" width="200" alt="MOSFET Driver Module and PTC Heater">
</div>

<p align="center">
Figure 4: MOSFET driver module & PTC heater
</p>

<div align="center">
  <img src="Image/DS18B20 temperature sensor.jpg" width="200" alt="DS18B20 Temperature Sensor">
</div>

<p align="center">
Figure 5: DS18B20 temperature sensor
</p>

<div align="center">
  <img src="Image/XD-7082 Peltier cooling plate.jpg" width="200" alt="XD-7082 Peltier Cooling Plate">
</div>

<p align="center">
Figure 6: XD-7082 Peltier cooling plate
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
### 3.1 Software Operation Process
Upon power-up, the smart medical transport car begins by initializing all critical modules—including motor drivers, vision tracker, obstacle sensors, temperature controller, and safety systems—while performing a self-diagnostic to confirm hardware readiness. Once verified, the system launches two concurrent threads: the main control loop and the safety monitoring process.
In the main control loop, the Raspberry Pi Camera captures continuous video frames, which are processed in real time using OpenCV. The algorithm performs grayscale conversion, Gaussian blur, thresholding, and Hough line detection to extract the line’s position and orientation relative to the robot. These parameters are then passed into a PID control algorithm that dynamically adjusts the speed and steering of the left and right motors through PWM output. This enables smooth, responsive line-following behavior, allowing the vehicle to stay centered on its route.
Simultaneously, the safety thread polls infrared and ultrasonic sensors every 100 milliseconds. Infrared sensors mounted at the front, left, and right detect nearby obstacles, while the ultrasonic module measures forward distance. If an obstacle enters the unsafe range, the SafetyController instantly overrides the motor commands, initiating a full stop or evasive turn. The system’s emergency response latency is less than 1 millisecond, ensuring safety in dynamic hospital environments.
Temperature control runs in parallel. The DS18B20 sensor continuously monitors the cargo bay temperature and reports readings to the TemperatureController, which employs a PID algorithm to drive the PTC heating module and XD-7082 cooling unit. If the temperature drifts outside the ±0.5°C target range around 25°C, the controller adjusts heating or cooling output accordingly. If unsafe thermal deviations persist, the safety module triggers a thermal emergency stop.
All recurring tasks—such as sensor polling, control loop execution, and system logging—are scheduled using the TimerManager with sub-millisecond accuracy, ensuring synchronized real-time performance. The system continues operating autonomously until it receives a shutdown signal (e.g., Ctrl+C or button press), at which point it gracefully terminates all threads, shuts down modules in reverse order, and secures system integrity.This tightly integrated workflow ensures that the vehicle navigates accurately, avoids hazards, regulates cargo temperature, and maintains safety—all without human intervention—making it highly suitable for autonomous medical logistics in hospitals.

## License
This project is licensed under the MIT License. See LICENSE for details.


## Follow us
More info is available in Ins: @ha.nn6012 

If you are interested in our design, feel free to contact us!
