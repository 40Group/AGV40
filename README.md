# 
# Introduction
With the rise of smart healthcare and intelligent hospitals, Automated Guided Vehicles (AGVs) have become a key part of modern medical logistics. Traditional manual transport is inefficient and error-prone, while AGVs enable the automatic and precise delivery of items such as medications, samples, and meals, easing staff workload and improving efficiency. To operate effectively in hospital environments, AGVs must ensure stable navigation, real-time obstacle avoidance, and temperature control. They often rely on black line tracking for navigation, ultrasonic sensors for safety, and onboard temperature regulation to protect sensitive medical items like vaccines or specimens.

This project presents the design and implementation of a Raspberry Pi-based intelligent vehicle control system tailored to medical AGV applications. The system is developed in C++ and integrates multiple functionalities including line following, ultrasonic obstacle avoidance, temperature monitoring and control, and lightweight web-based communication. OpenCV is used for real-time image acquisition and processing, while the path center is extracted using image moment calculations. A proportional control algorithm dynamically adjusts the servo angle, enabling the vehicle to follow a black line with high precision. An ultrasonic distance sensor, connected via GPIO, allows the system to detect obstacles in front of the vehicle and initiate appropriate avoidance maneuvers to prevent collisions. For temperature-sensitive transport, the system incorporates an I2C-based digital temperature sensor and uses a PID algorithm to control a cooling or heating module via PWM signals, ensuring stable cargo temperatures. On the communication side, an embedded HTTP server provides a live video stream and RESTful APIs, allowing users to monitor the vehicle’s status, adjust parameters, and control operation remotely through a browser interface.

# Hardware and Component Selection
![Table 1](Images/Table1.png)

![Table 2](Images/Table%202.png)

![Raspberry Pi 4B](Images/Raspberry%20Pi%204B.png)
Figure1: Raspberry Pi 4B+ main control board
![ESP32](Images/ESP%2032.jpg)
Figure2: ESP core board

#  System overall structure and operation logic


