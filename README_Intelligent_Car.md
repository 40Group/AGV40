# Intelligent Tracking Obstacle Avoidance and Temperature Control Trolley

## 1. Introduction

With the rise of smart healthcare and intelligent hospitals, Automated Guided Vehicles (AGVs) have become a key part of modern medical logistics. Traditional manual transport is inefficient and error-prone, while AGVs enable the automatic and precise delivery of items such as medications, samples, and meals, easing staff workload and improving efficiency. To operate effectively in hospital environments, AGVs must ensure stable navigation, real-time obstacle avoidance, and temperature control. They often rely on black line tracking for navigation, ultrasonic sensors for safety, and onboard temperature regulation to protect sensitive medical items like vaccines or specimens.

This project presents the design and implementation of a Raspberry Pi-based intelligent vehicle control system tailored to medical AGV applications. The system is developed in C++ and integrates multiple functionalities including line following, ultrasonic obstacle avoidance, temperature monitoring and control, and lightweight web-based communication. OpenCV is used for real-time image acquisition and processing, while the path center is extracted using image moment calculations. A proportional control algorithm dynamically adjusts the servo angle, enabling the vehicle to follow a black line with high precision. An ultrasonic distance sensor, connected via GPIO, allows the system to detect obstacles in front of the vehicle and initiate appropriate avoidance maneuvers to prevent collisions. For temperature-sensitive transport, the system incorporates an I2C-based digital temperature sensor and uses a PID algorithm to control a cooling or heating module via PWM signals, ensuring stable cargo temperatures. On the communication side, an embedded HTTP server provides a live video stream and RESTful APIs, allowing users to monitor the vehicle’s status, adjust parameters, and control operation remotely through a browser interface.

## 2. Hardware and Component Selection

![Raspberry Pi 4B+ main control board](./images/raspberry_pi.png)

![ESP32 Core Board](./images/esp32_core.png)

## 3. System Overall Structure and Operation Logic

### 3.1 Software Architecture

- **Multithreading + Asynchronous Framework**:
  - `capture_frames()`: Main logic for tracking/steering/obstacle avoidance, triggered every 30ms
  - `get_distance()`: Ultrasonic distance update every 100ms
  - `temp_control()`: PID temperature control every 200ms
- **HTTP Server**: Uses cpp-httplib
  - Serves static HTML/JS/CSS resources

### 3.2 Logical Flow

![System Flowchart](./images/flowchart.png)

## 4. System Functions and Implementation

### 4.1 Vehicle Operation

- **Forward**: Servo #008 → 2000, Servo #009 → 1000
- **Stop**: Both servos → 1500 (neutral)
- **Turn Left**: Servo #010 → 1000
- **Turn Right**: Servo #010 → 2000

These commands are sent via serial to the servo controller.

### 4.2 Image Processing

- Convert frame to grayscale
- Threshold at 50 to detect black lines
- Morphological open (denoise) + close (connect lines)
- Use `findContours()` to locate the largest contour
- If a valid contour is found, calculate centroid, generate control signal via PID, and send command via serial
- Visualize centerline on frame

### 4.3 Line Tracking

This module uses OpenCV to extract the path centroid using image moments. The centroid is used to compute the deviation from the center of the frame, which is fed into a proportional controller (Kp = 1.5625) to determine PWM values. If deviation is within ±5 pixels, a neutral PWM of 1500 is used. A visualization overlay shows the path centerline for debugging.

### 4.4 Ultrasonic Ranging

- **Sensor**: HC-SR04
- **Principle**:
  - Trigger ultrasonic pulse
  - Measure echo time-of-flight
  - Calculate distance using:  
    `Distance (cm) = Time (μs) * 0.0343 / 2`
- **Implementation**:
  - Uses Boost.Asio timers (100ms) for non-blocking background updates

### 4.5 Obstacle Avoidance

- During every timer cycle, `timer_handler` checks if the car is running
- If currently turning (sub-state), uses time elapsed to decide whether to switch from right to left turn, or back to tracking
- If not in avoidance state:
  - Under `STATE_LINE_FOLLOWING`, check distance; if obstacle detected, switch to `STATE_OBSTACLE_AVOIDANCE`
  - If in `STATE_STOPPED`, only update frame

### 4.6 Temperature Control

- `get_temp()` reads temperature via I2C (high + low byte)
- Converts to Celsius (raw / 100)
- PID Control: P = 25, I = D = 0
- Positive output → Heat (GPIO27), Negative output → Cool (GPIO17)
- 200ms asynchronous update via Boost.Asio

### 4.7 Web Frontend Visualization and Control

- In `main()`, initialize hardware and start 3 threads (camera, temp, ultrasonic)
- Create server using cpp-httplib:
  - `/`: serves index.html
  - `/video_feed`: MJPEG video stream
  - `/api/curtainSwitch`: POST interface for car state control
  - `/api/temperature`: GET current temp
  - `/api/targetTemperature`: GET/POST to read/set target temp
- Server listens on `0.0.0.0:5000` for local network access

## 5. Summary

This Raspberry Pi-based intelligent medical vehicle integrates black line tracking, ultrasonic obstacle avoidance, real-time temperature control, and web interface operation. It is suitable for the autonomous transport of medical supplies such as drugs, specimens, and meals within hospitals.

- **Tracking**: Image moment + proportional control for smooth line following
- **Steering**: Mechanical linkage for realistic vehicle turning (vs. differential)
- **Obstacle Avoidance**: Asynchronous thread measures and reacts to obstacles
- **Temperature**: I2C sensor + PID to drive Peltier module
- **Control Interface**: Lightweight HTTP server with RESTful API + video streaming

The project emphasizes functional integrity, multithreaded architecture, and practical hardware design. The asynchronous, concurrent architecture ensures real-time performance and modular robustness, demonstrating a highly integrated approach to embedded smart vehicle design.
