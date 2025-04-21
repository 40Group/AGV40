# Intelligent Tracking and Obstacle Avoidance Vehicle with Temperature Control

A Raspberry Pi-based smart vehicle system designed for automated delivery in medical environments. It features visual line tracking, ultrasonic obstacle detection, temperature regulation, and real-time web-based control.

---

## 📸 Appearance

![Custom-designed 3D printed shell](./images/shell.png)  
*Figure 1: Custom-designed 3D printed shell*

---

## 🔧 Features

- **Real-time Video Transmission**  
  View the USB camera feed in real time via browser. Equipped with a high-resolution lens and stable MJPEG streaming.

- **Webpage Remote Control**  
  Control the vehicle via webpage (start/stop/steering).

- **Personnel Detection Support**  
  I2C-based deep-learning-ready external module for personnel detection (optional).

---

## 🧱 Project Structure

The system is composed of multiple concurrent modules using C++ multithreading and Boost.Asio asynchronous timers.

### 1. Hardware Control Module (STM32)

This serves as the lower control layer of the vehicle, handling posture sensing, OLED display, self-balancing, button detection, etc.

![STM32 Board](./images/stm32.png)

---

### 2. Overall Architecture

- **capture_frames()**  
  Vision + tracking + state machine logic (30ms interval)

- **get_distance()**  
  Ultrasonic ranging, updates global `distance` variable (100ms interval)

- **temp_control()**  
  Temperature PID control and Peltier management (200ms interval)

- **HTTP Server**  
  Based on `cpp-httplib` for video streaming + JSON API control

```mermaid
flowchart TD
    A[Start Initialization] --> B[Start 3 Threads]
    B --> C[Capture Frames]
    B --> D[Distance Measure]
    B --> E[Temperature PID Control]
    C --> F[State Machine: Tracking or Avoidance]
    F -->|Obstacle| G[Right Turn]
    G --> H[Left Turn]
    F -->|Line Detected| I[Calculate Deviation → PWM Output]
    E --> J[Read Temp via I2C → PID → Peltier/Fan]

