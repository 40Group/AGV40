# AGV40

Key Pins and Variables
const int TRIG_PIN = 19;
const int ECHO_PIN = 26;
float distance = 0;
# 🏥 Medical Smart Car Project

> An autonomous obstacle-avoiding car designed for hospital environments.

---

## 🔧 Features

- **Real-time video transmission**  
  View live video stream via web interface using MJPEG format.

- **Webpage remote control**  
  Control movement (forward, backward, left, right) directly through browser.

- **Temperature regulation**  
  Supports heating and cooling for medical samples.

- **Single-servo steering**  
  Uses mechanical linkage to simplify control and reduce energy usage.

---

## 📦 Project Structure

This project consists of multiple functional modules:

### 1. Hardware Control (ESP32)

The ESP32 controls motors, servos, temperature logic and communicates with sensors.

![ESP32 Module](images/esp32.jpg)

### 2. Web Server (C++ + httplib)

- Hosts the front-end page
- Provides RESTful APIs for temperature, control, video feed

---

## 📸 Live Demo Screenshot

![Car Top View](images/top_view.png)

*Figure: Onboard camera feed and 3D-printed shell*

---

## 📁 File Structure

！[image](images/top_view.png)

!.[image].(blob/main/%E7%94%B5%E8%B7%AF.png)
