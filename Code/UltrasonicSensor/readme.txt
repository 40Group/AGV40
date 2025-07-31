=== Event-Driven Ultrasonic Sensor Module ===

PURPOSE:
True event-driven ultrasonic distance measurement system with GPIO interrupt handling for real-time obstacle detection in Smart Medical Car.

ARCHITECTURE:
✅ Pure Event-Driven Design - No polling loops
✅ GPIO Interrupt Based - Edge-triggered echo detection  
✅ Timer Event Integration - Scheduled measurement triggers
✅ Callback Response System - Immediate obstacle notifications
✅ gpiod.hpp Integration - Modern Linux GPIO handling

EVENT FLOW:
1. TimerManager triggers measurement request (10Hz)
2. Trigger pulse sent via gpiod output line
3. Echo rising edge interrupt → Start timing
4. Echo falling edge interrupt → Calculate distance
5. Distance processed → Obstacle callback if threshold exceeded
6. Measurement complete → Ready for next trigger

HARDWARE REQUIREMENTS:
- HC-SR04 Ultrasonic Sensor
- Trigger pin: GPIO 23 (default, configurable)
- Echo pin: GPIO 24 (default, configurable) 
- GPIO chip: /dev/gpiochip0 (default)
- 5V power supply + logic level conversion if needed

REAL-TIME FEATURES:
- Interrupt-driven echo detection (<1μs response)
- Zero polling overhead
- Precise microsecond timing
- Immediate callback execution
- Non-blocking measurement process

EVENT-DRIVEN API:
```cpp
// Initialization with TimerManager dependency
sensor.initialize(&timer_manager);

// Start event-driven mode (registers timer + interrupts)
sensor.startEventDriven();

// Register obstacle detection callback
sensor.registerCallback([](bool detected, double distance) {
    if (detected) {
        motor_controller.emergencyStop();
        safety_controller.reportObstacle(distance);
    }
});

// Configuration
sensor.setObstacleThreshold(15.0);  // Triggers callback on change

// Query current state (thread-safe)
double dist = sensor.getDistance();
bool obstacle = sensor.isObstacleDetected();

The UltrasonicSensor module performs event-driven distance measurement using GPIO-based trigger and echo pins, relying on libgpiod for efficient edge-interrupt handling. It periodically emits a 10 μs trigger pulse and waits for echo responses via rising and falling edge events. Upon detecting both edges, it calculates the pulse duration and derives the distance using the speed of sound. If the measured distance falls below a configurable threshold, a registered callback is triggered to notify the system of an obstacle. The design ensures non-blocking, low-latency operation suitable for real-time embedded systems.
