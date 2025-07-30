# Vision Tracker Module

## Purpose: True event-driven computer vision system for line following with GPIO integration and timer-based frame processing for Smart Medical Car autonomous navigation.

## ARCHITECTURE:
✅ Pure Event-Driven Design - No processing loops

✅ Timer-Triggered Capture - Scheduled frame acquisition

✅ GPIO Hardware Integration - External trigger capability

✅ Callback Response System - Immediate navigation events

✅ OpenCV + gpiod.hpp - Modern CV with Linux GPIO

## EVENT FLOW
1. TimerManager triggers frame capture request (10Hz)
2. Camera frame captured asynchronously
3. Frame stored and processing event scheduled
4. Image processing executed in event context
5. Line detection results trigger navigation callback
6. Optional GPIO external trigger for sync capture

## Hardware Requirements:
- Raspberry Pi Camera Module v2 or USB Camera
- GPIO trigger pin: GPIO 25 (default, configurable)
- GPIO chip: /dev/gpiochip0 (default)
- Adequate lighting for line visibility
- Stable camera mounting with vibration isolation

## REAL-TIME FEATURES:
- Event-driven frame processing
- Zero polling overhead for CPU efficiency  
- Immediate callback response to line changes
- Asynchronous image processing pipeline
- Hardware trigger synchronization capability

## All in all
This is a high-performance, low-latency, fully asynchronous vision recognition module, designed for fast-response embedded autonomous navigation systems, and particularly well-suited for line-following applications in smart medical transport vehicles.

## EVENT-DRIVEN API:
```cpp
// Initialization with TimerManager dependency
tracker.initialize(&timer_manager);

// Start event-driven mode (registers timer + GPIO events)
tracker.startEventDriven();

// Register line detection callback
tracker.registerCallback([](bool detected, double deviation) {
    if (detected) {
        if (abs(deviation) > 0.1) {
            motor_controller.adjustSteering(deviation);
        } else {
            motor_controller.driveStraight();
        }
    } else {
        motor_controller.searchForLine();
    }
});

// Configuration
tracker.setLineColor(hsv_min, hsv_max);  // Color-based detection
tracker.setCannyThresholds(50, 150);     // Edge detection tuning

// Query current state (thread-safe)
bool line = tracker.isLineDetected();
double dev = tracker.getLineDeviation();
bool active = tracker.isProcessingActive();
