#include "../common/Common.h"
#include "../MotorController/MotorController.h"
#include "../VisionTracker/VisionTracker.h"
#include "../UltrasonicSensor/UltrasonicSensor.h"
#include "../InfraredSensor/InfraredSensor.h"
#include "../TemperatureController/TemperatureController.h"
#include "../SafetyController/SafetyController.h"
#include "../TimerManager/TimerManager.h"
#include <signal.h>
#include <unistd.h>

// Global termination flag
volatile std::sig_atomic_t g_running = 1;

// Global system components (managed via smart pointers)
std::unique_ptr<TimerManager> g_timer;
std::unique_ptr<MotorController> g_motor;
std::unique_ptr<VisionTracker> g_vision;
std::unique_ptr<UltrasonicSensor> g_ultrasonic;
std::unique_ptr<InfraredSensor> g_infrared;
std::unique_ptr<TemperatureController> g_temperature;
std::unique_ptr<SafetyController> g_safety;

// Signal handler
void signalHandler(int signum) {
    LOG_INFO("Shutdown signal received");
    g_running = 0;
}

// System initialization
bool initializeSystem() {
    LOG_INFO("Initializing Smart Medical Car System (Event-Driven)...");
    
    try {
        // Create TimerManager
        g_timer = std::make_unique<TimerManager>();
        if (!g_timer->initialize()) {
            LOG_ERROR("TimerManager initialization failed");
            return false;
        }
        g_timer->start();
        
        // Create all components
        g_motor = std::make_unique<MotorController>();
        g_vision = std::make_unique<VisionTracker>();
        g_ultrasonic = std::make_unique<UltrasonicSensor>();
        g_infrared = std::make_unique<InfraredSensor>();
        g_temperature = std::make_unique<TemperatureController>();
        g_safety = std::make_unique<SafetyController>();
        
        // Initialise all components (pass TimerManager dependency)
        if (!g_motor->initialize(g_timer.get())) {
            LOG_ERROR("MotorController initialization failed");
            return false;
        }
        
        if (!g_vision->initialize(g_timer.get())) {
            LOG_ERROR("VisionTracker initialization failed");
            return false;
        }
        
        if (!g_ultrasonic->initialize(g_timer.get())) {
            LOG_ERROR("UltrasonicSensor initialization failed");
            return false;
        }
        
        if (!g_infrared->initialize(g_timer.get())) {
            LOG_ERROR("InfraredSensor initialization failed");
            return false;
        }
        
        if (!g_temperature->initialize(g_timer.get())) {
            LOG_ERROR("TemperatureController initialization failed");
            return false;
        }
        
        if (!g_safety->initialize(g_timer.get())) {
            LOG_ERROR("SafetyController initialization failed");
            return false;
        }
        
        LOG_INFO("All components initialized successfully (Event-Driven)");
        return true;
        
    } catch (const std::exception& e) {
        LOG_ERROR("System initialization failed: " + std::string(e.what()));
        return false;
    }
}

// Setup all callback functions (TRUE EVENT-DRIVEN CORE)
void setupCallbacks() {
    LOG_INFO("Setting up event-driven callbacks...");
    
    // Visual tracking callback - Control the car to follow the track
    g_vision->registerCallback([](bool lineDetected, double deviation) {
        g_safety->updateVisionHealth();
        
        if (g_emergency_stop.load()) return;
        
        if (lineDetected) {
            if (std::abs(deviation) < 0.1) {
                g_motor->requestMotion(MotionState::FORWARD);
            } else if (deviation > 0.1) {
                g_motor->requestMotion(MotionState::TURN_RIGHT);
            } else if (deviation < -0.1) {
                g_motor->requestMotion(MotionState::TURN_LEFT);
            }
        } else {
            g_motor->requestMotion(MotionState::STOP);
            LOG_WARNING("Line lost - stopping vehicle");
        }
    });
    
    // Ultrasonic obstacle avoidance callback
    g_ultrasonic->registerCallback([](bool obstacleDetected, double distance) {
        g_safety->updateUltrasonicHealth();
        
        if (obstacleDetected && distance < 15.0) {
            LOG_WARNING("Obstacle detected at " + std::to_string(distance) + "cm - executing avoidance");
            g_motor->requestMotion(MotionState::STOP);
            
            // Event-driven obstacle avoidance sequence
            g_timer->scheduleOnce(300, []() {
                if (!g_emergency_stop.load()) {
                    g_motor->requestMotion(MotionState::BACKWARD);
                    g_timer->scheduleOnce(500, []() {
                        if (!g_emergency_stop.load()) {
                            g_motor->requestMotion(MotionState::TURN_RIGHT);
                            g_timer->scheduleOnce(800, []() {
                                if (!g_emergency_stop.load()) {
                                    g_motor->requestMotion(MotionState::FORWARD);
                                }
                            });
                        }
                    });
                }
            });
        }
    });
    
    // Infrared boundary detection callback
    g_infrared->registerCallback([](bool leftDetected, bool rightDetected) {
        g_safety->updateInfraredHealth();
        
        if (leftDetected || rightDetected) {
            LOG_WARNING("Boundary detected - executing boundary avoidance");
            g_motor->requestMotion(MotionState::STOP);
            
            // Event-driven boundary avoidance
            g_timer->scheduleOnce(200, [leftDetected, rightDetected]() {
                if (!g_emergency_stop.load()) {
                    g_motor->requestMotion(MotionState::BACKWARD);
                    g_timer->scheduleOnce(600, [leftDetected, rightDetected]() {
                        if (!g_emergency_stop.load()) {
                            if (leftDetected) {
                                g_motor->requestMotion(MotionState::TURN_RIGHT);
                            } else {
                                g_motor->requestMotion(MotionState::TURN_LEFT);
                            }
                            g_timer->scheduleOnce(1000, []() {
                                if (!g_emergency_stop.load()) {
                                    g_motor->requestMotion(MotionState::FORWARD);
                                }
                            });
                        }
                    });
                }
            });
        }
    });
    
    // Temperature control callback
    g_temperature->registerCallback([](double temperature, TempControlState state) {
        g_safety->updateTemperatureHealth();
        
        if (temperature > 35.0 || temperature < 5.0) {
            LOG_WARNING("Temperature alert: " + std::to_string(temperature) + "°C");
        }
        
        // Temperature Status Change Log
        static TempControlState last_state = TempControlState::IDLE;
        if (state != last_state) {
            LOG_INFO("Temperature control state: " + std::to_string(static_cast<int>(state)));
            last_state = state;
        }
    });
    
    // Security control callback
    g_safety->registerCallback([](const std::string& emergency) {
        LOG_ERROR("SAFETY EMERGENCY: " + emergency);
        g_motor->emergencyStop();
        g_emergency_stop.store(true);
    });
    
    // Configure security parameters
    g_safety->setTemperatureLimits(40.0);
    g_safety->setMinimumDistance(5.0);
    g_safety->setMaxOperationTime(180); // 3小时
    
    // Configure temperature control
    g_temperature->setTargetTemperature(25.0);
    
    // Configure visual tracking (white line detection)
    g_vision->setLineColor(cv::Scalar(0, 0, 100), cv::Scalar(180, 30, 255));
    g_vision->setCannyThresholds(50, 150);
    
    // Configure ultrasonic detection thresholds
    g_ultrasonic->setObstacleThreshold(20.0);
    
    LOG_INFO("All event-driven callbacks configured successfully");
}

// Start all event-driven services
void startServices() {
    LOG_INFO("Starting all event-driven services...");
    
    g_system_state.store(SystemState::RUNNING);
    
    // Start all event-driven services
    g_safety->startEventDriven();
    g_temperature->startEventDriven();
    g_vision->startEventDriven();
    g_ultrasonic->startEventDriven();
    g_infrared->startEventDriven();
    g_motor->startEventDriven();
    
    LOG_INFO("All event-driven services started successfully");
}

// Cleanup system
void cleanupSystem() {
    LOG_INFO("Shutting down event-driven system...");
    
    g_system_state.store(SystemState::SHUTDOWN);
    g_emergency_stop.store(true);
    
    // Stop all event-driven services
    if (g_motor) g_motor->stopEventDriven();
    if (g_infrared) g_infrared->stopEventDriven();
    if (g_ultrasonic) g_ultrasonic->stopEventDriven();
    if (g_vision) g_vision->stopEventDriven();
    if (g_temperature) g_temperature->stopEventDriven();
    if (g_safety) g_safety->stopEventDriven();
    if (g_timer) g_timer->stop();
    
    LOG_INFO("Event-driven system shutdown complete");
}

// Main function - PURE EVENT-DRIVEN: Only initialization then sleep!
int main() {
    // Set signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    LOG_INFO("=== Smart Medical Car System Starting (Event-Driven) ===");
    
    // Initialize event-driven system
    if (!initializeSystem()) {
        LOG_ERROR("Event-driven system initialization failed!");
        return -1;
    }
    
    // Setup all event callbacks
    setupCallbacks();
    
    // Start all event-driven services
    startServices();
    
    LOG_INFO("=== Event-Driven System Ready - Medical Transport Car Active ===");
    LOG_INFO("Features: Line Tracking, Obstacle Avoidance, Temperature Control, Boundary Detection");
    LOG_INFO("Architecture: Pure Event-Driven with GPIO interrupts + Timer events");
    LOG_INFO("Press Ctrl+C to stop the system");
    
    // Main thread ONLY sleeps - ALL WORK DONE VIA EVENTS AND CALLBACKS!
    while (g_running && !g_emergency_stop.load()) {
        sleep(1);  // Pure sleep - system runs entirely via events
        
        // Minimal status display (every 30 seconds)
        static int counter = 0;
        if (++counter >= 30) {
            LOG_INFO("Status: " + 
                    std::to_string(g_safety->getOperationTimeMinutes()) + 
                    "min, Temp: " + 
                    std::to_string(g_temperature->getCurrentTemperature()) + "°C, " +
                    "Line: " + (g_vision->isLineDetected() ? "OK" : "LOST") + ", " +
                    "Distance: " + std::to_string(g_ultrasonic->getDistance()) + "cm");
            counter = 0;
        }
    }
    
    // Cleanup
    cleanupSystem();
    
    LOG_INFO("=== Smart Medical Car System Stopped ===");
    return 0;
}
