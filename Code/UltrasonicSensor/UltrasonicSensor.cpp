#include "UltrasonicSensor.h"
#include "../timer/TimerManager.h"
#include <stdexcept>

UltrasonicSensor::UltrasonicSensor(int trigger_pin, int echo_pin, 
                                   const std::string& chip_path)
    : trigger_pin_(trigger_pin), echo_pin_(echo_pin), chip_path_(chip_path),
      last_distance_(999.0), obstacle_detected_(false), measurement_active_(false),
      obstacle_threshold_cm_(20.0), timer_manager_(nullptr) {
}

UltrasonicSensor::~UltrasonicSensor() {
    stopEventDriven();
}

bool UltrasonicSensor::initialize(TimerManager* timer_manager) {
    try {
        if (!timer_manager) {
            LOG_ERROR("TimerManager is required for event-driven ultrasonic sensor");
            return false;
        }
        
        timer_manager_ = timer_manager;
        
        // Initialize the GPIO chip
        gpio_chip_ = std::make_unique<gpiod::chip>(chip_path_);
        if (!gpio_chip_->is_used()) {
            LOG_ERROR("Failed to initialize GPIO chip: " + chip_path_);
            return false;
        }
        
        setupGPIO();
        
        LOG_INFO("UltrasonicSensor initialized successfully (Event-Driven + gpiod)");
        return true;
        
    } catch (const std::exception& e) {
        LOG_ERROR("UltrasonicSensor initialization failed: " + std::string(e.what()));
        return false;
    }
}

void UltrasonicSensor::setupGPIO() {
    // Configure the trigger pin as output
    trigger_line_ = std::make_unique<gpiod::line>(gpio_chip_->get_line(trigger_pin_));
    gpiod::line_request trigger_config = {
        "ultrasonic_trigger",
        gpiod::line_request::DIRECTION_OUTPUT,
        0  // 初始低电平
    };
    trigger_line_->request(trigger_config);
    
    // Configure the echo pin as input and enable edge interrupt
    echo_line_ = std::make_unique<gpiod::line>(gpio_chip_->get_line(echo_pin_));
    gpiod::line_request echo_config = {
        "ultrasonic_echo",
        gpiod::line_request::EVENT_BOTH_EDGES,
        0
    };
    echo_line_->request(echo_config);
    
    LOG_INFO("GPIO configured - Trigger: GPIO" + std::to_string(trigger_pin_) + 
             ", Echo: GPIO" + std::to_string(echo_pin_));
}

void UltrasonicSensor::startEventDriven() {
    if (!timer_manager_) {
        LOG_ERROR("Cannot start event-driven mode without TimerManager");
        return;
    }
    
    // Register GPIO interrupt processing
    std::thread echo_monitor([this]() {
        while (!g_emergency_stop.load()) {
            try {
                // Wait for the echo pin interrupt event
                if (echo_line_->event_wait(std::chrono::milliseconds(100))) {
                    auto event = echo_line_->event_read();
                    
                    if (event.event_type == gpiod::line_event::RISING_EDGE) {
                        handleEchoRisingEdge();
                    } else if (event.event_type == gpiod::line_event::FALLING_EDGE) {
                        handleEchoFallingEdge();
                    }
                }
            } catch (const std::exception& e) {
                LOG_ERROR("Echo interrupt error: " + std::string(e.what()));
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    });
    echo_monitor.detach();
    
    // Register for timed ranging events (10Hz frequency)
    timer_manager_->scheduleRepeating(100, [this]() {
        requestMeasurement();
    });
    
    LOG_INFO("UltrasonicSensor started in event-driven mode");
}

void UltrasonicSensor::stopEventDriven() {
    // TimerManager will automatically clean up timer events
    LOG_INFO("UltrasonicSensor stopped");
}

void UltrasonicSensor::requestMeasurement() {
    if (measurement_active_.load()) {
        return; // The last measurement has not been completed
    }
    
    std::lock_guard<std::mutex> lock(measurement_mutex_);
    measurement_active_ = true;
    
    // Start ranging sequence
    triggerMeasurement();
}

void UltrasonicSensor::triggerMeasurement() {
    try {
        // Send 10μs trigger pulse
        trigger_line_->set_value(1);
        std::this_thread::sleep_for(std::chrono::microseconds(10));
        trigger_line_->set_value(0);
        
        // Setting timeout protection
        timer_manager_->scheduleOnce(50, [this]() {
            if (measurement_active_.load()) {
                LOG_WARNING("Ultrasonic measurement timeout");
                measurement_active_ = false;
            }
        });
        
    } catch (const std::exception& e) {
        LOG_ERROR("Trigger pulse error: " + std::string(e.what()));
        measurement_active_ = false;
    }
}

void UltrasonicSensor::handleEchoRisingEdge() {
    if (!measurement_active_.load()) {
        return;
    }
    
    // Record echo start time
    echo_start_time_ = std::chrono::high_resolution_clock::now();
}

void UltrasonicSensor::handleEchoFallingEdge() {
    if (!measurement_active_.load()) {
        return;
    }
    
    // Calculating pulse duration
    auto echo_end_time = std::chrono::high_resolution_clock::now();
    auto pulse_duration = std::chrono::duration_cast<std::chrono::microseconds>(
        echo_end_time - echo_start_time_);
    
    measurement_active_ = false;
    
    // Calculating distance
    calculateDistance(pulse_duration);
}

void UltrasonicSensor::calculateDistance(std::chrono::microseconds pulse_duration) {
    try {
        // Sound speed calculation: distance = (pulse_time * 0.034) / 2
        double distance_cm = (pulse_duration.count() * 0.034) / 2.0;
        
        // Verify measurement range
        if (distance_cm > 0 && distance_cm < 400) {
            processDistanceResult(distance_cm);
        } else {
            LOG_WARNING("Invalid distance measurement: " + std::to_string(distance_cm) + "cm");
        }
        
    } catch (const std::exception& e) {
        LOG_ERROR("Distance calculation error: " + std::string(e.what()));
    }
}

void UltrasonicSensor::processDistanceResult(double distance) {
    // Update distance data
    last_distance_ = distance;
    
    // Obstacle Detection
    bool obstacle = (distance < obstacle_threshold_cm_);
    bool prev_obstacle = obstacle_detected_.load();
    obstacle_detected_ = obstacle;
    
    // Trigger callback (when state changes)
    if (obstacle != prev_obstacle) {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        if (obstacle_callback_) {
            // Execute callback in TimerManager's event thread
            timer_manager_->scheduleOnce(0, [this, obstacle, distance]() {
                obstacle_callback_(obstacle, distance);
            });
        }
    }
    
    LOG_DEBUG("Distance: " + std::to_string(distance) + "cm, Obstacle: " + 
              (obstacle ? "Yes" : "No"));
}

void UltrasonicSensor::registerCallback(ObstacleCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    obstacle_callback_ = callback;
    LOG_INFO("Obstacle callback registered");
}

double UltrasonicSensor::getDistance() const {
    return last_distance_.load();
}

bool UltrasonicSensor::isObstacleDetected() const {
    return obstacle_detected_.load();
}

bool UltrasonicSensor::isMeasurementActive() const {
    return measurement_active_.load();
}

void UltrasonicSensor::setObstacleThreshold(double threshold_cm) {
    if (threshold_cm > 0 && threshold_cm < 200) {
        obstacle_threshold_cm_ = threshold_cm;
        LOG_INFO("Obstacle threshold set to " + std::to_string(threshold_cm) + "cm");
    } else {
        LOG_ERROR("Invalid threshold value: " + std::to_string(threshold_cm) + "cm");
    }
}
