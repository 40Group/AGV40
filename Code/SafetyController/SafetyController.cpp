#include "SafetyController.h"

SafetyController::SafetyController(int emergency_button_pin)
    : emergency_button_pin_(emergency_button_pin),
      running_(false), emergency_active_(false),
      max_temperature_(40.0), min_distance_(10.0), max_operation_time_minutes_(120),
      vision_healthy_(true), ultrasonic_healthy_(true), temperature_healthy_(true) {
    
    auto now = std::chrono::steady_clock::now();
    start_time_ = now;
    last_vision_update_.store(now);
    last_ultrasonic_update_.store(now);
    last_temperature_update_.store(now);
    
    // 初始化gpiod chip
    chip_ = std::make_unique<gpiod::chip>("gpiochip0");
}

SafetyController::~SafetyController() {
    stop();
}

bool SafetyController::initialize() {
    try {
        setupEmergencyButtonGPIO();
        LOG_INFO("SafetyController initialized with REAL GPIO event-driven emergency button");
        return true;
        
    } catch (const std::exception& e) {
        LOG_ERROR("SafetyController initialization failed: " + std::string(e.what()));
        return false;
    }
}

void SafetyController::setupEmergencyButtonGPIO() {
    try {
        // Obtain emergency button GPIO circuit
        emergency_button_line_ = std::make_unique<gpiod::line>(
            chip_->get_line(emergency_button_pin_)
        );
        
        // Configure as falling edge interrupt (triggered when button is pressed)
        emergency_button_line_->request({
            "emergency_button", 
            gpiod::line_request::EVENT_FALLING_EDGE,
            gpiod::line_request::FLAG_BIAS_PULL_UP
        });
        
        LOG_INFO("Emergency button GPIO configured for real hardware interrupts");
        
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to setup emergency button GPIO: " + std::string(e.what()));
    }
}

void SafetyController::start() {
    if (running_.load()) {
        return;
    }
    
    running_ = true;
    start_time_ = std::chrono::steady_clock::now();
    
    // Start the real GPIO event listening thread
    emergency_button_thread_ = std::thread(&SafetyController::realEmergencyButtonLoop, this);
    
    LOG_INFO("SafetyController started with REAL event-driven monitoring");
}

void SafetyController::stop() {
    running_ = false;
    
    if (emergency_button_thread_.joinable()) {
        emergency_button_thread_.join();
    }
    
    LOG_INFO("SafetyController stopped");
}

void SafetyController::realEmergencyButtonLoop() {
    LOG_INFO("Real GPIO emergency button event loop started");
    
    while (running_.load()) {
        try {
            // Real-event driven: Blocking wait for hardware GPIO interrupt
            if (emergency_button_line_->event_wait(std::chrono::seconds(1))) {
                auto event = emergency_button_line_->event_read();
                
                if (event.event_type == gpiod::line_event::FALLING_EDGE) {
                    handleEmergencyButtonEvent();
                }
            }
            
            // Event-driven health checks (non-polling)
            checkHealthEventDriven();
            
        } catch (const std::exception& e) {
            LOG_ERROR("Real emergency button event error: " + std::string(e.what()));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    LOG_INFO("Real GPIO emergency button event loop terminated");
}

void SafetyController::handleEmergencyButtonEvent() {
    LOG_WARNING("🔥 REAL HARDWARE EMERGENCY BUTTON PRESSED!");
    triggerEmergencyStop("Hardware emergency button activated");
}

void SafetyController::checkHealthEventDriven() {
    // Event-driven health checks: only check when there are health updates
    auto now = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(5);
    
    auto last_vision = last_vision_update_.load();
    auto last_ultrasonic = last_ultrasonic_update_.load();
    auto last_temperature = last_temperature_update_.load();
    
    bool health_changed = false;
    
    // Check the health status of each module Timeout
    if (now - last_vision > timeout && vision_healthy_.load()) {
        vision_healthy_ = false;
        health_changed = true;
        LOG_WARNING("Vision system timeout detected - health event triggered");
    }
    
    if (now - last_ultrasonic > timeout && ultrasonic_healthy_.load()) {
        ultrasonic_healthy_ = false;
        health_changed = true;
        LOG_WARNING("Ultrasonic sensor timeout detected - health event triggered");
    }
    
    if (now - last_temperature > timeout && temperature_healthy_.load()) {
        temperature_healthy_ = false;
        health_changed = true;
        LOG_WARNING("Temperature controller timeout detected - health event triggered");
    }
    
    // Trigger emergency stop only when actual health status changes
    if (health_changed && !isSystemHealthy() && !emergency_active_.load()) {
        triggerEmergencyStop("System health monitoring detected failure");
    }
}

void SafetyController::registerCallback(SafetyCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    safety_callback_ = callback;
    LOG_INFO("Safety callback registered for real-time emergency events");
}

// Event-driven security inspection method (triggered by external sensor callback)
void SafetyController::checkTemperatureEvent(double current_temp) {
    double max_temp = max_temperature_.load();
    
    if (current_temp > max_temp) {
        triggerEmergencyStop("Temperature limit exceeded: " + 
                           std::to_string(current_temp) + "°C > " + 
                           std::to_string(max_temp) + "°C");
    } else if (current_temp < -10.0) {
        triggerEmergencyStop("Temperature too low: " + 
                           std::to_string(current_temp) + "°C");
    }
}

void SafetyController::checkObstacleEvent(double distance) {
    double min_dist = min_distance_.load();
    
    if (distance > 0 && distance < min_dist) {
        triggerEmergencyStop("Critical obstacle detected at " + 
                           std::to_string(distance) + "cm");
    }
}

void SafetyController::checkOperationTimeEvent() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::minutes>(now - start_time_);
    
    if (elapsed.count() > max_operation_time_minutes_.load()) {
        triggerEmergencyStop("Maximum operation time exceeded: " + 
                           std::to_string(elapsed.count()) + " minutes");
    }
}

void SafetyController::triggerEmergencyStop(const std::string& reason) {
    if (emergency_active_.load()) {
        return; // Already in a state of emergency
    }
    
    emergency_active_ = true;
    g_emergency_stop.store(true);
    g_system_state.store(SystemState::EMERGENCY_STOP);
    
    LOG_ERROR("🚨 EMERGENCY STOP TRIGGERED: " + reason);
    
    // Immediately trigger security callback
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        if (safety_callback_) {
            safety_callback_(reason);
        }
    }
}

void SafetyController::manualEmergencyStop() {
    triggerEmergencyStop("Manual emergency stop activated");
}

void SafetyController::resetEmergency() {
    if (!emergency_active_.load()) {
        return;
    }
    
    emergency_active_ = false;
    g_emergency_stop.store(false);
    g_system_state.store(SystemState::RUNNING);
    
    // Reset health status
    vision_healthy_ = true;
    ultrasonic_healthy_ = true;
    temperature_healthy_ = true;
    
    auto now = std::chrono::steady_clock::now();
    last_vision_update_.store(now);
    last_ultrasonic_update_.store(now);
    last_temperature_update_.store(now);
    
    LOG_INFO("Emergency state reset - system ready for event-driven operation");
}

void SafetyController::setTemperatureLimits(double max_temp) {
    max_temperature_ = max_temp;
    LOG_INFO("Temperature limit set to " + std::to_string(max_temp) + "°C");
}

void SafetyController::setMinimumDistance(double min_dist) {
    min_distance_ = min_dist;
    LOG_INFO("Minimum distance set to " + std::to_string(min_dist) + "cm");
}

void SafetyController::setMaxOperationTime(int minutes) {
    max_operation_time_minutes_ = minutes;
    LOG_INFO("Maximum operation time set to " + std::to_string(minutes) + " minutes");
}

// health status updates
void SafetyController::updateVisionHealth() {
    last_vision_update_.store(std::chrono::steady_clock::now());
    vision_healthy_ = true;
    LOG_DEBUG("Vision health event received");
}

void SafetyController::updateUltrasonicHealth() {
    last_ultrasonic_update_.store(std::chrono::steady_clock::now());
    ultrasonic_healthy_ = true;
    LOG_DEBUG("Ultrasonic health event received");
}

void SafetyController::updateTemperatureHealth() {
    last_temperature_update_.store(std::chrono::steady_clock::now());
    temperature_healthy_ = true;
    LOG_DEBUG("Temperature health event received");
}

bool SafetyController::isEmergencyActive() const {
    return emergency_active_.load();
}

bool SafetyController::isSystemHealthy() const {
    return vision_healthy_.load() && ultrasonic_healthy_.load() && temperature_healthy_.load();
}

int SafetyController::getOperationTimeMinutes() const {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::minutes>(now - start_time_);
    return static_cast<int>(elapsed.count());
}

bool SafetyController::isInitialized() const {
    return running_.load();
}
