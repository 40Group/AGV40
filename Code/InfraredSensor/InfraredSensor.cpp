#include "InfraredSensor.h"

InfraredSensor::InfraredSensor(int left_pin, int right_pin)
    : left_pin_(left_pin), right_pin_(right_pin),
      running_(false), left_detected_(false), right_detected_(false) {
    
    // Initialize the gpiod component
    chip_ = std::make_unique<gpiod::chip>("gpiochip0");
}

InfraredSensor::~InfraredSensor() {
    stop();
}

bool InfraredSensor::initialize() {
    try {
        // Get GPIO lines
        left_line_ = std::make_unique<gpiod::line>(chip_->get_line(left_pin_));
        right_line_ = std::make_unique<gpiod::line>(chip_->get_line(right_pin_));
        
        setupGPIOEvents();
        
        LOG_INFO("InfraredSensor initialized successfully with real event-driven GPIO");
        return true;
        
    } catch (const std::exception& e) {
        LOG_ERROR("InfraredSensor initialization failed: " + std::string(e.what()));
        return false;
    }
}

void InfraredSensor::setupGPIOEvents() {
    try {
        // Configure the left sensor as a dual-edge interrupt (real hardware event)
        left_line_->request({
            "ir_left_sensor", 
            gpiod::line_request::EVENT_BOTH_EDGES,
            gpiod::line_request::FLAG_BIAS_PULL_UP
        });
        
        // Configure the right sensor as a dual-edge interrupt (real hardware event)
        right_line_->request({
            "ir_right_sensor", 
            gpiod::line_request::EVENT_BOTH_EDGES,
            gpiod::line_request::FLAG_BIAS_PULL_UP
        });
        
        LOG_INFO("GPIO event configuration completed - Real hardware interrupts enabled");
        
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to setup GPIO events: " + std::string(e.what()));
    }
}

void InfraredSensor::start() {
    if (running_.load()) {
        return;
    }
    
    running_ = true;
    
    // Start the real event-driven thread
    event_thread_ = std::thread(&InfraredSensor::realEventLoop, this);
    
    LOG_INFO("InfraredSensor started with REAL event-driven monitoring");
}

void InfraredSensor::stop() {
    running_ = false;
    
    if (event_thread_.joinable()) {
        event_thread_.join();
    }
    
    LOG_INFO("InfraredSensor stopped");
}

void InfraredSensor::realEventLoop() {
    LOG_INFO("Real GPIO event loop started - waiting for hardware interrupts");
    
    while (running_.load() && !g_emergency_stop.load()) {
        try {
            // True event-driven: monitoring hardware interrupts from two GPIO lines simultaneously
            std::vector<gpiod::line*> lines = {left_line_.get(), right_line_.get()};
            
            // Block waiting for real hardware events (timeout of 1 second to check running status)
            auto event_lines = gpiod::line::event_wait(lines, std::chrono::seconds(1));
            
            if (!event_lines.empty()) {
                // Handle all triggered hardware events
                for (auto* line : event_lines) {
                    if (line->event_wait(std::chrono::milliseconds(0))) {
                        auto event = line->event_read();
                        handleGPIOEvent(*line, event);
                    }
                }
            }
            
        } catch (const std::exception& e) {
            LOG_ERROR("Real event loop error: " + std::string(e.what()));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    LOG_INFO("Real GPIO event loop terminated");
}

void InfraredSensor::handleGPIOEvent(gpiod::line& line, const gpiod::line_event& event) {
    try {
        // Determine which sensor triggered the event
        bool is_left_sensor = (&line == left_line_.get());
        bool is_right_sensor = (&line == right_line_.get());
        
        // Infrared sensor logic: LOW = obstacle detected, HIGH = no obstacle
        bool obstacle_detected = (event.event_type == gpiod::line_event::FALLING_EDGE);
        
        // Update the corresponding sensor status (atomic operation)
        if (is_left_sensor) {
            left_detected_.store(obstacle_detected);
            LOG_DEBUG("Left IR sensor real event: " + std::string(obstacle_detected ? "OBSTACLE" : "CLEAR"));
        } else if (is_right_sensor) {
            right_detected_.store(obstacle_detected);
            LOG_DEBUG("Right IR sensor real event: " + std::string(obstacle_detected ? "OBSTACLE" : "CLEAR"));
        }
        
        // Trigger callback (if registered)
        {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            if (boundary_callback_) {
                boundary_callback_(left_detected_.load(), right_detected_.load());
            }
        }
        
    } catch (const std::exception& e) {
        LOG_ERROR("GPIO event handling error: " + std::string(e.what()));
    }
}

void InfraredSensor::registerCallback(std::function<void(bool, bool)> callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    boundary_callback_ = callback;
    LOG_INFO("IR sensor callback registered for real-time boundary detection");
}

// Thread-safe status query method
bool InfraredSensor::isLeftDetected() const {
    return left_detected_.load();
}

bool InfraredSensor::isRightDetected() const {
    return right_detected_.load();
}

bool InfraredSensor::isBoundaryDetected() const {
    return left_detected_.load() || right_detected_.load();
}

// Testing compatibility methods
bool InfraredSensor::isRunning() const {
    return running_.load();
}

bool InfraredSensor::selfTest() {
    try {
        // Simple self-test: Verify GPIO line availability
        if (!left_line_ || !right_line_) {
            return false;
        }
        
        // Read the current status to verify the hardware connection
        int left_value = left_line_->get_value();
        int right_value = right_line_->get_value();
        
        LOG_INFO("IR Sensor self-test passed - GPIO values: L=" + 
                std::to_string(left_value) + " R=" + std::to_string(right_value));
        
        return true;
        
    } catch (const std::exception& e) {
        LOG_ERROR("IR Sensor self-test failed: " + std::string(e.what()));
        return false;
    }
}

void InfraredSensor::setPollingInterval(std::chrono::milliseconds interval) {
    // Note: In real event-driven mode, this method is only for compatibility testing
    // In fact, no polling interval is required because it is hardware interrupt driven
    LOG_INFO("Note: Polling interval not applicable in real event-driven mode. Using hardware interrupts.");
}

void InfraredSensor::shutdown() {
    stop();
    LOG_INFO("InfraredSensor shutdown completed");
}

InfraredSensor::SensorStates InfraredSensor::getAllStates() const {
    return {
        left_detected_.load(),
        right_detected_.load(),
        left_detected_.load() || right_detected_.load()  // front = 任一传感器检测到
    };
}

bool InfraredSensor::isAnyObstacle() const {
    return isBoundaryDetected();
}
