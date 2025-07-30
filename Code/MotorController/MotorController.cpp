#include "MotorController.h"

MotorController::MotorController(int left_pin1, int left_pin2, 
                               int right_pin1, int right_pin2)
    : left_pin1_num_(left_pin1), left_pin2_num_(left_pin2),
      right_pin1_num_(right_pin1), right_pin2_num_(right_pin2),
      current_state_(MotionState::STOP), initialized_(false) {
    
    // Initialize the gpiod chip
    chip_ = std::make_unique<gpiod::chip>("gpiochip0");
}

MotorController::~MotorController() {
    shutdown();
}

bool MotorController::initialize() {
    try {
        setupGPIOLines();
        stopMotorsImmediate();
        initialized_ = true;
        
        LOG_INFO("MotorController initialized with real GPIO event-driven control");
        return true;
        
    } catch (const std::exception& e) {
        LOG_ERROR("MotorController initialization failed: " + std::string(e.what()));
        return false;
    }
}

void MotorController::setupGPIOLines() {
    try {
        // Obtain GPIO lines
        left_pin1_ = std::make_unique<gpiod::line>(chip_->get_line(left_pin1_num_));
        left_pin2_ = std::make_unique<gpiod::line>(chip_->get_line(left_pin2_num_));
        right_pin1_ = std::make_unique<gpiod::line>(chip_->get_line(right_pin1_num_));
        right_pin2_ = std::make_unique<gpiod::line>(chip_->get_line(right_pin2_num_));
        
        // Set to output mode
        left_pin1_->request({"motor_left_1", gpiod::line_request::DIRECTION_OUTPUT}, 0);
        left_pin2_->request({"motor_left_2", gpiod::line_request::DIRECTION_OUTPUT}, 0);
        right_pin1_->request({"motor_right_1", gpiod::line_request::DIRECTION_OUTPUT}, 0);
        right_pin2_->request({"motor_right_2", gpiod::line_request::DIRECTION_OUTPUT}, 0);
        
        LOG_INFO("GPIO lines configured for motor control");
        
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to setup GPIO lines: " + std::string(e.what()));
    }
}

void MotorController::shutdown() {
    if (initialized_.load()) {
        stopMotorsImmediate();
        initialized_ = false;
        LOG_INFO("MotorController shutdown complete");
    }
}

void MotorController::setGPIO(gpiod::line& line, bool state) {
    try {
        line.set_value(state ? 1 : 0);
    } catch (const std::exception& e) {
        LOG_ERROR("GPIO set error: " + std::string(e.what()));
    }
}

void MotorController::stopMotorsImmediate() {
    std::lock_guard<std::mutex> lock(control_mutex_);
    
    if (!initialized_.load()) return;
    
    try {
        // Immediately stop all motors (without delay)
        setGPIO(*left_pin1_, false);
        setGPIO(*left_pin2_, false);
        setGPIO(*right_pin1_, false);
        setGPIO(*right_pin2_, false);
        
        current_state_ = MotionState::STOP;
        
    } catch (const std::exception& e) {
        LOG_ERROR("Stop motors error: " + std::string(e.what()));
    }
}

void MotorController::executeMotion(MotionState state) {
    // Emergency stop check (highest priority)
    if (g_emergency_stop.load()) {
        emergencyStop();
        return;
    }
    
    if (!initialized_.load()) {
        LOG_ERROR("MotorController not initialized");
        return;
    }
    
    std::lock_guard<std::mutex> lock(control_mutex_);
    
    try {
        // First, stop all motors (safe operation)
        setGPIO(*left_pin1_, false);
        setGPIO(*left_pin2_, false);
        setGPIO(*right_pin1_, false);
        setGPIO(*right_pin2_, false);
        
        // Set the motor according to the status (immediate response, no delay)
        switch (state) {
            case MotionState::FORWARD:
                setGPIO(*left_pin1_, true);   // left wheel forward
                setGPIO(*right_pin1_, true);  // right wheel forward
                LOG_DEBUG("Motors set to FORWARD");
                break;
                
            case MotionState::BACKWARD:
                setGPIO(*left_pin2_, true);   // Left wheel back
                setGPIO(*right_pin2_, true);  // right wheel back
                LOG_DEBUG("Motors set to BACKWARD");
                break;
                
            case MotionState::TURN_LEFT:
                setGPIO(*left_pin2_, true);   // Left wheel reverse
                setGPIO(*right_pin1_, true);  // right wheel forward
                LOG_DEBUG("Motors set to TURN_LEFT");
                break;
                
            case MotionState::TURN_RIGHT:
                setGPIO(*left_pin1_, true);   // left wheel forward
                setGPIO(*right_pin2_, true);  // right wheel reverse
                LOG_DEBUG("Motors set to TURN_RIGHT");
                break;
                
            case MotionState::STOP:
            default:
                // All motors have stopped above.
                LOG_DEBUG("Motors set to STOP");
                break;
        }
        
        current_state_ = state;
        LOG_INFO("Motor state immediately changed to: " + std::to_string(static_cast<int>(state)));
        
    } catch (const std::exception& e) {
        LOG_ERROR("Execute motion error: " + std::string(e.what()));
        stopMotorsImmediate();  // Stop immediately when an error occurs
    }
}

void MotorController::emergencyStop() {
    // Emergency stop: Highest priority, lock-free operation, immediate response
    try {
        if (initialized_.load()) {
            setGPIO(*left_pin1_, false);
            setGPIO(*left_pin2_, false);
            setGPIO(*right_pin1_, false);
            setGPIO(*right_pin2_, false);
            
            current_state_ = MotionState::STOP;
        }
        
        LOG_WARNING("EMERGENCY STOP activated - Motors stopped immediately!");
        
    } catch (const std::exception& e) {
        LOG_ERROR("Emergency stop error: " + std::string(e.what()));
    }
}

MotionState MotorController::getCurrentState() const {
    return current_state_.load();
}

bool MotorController::isInitialized() const {
    return initialized_.load();
}

void MotorController::setMotionState(MotionState state) {
    // Pure event-driven interface: Set status immediately, no sequence control
    executeMotion(state);
}

void MotorController::executeEmergencyBrake() {
    // Emergency braking: more powerful than normal stopping
    emergencyStop();
}
