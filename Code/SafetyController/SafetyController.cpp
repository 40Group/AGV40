#include "../include/SafetyController.h"

SafetyController::SafetyController()
    : motor_controller_(nullptr), ultrasonic_sensor_(nullptr), ir_sensor_(nullptr),
      temp_controller_(nullptr), running_(false), current_state_(SafetyState::NORMAL),
      emergency_stop_active_(false), safe_distance_threshold_(Constants::MAX_SAFE_DISTANCE),
      critical_distance_threshold_(Constants::MIN_SAFE_DISTANCE),
      max_safe_temperature_(45.0), min_safe_temperature_(10.0) {
}

SafetyController::~SafetyController() {
    shutdown();
}

bool SafetyController::initialize(MotorController* motor, UltrasonicSensor* ultrasonic,
                                 InfraredSensor* ir, TemperatureController* temp) {
    if (!motor || !ultrasonic || !ir || !temp) {
        std::cerr << "SafetyController: Invalid component pointers" << std::endl;
        return false;
    }
    
    motor_controller_ = motor;
    ultrasonic_sensor_ = ultrasonic;
    ir_sensor_ = ir;
    temp_controller_ = temp;
    
    running_ = true;
    current_state_ = SafetyState::NORMAL;
    
    // Start the security monitoring thread
    safety_monitor_thread_ = std::thread(&SafetyController::safetyMonitorLoop, this);
    
    std::cout << "SafetyController initialized successfully" << std::endl;
    return true;
}

void SafetyController::shutdown() {
    running_ = false;
    
    if (safety_monitor_thread_.joinable()) {
        safety_monitor_thread_.join();
    }
    
    std::cout << "SafetyController shutdown" << std::endl;
}

void SafetyController::safetyMonitorLoop() {
    while (running_.load()) {
        try {
            bool obstacle_safe = checkObstacleSafety();
            bool temperature_safe = checkTemperatureSafety();
            bool system_healthy = checkSystemHealth();
            
            // Updated security status based on inspection results
            if (!system_healthy) {
                current_state_ = SafetyState::SYSTEM_ERROR;
                handleSystemError();
            } else if (!temperature_safe) {
                current_state_ = SafetyState::TEMPERATURE_EMERGENCY;
                handleTemperatureEmergency();
            } else if (!obstacle_safe) {
                current_state_ = SafetyState::OBSTACLE_DETECTED;
                handleObstacleDetection();
            } else {
                current_state_ = SafetyState::NORMAL;
            }
            
        } catch (const std::exception& e) {
            std::cerr << "SafetyController error: " << e.what() << std::endl;
            logSafetyEvent("Exception in safety monitor: " + std::string(e.what()));
        }
        
        // Check every 100ms
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

bool SafetyController::checkObstacleSafety() {
    // Checking the ultrasonic sensor
    double distance = ultrasonic_sensor_->getLatestDistance();
    bool ultrasonic_safe = distance > critical_distance_threshold_;
    
    // Check the infrared sensor
    bool ir_safe = !ir_sensor_->isAnyObstacle();
    
    return ultrasonic_safe && ir_safe;
}

bool SafetyController::checkTemperatureSafety() {
    if (!temp_controller_->isRunning()) {
        return false;
    }
    
    double current_temp = temp_controller_->getCurrentTemperature();
    
    // Check that the temperature is within the safe range
    bool temp_safe = (current_temp >= min_safe_temperature_ && 
                     current_temp <= max_safe_temperature_);
    
    return temp_safe;
}

bool SafetyController::checkSystemHealth() {
    // Check that the components are functioning correctly
    return motor_controller_->isRunning() &&
           ultrasonic_sensor_->isRunning() &&
           ir_sensor_->isRunning() &&
           temp_controller_->isRunning();
}

void SafetyController::handleObstacleDetection() {
    std::lock_guard<std::mutex> lock(safety_mutex_);
    
    // Stop the motor immediately
    motor_controller_->emergencyStop();
    
    std::string reason = "Obstacle detected - ";
    
    // Get detailed information on obstacles
    double distance = ultrasonic_sensor_->getLatestDistance();
    auto ir_state = ir_sensor_->getAllStates();
    
    if (distance <= critical_distance_threshold_) {
        reason += "Ultrasonic distance: " + std::to_string(distance) + "cm ";
    }
    
    if (ir_state.front) reason += "Front IR ";
    if (ir_state.left) reason += "Left IR ";
    if (ir_state.right) reason += "Right IR ";
    
    logSafetyEvent(reason);
    std::cout << "SAFETY: " << reason << std::endl;
}

void SafetyController::handleTemperatureEmergency() {
    std::lock_guard<std::mutex> lock(safety_mutex_);
    
    double current_temp = temp_controller_->getCurrentTemperature();
    std::string reason = "Temperature emergency: " + std::to_string(current_temp) + "°C";
    
    // Turn off the temperature control system
    temp_controller_->turnOffControl();
    
    // If the temperature is too high, stop the movement to reduce heat production
    if (current_temp > max_safe_temperature_) {
        motor_controller_->emergencyStop();
        reason += " (overheating)";
    }
    
    logSafetyEvent(reason);
    std::cout << "SAFETY: " << reason << std::endl;
}

void SafetyController::handleSystemError() {
    std::lock_guard<std::mutex> lock(safety_mutex_);
    
    // Stop all operations in case of system error
    motor_controller_->emergencyStop();
    temp_controller_->turnOffControl();
    
    std::string reason = "System component failure detected";
    logSafetyEvent(reason);
    std::cout << "SAFETY: " << reason << std::endl;
}

void SafetyController::handleEmergencyStop() {
    std::lock_guard<std::mutex> lock(safety_mutex_);
    
    motor_controller_->emergencyStop();
    emergency_stop_active_ = true;
    emergency_start_time_ = std::chrono::steady_clock::now();
    current_state_ = SafetyState::EMERGENCY_STOP;
    
    logSafetyEvent("Manual emergency stop activated");
    std::cout << "EMERGENCY STOP ACTIVATED!" << std::endl;
}

void SafetyController::triggerEmergencyStop(const std::string& reason) {
    handleEmergencyStop();
    logSafetyEvent("Emergency stop reason: " + reason);
}

void SafetyController::clearEmergencyStop() {
    std::lock_guard<std::mutex> lock(safety_mutex_);
    
    if (emergency_stop_active_) {
        // Check for safe recovery
        if (checkObstacleSafety() && checkTemperatureSafety() && checkSystemHealth()) {
            emergency_stop_active_ = false;
            motor_controller_->resumeFromEmergency();
            current_state_ = SafetyState::NORMAL;
            
            logSafetyEvent("Emergency stop cleared");
            std::cout << "Emergency stop cleared - system resumed" << std::endl;
        } else {
            std::cout << "Cannot clear emergency stop - unsafe conditions persist" << std::endl;
        }
    }
}

bool SafetyController::isSafeToMove() {
    return current_state_.load() == SafetyState::NORMAL && 
           checkObstacleSafety() && 
           !emergency_stop_active_.load();
}

bool SafetyController::isSafeToOperate() {
    return current_state_.load() != SafetyState::SYSTEM_ERROR &&
           current_state_.load() != SafetyState::EMERGENCY_STOP &&
           checkSystemHealth();
}

bool SafetyController::checkMotorCommand(const MotorCommand& command) {
    // Check for emergency stop
    if (emergency_stop_active_.load()) {
        return false;
    }
    
    // Checking for safe movement
    if (!isSafeToMove() && (command.left_speed != 0 || command.right_speed != 0)) {
        return false;
    }
    
    // Check that the speed is within safe limits
    int max_safe_speed = Constants::MAX_MOTOR_SPEED;
    if (std::abs(command.left_speed) > max_safe_speed || 
        std::abs(command.right_speed) > max_safe_speed) {
        return false;
    }
    
    return true;
}

MotorCommand SafetyController::applySafetyConstraints(const MotorCommand& command) {
    MotorCommand safe_command = command;
    
    // If unsafe, force stop
    if (!checkMotorCommand(command)) {
        safe_command.left_speed = 0;
        safe_command.right_speed = 0;
        safe_command.emergency_stop = true;
        return safe_command;
    }
    
    // Adjusts speed according to obstacle distance
    double distance = ultrasonic_sensor_->getLatestDistance();
    if (distance < safe_distance_threshold_) {
        double speed_factor = distance / safe_distance_threshold_;
        speed_factor = std::max(0.3, speed_factor); // Minimum 30 per cent speed
        
        safe_command.left_speed = static_cast<int>(command.left_speed * speed_factor);
        safe_command.right_speed = static_cast<int>(command.right_speed * speed_factor);
    }
    
    return safe_command;
}

void SafetyController::setSafetyThresholds(double safe_distance, double critical_distance,
                                         double max_temp, double min_temp) {
    safe_distance_threshold_ = safe_distance;
    critical_distance_threshold_ = critical_distance;
    max_safe_temperature_ = max_temp;
    min_safe_temperature_ = min_temp;
    
    std::cout << "Safety thresholds updated - Distance: " << safe_distance 
              << "/" << critical_distance << " cm, Temp: " << min_temp 
              << "-" << max_temp << "°C" << std::endl;
}

std::string SafetyController::getStateDescription() const {
    switch (current_state_.load()) {
        case SafetyState::NORMAL:
            return "Normal operation";
        case SafetyState::OBSTACLE_DETECTED:
            return "Obstacle detected - movement restricted";
        case SafetyState::TEMPERATURE_EMERGENCY:
            return "Temperature emergency - thermal protection active";
        case SafetyState::SYSTEM_ERROR:
            return "System error - components malfunction";
        case SafetyState::EMERGENCY_STOP:
            return "Emergency stop - manual intervention required";
        default:
            return "Unknown state";
    }
}

std::chrono::milliseconds SafetyController::getEmergencyDuration() const {
    if (!emergency_stop_active_.load()) {
        return std::chrono::milliseconds(0);
    }
    
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - emergency_start_time_);
}

void SafetyController::logSafetyEvent(const std::string& event) {
    std::lock_guard<std::mutex> lock(log_mutex_);
    
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    
    std::string timestamp = std::ctime(&time_t);
    timestamp.pop_back(); // Remove line breaks
    
    std::string log_entry = "[" + timestamp + "] " + event;
    safety_log_.push_back(log_entry);
    
    // Limit log size
    if (safety_log_.size() > 1000) {
        safety_log_.erase(safety_log_.begin());
    }
}

std::vector<std::string> SafetyController::getSafetyLog() const {
    std::lock_guard<std::mutex> lock(log_mutex_);
    return safety_log_;
}

void SafetyController::clearSafetyLog() {
    std::lock_guard<std::mutex> lock(log_mutex_);
    safety_log_.clear();
}

bool SafetyController::selfTest() {
    std::cout << "Starting safety controller self-test..." << std::endl;
    
    if (!running_.load()) {
        std::cout << "Safety controller not initialized" << std::endl;
        return false;
    }
    
    // Testing of individual check functions
    std::cout << "Testing obstacle safety check..." << std::endl;
    bool obstacle_safe = checkObstacleSafety();
    std::cout << "Obstacle safety: " << (obstacle_safe ? "OK" : "FAIL") << std::endl;
    
    std::cout << "Testing temperature safety check..." << std::endl;
    bool temp_safe = checkTemperatureSafety();
    std::cout << "Temperature safety: " << (temp_safe ? "OK" : "FAIL") << std::endl;
    
    std::cout << "Testing system health check..." << std::endl;
    bool system_healthy = checkSystemHealth();
    std::cout << "System health: " << (system_healthy ? "OK" : "FAIL") << std::endl;
    
    // Test Emergency Stop
    std::cout << "Testing emergency stop..." << std::endl;
    triggerEmergencyStop("Self-test");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    clearEmergencyStop();
    
    std::cout << "Current safety state: " << getStateDescription() << std::endl;
    std::cout << "Safety controller self-test completed" << std::endl;
    
    return true;
}