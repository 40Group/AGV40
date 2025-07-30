#include "TemperatureController.h"
#include <cmath>

TemperatureController::TemperatureController(int temp_pin, int heater_pin, int cooler_pin)
    : temp_sensor_pin_(temp_pin), heater_pin_(heater_pin), cooler_pin_(cooler_pin),
      running_(false), initialized_(false),
      current_temperature_(25.0), target_temperature_(25.0),
      control_state_(TempControlState::IDLE),
      kp_(2.0), ki_(0.1), kd_(0.05),
      integral_(0.0), previous_error_(0.0) {
    
    // Initialize the gpiod chip
    chip_ = std::make_unique<gpiod::chip>("gpiochip0");
}

TemperatureController::~TemperatureController() {
    stop();
}

bool TemperatureController::initialize() {
    try {
        setupGPIOLines();
        
        // Initial state: Heating and cooling turned off
        setHeaterState(false);
        setCoolerState(false);
        
        initialized_ = true;
        
        LOG_INFO("TemperatureController initialized with REAL GPIO event-driven control");
        return true;
        
    } catch (const std::exception& e) {
        LOG_ERROR("TemperatureController initialization failed: " + std::string(e.what()));
        return false;
    }
}

void TemperatureController::setupGPIOLines() {
    try {
        // Obtain GPIO lines
        heater_line_ = std::make_unique<gpiod::line>(chip_->get_line(heater_pin_));
        cooler_line_ = std::make_unique<gpiod::line>(chip_->get_line(cooler_pin_));
        
        // Set to output mode
        heater_line_->request({"temp_heater", gpiod::line_request::DIRECTION_OUTPUT}, 0);
        cooler_line_->request({"temp_cooler", gpiod::line_request::DIRECTION_OUTPUT}, 0);
        
        LOG_INFO("Temperature control GPIO lines configured");
        
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to setup GPIO lines: " + std::string(e.what()));
    }
}

void TemperatureController::start() {
    if (running_.load() || !initialized_.load()) {
        return;
    }
    
    running_ = true;
    
    if (timer_callback_) {
        // Register a 1-second timer event for temperature control
        timer_callback_([this]() {
            if (running_.load()) {
                processTemperatureControlEvent();
                
                // Re-register the next timer event
                if (timer_callback_) {
                    timer_callback_([this]() { processTemperatureControlEvent(); }, 
                                   std::chrono::milliseconds(1000));
                }
            }
        }, std::chrono::milliseconds(1000));
    }
    
    LOG_INFO("TemperatureController started with REAL event-driven control");
}

void TemperatureController::stop() {
    running_ = false;
    
    if (initialized_.load()) {
        // Immediately turn off heating and cooling (safe operation)
        setHeaterState(false);
        setCoolerState(false);
        control_state_ = TempControlState::IDLE;
    }
    
    LOG_INFO("TemperatureController stopped");
}

void TemperatureController::registerCallback(TemperatureCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    temp_callback_ = callback;
    LOG_INFO("Temperature callback registered for real-time events");
}

void TemperatureController::registerTimerCallback(
    std::function<void(std::function<void()>, std::chrono::milliseconds)> timer_cb) {
    timer_callback_ = timer_cb;
    LOG_INFO("Timer callback registered for event-driven temperature control");
}

// Core event-driven approach: triggered by external timer events
void TemperatureController::processTemperatureControlEvent() {
    if (!running_.load() || g_emergency_stop.load()) {
        return;
    }
    
    try {
        // Event 1: Read temperature sensor
        handleTemperatureReadEvent();
        
        // Event 2: Execute control calculation
        handleControlCalculationEvent();
        
    } catch (const std::exception& e) {
        LOG_ERROR("Temperature control event error: " + std::string(e.what()));
        emergencyTemperatureShutdown();
    }
}

void TemperatureController::handleTemperatureReadEvent() {
    try {
        double temp = readTemperatureSensor();
        current_temperature_ = temp;
        
        // Immediately trigger temperature callback 
        triggerTemperatureCallback(temp);
        
        LOG_DEBUG("Temperature read event: " + std::to_string(temp) + "°C");
        
    } catch (const std::exception& e) {
        LOG_ERROR("Temperature read event error: " + std::string(e.what()));
    }
}

void TemperatureController::handleControlCalculationEvent() {
    try {
        double temp = current_temperature_.load();
        double target = target_temperature_.load();
        double error = target - temp;
        
        // PID control calculation (event-driven)
        double control_output = calculatePIDEvent(error);
        
        // Execute control action immediately (no delay)
        std::lock_guard<std::mutex> lock(control_mutex_);
        
        if (std::abs(error) < 0.5) {
            // Temperature is within the target range.
            control_state_ = TempControlState::MAINTAINING;
            setHeaterState(false);
            setCoolerState(false);
            LOG_DEBUG("Temperature maintaining at target");
            
        } else if (control_output > 0) {
            // need heating
            control_state_ = TempControlState::HEATING;
            setHeaterState(true);
            setCoolerState(false);
            LOG_DEBUG("Heating activated (error: " + std::to_string(error) + "°C)");
            
        } else {
            // Refrigeration required
            control_state_ = TempControlState::COOLING;
            setHeaterState(false);
            setCoolerState(true);
            LOG_DEBUG("Cooling activated (error: " + std::to_string(error) + "°C)");
        }
        
    } catch (const std::exception& e) {
        LOG_ERROR("Control calculation event error: " + std::string(e.what()));
        emergencyTemperatureShutdown();
    }
}

double TemperatureController::readTemperatureSensor() {
    try {
        // Simulated temperature sensor reading (replace with actual SPI/I2C reading in real applications)
        // Here, it is simplified to ambient temperature + some variations.
        static double simulated_temp = 25.0;
        
        // Simulate the physical response of a temperature sensor
        double target = target_temperature_.load();
        auto state = control_state_.load();
        
        if (state == TempControlState::HEATING) {
            simulated_temp += 0.5;  // Temperature rise during heating
        } else if (state == TempControlState::COOLING) {
            simulated_temp -= 0.3;  // Temperature drop during cooling
        } else {
            // Environmental impact: Tending toward 25°C
            if (simulated_temp > 25.0) {
                simulated_temp -= 0.1;
            } else if (simulated_temp < 25.0) {
                simulated_temp += 0.1;
            }
        }
        
        // Add a small amount of noise
        simulated_temp += (rand() % 20 - 10) / 100.0;
        
        return simulated_temp;
        
    } catch (const std::exception& e) {
        LOG_ERROR("Temperature sensor reading error: " + std::string(e.what()));
        return current_temperature_.load(); // Return the previous value
    }
}

void TemperatureController::setHeaterState(bool state) {
    try {
        if (initialized_.load()) {
            heater_line_->set_value(state ? 1 : 0);
            LOG_DEBUG("Heater " + std::string(state ? "ON" : "OFF"));
        }
    } catch (const std::exception& e) {
        LOG_ERROR("Heater control error: " + std::string(e.what()));
    }
}

void TemperatureController::setCoolerState(bool state) {
    try {
        if (initialized_.load()) {
            cooler_line_->set_value(state ? 1 : 0);
            LOG_DEBUG("Cooler " + std::string(state ? "ON" : "OFF"));
        }
    } catch (const std::exception& e) {
        LOG_ERROR("Cooler control error: " + std::string(e.what()));
    }
}

double TemperatureController::calculatePIDEvent(double error) {
    double current_integral = integral_.load();
    double current_previous_error = previous_error_.load();
    
    // PID calculation
    current_integral += error;
    double derivative = error - current_previous_error;
    
    double kp = kp_.load();
    double ki = ki_.load();
    double kd = kd_.load();
    
    double output = kp * error + ki * current_integral + kd * derivative;
    
    // Restrict points to prevent points from reaching saturation
    if (current_integral > 100) current_integral = 100;
    if (current_integral < -100) current_integral = -100;
    
    // Atomic update of PID status
    integral_.store(current_integral);
    previous_error_.store(error);
    
    return output;
}

void TemperatureController::triggerTemperatureCallback(double temperature) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (temp_callback_) {
        temp_callback_(temperature);
    }
}

void TemperatureController::setTargetTemperature(double target) {
    target_temperature_ = target;
    
    // Reset PID integral term to avoid sudden jumps
    resetPIDIntegral();
    
    LOG_INFO("Target temperature set to " + std::to_string(target) + "°C");
}

void TemperatureController::emergencyTemperatureShutdown() {
    LOG_WARNING("Emergency temperature shutdown activated!");
    
    // Immediately turn off all heating/cooling equipment.
    setHeaterState(false);
    setCoolerState(false);
    control_state_ = TempControlState::IDLE;
}

// Status query method (atomic operation)
double TemperatureController::getCurrentTemperature() const {
    return current_temperature_.load();
}

double TemperatureController::getTargetTemperature() const {
    return target_temperature_.load();
}

TempControlState TemperatureController::getControlState() const {
    return control_state_.load();
}

bool TemperatureController::isInitialized() const {
    return initialized_.load();
}

void TemperatureController::setPIDParameters(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    
    // Reset PID status
    resetPIDIntegral();
    
    LOG_INFO("PID parameters updated: Kp=" + std::to_string(kp) + 
             " Ki=" + std::to_string(ki) + " Kd=" + std::to_string(kd));
}

void TemperatureController::resetPIDIntegral() {
    integral_ = 0.0;
    previous_error_ = 0.0;
}
