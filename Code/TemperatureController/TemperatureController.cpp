#include "../include/TemperatureController.h"
#include <fstream>
#include <filesystem>
#include <algorithm>
#include <sstream>

TemperatureController::TemperatureController(int sensor_pin, int heater_pin, int cooler_pin, int sensor_power_pin)
    : sensor_pin_(sensor_pin), heater_pin_(heater_pin), cooler_pin_(cooler_pin), 
      sensor_power_pin_(sensor_power_pin), running_(false), current_temperature_(20.0),
      target_temperature_(Constants::TARGET_TEMPERATURE), kp_(2.0), ki_(0.1), kd_(0.5),
      integral_error_(0.0), last_error_(0.0), max_heating_power_(0.8), max_cooling_power_(0.8),
      max_safe_temperature_(50.0), min_safe_temperature_(5.0), 
      sensor_fd_(-1), epoll_fd_(-1) {
}

TemperatureController::~TemperatureController() {
    shutdown();
}

bool TemperatureController::initialize() {
    if (wiringPiSetup() == -1) {
        std::cerr << "Failed to initialize WiringPi for temperature controller" << std::endl;
        return false;
    }
    
    // Set GPIO pins
    pinMode(heater_pin_, PWM_OUTPUT);
    pinMode(cooler_pin_, PWM_OUTPUT);
    pinMode(sensor_power_pin_, OUTPUT);
    
    // Initialize PWM
    pwmSetMode(PWM_MODE_MS);
    pwmSetClock(384);
    pwmSetRange(1024);
    
    // Turn on the sensor power supply.
    digitalWrite(sensor_power_pin_, HIGH);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Find and open the DS18B20 sensor file - event-driven method
    if (!openTemperatureSensorFile()) {
        std::cerr << "Failed to open DS18B20 temperature sensor file" << std::endl;
        return false;
    }
    
    // Set epoll for event-driven reading
    if (!setupEventDrivenReading()) {
        std::cerr << "Failed to setup event-driven temperature reading" << std::endl;
        return false;
    }
    
    // Initialize PWM output to 0
    pwmWrite(heater_pin_, 0);
    pwmWrite(cooler_pin_, 0);
    
    // Reset PID status
    resetPIDState();
    
    // Start event-driven temperature monitoring thread
    running_ = true;
    temperature_thread_ = std::thread(&TemperatureController::eventDrivenTemperatureLoop, this);
    
    std::cout << "TemperatureController initialized successfully (Event-driven)" << std::endl;
    return true;
}

void TemperatureController::shutdown() {
    running_ = false;
    
    // Stop heating and cooling
    if (heater_pin_ >= 0) pwmWrite(heater_pin_, 0);
    if (cooler_pin_ >= 0) pwmWrite(cooler_pin_, 0);
    
    // Turn off the sensor power supply.
    if (sensor_power_pin_ >= 0) digitalWrite(sensor_power_pin_, LOW);
    
    // Close file descriptor
    if (sensor_fd_ != -1) {
        close(sensor_fd_);
        sensor_fd_ = -1;
    }
    
    if (epoll_fd_ != -1) {
        close(epoll_fd_);
        epoll_fd_ = -1;
    }
    
    // Wait for the temperature monitoring thread to finish
    if (temperature_thread_.joinable()) {
        temperature_thread_.join();
    }
    
    std::cout << "TemperatureController shutdown (Event-driven)" << std::endl;
}

bool TemperatureController::openTemperatureSensorFile() {
    // DS18B20 devices are typically found in the /sys/bus/w1/devices/ directory.
    std::string devices_path = "/sys/bus/w1/devices/";
    
    try {
        for (const auto& entry : std::filesystem::directory_iterator(devices_path)) {
            std::string device_name = entry.path().filename().string();
            
            // DS18B20 device names usually start with 28-
            if (device_name.substr(0, 3) == "28-") {
                sensor_device_path_ = entry.path().string() + "/w1_slave";
                
                // Open a file in a non-blocking manner
                sensor_fd_ = open(sensor_device_path_.c_str(), O_RDONLY | O_NONBLOCK);
                if (sensor_fd_ == -1) {
                    std::cerr << "Failed to open sensor file: " << sensor_device_path_ << std::endl;
                    return false;
                }
                
                std::cout << "Found DS18B20 sensor: " << device_name << std::endl;
                return true;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error searching for DS18B20 sensor: " << e.what() << std::endl;
    }
    
    return false;
}

bool TemperatureController::setupEventDrivenReading() {
    // Create an epoll instance
    epoll_fd_ = epoll_create1(0);
    if (epoll_fd_ == -1) {
        std::cerr << "Failed to create epoll instance: " << strerror(errno) << std::endl;
        return false;
    }
    
    // Add sensor file descriptors to epoll
    struct epoll_event ev;
    ev.events = EPOLLIN | EPOLLPRI;  // Monitoring readable and urgent data
    ev.data.fd = sensor_fd_;
    
    if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, sensor_fd_, &ev) == -1) {
        std::cerr << "Failed to add sensor fd to epoll: " << strerror(errno) << std::endl;
        return false;
    }
    
    return true;
}

void TemperatureController::eventDrivenTemperatureLoop() {
    const int MAX_EVENTS = 1;
    struct epoll_event events[MAX_EVENTS];
    
    while (running_.load()) {
        // Wait for sensor data event, timeout 1 second
        int nfds = epoll_wait(epoll_fd_, events, MAX_EVENTS, 1000);
        
        if (nfds == -1) {
            if (errno != EINTR) {
                std::cerr << "epoll_wait failed: " << strerror(errno) << std::endl;
                break;
            }
            continue;
        }
        
        if (nfds > 0) {
            // There is a sensor data event.
            for (int i = 0; i < nfds; i++) {
                if (events[i].data.fd == sensor_fd_) {
                    // Read temperature data
                    double temp = readDS18B20TemperatureEventDriven();
                    
                    if (temp > -999.0) {
                        std::lock_guard<std::mutex> lock(temp_mutex_);
                        current_temperature_.store(temp);
                        
                        // security check
                        if (!isSafeTemperature(temp)) {
                            std::cerr << "Temperature out of safe range: " << temp << "°C" << std::endl;
                            turnOffControl();
                            continue;
                        }
                        
                        // PID control
                        double target = target_temperature_.load();
                        double pid_output = calculatePIDOutput(temp, target);
                        applyTemperatureControl(pid_output);
                    }
                }
            }
        } else {
            // If there are no events, perform a temperature check (as a backup).
            double temp = readDS18B20TemperatureEventDriven();
            if (temp > -999.0) {
                std::lock_guard<std::mutex> lock(temp_mutex_);
                current_temperature_.store(temp);
                
                // Simple temperature control
                double target = target_temperature_.load();
                double pid_output = calculatePIDOutput(temp, target);
                applyTemperatureControl(pid_output);
            }
        }
    }
}

double TemperatureController::readDS18B20TemperatureEventDriven() {
    if (sensor_fd_ == -1) return -999.0;
    
    // Reset the file pointer to the beginning
    lseek(sensor_fd_, 0, SEEK_SET);
    
    char buffer[256];
    ssize_t bytes_read = read(sensor_fd_, buffer, sizeof(buffer) - 1);
    
    if (bytes_read <= 0) {
        return -999.0;
    }
    
    buffer[bytes_read] = '\0';
    std::string content(buffer);
    
    // Analyzing DS18B20 Output
    std::istringstream iss(content);
    std::string line1, line2;
    std::getline(iss, line1);
    std::getline(iss, line2);
    
    // Check CRC checksum
    if (line1.find("YES") == std::string::npos) {
        std::cerr << "Temperature sensor CRC check failed" << std::endl;
        return -999.0;
    }
    
    // Analyzing temperature values
    size_t pos = line2.find("t=");
    if (pos != std::string::npos) {
        std::string temp_str = line2.substr(pos + 2);
        int temp_raw = std::stoi(temp_str);
        double temperature = temp_raw / 1000.0;
        
        // Check whether the temperature is within a reasonable range.
        if (temperature >= -55.0 && temperature <= 125.0) {
            return temperature;
        }
    }
    
    std::cerr << "Invalid temperature reading" << std::endl;
    return -999.0;
}

double TemperatureController::calculatePIDOutput(double current_temp, double target_temp) {
    auto now = std::chrono::steady_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_).count() / 1000.0;
    
    if (dt <= 0) dt = 1.0; // Prevent zero removal
    
    // computational error
    double error = target_temp - current_temp;
    
    // Integral term
    integral_error_ += error * dt;
    
    // Integration saturation protection
    double max_integral = 100.0;
    integral_error_ = std::max(-max_integral, std::min(max_integral, integral_error_));
    
    // differential term
    double derivative = (error - last_error_) / dt;
    
    // PID output
    double output = kp_ * error + ki_ * integral_error_ + kd_ * derivative;
    
    // Update status
    last_error_ = error;
    last_update_ = now;
    
    // Limit output range [-1.0, 1.0]
    return std::max(-1.0, std::min(1.0, output));
}

void TemperatureController::applyTemperatureControl(double pid_output) {
    if (pid_output > 0) {
        // Needs heating
        double heating_power = std::min(pid_output * max_heating_power_, max_heating_power_);
        setHeatingPower(heating_power);
        setCoolingPower(0.0);
    } else if (pid_output < 0) {
        // Refrigeration required
        double cooling_power = std::min(std::abs(pid_output) * max_cooling_power_, max_cooling_power_);
        setCoolingPower(cooling_power);
        setHeatingPower(0.0);
    } else {
        // Turn off heating and cooling
        setHeatingPower(0.0);
        setCoolingPower(0.0);
    }
}

void TemperatureController::setTargetTemperature(double target) {
    target_temperature_.store(target);
    resetPIDState(); // Reset PID status to avoid integral saturation
    std::cout << "Target temperature set to: " << target << "°C" << std::endl;
}

void TemperatureController::setPIDParameters(double kp, double ki, double kd) {
    std::lock_guard<std::mutex> lock(temp_mutex_);
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    resetPIDState();
    std::cout << "PID parameters updated: Kp=" << kp << ", Ki=" << ki << ", Kd=" << kd << std::endl;
}

void TemperatureController::resetPIDState() {
    integral_error_ = 0.0;
    last_error_ = 0.0;
    last_update_ = std::chrono::steady_clock::now();
}

void TemperatureController::setHeatingPower(double power) {
    power = std::max(0.0, std::min(1.0, power));
    int pwm_value = static_cast<int>(power * 1024);
    pwmWrite(heater_pin_, pwm_value);
}

void TemperatureController::setCoolingPower(double power) {
    power = std::max(0.0, std::min(1.0, power));
    int pwm_value = static_cast<int>(power * 1024);
    pwmWrite(cooler_pin_, pwm_value);
}

void TemperatureController::turnOffControl() {
    setHeatingPower(0.0);
    setCoolingPower(0.0);
    resetPIDState();
}

bool TemperatureController::isSafeTemperature(double temp) {
    return temp >= min_safe_temperature_ && temp <= max_safe_temperature_;
}

bool TemperatureController::isTemperatureStable(double tolerance) {
    double current = current_temperature_.load();
    double target = target_temperature_.load();
    return std::abs(current - target) <= tolerance;
}

bool TemperatureController::isWithinSafeRange() const {
    double temp = current_temperature_.load();
    return temp >= min_safe_temperature_ && temp <= max_safe_temperature_;
}

TemperatureController::TemperatureStats TemperatureController::getTemperatureStats() {
    // Simplified implementation; historical data can be maintained in actual projects.
    TemperatureStats stats;
    double current = current_temperature_.load();
    stats.min_temp = current;
    stats.max_temp = current;
    stats.avg_temp = current;
    stats.readings_count = 1;
    stats.last_update = std::chrono::steady_clock::now();
    return stats;
}

void TemperatureController::setTemperatureCallback(TemperatureCallback callback) {
    // Callback functionality can be added when needed.
}

bool TemperatureController::selfTest() {
    std::cout << "Starting temperature controller self-test (Event-driven)..." << std::endl;
    
    if (!running_.load()) {
        std::cout << "Temperature controller not initialized" << std::endl;
        return false;
    }
    
    // Test temperature reading
    double temp = readDS18B20TemperatureEventDriven();
    std::cout << "Current temperature: " << temp << "°C" << std::endl;
    
    if (temp <= -999.0) {
        std::cout << "Failed to read temperature" << std::endl;
        return false;
    }
    
    // Test heater
    std::cout << "Testing heater (10% power for 2 seconds)..." << std::endl;
    setHeatingPower(0.1);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    setHeatingPower(0.0);
    
    // Testing the cooler
    std::cout << "Testing cooler (10% power for 2 seconds)..." << std::endl;
    setCoolingPower(0.1);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    setCoolingPower(0.0);
    
    // Check temperature changes
    std::this_thread::sleep_for(std::chrono::seconds(2));
    double new_temp = readDS18B20TemperatureEventDriven();
    std::cout << "Temperature after test: " << new_temp << "°C" << std::endl;
    
    std::cout << "Temperature controller self-test completed (Event-driven)" << std::endl;
    return true;
}

void TemperatureController::calibrateSensor() {
    std::cout << "Temperature sensor calibration not implemented" << std::endl;
}