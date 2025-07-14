#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <functional>
#include <vector>
#include <memory>
#include <cmath>
#include <string>
#include <numeric>
#include <algorithm>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <opencv2/opencv.hpp>

// GPIO Pin definition
namespace GPIOPins {
    // Motor control pins
    const int MOTOR_LEFT_PWM = 1;
    const int MOTOR_LEFT_DIR1 = 2;
    const int MOTOR_LEFT_DIR2 = 3;
    const int MOTOR_RIGHT_PWM = 4;
    const int MOTOR_RIGHT_DIR1 = 5;
    const int MOTOR_RIGHT_DIR2 = 6;
    
    // Ultrasonic sensor pins
    const int ULTRASONIC_TRIG = 7;
    const int ULTRASONIC_ECHO = 8;
    
    // Infrared sensor pins
    const int IR_LEFT = 9;
    const int IR_RIGHT = 10;
    const int IR_FRONT = 11;
    
    // Temperature control system pins
    const int TEMP_SENSOR = 12;
    const int HEATER_PWM = 13;
    const int COOLER_PWM = 14;
    const int TEMP_SENSOR_POWER = 15;
    
    // Added sensor pins
    const int LIGHT_SENSOR = 16;
    const int PRESSURE_SENSOR = 17;
    
    // LED pin
    const int LED_RED = 18;
    const int LED_GREEN = 19;
    const int LED_BLUE = 20;
    const int LED_STATUS = 21;
    
    // Buzzer pins
    const int BUZZER = 22;
    
    // button pins
    const int BUTTON_EMERGENCY = 23;
    const int BUTTON_START = 24;
    const int BUTTON_STOP = 25;
}

// System constants
namespace Constants {
    const double MAX_SAFE_DISTANCE = 30.0; // cm
    const double MIN_SAFE_DISTANCE = 10.0; // cm
    const double TARGET_TEMPERATURE = 25.0; // °C
    const double TEMP_TOLERANCE = 1.0; // °C
    const int CAMERA_WIDTH = 640;
    const int CAMERA_HEIGHT = 480;
    const int MAX_MOTOR_SPEED = 255;
    const int MIN_MOTOR_SPEED = 50;
    
    // Real-time performance requirements
    const int MAX_VISION_LATENCY_US = 50000;      // 50ms
    const int MAX_SENSOR_LATENCY_US = 10000;      // 10ms
    const int MAX_CONTROL_LATENCY_US = 5000;      // 5ms
    const int MAX_SAFETY_LATENCY_US = 1000;       // 1ms
}

// Latency Monitoring - Used for real-time performance evaluation
class LatencyMonitor {
private:
    std::chrono::steady_clock::time_point start_time_;
    std::vector<double> latencies_;
    std::string operation_name_;
    std::mutex latency_mutex_;
    int max_allowed_latency_us_;
    
public:
    LatencyMonitor(const std::string& operation = "Unknown", int max_latency_us = 50000) 
        : operation_name_(operation), max_allowed_latency_us_(max_latency_us) {
    }
    
    void startMeasurement(const std::string& operation = "") {
        if (!operation.empty()) {
            operation_name_ = operation;
        }
        start_time_ = std::chrono::steady_clock::now();
    }
    
    double endMeasurement() {
        auto end_time = std::chrono::steady_clock::now();
        auto latency = std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - start_time_).count();
        
        {
            std::lock_guard<std::mutex> lock(latency_mutex_);
            latencies_.push_back(latency);
        }
        
        std::cout << operation_name_ << " latency: " << latency << "μs" << std::endl;
        
        // Check if the real-time requirements are exceeded
        if (latency > max_allowed_latency_us_) {
            std::cerr << "⚠️  WARNING: " << operation_name_ 
                      << " latency " << latency << "μs exceeded real-time threshold " 
                      << max_allowed_latency_us_ << "μs!" << std::endl;
        }
        
        return latency;
    }
    
    void printStatistics() {
        std::lock_guard<std::mutex> lock(latency_mutex_);
        
        if (latencies_.empty()) {
            std::cout << "No latency data for " << operation_name_ << std::endl;
            return;
        }
        
        double avg = std::accumulate(latencies_.begin(), latencies_.end(), 0.0) / latencies_.size();
        double max_lat = *std::max_element(latencies_.begin(), latencies_.end());
        double min_lat = *std::min_element(latencies_.begin(), latencies_.end());
        
        // Count the number of times the threshold is exceeded
        int violations = std::count_if(latencies_.begin(), latencies_.end(),
                                     [this](double lat) { return lat > max_allowed_latency_us_; });
        
        std::cout << "\n=== " << operation_name_ << " Performance Statistics ===" << std::endl;
        std::cout << "Average: " << avg << "μs" << std::endl;
        std::cout << "Maximum: " << max_lat << "μs" << std::endl;
        std::cout << "Minimum: " << min_lat << "μs" << std::endl;
        std::cout << "Total samples: " << latencies_.size() << std::endl;
        std::cout << "Threshold violations: " << violations << " (" 
                  << (100.0 * violations / latencies_.size()) << "%)" << std::endl;
        std::cout << "Real-time compliance: " 
                  << (violations == 0 ? "✅ PASS" : "❌ FAIL") << std::endl;
        std::cout << "=================================================" << std::endl;
    }
    
    void clearStatistics() {
        std::lock_guard<std::mutex> lock(latency_mutex_);
        latencies_.clear();
    }
    
    size_t getSampleCount() const {
        std::lock_guard<std::mutex> lock(latency_mutex_);
        return latencies_.size();
    }
    
    double getAverageLatency() const {
        std::lock_guard<std::mutex> lock(latency_mutex_);
        if (latencies_.empty()) return 0.0;
        return std::accumulate(latencies_.begin(), latencies_.end(), 0.0) / latencies_.size();
    }
    
    bool isRealTimeCompliant() const {
        std::lock_guard<std::mutex> lock(latency_mutex_);
        return std::all_of(latencies_.begin(), latencies_.end(),
                          [this](double lat) { return lat <= max_allowed_latency_us_; });
    }
};


// 
class ISensor {
public:
    virtual ~ISensor() = default;
    virtual bool initialize() = 0;
    virtual void shutdown() = 0;
    virtual bool isRunning() const = 0;
    virtual bool selfTest() = 0;
    
    // Obtain sensor type information
    virtual std::string getSensorType() const = 0;
    virtual std::string getSensorStatus() const = 0;
};

// Proximity sensor interface
class IDistanceSensor : public ISensor {
public:
    virtual double getDistance() = 0;
    virtual bool isObstacleDetected(double threshold) = 0;
};

// Environmental sensor interface    
class IEnvironmentSensor : public ISensor {
public:
    virtual double getCurrentValue() = 0;
    virtual bool isWithinNormalRange() = 0;
};

// Vision sensor interface
class IVisionSensor : public ISensor {
public:
    virtual VisionResult processFrame() = 0;
    virtual VisionResult getLatestResult() = 0;
    virtual bool isCameraReady() const = 0;
};

// Controller interface
class IController {
public:
    virtual ~IController() = default;
    virtual bool initialize() = 0;
    virtual void shutdown() = 0;
    virtual bool isRunning() const = 0;
    virtual bool selfTest() = 0;
    virtual std::string getControllerType() const = 0;
};

// Motor controller interface
class IMotorController : public IController {
public:
    virtual void executeCommand(const MotorCommand& command) = 0;
    virtual void emergencyStop() = 0;
    virtual void resumeFromEmergency() = 0;
    virtual bool isEmergencyStopped() const = 0;
    virtual MotorCommand getCurrentCommand() const = 0;
};

// Temperature controller interface
class ITemperatureController : public IController {
public:
    virtual void setTargetTemperature(double target) = 0;
    virtual double getCurrentTemperature() const = 0;
    virtual double getTargetTemperature() const = 0;
    virtual bool isTemperatureStable(double tolerance = 1.0) = 0;
};

// Safety controller interface
class ISafetyController : public IController {
public:
    virtual bool isSafeToMove() = 0;
    virtual bool isSafeToOperate() = 0;
    virtual void triggerEmergencyStop(const std::string& reason) = 0;
    virtual void clearEmergencyStop() = 0;
    virtual bool isEmergencyActive() const = 0;
};
// 

// Global Latency Monitor instance
namespace PerformanceMonitors {
    extern LatencyMonitor vision_monitor;
    extern LatencyMonitor sensor_monitor;
    extern LatencyMonitor control_monitor;
    extern LatencyMonitor safety_monitor;
}

// Motion control structures
struct MotorCommand {
    int left_speed;   // -255 to 255
    int right_speed;  // -255 to 255
    bool emergency_stop;
    std::chrono::steady_clock::time_point timestamp;
    
    MotorCommand() : left_speed(0), right_speed(0), emergency_stop(false),
                     timestamp(std::chrono::steady_clock::now()) {}
    MotorCommand(int l, int r) : left_speed(l), right_speed(r), emergency_stop(false),
                                timestamp(std::chrono::steady_clock::now()) {}
};

// Sensor data structures
struct SensorData {
    double ultrasonic_distance;
    bool ir_left_obstacle;
    bool ir_right_obstacle;
    bool ir_front_obstacle;
    double temperature;
    int light_level;
    double pressure_weight;
    std::chrono::steady_clock::time_point timestamp;
    
    SensorData() : ultrasonic_distance(999.0), ir_left_obstacle(false), 
                   ir_right_obstacle(false), ir_front_obstacle(false), 
                   temperature(20.0), light_level(500), pressure_weight(0.0),
                   timestamp(std::chrono::steady_clock::now()) {}
};

// Visual processing results
struct VisionResult {
    bool line_detected;
    double line_offset;  // -1.0 (left) to 1.0 (right)
    double line_angle;   // -90 to 90 degrees
    cv::Point2f line_center;
    double confidence;   // 0.0 to 1.0
    int processing_time_us;  // 处理时间（微秒）
    std::chrono::steady_clock::time_point timestamp;
    
    VisionResult() : line_detected(false), line_offset(0.0), line_angle(0.0), 
                     line_center(0, 0), confidence(0.0), processing_time_us(0),
                     timestamp(std::chrono::steady_clock::now()) {}
};

// Performance benchmark structure
struct PerformanceBenchmark {
    std::string component_name;
    double average_latency_us;
    double max_latency_us;
    double min_latency_us;
    size_t sample_count;
    double real_time_compliance_rate;
    std::chrono::steady_clock::time_point benchmark_time;
    
    PerformanceBenchmark(const std::string& name = "Unknown") 
        : component_name(name), average_latency_us(0.0), max_latency_us(0.0),
          min_latency_us(0.0), sample_count(0), real_time_compliance_rate(0.0),
          benchmark_time(std::chrono::steady_clock::now()) {}
};

// System health status
struct SystemHealth {
    bool motor_healthy;
    bool vision_healthy;
    bool sensors_healthy;
    bool temperature_healthy;
    bool safety_healthy;
    double system_uptime_seconds;
    std::chrono::steady_clock::time_point last_check;
    
    SystemHealth() : motor_healthy(false), vision_healthy(false), 
                     sensors_healthy(false), temperature_healthy(false),
                     safety_healthy(false), system_uptime_seconds(0.0),
                     last_check(std::chrono::steady_clock::now()) {}
    
    bool isSystemHealthy() const {
        return motor_healthy && vision_healthy && sensors_healthy && 
               temperature_healthy && safety_healthy;
    }
};

// Utility functions
namespace Utils {
    //  Timestamps to strings
    inline std::string timestampToString(const std::chrono::steady_clock::time_point& tp) {
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            tp.time_since_epoch()).count();
        return std::to_string(ms) + "ms";
    }
    
    // Limit the numeric range
    template<typename T>
    inline T clamp(T value, T min_val, T max_val) {
        return std::max(min_val, std::min(max_val, value));
    }
    
    // Angles are normalized to[-180, 180]
    inline double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
}

#endif // COMMON_H
