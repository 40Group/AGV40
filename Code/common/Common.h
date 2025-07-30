#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <memory>
#include <functional>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <vector>
#include <string>

enum class SystemState {
    INITIALIZING,
    RUNNING,
    EMERGENCY_STOP,
    SHUTDOWN
};

// Movement states are enumerated
enum class MotionState {
    STOP,
    FORWARD,
    BACKWARD,
    TURN_LEFT,
    TURN_RIGHT
};

// Temperature control state
enum class TempControlState {
    IDLE,
    HEATING,
    COOLING,
    MAINTAINING
};

// Sensor data structures
struct SensorData {
    double distance_cm;
    bool obstacle_detected;
    bool line_detected;
    double temperature_celsius;
    std::chrono::steady_clock::time_point timestamp;
    
    // constructor to ensure that the timestamp is initialized
    SensorData() : distance_cm(0.0), obstacle_detected(false), 
                   line_detected(false), temperature_celsius(25.0),
                   timestamp(std::chrono::steady_clock::now()) {}
};


using VisionCallback = std::function<void(bool lineDetected, double deviation)>;
using ObstacleCallback = std::function<void(bool obstacleDetected, double distance)>;
using TemperatureCallback = std::function<void(double temperature)>;
using SafetyCallback = std::function<void(const std::string& emergency)>;
using TimerCallback = std::function<void()>;

// IR sensor callback
using BoundaryCallback = std::function<void(bool leftDetected, bool rightDetected)>;

// Global system state
extern std::atomic<SystemState> g_system_state;
extern std::atomic<bool> g_emergency_stop;
extern std::atomic<bool> g_running;  // 添加全局运行标志

// Log Hong
#define LOG_INFO(msg) std::cout << "[INFO] " << msg << std::endl
#define LOG_ERROR(msg) std::cerr << "[ERROR] " << msg << std::endl
#define LOG_WARNING(msg) std::cout << "[WARNING] " << msg << std::endl
#define LOG_DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl

// Event-driven architecture constants
namespace EventDriven {
    // GPIO event timeout setting
    constexpr auto GPIO_EVENT_TIMEOUT = std::chrono::milliseconds(1000);
    
    // Sensor health check interval
    constexpr auto HEALTH_CHECK_INTERVAL = std::chrono::seconds(5);
    
    // Emergency stop response time requirements
    constexpr auto EMERGENCY_RESPONSE_TIME = std::chrono::milliseconds(100);
}

#endif // COMMON_H
