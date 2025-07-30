#ifndef INFRAREDSENSOR_H
#define INFRAREDSENSOR_H

#include "../common/Common.h"
#include <gpiod.hpp>
#include <atomic>
#include <thread>
#include <functional>
#include <mutex>
#include <chrono>

class InfraredSensor {
private:
    // GPIO Configuration
    std::unique_ptr<gpiod::chip> chip_;
    std::unique_ptr<gpiod::line> left_line_;
    std::unique_ptr<gpiod::line> right_line_;
    
    // Threads and States
    std::thread event_thread_;
    std::atomic<bool> running_;
    std::atomic<bool> left_detected_;
    std::atomic<bool> right_detected_;
    
    // Callback system
    std::function<void(bool, bool)> boundary_callback_;
    std::mutex callback_mutex_;
    
    // GPIO pins
    int left_pin_;
    int right_pin_;
    
    // True event-driven approach
    void realEventLoop();
    void setupGPIOEvents();
    void handleGPIOEvent(gpiod::line& line, const gpiod::line_event& event);
    
public:
    InfraredSensor(int left_pin = 19, int right_pin = 26);
    ~InfraredSensor();
    
    // Initialization and control
    bool initialize();
    void start();
    void stop();
    
    // Callback Registration
    void registerCallback(std::function<void(bool, bool)> callback);
    
    // Status query (atomic operation, thread-safe)
    bool isLeftDetected() const;
    bool isRightDetected() const;
    bool isBoundaryDetected() const;
    
    // Test related methods (maintain compatibility)
    bool isRunning() const;
    bool selfTest();
    void setPollingInterval(std::chrono::milliseconds interval); // 现在只是占位符
    void shutdown();
    
    // Extended functionality (keep test compatible)
    struct SensorStates {
        bool left;
        bool right;
        bool front;  // For compatibility testing, front = left || right
    };
    SensorStates getAllStates() const;
    bool isAnyObstacle() const;
};

#endif // INFRAREDSENSOR_H
