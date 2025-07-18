#ifndef INFRAREDSENSOR_H
#define INFRAREDSENSOR_H

#include "Common.h"

class InfraredSensor {
private:
    int left_pin_;
    int right_pin_;
    int front_pin_;
    std::atomic<bool> running_;
    
    // Sensor status
    std::atomic<bool> left_obstacle_;
    std::atomic<bool> right_obstacle_;
    std::atomic<bool> front_obstacle_;
    
    // Timer-related
    std::thread timer_thread_;
    std::chrono::milliseconds polling_interval_;
    
    // Timer polling loop
    void timerPollingLoop();
    void readAllSensors();

public:
    InfraredSensor(int left_pin = GPIOPins::IR_LEFT, 
                   int right_pin = GPIOPins::IR_RIGHT,
                   int front_pin = GPIOPins::IR_FRONT);
    ~InfraredSensor();
    
    bool initialize();
    void shutdown();
    
    // Main detection methods
    bool isLeftObstacle() const { return left_obstacle_.load(); }
    bool isRightObstacle() const { return right_obstacle_.load(); }
    bool isFrontObstacle() const { return front_obstacle_.load(); }
    bool isAnyObstacle() const;
    
    // Batch status retrieval
    struct ObstacleState {
        bool left;
        bool right;
        bool front;
        std::chrono::steady_clock::time_point timestamp;
    };
    ObstacleState getAllStates();
    
    // Parameter settings
    void setPollingInterval(std::chrono::milliseconds interval);
    
    // Status Inquiry
    bool isRunning() const { return running_.load(); }
    
    // Testing method
    bool selfTest();
};

#endif // INFRAREDSENSOR_H
