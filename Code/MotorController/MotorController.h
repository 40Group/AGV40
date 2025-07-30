#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "../common/Common.h"
#include <gpiod.hpp>
#include <atomic>
#include <mutex>
#include <memory>

class MotorController {
private:
    // GPIO configuration - Using modern gpiod
    std::unique_ptr<gpiod::chip> chip_;
    std::unique_ptr<gpiod::line> left_pin1_;
    std::unique_ptr<gpiod::line> left_pin2_;
    std::unique_ptr<gpiod::line> right_pin1_;
    std::unique_ptr<gpiod::line> right_pin2_;
    
    // pin number
    int left_pin1_num_, left_pin2_num_;
    int right_pin1_num_, right_pin2_num_;
    
    // status management
    std::atomic<MotionState> current_state_;
    std::mutex control_mutex_;
    std::atomic<bool> initialized_;
    
    // Internal hardware control methods
    void setGPIO(gpiod::line& line, bool state);
    void stopMotorsImmediate();
    void setupGPIOLines();
    
public:
    MotorController(int left_pin1 = 17, int left_pin2 = 18, 
                   int right_pin1 = 22, int right_pin2 = 23);
    ~MotorController();
    
    // Initialization and cleanup
    bool initialize();
    void shutdown();
    
    // Core motion control interface (pure callback-driven, no delay)
    void executeMotion(MotionState state);
    void emergencyStop();
    
    // Status retrieval (thread-safe)
    MotionState getCurrentState() const;
    bool isInitialized() const;
    
    void setMotionState(MotionState state);  // Set status immediately
    void executeEmergencyBrake();           // emergency braking
};

#endif // MOTORCONTROLLER_H