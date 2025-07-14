#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "Common.h"

class MotorController {
private:
    std::mutex motor_mutex_;
    std::atomic<bool> emergency_stop_;
    std::atomic<bool> running_;
    MotorCommand current_command_;
    
    // Motor parameters
    int max_speed_;
    int min_speed_;
    
    // PWM smoothing control
    int current_left_speed_;
    int current_right_speed_;
    const int speed_step_ = 5;
    
    void setMotorSpeed(int motor_pin, int dir_pin1, int dir_pin2, int speed);
    void smoothSpeedTransition();

public:
    MotorController();
    ~MotorController();
    
    bool initialize();
    void shutdown();
    
    // Core control methods
    void executeCommand(const MotorCommand& command);
    void emergencyStop();
    void resumeFromEmergency();
    
    // Basic exercise methods
    void moveForward(int speed = 100);
    void moveBackward(int speed = 100);
    void turnLeft(int speed = 80);
    void turnRight(int speed = 80);
    void stop();
    
    // Advanced control methods
    void setDifferentialSpeed(int left_speed, int right_speed);
    void smoothTurn(double turn_ratio); // -1.0 to 1.0
    
    // Status Check
    bool isEmergencyStopped() const { return emergency_stop_.load(); }
    bool isRunning() const { return running_.load(); }
    MotorCommand getCurrentCommand() const;
    
    // Testing methods
    bool selfTest();
};

#endif // MOTORCONTROLLER_H