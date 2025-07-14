#ifndef SAFETYCONTROLLER_H
#define SAFETYCONTROLLER_H

#include "Common.h"
#include "MotorController.h"
#include "UltrasonicSensor.h"
#include "InfraredSensor.h"
#include "TemperatureController.h"

class SafetyController {
public:
    enum class Priority {
        EMERGENCY_STOP = 0,     // highest priority
        OBSTACLE_AVOIDANCE = 1,
        TEMPERATURE_CONTROL = 2,
        NORMAL_OPERATION = 3    // lowest priority
    };
    
    enum class SafetyState {
        NORMAL,
        OBSTACLE_DETECTED,
        TEMPERATURE_EMERGENCY,
        SYSTEM_ERROR,
        EMERGENCY_STOP
    };

private:
    // System Component References
    MotorController* motor_controller_;
    UltrasonicSensor* ultrasonic_sensor_;
    InfraredSensor* ir_sensor_;
    TemperatureController* temp_controller_;
    
    std::atomic<bool> running_;
    std::atomic<SafetyState> current_state_;
    std::mutex safety_mutex_;
    
    // Security Monitoring Thread
    std::thread safety_monitor_thread_;
    
    // Emergency stop related
    std::atomic<bool> emergency_stop_active_;
    std::chrono::steady_clock::time_point emergency_start_time_;
    
    // safety parameter
    double safe_distance_threshold_;
    double critical_distance_threshold_;
    double max_safe_temperature_;
    double min_safe_temperature_;
    
    // Safety Check Methods
    bool checkObstacleSafety();
    bool checkTemperatureSafety();
    bool checkSystemHealth();
    
    // Security response methods
    void handleObstacleDetection();
    void handleTemperatureEmergency();
    void handleSystemError();
    void handleEmergencyStop();
    
    // Security Monitoring Main Cycle
    void safetyMonitorLoop();
    
    // Priority management
    bool canExecuteWithPriority(Priority required_priority);
    void blockLowerPriority(Priority current_priority);
    
    // Logs and reports
    void logSafetyEvent(const std::string& event);
    std::vector<std::string> safety_log_;
    std::mutex log_mutex_;

public:
    SafetyController();
    ~SafetyController();
    
    bool initialize(MotorController* motor, UltrasonicSensor* ultrasonic,
                   InfraredSensor* ir, TemperatureController* temp);
    void shutdown();
    
    // Main safety check methods
    bool isSafeToMove();
    bool isSafeToOperate();
    SafetyState getCurrentState() const { return current_state_.load(); }
    
    // emergency control
    void triggerEmergencyStop(const std::string& reason);
    void clearEmergencyStop();
    bool isEmergencyActive() const { return emergency_stop_active_.load(); }
    
    // Campaign Safety Check
    bool checkMotorCommand(const MotorCommand& command);
    MotorCommand applySafetyConstraints(const MotorCommand& command);
    
    // parameter setting
    void setSafetyThresholds(double safe_distance, double critical_distance,
                           double max_temp, double min_temp);
    
    // status inquiry
    bool isRunning() const { return running_.load(); }
    std::string getStateDescription() const;
    std::chrono::milliseconds getEmergencyDuration() const;
    
    // Callback registration
    using SafetyCallback = std::function<void(SafetyState state, const std::string& reason)>;
    void setSafetyCallback(SafetyCallback callback);
    
    // Statistics and logs
    std::vector<std::string> getSafetyLog() const;
    void clearSafetyLog();
    
    // testing method
    bool selfTest();
    void simulateEmergency(const std::string& type);
};

#endif // SAFETYCONTROLLER_H