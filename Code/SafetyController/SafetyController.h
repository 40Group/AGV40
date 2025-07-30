#ifndef SAFETYCONTROLLER_H
#define SAFETYCONTROLLER_H

#include "../common/Common.h"

class SafetyController {
private:
    std::thread safety_thread_;
    std::atomic<bool> running_;
    std::atomic<bool> emergency_active_;
    
    SafetyCallback safety_callback_;
    std::mutex callback_mutex_;
    
    // Safety monitoring parameters
    std::atomic<double> max_temperature_;
    std::atomic<double> min_distance_;
    std::atomic<int> max_operation_time_minutes_;
    
    // System Status Monitoring
    std::chrono::steady_clock::time_point start_time_;
    std::atomic<bool> vision_healthy_;
    std::atomic<bool> ultrasonic_healthy_;
    std::atomic<bool> temperature_healthy_;
    
    // internal methods
    void safetyMonitoringLoop();
    void checkSystemHealth();
    void checkTemperatureLimits(double current_temp);
    void checkObstacleSafety(double distance);
    void checkOperationTime();
    void triggerEmergencyStop(const std::string& reason);
    
    // Health check-up time exceeded
    std::chrono::steady_clock::time_point last_vision_update_;
    std::chrono::steady_clock::time_point last_ultrasonic_update_;
    std::chrono::steady_clock::time_point last_temperature_update_;
    
public:
    SafetyController();
    ~SafetyController();
    
    // Initialisation and control
    bool initialize();
    void start();
    void stop();
    
    // Callback registration
    void registerCallback(SafetyCallback callback);
    
    // Safety parameter settings
    void setTemperatureLimits(double max_temp);
    void setMinimumDistance(double min_dist);
    void setMaxOperationTime(int minutes);
    
    // System Health Status Update (called by other modules)
    void updateVisionHealth();
    void updateUltrasonicHealth();
    void updateTemperatureHealth();
    
    // Manual emergency stop
    void manualEmergencyStop();
    void resetEmergency();
    
    // status inquiry
    bool isEmergencyActive() const;
    bool isSystemHealthy() const;
    int getOperationTimeMinutes() const;
};

#endif // SAFETYCONTROLLER_H
