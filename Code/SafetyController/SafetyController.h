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
    
    // 安全监控参数
    std::atomic<double> max_temperature_;
    std::atomic<double> min_distance_;
    std::atomic<int> max_operation_time_minutes_;
    
    // 系统状态监控
    std::chrono::steady_clock::time_point start_time_;
    std::atomic<bool> vision_healthy_;
    std::atomic<bool> ultrasonic_healthy_;
    std::atomic<bool> temperature_healthy_;
    
    // 内部方法
    void safetyMonitoringLoop();
    void checkSystemHealth();
    void checkTemperatureLimits(double current_temp);
    void checkObstacleSafety(double distance);
    void checkOperationTime();
    void triggerEmergencyStop(const std::string& reason);
    
    // 健康检查超时
    std::chrono::steady_clock::time_point last_vision_update_;
    std::chrono::steady_clock::time_point last_ultrasonic_update_;
    std::chrono::steady_clock::time_point last_temperature_update_;
    
public:
    SafetyController();
    ~SafetyController();
    
    // 初始化和控制
    bool initialize();
    void start();
    void stop();
    
    // 回调注册
    void registerCallback(SafetyCallback callback);
    
    // 安全参数设置
    void setTemperatureLimits(double max_temp);
    void setMinimumDistance(double min_dist);
    void setMaxOperationTime(int minutes);
    
    // 系统健康状态更新（由其他模块调用）
    void updateVisionHealth();
    void updateUltrasonicHealth();
    void updateTemperatureHealth();
    
    // 手动紧急停止
    void manualEmergencyStop();
    void resetEmergency();
    
    // 状态查询
    bool isEmergencyActive() const;
    bool isSystemHealthy() const;
    int getOperationTimeMinutes() const;
};

#endif // SAFETYCONTROLLER_H