#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#include "../common/Common.h"
#include <gpiod.hpp>
#include <memory>
#include <chrono>

class TimerManager; // 前向声明

class UltrasonicSensor {
private:
    // GPIO 配置
    int trigger_pin_;
    int echo_pin_;
    std::string chip_path_;
    
    // gpiod 对象
    std::unique_ptr<gpiod::chip> gpio_chip_;
    std::unique_ptr<gpiod::line> trigger_line_;
    std::unique_ptr<gpiod::line> echo_line_;
    
    // 状态数据
    std::atomic<double> last_distance_;
    std::atomic<bool> obstacle_detected_;
    std::atomic<bool> measurement_active_;
    
    // 回调系统
    ObstacleCallback obstacle_callback_;
    std::mutex callback_mutex_;
    
    // 配置参数
    double obstacle_threshold_cm_;
    
    // 测距状态
    std::chrono::high_resolution_clock::time_point echo_start_time_;
    std::mutex measurement_mutex_;
    
    // TimerManager引用
    TimerManager* timer_manager_;
    
    // GPIO中断处理
    void handleEchoRisingEdge();
    void handleEchoFallingEdge();
    
    // 内部方法
    void setupGPIO();
    void triggerMeasurement();
    void calculateDistance(std::chrono::microseconds pulse_duration);
    void processDistanceResult(double distance);
    
public:
    UltrasonicSensor(int trigger_pin = 23, int echo_pin = 24, 
                     const std::string& chip_path = "/dev/gpiochip0");
    ~UltrasonicSensor();
    
    // 初始化
    bool initialize(TimerManager* timer_manager);
    
    // 事件驱动启动（注册定时测距事件）
    void startEventDriven();
    void stopEventDriven();
    
    // 回调注册
    void registerCallback(ObstacleCallback callback);
    
    // 状态查询
    double getDistance() const;
    bool isObstacleDetected() const;
    bool isMeasurementActive() const;
    
    // 配置
    void setObstacleThreshold(double threshold_cm);
    
    // 手动触发测距（用于测试）
    void requestMeasurement();
};

#endif // ULTRASONICSENSOR_H