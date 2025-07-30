#ifndef TEMPERATURECONTROLLER_H
#define TEMPERATURECONTROLLER_H

#include "../common/Common.h"
#include <gpiod.hpp>
#include <atomic>
#include <mutex>
#include <memory>
#include <functional>

class TemperatureController {
private:
    // GPIO配置 - 使用现代gpiod
    std::unique_ptr<gpiod::chip> chip_;
    std::unique_ptr<gpiod::line> heater_line_;
    std::unique_ptr<gpiod::line> cooler_line_;
    
    // 引脚编号
    int temp_sensor_pin_;
    int heater_pin_;
    int cooler_pin_;
    
    // 状态管理（原子操作）
    std::atomic<bool> running_;
    std::atomic<bool> initialized_;
    std::atomic<double> current_temperature_;
    std::atomic<double> target_temperature_;
    std::atomic<TempControlState> control_state_;
    
    // 回调系统
    TemperatureCallback temp_callback_;
    std::mutex callback_mutex_;
    std::mutex control_mutex_;
    
    // PID控制参数（事件驱动）
    std::atomic<double> kp_, ki_, kd_;
    std::atomic<double> integral_;
    std::atomic<double> previous_error_;
    
    // 事件驱动定时器（外部触发）
    std::function<void(std::function<void()>, std::chrono::milliseconds)> timer_callback_;
    
    // 内部事件驱动方法
    void handleTemperatureReadEvent();
    void handleControlCalculationEvent();
    void setupGPIOLines();
    double readTemperatureSensor();
    void setHeaterState(bool state);
    void setCoolerState(bool state);
    double calculatePIDEvent(double error);
    void triggerTemperatureCallback(double temperature);
    
public:
    TemperatureController(int temp_pin = 4, int heater_pin = 5, int cooler_pin = 6);
    ~TemperatureController();
    
    // 初始化和控制
    bool initialize();
    void start();
    void stop();
    
    // 回调注册
    void registerCallback(TemperatureCallback callback);
    void registerTimerCallback(std::function<void(std::function<void()>, std::chrono::milliseconds)> timer_cb);
    
    // 事件驱动温度控制（由外部定时器事件触发）
    void processTemperatureControlEvent();
    
    // 立即温度控制（事件响应）
    void setTargetTemperature(double target);
    void emergencyTemperatureShutdown();
    
    // 状态查询（原子操作）
    double getCurrentTemperature() const;
    double getTargetTemperature() const;
    TempControlState getControlState() const;
    bool isInitialized() const;
    
    // PID参数设置（原子操作）
    void setPIDParameters(double kp, double ki, double kd);
    void resetPIDIntegral();
};

#endif // TEMPERATURECONTROLLER_H