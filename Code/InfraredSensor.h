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
    // GPIO配置
    std::unique_ptr<gpiod::chip> chip_;
    std::unique_ptr<gpiod::line> left_line_;
    std::unique_ptr<gpiod::line> right_line_;
    
    // 线程和状态
    std::thread event_thread_;
    std::atomic<bool> running_;
    std::atomic<bool> left_detected_;
    std::atomic<bool> right_detected_;
    
    // 回调系统
    std::function<void(bool, bool)> boundary_callback_;
    std::mutex callback_mutex_;
    
    // GPIO引脚
    int left_pin_;
    int right_pin_;
    
    // 真实事件驱动方法
    void realEventLoop();
    void setupGPIOEvents();
    void handleGPIOEvent(gpiod::line& line, const gpiod::line_event& event);
    
public:
    InfraredSensor(int left_pin = 19, int right_pin = 26);
    ~InfraredSensor();
    
    // 初始化和控制
    bool initialize();
    void start();
    void stop();
    
    // 回调注册
    void registerCallback(std::function<void(bool, bool)> callback);
    
    // 状态查询（原子操作，线程安全）
    bool isLeftDetected() const;
    bool isRightDetected() const;
    bool isBoundaryDetected() const;
    
    // 测试相关方法（保持兼容性）
    bool isRunning() const;
    bool selfTest();
    void setPollingInterval(std::chrono::milliseconds interval); // 现在只是占位符
    void shutdown();
    
    // 扩展功能（保持测试兼容）
    struct SensorStates {
        bool left;
        bool right;
        bool front;  // 为了兼容测试，front = left || right
    };
    SensorStates getAllStates() const;
    bool isAnyObstacle() const;
};

#endif // INFRAREDSENSOR_H