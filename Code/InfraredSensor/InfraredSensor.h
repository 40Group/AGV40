#ifndef INFRAREDSENSOR_H
#define INFRAREDSENSOR_H

#include "Common.h"

class InfraredSensor {
private:
    int left_pin_;
    int right_pin_;
    int front_pin_;
    std::atomic<bool> running_;
    
    // 传感器状态
    std::atomic<bool> left_obstacle_;
    std::atomic<bool> right_obstacle_;
    std::atomic<bool> front_obstacle_;
    
    // 定时器相关
    std::thread timer_thread_;
    std::chrono::milliseconds polling_interval_;
    
    // 定时器轮询循环
    void timerPollingLoop();
    void readAllSensors();

public:
    InfraredSensor(int left_pin = GPIOPins::IR_LEFT, 
                   int right_pin = GPIOPins::IR_RIGHT,
                   int front_pin = GPIOPins::IR_FRONT);
    ~InfraredSensor();
    
    bool initialize();
    void shutdown();
    
    // 主要检测方法
    bool isLeftObstacle() const { return left_obstacle_.load(); }
    bool isRightObstacle() const { return right_obstacle_.load(); }
    bool isFrontObstacle() const { return front_obstacle_.load(); }
    bool isAnyObstacle() const;
    
    // 批量状态获取
    struct ObstacleState {
        bool left;
        bool right;
        bool front;
        std::chrono::steady_clock::time_point timestamp;
    };
    ObstacleState getAllStates();
    
    // 参数设置
    void setPollingInterval(std::chrono::milliseconds interval);
    
    // 状态查询
    bool isRunning() const { return running_.load(); }
    
    // 测试方法
    bool selfTest();
};

#endif // INFRAREDSENSOR_H