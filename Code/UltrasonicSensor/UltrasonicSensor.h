#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#include "Common.h"

class UltrasonicSensor : public IDistanceSensor {
private:
    int trig_pin_;
    int echo_pin_;
    std::atomic<bool> running_;
    std::mutex distance_mutex_;
    std::atomic<double> latest_distance_;
    
    // 阻塞I/O线程
    std::thread measurement_thread_;
    
    // 测量参数
    static const int TIMEOUT_US = 30000; // 30ms timeout
    static const double SOUND_SPEED = 34300.0; // cm/s
    static const int STABLE_READINGS = 3;
    
    // 阻塞式距离测量循环和方法
    void blockingMeasurementLoop();
    double measureDistanceBlocking();
    
    // 中断处理相关（保留作为备用）
    static UltrasonicSensor* instance_;
    static void echoInterruptHandler();
    volatile unsigned long echo_start_time_;
    volatile unsigned long echo_end_time_;
    volatile bool echo_received_;

public:
    UltrasonicSensor(int trig_pin = GPIOPins::ULTRASONIC_TRIG, 
                     int echo_pin = GPIOPins::ULTRASONIC_ECHO);
    ~UltrasonicSensor();
    
    // 实现ISensor接口
    bool initialize() override;
    void shutdown() override;
    bool isRunning() const override { return running_.load(); }
    bool selfTest() override;
    std::string getSensorType() const override { return "Ultrasonic Distance Sensor"; }
    std::string getSensorStatus() const override;
    
    // 实现IDistanceSensor接口
    double getDistance() override { return latest_distance_.load(); }
    bool isObstacleDetected(double threshold) override;
    
    // 原有的特定方法
    double getStableDistance(); // 多次测量取平均
    double getLatestDistance() const { return latest_distance_.load(); }
};

#endif // ULTRASONICSENSOR_H