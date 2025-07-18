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
    
    // Blocking I/O thread
    std::thread measurement_thread_;
    
    // measure parameters
    static const int TIMEOUT_US = 30000; // 30ms timeout
    static const double SOUND_SPEED = 34300.0; // cm/s
    static const int STABLE_READINGS = 3;
    
    // Blocking-type distance measurement loop and method
    void blockingMeasurementLoop();
    double measureDistanceBlocking();
    
    // Interrupt handling related (reserved as a backup)
    static UltrasonicSensor* instance_;
    static void echoInterruptHandler();
    volatile unsigned long echo_start_time_;
    volatile unsigned long echo_end_time_;
    volatile bool echo_received_;

public:
    UltrasonicSensor(int trig_pin = GPIOPins::ULTRASONIC_TRIG, 
                     int echo_pin = GPIOPins::ULTRASONIC_ECHO);
    ~UltrasonicSensor();
    
    // Implement the ISensor interface
    bool initialize() override;
    void shutdown() override;
    bool isRunning() const override { return running_.load(); }
    bool selfTest() override;
    std::string getSensorType() const override { return "Ultrasonic Distance Sensor"; }
    std::string getSensorStatus() const override;
    
    // Implement the IDistanceSensor interface
    double getDistance() override { return latest_distance_.load(); }
    bool isObstacleDetected(double threshold) override;
    
    // The original specific method
    double getStableDistance(); // Take the average of multiple measurements
    double getLatestDistance() const { return latest_distance_.load(); }
};

#endif // ULTRASONICSENSOR_H
