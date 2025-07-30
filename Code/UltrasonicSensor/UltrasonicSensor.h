#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#include "../common/Common.h"
#include <gpiod.hpp>
#include <memory>
#include <chrono>

class TimerManager; // Forward declaration

class UltrasonicSensor {
private:
    // GPIO Configuration
    int trigger_pin_;
    int echo_pin_;
    std::string chip_path_;
    
    // gpiod Object
    std::unique_ptr<gpiod::chip> gpio_chip_;
    std::unique_ptr<gpiod::line> trigger_line_;
    std::unique_ptr<gpiod::line> echo_line_;
    
    // Status data
    std::atomic<double> last_distance_;
    std::atomic<bool> obstacle_detected_;
    std::atomic<bool> measurement_active_;
    
    // Callback system
    ObstacleCallback obstacle_callback_;
    std::mutex callback_mutex_;
    
    // Configuration parameters
    double obstacle_threshold_cm_;
    
    // Ranging status
    std::chrono::high_resolution_clock::time_point echo_start_time_;
    std::mutex measurement_mutex_;
    
    // TimerManager References
    TimerManager* timer_manager_;
    
    // GPIO Interrupt handling
    void handleEchoRisingEdge();
    void handleEchoFallingEdge();
    
    // Internal methods
    void setupGPIO();
    void triggerMeasurement();
    void calculateDistance(std::chrono::microseconds pulse_duration);
    void processDistanceResult(double distance);
    
public:
    UltrasonicSensor(int trigger_pin = 23, int echo_pin = 24, 
                     const std::string& chip_path = "/dev/gpiochip0");
    ~UltrasonicSensor();
    
    // initialization
    bool initialize(TimerManager* timer_manager);
    
    // Event-driven startup (registering timed ranging events)
    void startEventDriven();
    void stopEventDriven();
    
    // Callback Registration
    void registerCallback(ObstacleCallback callback);
    
    // Status Query
    double getDistance() const;
    bool isObstacleDetected() const;
    bool isMeasurementActive() const;
    
    // Configuration
    void setObstacleThreshold(double threshold_cm);
    
    // Manual trigger ranging (for testing)
    void requestMeasurement();
};

#endif // ULTRASONICSENSOR_H
