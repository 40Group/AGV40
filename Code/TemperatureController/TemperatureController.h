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
    // GPIO configuration - Using modern gpiod
    std::unique_ptr<gpiod::chip> chip_;
    std::unique_ptr<gpiod::line> heater_line_;
    std::unique_ptr<gpiod::line> cooler_line_;
    
    // pin number
    int temp_sensor_pin_;
    int heater_pin_;
    int cooler_pin_;
    
    // State management (atomic operations)
    std::atomic<bool> running_;
    std::atomic<bool> initialized_;
    std::atomic<double> current_temperature_;
    std::atomic<double> target_temperature_;
    std::atomic<TempControlState> control_state_;
    
    // callback system
    TemperatureCallback temp_callback_;
    std::mutex callback_mutex_;
    std::mutex control_mutex_;
    
    // PID control parameters (event-driven)
    std::atomic<double> kp_, ki_, kd_;
    std::atomic<double> integral_;
    std::atomic<double> previous_error_;
    
    // Event-driven timer (external trigger)
    std::function<void(std::function<void()>, std::chrono::milliseconds)> timer_callback_;
    
    // Internal event-driven approach
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
    
    // Initialization and control
    bool initialize();
    void start();
    void stop();
    
    // callback registration
    void registerCallback(TemperatureCallback callback);
    void registerTimerCallback(std::function<void(std::function<void()>, std::chrono::milliseconds)> timer_cb);
    
    // Event-driven temperature control (triggered by external timer events)
    void processTemperatureControlEvent();
    
    // Immediate temperature control (event response)
    void setTargetTemperature(double target);
    void emergencyTemperatureShutdown();
    
    // Status query (atomic operation)
    double getCurrentTemperature() const;
    double getTargetTemperature() const;
    TempControlState getControlState() const;
    bool isInitialized() const;
    
    // PID parameter settings (atomic operations)
    void setPIDParameters(double kp, double ki, double kd);
    void resetPIDIntegral();
};

#endif // TEMPERATURECONTROLLER_H