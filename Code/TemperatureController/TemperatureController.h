#ifndef TEMPERATURECONTROLLER_H
#define TEMPERATURECONTROLLER_H

#include "Common.h"
#include <sys/epoll.h>
#include <fcntl.h>
#include <unistd.h>
#include <filesystem>

class TemperatureController {
private:
    int sensor_pin_;
    int heater_pin_;
    int cooler_pin_;
    int sensor_power_pin_;
    
    std::atomic<bool> running_;
    std::atomic<double> current_temperature_;
    std::atomic<double> target_temperature_;
    
    // PID control parameters
    double kp_, ki_, kd_;
    double integral_error_;
    double last_error_;
    std::chrono::steady_clock::time_point last_update_;
    
    // Event-driven temperature reading
    std::thread temperature_thread_;
    std::mutex temp_mutex_;
    
    // DS18B20 Temperature Sensor Related - Event Driven
    std::string sensor_device_path_;
    int sensor_fd_;        // Sensor file descriptor
    int epoll_fd_;         // epoll file descriptor
    
    bool openTemperatureSensorFile();
    bool setupEventDrivenReading();
    double readDS18B20TemperatureEventDriven();
    
    // PID control
    double calculatePIDOutput(double current_temp, double target_temp);
    void applyTemperatureControl(double pid_output);
    
    // Event-driven temperature monitoring thread
    void eventDrivenTemperatureLoop();
    
    // Safety protection
    double max_heating_power_;
    double max_cooling_power_;
    double max_safe_temperature_;
    double min_safe_temperature_;
    
    bool isSafeTemperature(double temp);

public:
    TemperatureController(int sensor_pin = GPIOPins::TEMP_SENSOR,
                         int heater_pin = GPIOPins::HEATER_PWM,
                         int cooler_pin = GPIOPins::COOLER_PWM,
                         int sensor_power_pin = GPIOPins::TEMP_SENSOR_POWER);
    ~TemperatureController();
    
    bool initialize();
    void shutdown();
    
    // Main control methods
    void setTargetTemperature(double target);
    double getCurrentTemperature() const { return current_temperature_.load(); }
    double getTargetTemperature() const { return target_temperature_.load(); }
    
    // PID parameter settings
    void setPIDParameters(double kp, double ki, double kd);
    void resetPIDState();
    
    // manual control
    void setHeatingPower(double power); // 0.0 to 1.0
    void setCoolingPower(double power); // 0.0 to 1.0
    void turnOffControl();
    
    // Status Inquiry
    bool isRunning() const { return running_.load(); }
    bool isTemperatureStable(double tolerance = Constants::TEMP_TOLERANCE);
    bool isWithinSafeRange() const;
    
    // Temperature history and statistics
    struct TemperatureStats {
        double min_temp;
        double max_temp;
        double avg_temp;
        int readings_count;
        std::chrono::steady_clock::time_point last_update;
    };
    TemperatureStats getTemperatureStats();
    
    // Callback registration
    using TemperatureCallback = std::function<void(double current, double target)>;
    void setTemperatureCallback(TemperatureCallback callback);
    
    // Testing and calibration
    bool selfTest();
    void calibrateSensor();
};

#endif // TEMPERATURECONTROLLER_H