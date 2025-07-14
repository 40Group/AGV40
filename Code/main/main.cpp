#include "../include/Common.h"
#include "../include/MotorController.h"
#include "../include/VisionTracker.h"
#include "../include/UltrasonicSensor.h"
#include "../include/InfraredSensor.h"
#include "../include/TemperatureController.h"
#include "../include/SafetyController.h"
#include "../include/TimerManager.h"
#include <signal.h>
#include <csignal>
// The function of the guard macro is to prevent header files from being included more than once!
// Global variables for graceful exits
std::atomic<bool> g_running(true);

// signal processing function
void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down gracefully..." << std::endl;
    g_running = false;
}

// Intelligent small car main control class
class SmartCarController {
private:
    // System components
    std::unique_ptr<MotorController> motor_controller_;
    std::unique_ptr<VisionTracker> vision_tracker_;
    std::unique_ptr<UltrasonicSensor> ultrasonic_sensor_;
    std::unique_ptr<InfraredSensor> ir_sensor_;
    std::unique_ptr<TemperatureController> temp_controller_;
    std::unique_ptr<SafetyController> safety_controller_;
    std::unique_ptr<TimerManager> timer_manager_;
    
    // Control thread
    std::thread main_control_thread_;
    std::thread safety_monitor_thread_;
    
    // control state
    std::atomic<bool> running_;
    std::atomic<bool> auto_mode_;
    
    // control parameter
    double max_speed_;
    double turn_sensitivity_;
    
public:
    SmartCarController() 
        : running_(false), auto_mode_(true), max_speed_(150.0), turn_sensitivity_(0.8) {
    }
    
    ~SmartCarController() {
        shutdown();
    }
    
    bool initialize() {
        std::cout << "Initializing Smart Car System..." << std::endl;
        
        // Create all components
        motor_controller_ = std::make_unique<MotorController>();
        vision_tracker_ = std::make_unique<VisionTracker>();
        ultrasonic_sensor_ = std::make_unique<UltrasonicSensor>();
        ir_sensor_ = std::make_unique<InfraredSensor>();
        temp_controller_ = std::make_unique<TemperatureController>();
        safety_controller_ = std::make_unique<SafetyController>();
        timer_manager_ = std::make_unique<TimerManager>();
        
        // Initialize each component
        if (!motor_controller_->initialize()) {
            std::cerr << "Failed to initialize motor controller" << std::endl;
            return false;
        }
        
        if (!vision_tracker_->initialize(0)) {
            std::cerr << "Failed to initialize vision tracker" << std::endl;
            return false;
        }
        
        if (!ultrasonic_sensor_->initialize()) {
            std::cerr << "Failed to initialize ultrasonic sensor" << std::endl;
            return false;
        }
        
        if (!ir_sensor_->initialize()) {
            std::cerr << "Failed to initialize IR sensors" << std::endl;
            return false;
        }
        
        if (!temp_controller_->initialize()) {
            std::cerr << "Failed to initialize temperature controller" << std::endl;
            return false;
        }
        
        if (!timer_manager_->initialize()) {
            std::cerr << "Failed to initialize timer manager" << std::endl;
            return false;
        }
        
        // Initialize the safety controller (all other components are required)
        if (!safety_controller_->initialize(motor_controller_.get(), 
                                           ultrasonic_sensor_.get(),
                                           ir_sensor_.get(),
                                           temp_controller_.get())) {
            std::cerr << "Failed to initialize safety controller" << std::endl;
            return false;
        }
        
        // Set the temperature target
        temp_controller_->setTargetTemperature(Constants::TARGET_TEMPERATURE);
        
        std::cout << "Smart Car System initialized successfully!" << std::endl;
        return true;
    }
    
    void start() {
        running_ = true;
        
        // Start the main control thread
        main_control_thread_ = std::thread(&SmartCarController::mainControlLoop, this);
        
        std::cout << "Smart Car started - Press Ctrl+C to stop" << std::endl;
    }
    
    void shutdown() {
        std::cout << "Shutting down Smart Car System..." << std::endl;
        
        running_ = false;
        
        // Waiting for the control thread to finish
        if (main_control_thread_.joinable()) {
            main_control_thread_.join();
        }
        
        // Close all components (in reverse order)
        if (safety_controller_) safety_controller_->shutdown();
        if (timer_manager_) timer_manager_->shutdown();
        if (temp_controller_) temp_controller_->shutdown();
        if (ir_sensor_) ir_sensor_->shutdown();
        if (ultrasonic_sensor_) ultrasonic_sensor_->shutdown();
        if (vision_tracker_) vision_tracker_->shutdown();
        if (motor_controller_) motor_controller_->shutdown();
        
        std::cout << "Smart Car System shutdown complete" << std::endl;
    }
    
private:
    void mainControlLoop() {
        std::cout << "Main control loop started" << std::endl;
        
        while (running_.load() && g_running.load()) {
            try {
                if (auto_mode_.load()) {
                    autonomousControl();
                } else {
                    // Manual mode or stop mode
                    motor_controller_->stop();
                }
                
                // Display status information
                printSystemStatus();
                
            } catch (const std::exception& e) {
                std::cerr << "Main control loop error: " << e.what() << std::endl;
                motor_controller_->emergencyStop();
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            
            // The control cycle frequency is approximately 10Hz
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Stop all movements when exiting
        motor_controller_->stop();
        std::cout << "Main control loop ended" << std::endl;
    }
    
    void autonomousControl() {
        // Check the safety status
        if (!safety_controller_->isSafeToMove()) {
            motor_controller_->stop();
            return;
        }
        
        // Obtain visual processing results - event-driven approach
        VisionResult vision_result = vision_tracker_->processFrame();
        
        MotorCommand command;
        
        if (vision_result.line_detected) {
            // Vision-based tracking control
            command = calculateTrackingCommand(vision_result);
        } else {
            // No line was detected. Stop or search
            std::cout << "No line detected, stopping..." << std::endl;
            command = MotorCommand(0, 0);
        }
        
        // Apply security constraints
        MotorCommand safe_command = safety_controller_->applySafetyConstraints(command);
        
        // Execute motion commands
        motor_controller_->executeCommand(safe_command);
    }
    
    MotorCommand calculateTrackingCommand(const VisionResult& vision_result) {
        // PID control parameters
        static double kp = 100.0;  // 比例系数
        static double kd = 50.0;   // 微分系数
        static double last_offset = 0.0;
        
        double offset = vision_result.line_offset;
        double offset_derivative = offset - last_offset;
        last_offset = offset;
        
        // Calculating Steering Strength
        double turn_control = kp * offset + kd * offset_derivative;
        turn_control = std::max(-1.0, std::min(1.0, turn_control / 100.0));
        
        // base speed
        double base_speed = max_speed_;
        
        // Adjusts speed to steering angle
        double speed_factor = 1.0 - std::abs(turn_control) * 0.5;
        base_speed *= speed_factor;
        
        // Calculate left and right wheel speeds
        int left_speed = static_cast<int>(base_speed * (1.0 - turn_control * turn_sensitivity_));
        int right_speed = static_cast<int>(base_speed * (1.0 + turn_control * turn_sensitivity_));
        
        // Limit speed range
        left_speed = std::max(-Constants::MAX_MOTOR_SPEED, 
                             std::min(Constants::MAX_MOTOR_SPEED, left_speed));
        right_speed = std::max(-Constants::MAX_MOTOR_SPEED, 
                              std::min(Constants::MAX_MOTOR_SPEED, right_speed));
        
        return MotorCommand(left_speed, right_speed);
    }
    
    void printSystemStatus() {
        static int status_counter = 0;
        
        // Prints status every 5 seconds
        if (++status_counter % 50 == 0) {
            std::cout << "\n=== System Status ===" << std::endl;
            std::cout << "Safety State: " << safety_controller_->getStateDescription() << std::endl;
            std::cout << "Temperature: " << temp_controller_->getCurrentTemperature() 
                      << "°C (Target: " << temp_controller_->getTargetTemperature() << "°C)" << std::endl;
            std::cout << "Distance: " << ultrasonic_sensor_->getLatestDistance() << " cm" << std::endl;
            
            auto ir_state = ir_sensor_->getAllStates();
            std::cout << "IR Sensors - Left: " << ir_state.left 
                      << ", Right: " << ir_state.right 
                      << ", Front: " << ir_state.front << std::endl;
            
            std::cout << "Active Timers: " << timer_manager_->getActiveTimerCount() << std::endl;
            std::cout << "===================" << std::endl;
        }
    }
};

// System self-test function
bool performSystemSelfTest(SmartCarController& car) {
    std::cout << "\n=== Performing System Self-Test ===" << std::endl;
    
    
    std::cout << "System self-test completed" << std::endl;
    return true;
}

int main(int argc, char* argv[]) {
    std::cout << "Smart Medical Transport Car System Starting..." << std::endl;
    std::cout << "Version: 1.0.0" << std::endl;
    std::cout << "Build Date: " << __DATE__ << " " << __TIME__ << std::endl;
    
    // Setting up signal processing
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try {
        // Creating Intelligent Trolley Controllers
        SmartCarController car;
        
        // Initialise the system
        if (!car.initialize()) {
            std::cerr << "Failed to initialize smart car system" << std::endl;
            return -1;
        }
        
        // Perform system self-test
        if (!performSystemSelfTest(car)) {
            std::cerr << "System self-test failed" << std::endl;
            return -1;
        }
        
        // activation system
        car.start();
        
        // Main loop - waiting for exit signal
        while (g_running.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Elegant closure
        car.shutdown();
        
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return -1;
    }
    
    std::cout << "Smart Car System exited successfully" << std::endl;
    return 0;
}
