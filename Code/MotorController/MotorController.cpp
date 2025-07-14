#include "../include/MotorController.h"
#include <algorithm>
#include <thread>
#include <chrono>

MotorController::MotorController() 
    : emergency_stop_(false), running_(false), max_speed_(Constants::MAX_MOTOR_SPEED),
      min_speed_(Constants::MIN_MOTOR_SPEED), current_left_speed_(0), current_right_speed_(0) {
}

MotorController::~MotorController() {
    shutdown();
}

bool MotorController::initialize() {
    if (wiringPiSetup() == -1) {
        std::cerr << "Failed to initialize WiringPi" << std::endl;
        return false;
    }
    
    // Set the motor control pin as output.
    pinMode(GPIOPins::MOTOR_LEFT_PWM, PWM_OUTPUT);
    pinMode(GPIOPins::MOTOR_LEFT_DIR1, OUTPUT);
    pinMode(GPIOPins::MOTOR_LEFT_DIR2, OUTPUT);
    pinMode(GPIOPins::MOTOR_RIGHT_PWM, PWM_OUTPUT);
    pinMode(GPIOPins::MOTOR_RIGHT_DIR1, OUTPUT);
    pinMode(GPIOPins::MOTOR_RIGHT_DIR2, OUTPUT);
    
    // Initialize PWM
    pwmSetMode(PWM_MODE_MS);
    pwmSetClock(384);
    pwmSetRange(1024);
    
    // Initial state: Stopped
    stop();
    
    running_ = true;
    std::cout << "MotorController initialized successfully" << std::endl;
    return true;
}

void MotorController::shutdown() {
    running_ = false;
    emergencyStop();
    std::cout << "MotorController shutdown" << std::endl;
}

void MotorController::setMotorSpeed(int motor_pin, int dir_pin1, int dir_pin2, int speed) {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    
    if (emergency_stop_.load()) {
        pwmWrite(motor_pin, 0);
        digitalWrite(dir_pin1, LOW);
        digitalWrite(dir_pin2, LOW);
        return;
    }
    
    // Speed limit range
    speed = std::max(-max_speed_, std::min(max_speed_, speed));
    
    if (speed > 0) {
        // forward rotation
        digitalWrite(dir_pin1, HIGH);
        digitalWrite(dir_pin2, LOW);
        pwmWrite(motor_pin, std::abs(speed) * 1024 / 255);
    } else if (speed < 0) {
        // Reverse rotation
        digitalWrite(dir_pin1, LOW);
        digitalWrite(dir_pin2, HIGH);
        pwmWrite(motor_pin, std::abs(speed) * 1024 / 255);
    } else {
        // Stop
        digitalWrite(dir_pin1, LOW);
        digitalWrite(dir_pin2, LOW);
        pwmWrite(motor_pin, 0);
    }
}

void MotorController::executeCommand(const MotorCommand& command) {
    if (!running_.load()) return;
    
    current_command_ = command;
    
    if (command.emergency_stop) {
        emergencyStop();
        return;
    }
    
    setMotorSpeed(GPIOPins::MOTOR_LEFT_PWM, GPIOPins::MOTOR_LEFT_DIR1, 
                  GPIOPins::MOTOR_LEFT_DIR2, command.left_speed);
    setMotorSpeed(GPIOPins::MOTOR_RIGHT_PWM, GPIOPins::MOTOR_RIGHT_DIR1, 
                  GPIOPins::MOTOR_RIGHT_DIR2, command.right_speed);
    
    current_left_speed_ = command.left_speed;
    current_right_speed_ = command.right_speed;
}

void MotorController::emergencyStop() {
    emergency_stop_ = true;
    
    std::lock_guard<std::mutex> lock(motor_mutex_);
    
    // Stop all motors immediately.
    pwmWrite(GPIOPins::MOTOR_LEFT_PWM, 0);
    pwmWrite(GPIOPins::MOTOR_RIGHT_PWM, 0);
    digitalWrite(GPIOPins::MOTOR_LEFT_DIR1, LOW);
    digitalWrite(GPIOPins::MOTOR_LEFT_DIR2, LOW);
    digitalWrite(GPIOPins::MOTOR_RIGHT_DIR1, LOW);
    digitalWrite(GPIOPins::MOTOR_RIGHT_DIR2, LOW);
    
    current_left_speed_ = 0;
    current_right_speed_ = 0;
    
    std::cout << "Emergency stop activated!" << std::endl;
}

void MotorController::resumeFromEmergency() {
    emergency_stop_ = false;
    std::cout << "Emergency stop cleared" << std::endl;
}

void MotorController::moveForward(int speed) {
    MotorCommand cmd(speed, speed);
    executeCommand(cmd);
}

void MotorController::moveBackward(int speed) {
    MotorCommand cmd(-speed, -speed);
    executeCommand(cmd);
}

void MotorController::turnLeft(int speed) {
    MotorCommand cmd(-speed/2, speed);
    executeCommand(cmd);
}

void MotorController::turnRight(int speed) {
    MotorCommand cmd(speed, -speed/2);
    executeCommand(cmd);
}

void MotorController::stop() {
    MotorCommand cmd(0, 0);
    executeCommand(cmd);
}

void MotorController::setDifferentialSpeed(int left_speed, int right_speed) {
    MotorCommand cmd(left_speed, right_speed);
    executeCommand(cmd);
}

void MotorController::smoothTurn(double turn_ratio) {
    // turn_ratio: -1.0 (turn completely left) 到 1.0 (turn comletely right)
    turn_ratio = std::max(-1.0, std::min(1.0, turn_ratio));
    
    int base_speed = 100;
    int left_speed = base_speed;
    int right_speed = base_speed;
    
    if (turn_ratio > 0) {
        // Turn right: Reduce the speed of the right wheel.
        right_speed = base_speed * (1.0 - turn_ratio);
    } else if (turn_ratio < 0) {
        // Turn left: Reduce the speed of the left wheel.
        left_speed = base_speed * (1.0 + turn_ratio);
    }
    
    setDifferentialSpeed(left_speed, right_speed);
}

MotorCommand MotorController::getCurrentCommand() const {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    return current_command_;
}

bool MotorController::selfTest() {
    std::cout << "Starting motor self-test..." << std::endl;
    
    if (!running_.load()) {
        std::cout << "Motor controller not initialized" << std::endl;
        return false;
    }
    
    // Test movement in all directions
    std::cout << "Testing forward motion..." << std::endl;
    moveForward(50);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    std::cout << "Testing backward motion..." << std::endl;
    moveBackward(50);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    std::cout << "Testing left turn..." << std::endl;
    turnLeft(50);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    std::cout << "Testing right turn..." << std::endl;
    turnRight(50);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    std::cout << "Testing stop..." << std::endl;
    stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    std::cout << "Testing emergency stop..." << std::endl;
    emergencyStop();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    resumeFromEmergency();
    
    std::cout << "Motor self-test completed successfully" << std::endl;
    return true;
}