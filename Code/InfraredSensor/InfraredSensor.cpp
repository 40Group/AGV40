#include "../include/InfraredSensor.h"

InfraredSensor::InfraredSensor(int left_pin, int right_pin, int front_pin)
    : left_pin_(left_pin), right_pin_(right_pin), front_pin_(front_pin), running_(false),
      left_obstacle_(false), right_obstacle_(false), front_obstacle_(false),
      polling_interval_(std::chrono::milliseconds(50)) {
}

InfraredSensor::~InfraredSensor() {
    shutdown();
}

bool InfraredSensor::initialize() {
    if (wiringPiSetup() == -1) {
        std::cerr << "Failed to initialize WiringPi for IR sensors" << std::endl;
        return false;
    }
    
    // Set the pin to input mode and enable the pull-up resistor
    pinMode(left_pin_, INPUT);
    pinMode(right_pin_, INPUT);
    pinMode(front_pin_, INPUT);
    
    pullUpDnControl(left_pin_, PUD_UP);
    pullUpDnControl(right_pin_, PUD_UP);
    pullUpDnControl(front_pin_, PUD_UP);
    
    running_ = true;
    
    // Start the timer polling thread
    timer_thread_ = std::thread(&InfraredSensor::timerPollingLoop, this);
    
    std::cout << "InfraredSensor initialized successfully (Timer polling mode)" << std::endl;
    return true;
}

void InfraredSensor::shutdown() {
    running_ = false;
    
    if (timer_thread_.joinable()) {
        timer_thread_.join();
    }
    
    std::cout << "InfraredSensor shutdown" << std::endl;
}

void InfraredSensor::timerPollingLoop() {
    while (running_.load()) {
        readAllSensors();
        std::this_thread::sleep_for(polling_interval_);
    }
}

void InfraredSensor::readAllSensors() {
    // Read the status of all sensors (LOW indicates the detection of obstacles)
    bool left = (digitalRead(left_pin_) == LOW);
    bool right = (digitalRead(right_pin_) == LOW);
    bool front = (digitalRead(front_pin_) == LOW);
    
    // Update the status (output information only when the status changes)
    if (left != left_obstacle_.load()) {
        left_obstacle_.store(left);
        if (left) std::cout << "Left obstacle detected!" << std::endl;
        else std::cout << "Left obstacle cleared!" << std::endl;
    }
    
    if (right != right_obstacle_.load()) {
        right_obstacle_.store(right);
        if (right) std::cout << "Right obstacle detected!" << std::endl;
        else std::cout << "Right obstacle cleared!" << std::endl;
    }
    
    if (front != front_obstacle_.load()) {
        front_obstacle_.store(front);
        if (front) std::cout << "Front obstacle detected!" << std::endl;
        else std::cout << "Front obstacle cleared!" << std::endl;
    }
}

bool InfraredSensor::isAnyObstacle() const {
    return left_obstacle_.load() || right_obstacle_.load() || front_obstacle_.load();
}

InfraredSensor::ObstacleState InfraredSensor::getAllStates() {
    ObstacleState state;
    state.left = left_obstacle_.load();
    state.right = right_obstacle_.load();
    state.front = front_obstacle_.load();
    state.timestamp = std::chrono::steady_clock::now();
    return state;
}

void InfraredSensor::setPollingInterval(std::chrono::milliseconds interval) {
    polling_interval_ = interval;
}

bool InfraredSensor::selfTest() {
    std::cout << "Starting IR sensor self-test (Timer polling mode)..." << std::endl;
    
    if (!running_.load()) {
        std::cout << "IR sensors not initialized" << std::endl;
        return false;
    }
    
    // Monitor status changes
    std::cout << "Monitoring for 5 seconds..." << std::endl;
    auto start_time = std::chrono::steady_clock::now();
    
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(5)) {
        ObstacleState state = getAllStates();
        std::cout << "IR States - Left: " << state.left 
                  << ", Right: " << state.right 
                  << ", Front: " << state.front << std::endl;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    std::cout << "IR sensor self-test completed" << std::endl;
    return true;
}
