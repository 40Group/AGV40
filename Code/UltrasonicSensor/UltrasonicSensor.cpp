#include "../include/UltrasonicSensor.h"
#include <sys/select.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>

UltrasonicSensor* UltrasonicSensor::instance_ = nullptr;

UltrasonicSensor::UltrasonicSensor(int trig_pin, int echo_pin)
    : trig_pin_(trig_pin), echo_pin_(echo_pin), running_(false), 
      latest_distance_(999.0), echo_start_time_(0), echo_end_time_(0), echo_received_(false) {
    instance_ = this;
}

UltrasonicSensor::~UltrasonicSensor() {
    shutdown();
    instance_ = nullptr;
}

bool UltrasonicSensor::initialize() {
    if (wiringPiSetup() == -1) {
        std::cerr << "Failed to initialize WiringPi for ultrasonic sensor" << std::endl;
        return false;
    }
    
    // Set pin mode
    pinMode(trig_pin_, OUTPUT);
    pinMode(echo_pin_, INPUT);
    
    // Initialize pin states
    digitalWrite(trig_pin_, LOW);
    
    running_ = true;
    
    // Start the blocking I/O measurement thread
    measurement_thread_ = std::thread(&UltrasonicSensor::blockingMeasurementLoop, this);
    
    std::cout << "UltrasonicSensor initialized successfully (Blocking I/O)" << std::endl;
    return true;
}

void UltrasonicSensor::shutdown() {
    running_ = false;
    
    if (measurement_thread_.joinable()) {
        measurement_thread_.join();
    }
    
    std::cout << "UltrasonicSensor shutdown" << std::endl;
}

void UltrasonicSensor::blockingMeasurementLoop() {  // ✅ Correct the class name
    while (running_.load()) {
        double distance = measureDistanceBlocking();
        
        {
            std::lock_guard<std::mutex> lock(distance_mutex_);
            latest_distance_.store(distance);
        }
        
        // Waiting for the next measurement - The interval of the blocking I/O mode
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

double UltrasonicSensor::measureDistanceBlocking() {
    std::lock_guard<std::mutex> lock(distance_mutex_);
    
    // Send the triggering signal
    digitalWrite(trig_pin_, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin_, LOW);
    
    // Wait for the rising edge of the echo signal.
    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::microseconds(TIMEOUT_US);
    
    // Wait for the echo pin to go high
    while (digitalRead(echo_pin_) == LOW) {
        auto current_time = std::chrono::steady_clock::now();
        if (current_time - start_time > timeout) {
            std::cerr << "Ultrasonic sensor timeout (waiting for HIGH)" << std::endl;
            return 999.0;
        }
        delayMicroseconds(1);  // A brief delay to avoid overly frequent polling
    }
    
    auto echo_start = std::chrono::steady_clock::now();
    
    // Blocking and waiting for the echo pin to go low
    while (digitalRead(echo_pin_) == HIGH) {
        auto current_time = std::chrono::steady_clock::now();
        if (current_time - start_time > timeout) {
            std::cerr << "Ultrasonic sensor timeout (waiting for LOW)" << std::endl;
            return 999.0;
        }
        delayMicroseconds(1);  // A brief delay to avoid overly frequent polling
    }
    
    auto echo_end = std::chrono::steady_clock::now();
    
    // Calculate the distance
    auto pulse_duration = std::chrono::duration_cast<std::chrono::microseconds>(echo_end - echo_start);
    double distance = (pulse_duration.count() * SOUND_SPEED) / (2.0 * 1000000.0);
    
    return distance < 400.0 ? distance : 999.0;
}

double UltrasonicSensor::getStableDistance() {
    std::vector<double> readings;
    
    for (int i = 0; i < STABLE_READINGS; ++i) {
        double distance = getDistance();  // Now, it is time to asynchronously obtain the latest value.
        if (distance < 400.0) { // Valid reading
            readings.push_back(distance);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    if (readings.empty()) {
        return 999.0;
    }
    
    // calculate the average amount
    double sum = 0.0;
    for (double reading : readings) {
        sum += reading;
    }
    
    return sum / readings.size();
}

bool UltrasonicSensor::isObstacleDetected(double threshold) {
    double distance = getDistance();
    return distance < threshold;
}

bool UltrasonicSensor::selfTest() {
    std::cout << "Starting ultrasonic sensor self-test (Blocking I/O)..." << std::endl;
    
    if (!running_.load()) {
        std::cout << "Ultrasonic sensor not initialized" << std::endl;
        return false;
    }
    
    // Test distance measurement
    for (int i = 0; i < 5; ++i) {
        double distance = getDistance();
        std::cout << "Distance reading " << i + 1 << ": " << distance << " cm" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    // Test for stable distance measurement
    double stable_distance = getStableDistance();
    std::cout << "Stable distance: " << stable_distance << " cm" << std::endl;
    
    std::cout << "Ultrasonic sensor self-test completed" << std::endl;
    return true;
}

// Keep the interrupt handling function as a backup (although currently using blocking I/O)
void UltrasonicSensor::echoInterruptHandler() {
    if (instance_ == nullptr) return;
    
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    unsigned long current_time = tv.tv_sec * 1000000 + tv.tv_usec;
    
    if (digitalRead(instance_->echo_pin_) == HIGH) {
        // Echo signal rising edge
        instance_->echo_start_time_ = current_time;
    } else {
        // Echo signal falling edge
        instance_->echo_end_time_ = current_time;
        instance_->echo_received_ = true;
    }
}
