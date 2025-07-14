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
    
    // 设置引脚模式
    pinMode(trig_pin_, OUTPUT);
    pinMode(echo_pin_, INPUT);
    
    // 初始化引脚状态
    digitalWrite(trig_pin_, LOW);
    
    running_ = true;
    
    // 启动阻塞I/O测量线程
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

void UltrasonicSensor::blockingMeasurementLoop() {  // ✅ 修正类名
    while (running_.load()) {
        double distance = measureDistanceBlocking();
        
        {
            std::lock_guard<std::mutex> lock(distance_mutex_);
            latest_distance_.store(distance);
        }
        
        // 等待下次测量 - 阻塞I/O方式的间隔
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

double UltrasonicSensor::measureDistanceBlocking() {
    std::lock_guard<std::mutex> lock(distance_mutex_);
    
    // 发送触发信号
    digitalWrite(trig_pin_, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin_, LOW);
    
    // 阻塞等待echo信号上升沿
    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::microseconds(TIMEOUT_US);
    
    // 等待echo引脚变高
    while (digitalRead(echo_pin_) == LOW) {
        auto current_time = std::chrono::steady_clock::now();
        if (current_time - start_time > timeout) {
            std::cerr << "Ultrasonic sensor timeout (waiting for HIGH)" << std::endl;
            return 999.0;
        }
        delayMicroseconds(1);  // 短暂延迟，避免过于频繁轮询
    }
    
    auto echo_start = std::chrono::steady_clock::now();
    
    // 阻塞等待echo引脚变低
    while (digitalRead(echo_pin_) == HIGH) {
        auto current_time = std::chrono::steady_clock::now();
        if (current_time - start_time > timeout) {
            std::cerr << "Ultrasonic sensor timeout (waiting for LOW)" << std::endl;
            return 999.0;
        }
        delayMicroseconds(1);  // 短暂延迟，避免过于频繁轮询
    }
    
    auto echo_end = std::chrono::steady_clock::now();
    
    // 计算距离
    auto pulse_duration = std::chrono::duration_cast<std::chrono::microseconds>(echo_end - echo_start);
    double distance = (pulse_duration.count() * SOUND_SPEED) / (2.0 * 1000000.0);
    
    return distance < 400.0 ? distance : 999.0;
}

double UltrasonicSensor::getStableDistance() {
    std::vector<double> readings;
    
    for (int i = 0; i < STABLE_READINGS; ++i) {
        double distance = getDistance();  // 现在是非阻塞获取最新值
        if (distance < 400.0) { // 有效读数
            readings.push_back(distance);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    if (readings.empty()) {
        return 999.0;
    }
    
    // 计算平均值
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
    
    // 测试距离测量
    for (int i = 0; i < 5; ++i) {
        double distance = getDistance();
        std::cout << "Distance reading " << i + 1 << ": " << distance << " cm" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    // 测试稳定距离测量
    double stable_distance = getStableDistance();
    std::cout << "Stable distance: " << stable_distance << " cm" << std::endl;
    
    std::cout << "Ultrasonic sensor self-test completed" << std::endl;
    return true;
}

// 保留中断处理函数作为备用（虽然现在用阻塞I/O）
void UltrasonicSensor::echoInterruptHandler() {
    if (instance_ == nullptr) return;
    
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    unsigned long current_time = tv.tv_sec * 1000000 + tv.tv_usec;
    
    if (digitalRead(instance_->echo_pin_) == HIGH) {
        // Echo信号上升沿
        instance_->echo_start_time_ = current_time;
    } else {
        // Echo信号下降沿
        instance_->echo_end_time_ = current_time;
        instance_->echo_received_ = true;
    }
}