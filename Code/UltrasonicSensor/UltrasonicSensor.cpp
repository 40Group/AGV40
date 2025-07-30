#include "UltrasonicSensor.h"
#include "../timer/TimerManager.h"
#include <stdexcept>

UltrasonicSensor::UltrasonicSensor(int trigger_pin, int echo_pin, 
                                   const std::string& chip_path)
    : trigger_pin_(trigger_pin), echo_pin_(echo_pin), chip_path_(chip_path),
      last_distance_(999.0), obstacle_detected_(false), measurement_active_(false),
      obstacle_threshold_cm_(20.0), timer_manager_(nullptr) {
}

UltrasonicSensor::~UltrasonicSensor() {
    stopEventDriven();
}

bool UltrasonicSensor::initialize(TimerManager* timer_manager) {
    try {
        if (!timer_manager) {
            LOG_ERROR("TimerManager is required for event-driven ultrasonic sensor");
            return false;
        }
        
        timer_manager_ = timer_manager;
        
        // 初始化GPIO芯片
        gpio_chip_ = std::make_unique<gpiod::chip>(chip_path_);
        if (!gpio_chip_->is_used()) {
            LOG_ERROR("Failed to initialize GPIO chip: " + chip_path_);
            return false;
        }
        
        setupGPIO();
        
        LOG_INFO("UltrasonicSensor initialized successfully (Event-Driven + gpiod)");
        return true;
        
    } catch (const std::exception& e) {
        LOG_ERROR("UltrasonicSensor initialization failed: " + std::string(e.what()));
        return false;
    }
}

void UltrasonicSensor::setupGPIO() {
    // 配置trigger引脚为输出
    trigger_line_ = std::make_unique<gpiod::line>(gpio_chip_->get_line(trigger_pin_));
    gpiod::line_request trigger_config = {
        "ultrasonic_trigger",
        gpiod::line_request::DIRECTION_OUTPUT,
        0  // 初始低电平
    };
    trigger_line_->request(trigger_config);
    
    // 配置echo引脚为输入，启用边沿中断
    echo_line_ = std::make_unique<gpiod::line>(gpio_chip_->get_line(echo_pin_));
    gpiod::line_request echo_config = {
        "ultrasonic_echo",
        gpiod::line_request::EVENT_BOTH_EDGES,
        0
    };
    echo_line_->request(echo_config);
    
    LOG_INFO("GPIO configured - Trigger: GPIO" + std::to_string(trigger_pin_) + 
             ", Echo: GPIO" + std::to_string(echo_pin_));
}

void UltrasonicSensor::startEventDriven() {
    if (!timer_manager_) {
        LOG_ERROR("Cannot start event-driven mode without TimerManager");
        return;
    }
    
    // 注册GPIO中断处理
    std::thread echo_monitor([this]() {
        while (!g_emergency_stop.load()) {
            try {
                // 等待echo引脚中断事件
                if (echo_line_->event_wait(std::chrono::milliseconds(100))) {
                    auto event = echo_line_->event_read();
                    
                    if (event.event_type == gpiod::line_event::RISING_EDGE) {
                        handleEchoRisingEdge();
                    } else if (event.event_type == gpiod::line_event::FALLING_EDGE) {
                        handleEchoFallingEdge();
                    }
                }
            } catch (const std::exception& e) {
                LOG_ERROR("Echo interrupt error: " + std::string(e.what()));
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    });
    echo_monitor.detach();
    
    // 注册定时测距事件（10Hz频率）
    timer_manager_->scheduleRepeating(100, [this]() {
        requestMeasurement();
    });
    
    LOG_INFO("UltrasonicSensor started in event-driven mode");
}

void UltrasonicSensor::stopEventDriven() {
    // TimerManager会自动清理定时器事件
    LOG_INFO("UltrasonicSensor stopped");
}

void UltrasonicSensor::requestMeasurement() {
    if (measurement_active_.load()) {
        return; // 上次测量还未完成
    }
    
    std::lock_guard<std::mutex> lock(measurement_mutex_);
    measurement_active_ = true;
    
    // 启动测距序列
    triggerMeasurement();
}

void UltrasonicSensor::triggerMeasurement() {
    try {
        // 发送10μs触发脉冲
        trigger_line_->set_value(1);
        std::this_thread::sleep_for(std::chrono::microseconds(10));
        trigger_line_->set_value(0);
        
        // 设置超时保护
        timer_manager_->scheduleOnce(50, [this]() {
            if (measurement_active_.load()) {
                LOG_WARNING("Ultrasonic measurement timeout");
                measurement_active_ = false;
            }
        });
        
    } catch (const std::exception& e) {
        LOG_ERROR("Trigger pulse error: " + std::string(e.what()));
        measurement_active_ = false;
    }
}

void UltrasonicSensor::handleEchoRisingEdge() {
    if (!measurement_active_.load()) {
        return;
    }
    
    // 记录echo开始时间
    echo_start_time_ = std::chrono::high_resolution_clock::now();
}

void UltrasonicSensor::handleEchoFallingEdge() {
    if (!measurement_active_.load()) {
        return;
    }
    
    // 计算脉冲持续时间
    auto echo_end_time = std::chrono::high_resolution_clock::now();
    auto pulse_duration = std::chrono::duration_cast<std::chrono::microseconds>(
        echo_end_time - echo_start_time_);
    
    measurement_active_ = false;
    
    // 计算距离
    calculateDistance(pulse_duration);
}

void UltrasonicSensor::calculateDistance(std::chrono::microseconds pulse_duration) {
    try {
        // 声速计算：distance = (pulse_time * 0.034) / 2
        double distance_cm = (pulse_duration.count() * 0.034) / 2.0;
        
        // 验证测量范围
        if (distance_cm > 0 && distance_cm < 400) {
            processDistanceResult(distance_cm);
        } else {
            LOG_WARNING("Invalid distance measurement: " + std::to_string(distance_cm) + "cm");
        }
        
    } catch (const std::exception& e) {
        LOG_ERROR("Distance calculation error: " + std::string(e.what()));
    }
}

void UltrasonicSensor::processDistanceResult(double distance) {
    // 更新距离数据
    last_distance_ = distance;
    
    // 障碍物检测
    bool obstacle = (distance < obstacle_threshold_cm_);
    bool prev_obstacle = obstacle_detected_.load();
    obstacle_detected_ = obstacle;
    
    // 触发回调（状态变化时）
    if (obstacle != prev_obstacle) {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        if (obstacle_callback_) {
            // 在TimerManager的事件线程中执行回调
            timer_manager_->scheduleOnce(0, [this, obstacle, distance]() {
                obstacle_callback_(obstacle, distance);
            });
        }
    }
    
    LOG_DEBUG("Distance: " + std::to_string(distance) + "cm, Obstacle: " + 
              (obstacle ? "Yes" : "No"));
}

void UltrasonicSensor::registerCallback(ObstacleCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    obstacle_callback_ = callback;
    LOG_INFO("Obstacle callback registered");
}

double UltrasonicSensor::getDistance() const {
    return last_distance_.load();
}

bool UltrasonicSensor::isObstacleDetected() const {
    return obstacle_detected_.load();
}

bool UltrasonicSensor::isMeasurementActive() const {
    return measurement_active_.load();
}

void UltrasonicSensor::setObstacleThreshold(double threshold_cm) {
    if (threshold_cm > 0 && threshold_cm < 200) {
        obstacle_threshold_cm_ = threshold_cm;
        LOG_INFO("Obstacle threshold set to " + std::to_string(threshold_cm) + "cm");
    } else {
        LOG_ERROR("Invalid threshold value: " + std::to_string(threshold_cm) + "cm");
    }
}
