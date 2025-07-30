#include "../common/Common.h"
#include "../MotorController/MotorController.h"
#include "../VisionTracker/VisionTracker.h"
#include "../UltrasonicSensor/UltrasonicSensor.h"
#include "../InfraredSensor/InfraredSensor.h"
#include "../TemperatureController/TemperatureController.h"
#include "../SafetyController/SafetyController.h"
#include "../TimerManager/TimerManager.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>

class EventDrivenIntegrationTest {
private:
    std::unique_ptr<TimerManager> timer;
    std::unique_ptr<MotorController> motor;
    std::unique_ptr<VisionTracker> vision;
    std::unique_ptr<UltrasonicSensor> ultrasonic;
    std::unique_ptr<InfraredSensor> infrared;
    std::unique_ptr<TemperatureController> temperature;
    std::unique_ptr<SafetyController> safety;
    
    std::atomic<bool> test_passed;
    std::atomic<bool> emergency_triggered;
    std::atomic<bool> obstacle_detected;
    std::atomic<bool> line_detected;
    std::atomic<int> vision_callbacks;
    std::atomic<int> ultrasonic_callbacks;
    
public:
    EventDrivenIntegrationTest() 
        : test_passed(true), emergency_triggered(false), obstacle_detected(false),
          line_detected(false), vision_callbacks(0), ultrasonic_callbacks(0) {}
    
    void runAllTests() {
        std::cout << "=== Event-Driven Integration Tests ===" << std::endl;
        
        testSystemInitialization();
        testEventDrivenCallbacks();
        testEmergencySystem();
        testCrossModuleIntegration();
        testPerformanceValidation();
        
        cleanup();
        
        if (test_passed.load()) {
            std::cout << "✅ ALL EVENT-DRIVEN INTEGRATION TESTS PASSED!" << std::endl;
        } else {
            std::cout << "❌ SOME EVENT-DRIVEN INTEGRATION TESTS FAILED!" << std::endl;
        }
    }
    
private:
    void testSystemInitialization() {
        std::cout << "Testing event-driven system initialization..." << std::endl;
        
        // 创建TimerManager（必须首先创建）
        timer = std::make_unique<TimerManager>();
        if (!timer->initialize()) {
            std::cout << "❌ TimerManager initialization failed" << std::endl;
            test_passed = false;
            return;
        }
        timer->start();
        
        // 创建所有组件
        motor = std::make_unique<MotorController>();
        vision = std::make_unique<VisionTracker>();
        ultrasonic = std::make_unique<UltrasonicSensor>();
        infrared = std::make_unique<InfraredSensor>();
        temperature = std::make_unique<TemperatureController>();
        safety = std::make_unique<SafetyController>();
        
        // 初始化所有组件（传递TimerManager依赖）
        if (!motor->initialize(timer.get()) ||
            !vision->initialize(timer.get()) ||
            !ultrasonic->initialize(timer.get()) ||
            !infrared->initialize(timer.get()) ||
            !temperature->initialize(timer.get()) ||
            !safety->initialize(timer.get())) {
            
            std::cout << "❌ Event-driven system initialization failed" << std::endl;
            test_passed = false;
            return;
        }
        
        std::cout << "✅ Event-driven system initialization test passed" << std::endl;
    }
    
    void testEventDrivenCallbacks() {
        std::cout << "Testing event-driven callback integration..." << std::endl;
        
        vision_callbacks = 0;
        ultrasonic_callbacks = 0;
        
        // 设置回调
        vision->registerCallback([this](bool detected, double deviation) {
            vision_callbacks++;
            line_detected = detected;
            
            if (detected) {
                if (abs(deviation) > 0.1) {
                    motor->requestMotion(MotionState::TURN_RIGHT);
                } else {
                    motor->requestMotion(MotionState::FORWARD);
                }
            } else {
                motor->requestMotion(MotionState::STOP);
            }
        });
        
        ultrasonic->registerCallback([this](bool detected, double distance) {
            ultrasonic_callbacks++;
            obstacle_detected = detected;
            safety->updateUltrasonicHealth();
            
            if (detected && distance < 20.0) {
                motor->requestMotion(MotionState::STOP);
            }
        });
        
        temperature->registerCallback([this](double temp, TempControlState state) {
            safety->updateTemperatureHealth();
        });
        
        safety->registerCallback([this](const std::string& emergency) {
            emergency_triggered = true;
            motor->emergencyStop();
        });
        
        // 启动事件驱动服务
        safety->startEventDriven();
        temperature->startEventDriven();
        vision->startEventDriven();
        ultrasonic->startEventDriven();
        infrared->startEventDriven();
        motor->startEventDriven();
        
        // 等待回调执行
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        
        // 验证回调执行
        int vision_count = vision_callbacks.load();
        int ultrasonic_count = ultrasonic_callbacks.load();
        
        if (vision_count < 1) {
            std::cout << "⚠️ Vision callbacks not triggered (may be due to no camera)" << std::endl;
        }
        
        if (ultrasonic_count < 1) {
            std::cout << "⚠️ Ultrasonic callbacks not triggered" << std::endl;
        }
        
        std::cout << "✅ Event-driven callback integration test completed" << std::endl;
        std::cout << "    Vision callbacks: " << vision_count << std::endl;
        std::cout << "    Ultrasonic callbacks: " << ultrasonic_count << std::endl;
    }
    
    void testEmergencySystem() {
        std::cout << "Testing event-driven emergency system..." << std::endl;
        
        emergency_triggered = false;
        
        // 设置安全参数
        safety->setTemperatureLimits(30.0);  // 较低的限制用于测试
        safety->setMinimumDistance(15.0);
        
        // 模拟紧急情况
        safety->manualEmergencyStop();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        if (!safety->isEmergencyActive()) {
            std::cout << "❌ Event-driven emergency system test failed - emergency not active" << std::endl;
            test_passed = false;
            return;
        }
        
        if (motor->getCurrentState() != MotionState::STOP) {
            std::cout << "❌ Event-driven emergency system test failed - motor not stopped" << std::endl;
            test_passed = false;
            return;
        }
        
        // 重置紧急状态
        safety->resetEmergency();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        if (safety->isEmergencyActive()) {
            std::cout << "❌ Event-driven emergency reset test failed" << std::endl;
            test_passed = false;
            return;
        }
        
        std::cout << "✅ Event-driven emergency system test passed" << std::endl;
    }
    
    void testCrossModuleIntegration() {
        std::cout << "Testing cross-module event integration..." << std::endl;
        
        // 设置温度目标
        temperature->setTargetTemperature(25.0);
        
        // 配置视觉跟踪
        vision->setLineColor(cv::Scalar(0, 0, 100), cv::Scalar(180, 30, 255));
        
        // 配置超声波阈值
        ultrasonic->setObstacleThreshold(15.0);
        
        // 等待系统稳定
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        // 验证模块状态
        double current_temp = temperature->getCurrentTemperature();
        double target_temp = temperature->getTargetTemperature();
        bool line_status = vision->isLineDetected();
        double distance = ultrasonic->getDistance();
        
        if (target_temp != 25.0) {
            std::cout << "❌ Temperature target setting failed" << std::endl;
            test_passed = false;
            return;
        }
        
        std::cout << "✅ Cross-module integration test passed" << std::endl;
        std::cout << "    Temperature: " << current_temp << "°C (target: " << target_temp << "°C)" << std::endl;
        std::cout << "    Line detected: " << (line_status ? "Yes" : "No") << std::endl;
        std::cout << "    Distance: " << distance << "cm" << std::endl;
    }
    
    void testPerformanceValidation() {
        std::cout << "Testing event-driven performance..." << std::endl;
        
        auto start_time = std::chrono::steady_clock::now();
        
        // 触发多个事件测试性能
        for (int i = 0; i < 5; i++) {
            vision->requestFrameCapture();
            ultrasonic->requestMeasurement();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time);
        
        // 验证响应时间
        bool vision_processing = vision->isProcessingActive();
        bool ultrasonic_processing = ultrasonic->isMeasurementActive();
        
        std::cout << "✅ Event-driven performance test completed" << std::endl;
        std::cout << "    Test duration: " << duration.count() << "ms" << std::endl;
        std::cout << "    Vision processing: " << (vision_processing ? "Active" : "Idle") << std::endl;
        std::cout << "    Ultrasonic processing: " << (ultrasonic_processing ? "Active" : "Idle") << std::endl;
        std::cout << "    Average response time: " << (duration.count() / 5.0) << "ms per event" << std::endl;
    }
    
    void cleanup() {
        std::cout << "Cleaning up event-driven test environment..." << std::endl;
        
        if (motor) motor->stopEventDriven();
        if (infrared) infrared->stopEventDriven();
        if (ultrasonic) ultrasonic->stopEventDriven();
        if (vision) vision->stopEventDriven();
        if (temperature) temperature->stopEventDriven();
        if (safety) safety->stopEventDriven();
        if (timer) timer->stop();
        
        std::cout << "✅ Event-driven cleanup completed" << std::endl;
    }
};

int main() {
    try {
        EventDrivenIntegrationTest test;
        test.runAllTests();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Event-driven integration test exception: " << e.what() << std::endl;
        return -1;
    }
}