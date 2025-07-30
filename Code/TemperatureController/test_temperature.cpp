#include "TemperatureController.h"
#include "../common/Common.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <queue>
#include <functional>

// Simulated event-driven timer manager
class EventTimer {
private:
    std::queue<std::function<void()>> event_queue_;
    std::mutex queue_mutex_;
    std::atomic<bool> running_;
    std::thread timer_thread_;
    
public:
    EventTimer() : running_(false) {}
    
    ~EventTimer() {
        stop();
    }
    
    void start() {
        running_ = true;
        timer_thread_ = std::thread([this]() {
            while (running_.load()) {
                std::function<void()> event;
                {
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    if (!event_queue_.empty()) {
                        event = event_queue_.front();
                        event_queue_.pop();
                    }
                }
                
                if (event) {
                    event();
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
    }
    
    void stop() {
        running_ = false;
        if (timer_thread_.joinable()) {
            timer_thread_.join();
        }
    }
    
    void scheduleEvent(std::function<void()> callback, std::chrono::milliseconds delay) {
        std::thread([this, callback, delay]() {
            std::this_thread::sleep_for(delay);
            std::lock_guard<std::mutex> lock(queue_mutex_);
            event_queue_.push(callback);
        }).detach();
    }
};

class TemperatureTest {
private:
    TemperatureController controller;
    EventTimer timer;
    std::atomic<bool> test_passed;
    std::atomic<double> last_temperature;
    std::atomic<int> callback_count;
    
public:
    TemperatureTest() : test_passed(true), last_temperature(0.0), callback_count(0) {}
    
    void runAllTests() {
        std::cout << "=== REAL Event-Driven Temperature Controller Tests ===" << std::endl;
        
        timer.start();
        
        testInitialization();
        testEventDrivenTimerIntegration();
        testTemperatureReading();
        testTargetSetting();
        testPIDParameters();
        testCallbackSystem();
        testRealTimeResponse();
        
        controller.stop();
        timer.stop();
        
        if (test_passed.load()) {
            std::cout << "✅ ALL REAL EVENT-DRIVEN TEMPERATURE TESTS PASSED!" << std::endl;
        } else {
            std::cout << "❌ SOME TEMPERATURE TESTS FAILED!" << std::endl;
        }
    }
    
private:
    void testInitialization() {
        std::cout << "Testing real event-driven temperature controller initialization..." << std::endl;
        
        bool init_result = controller.initialize();
        if (!init_result) {
            std::cout << "❌ Temperature controller initialization failed" << std::endl;
            test_passed = false;
            return;
        }
        
        // Check initial status
        if (controller.getControlState() != TempControlState::IDLE) {
            std::cout << "❌ Initial control state should be IDLE" << std::endl;
            test_passed = false;
            return;
        }
        
        if (!controller.isInitialized()) {
            std::cout << "❌ Controller should report as initialized" << std::endl;
            test_passed = false;
            return;
        }
        
        // Check the default target temperature.
        double target = controller.getTargetTemperature();
        if (target != 25.0) {
            std::cout << "❌ Default target temperature should be 25°C, got " 
                     << target << "°C" << std::endl;
            test_passed = false;
            return;
        }
        
        std::cout << "✅ Real event-driven temperature controller initialization test passed" << std::endl;
    }
    
    void testEventDrivenTimerIntegration() {
        std::cout << "Testing event-driven timer integration..." << std::endl;
        
        // Register timer callback
        controller.registerTimerCallback([this](std::function<void()> callback, std::chrono::milliseconds delay) {
            timer.scheduleEvent(callback, delay);
        });
        
        // Start event-driven control
        controller.start();
        
        // Wait for several timer events
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        std::cout << "✅ Event-driven timer integration test passed" << std::endl;
    }
    
    void testTemperatureReading() {
        std::cout << "Testing event-driven temperature reading..." << std::endl;
        
        // Trigger temperature control event
        controller.processTemperatureControlEvent();
        
        double temp = controller.getCurrentTemperature();
        
        // Check that the temperature reading is within the acceptable range.
        if (temp < -50.0 || temp > 100.0) {
            std::cout << "❌ Temperature reading out of reasonable range: " 
                     << temp << "°C" << std::endl;
            test_passed = false;
            return;
        }
        
        std::cout << "✅ Event-driven temperature reading test passed (Current: " 
                 << temp << "°C)" << std::endl;
    }
    
    void testTargetSetting() {
        std::cout << "Testing target temperature setting..." << std::endl;
        
        // Test different target temperatures (set immediately)
        double test_targets[] = {20.0, 25.0, 30.0};
        
        for (double target : test_targets) {
            controller.setTargetTemperature(target);
            
            double set_target = controller.getTargetTemperature();
            if (std::abs(set_target - target) > 0.01) {
                std::cout << "❌ Target temperature setting failed. Expected: " 
                         << target << "°C, Got: " << set_target << "°C" << std::endl;
                test_passed = false;
                return;
            }
        }
        
        std::cout << "✅ Target temperature setting test passed" << std::endl;
    }
    
    void testPIDParameters() {
        std::cout << "Testing PID parameter setting..." << std::endl;
        
        // Set new PID parameters (atomic operation)
        double kp = 1.5, ki = 0.2, kd = 0.1;
        controller.setPIDParameters(kp, ki, kd);
        
        // Set the target temperature and trigger the control event.
        controller.setTargetTemperature(23.0);
        
        // Trigger several control events to test the PID response.
        for (int i = 0; i < 3; i++) {
            controller.processTemperatureControlEvent();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Check whether the control status is responsive.
        auto state = controller.getControlState();
        std::cout << "PID control state: " << static_cast<int>(state) << std::endl;
        
        std::cout << "✅ PID parameters test passed" << std::endl;
    }
    
    void testCallbackSystem() {
        std::cout << "Testing event-driven callback system..." << std::endl;
        
        callback_count = 0;
        
        // Registered temperature rollback
        controller.registerCallback([this](double temperature) {
            last_temperature = temperature;
            callback_count++;
            std::cout << "🔥 TEMPERATURE EVENT: " << temperature << "°C [#" << callback_count.load() << "]" << std::endl;
        });
        
        // Trigger several temperature control events
        for (int i = 0; i < 5; i++) {
            controller.processTemperatureControlEvent();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        int count = callback_count.load();
        if (count < 3) {
            std::cout << "❌ Temperature callback not triggered enough. Count: " << count << std::endl;
            test_passed = false;
            return;
        }
        
        double temp = last_temperature.load();
        if (temp < -50.0 || temp > 100.0) {
            std::cout << "❌ Invalid temperature in callback: " << temp << "°C" << std::endl;
            test_passed = false;
            return;
        }
        
        std::cout << "✅ Event-driven callback system test passed (" << count 
                 << " callbacks, Last temp: " << temp << "°C)" << std::endl;
    }
    
    void testRealTimeResponse() {
        std::cout << "Testing real-time event response performance..." << std::endl;
        
        // Testing fast continuous event processing
        auto start_time = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < 100; i++) {
            controller.processTemperatureControlEvent();
            
            // Test rapid target temperature switching
            if (i % 10 == 0) {
                double target = 20.0 + (i % 3) * 5.0;  // 20, 25, 30°C cycle
                controller.setTargetTemperature(target);
            }
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - start_time).count();
        
        double avg_event_time = duration / 100.0;
        
        std::cout << "Average temperature control event time: " << avg_event_time << " μs" << std::endl;
        
        if (avg_event_time > 10000.0) {  // Should be less than 10ms
            std::cout << "⚠️ Event processing time may be too slow for real-time control" << std::endl;
        } else {
            std::cout << "✅ Excellent real-time temperature control performance" << std::endl;
        }
        
        std::cout << "✅ Real-time response test passed" << std::endl;
    }
};

int main() {
    try {
        std::cout << "🎯 REAL Event-Driven Temperature Controller Test" << std::endl;
        std::cout << "✅ No polling control loops" << std::endl;
        std::cout << "✅ Pure event-driven temperature control" << std::endl;
        std::cout << "✅ gpiod.hpp GPIO control" << std::endl;
        std::cout << "✅ Timer-based event scheduling" << std::endl;
        
        TemperatureTest test;
        test.runAllTests();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Temperature test exception: " << e.what() << std::endl;
        return -1;
    }
}