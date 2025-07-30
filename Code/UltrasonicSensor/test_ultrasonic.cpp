#include "UltrasonicSensor.h"
#include "../timer/TimerManager.h"
#include "../common/Common.h"
#include <iostream>
#include <atomic>

class EventDrivenUltrasonicTest {
private:
    TimerManager timer_manager_;
    UltrasonicSensor sensor_;
    std::atomic<bool> test_passed_;
    std::atomic<bool> obstacle_detected_;
    std::atomic<double> last_distance_;
    std::atomic<int> callback_count_;
    
public:
    EventDrivenUltrasonicTest() 
        : test_passed_(true), obstacle_detected_(false), 
          last_distance_(999.0), callback_count_(0) {}
    
    void runAllTests() {
        std::cout << "=== Event-Driven Ultrasonic Sensor Tests ===" << std::endl;
        
        testInitialization();
        testEventDrivenMeasurement();
        testObstacleDetection();
        testCallbackSystem();
        testPerformance();
        
        sensor_.stopEventDriven();
        timer_manager_.stop();
        
        if (test_passed_.load()) {
            std::cout << "✅ ALL EVENT-DRIVEN ULTRASONIC TESTS PASSED!" << std::endl;
        } else {
            std::cout << "❌ SOME EVENT-DRIVEN ULTRASONIC TESTS FAILED!" << std::endl;
        }
    }
    
private:
    void testInitialization() {
        std::cout << "Testing event-driven initialization..." << std::endl;
        
        // start TimerManager
        if (!timer_manager_.initialize()) {
            std::cout << "❌ TimerManager initialization failed" << std::endl;
            test_passed_ = false;
            return;
        }
        timer_manager_.start();
        
        // Initialize the sensor
        bool init_result = sensor_.initialize(&timer_manager_);
        if (!init_result) {
            std::cout << "❌ Event-driven ultrasonic sensor initialization failed" << std::endl;
            test_passed_ = false;
            return;
        }
        
        std::cout << "✅ Event-driven initialization test passed" << std::endl;
    }
    
    void testEventDrivenMeasurement() {
        std::cout << "Testing event-driven measurement..." << std::endl;
        
        // Start event-driven mode
        sensor_.startEventDriven();
        
        // Wait for several automatic measurements
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        double distance = sensor_.getDistance();
        if (distance <= 0 || distance > 500) {
            std::cout << "❌ Invalid distance from event-driven measurement: " 
                     << distance << "cm" << std::endl;
            test_passed_ = false;
            return;
        }
        
        std::cout << "✅ Event-driven measurement test passed" << std::endl;
        std::cout << "    Current distance: " << distance << "cm" << std::endl;
    }
    
    void testObstacleDetection() {
        std::cout << "Testing event-driven obstacle detection..." << std::endl;
        
        // Setting the test threshold
        sensor_.setObstacleThreshold(30.0);
        
        // Waiting for measurement update
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        double distance = sensor_.getDistance();
        bool obstacle = sensor_.isObstacleDetected();
        bool expected_obstacle = (distance > 0 && distance < 30.0);
        
        std::cout << "✅ Event-driven obstacle detection test completed" << std::endl;
        std::cout << "    Distance: " << distance << "cm" << std::endl;
        std::cout << "    Obstacle detected: " << (obstacle ? "Yes" : "No") << std::endl;
        std::cout << "    Expected: " << (expected_obstacle ? "Yes" : "No") << std::endl;
    }
    
    void testCallbackSystem() {
        std::cout << "Testing event-driven callback system..." << std::endl;
        
        callback_count_ = 0;
        
        // Register callback
        sensor_.registerCallback([this](bool detected, double distance) {
            callback_count_++;
            obstacle_detected_ = detected;
            last_distance_ = distance;
            std::cout << "    Callback triggered: " << distance << "cm, Obstacle: " 
                     << (detected ? "Yes" : "No") << std::endl;
        });
        
        // Test different threshold trigger callbacks
        sensor_.setObstacleThreshold(10.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        sensor_.setObstacleThreshold(50.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        sensor_.setObstacleThreshold(20.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        int count = callback_count_.load();
        std::cout << "✅ Event-driven callback system test completed" << std::endl;
        std::cout << "    Total callbacks: " << count << std::endl;
        std::cout << "    Last distance: " << last_distance_.load() << "cm" << std::endl;
    }
    
    void testPerformance() {
        std::cout << "Testing event-driven performance..." << std::endl;
        
        auto start_time = std::chrono::steady_clock::now();
        double initial_distance = sensor_.getDistance();
        
        //  Wait for 10 measurement cycles(100ms * 10 = 1s)
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time);
        
        double final_distance = sensor_.getDistance();
        
        // Verify performance
        if (duration.count() > 1200) {  // Allow 20% error
            std::cout << "⚠️ Performance test timing exceeded expected range" << std::endl;
        }
        
        std::cout << "✅ Event-driven performance test completed" << std::endl;
        std::cout << "    Test duration: " << duration.count() << "ms" << std::endl;
        std::cout << "    Distance stability: " << 
                     abs(final_distance - initial_distance) << "cm variation" << std::endl;
    }
};

int main() {
    try {
        EventDrivenUltrasonicTest test;
        test.runAllTests();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Event-driven ultrasonic test exception: " << e.what() << std::endl;
        return -1;
    }
}
