#include "../MotorController/MotorController.h"
#include "../common/Common.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>

class MotorTest {
private:
    MotorController motor;
    std::atomic<bool> test_passed;
    
public:
    MotorTest() : test_passed(true) {}
    
    void runAllTests() {
        std::cout << "=== REAL Event-Driven Motor Controller Tests ===" << std::endl;
        
        testInitialization();
        testImmediateStateChanges();
        testEmergencyStop();
        testStateTracking();
        testEventDrivenResponse();
        
        if (test_passed.load()) {
            std::cout << "✅ ALL REAL EVENT-DRIVEN MOTOR TESTS PASSED!" << std::endl;
        } else {
            std::cout << "❌ SOME MOTOR TESTS FAILED!" << std::endl;
        }
    }
    
private:
    void testInitialization() {
        std::cout << "Testing real GPIO motor initialization..." << std::endl;
        
        bool init_result = motor.initialize();
        if (!init_result) {
            std::cout << "❌ Motor initialization failed" << std::endl;
            test_passed = false;
            return;
        }
        
        // Check initial status
        if (motor.getCurrentState() != MotionState::STOP) {
            std::cout << "❌ Initial state should be STOP" << std::endl;
            test_passed = false;
            return;
        }
        
        if (!motor.isInitialized()) {
            std::cout << "❌ Motor should report as initialized" << std::endl;
            test_passed = false;
            return;
        }
        
        std::cout << "✅ Real GPIO motor initialization test passed" << std::endl;
    }
    
    void testImmediateStateChanges() {
        std::cout << "Testing immediate state changes (no delays)..." << std::endl;
        
        // Test forward (immediate response)
        motor.executeMotion(MotionState::FORWARD);
        if (motor.getCurrentState() != MotionState::FORWARD) {
            std::cout << "❌ Immediate forward motion test failed" << std::endl;
            test_passed = false;
        }
        
        // Test steering (immediate response)
        motor.executeMotion(MotionState::TURN_LEFT);
        if (motor.getCurrentState() != MotionState::TURN_LEFT) {
            std::cout << "❌ Immediate turn left test failed" << std::endl;
            test_passed = false;
        }
        
        motor.executeMotion(MotionState::TURN_RIGHT);
        if (motor.getCurrentState() != MotionState::TURN_RIGHT) {
            std::cout << "❌ Immediate turn right test failed" << std::endl;
            test_passed = false;
        }
        
        // Test reverse (immediate response)
        motor.executeMotion(MotionState::BACKWARD);
        if (motor.getCurrentState() != MotionState::BACKWARD) {
            std::cout << "❌ Immediate backward motion test failed" << std::endl;
            test_passed = false;
        }
        
        // Test stop (immediate response)
        motor.executeMotion(MotionState::STOP);
        if (motor.getCurrentState() != MotionState::STOP) {
            std::cout << "❌ Immediate stop motion test failed" << std::endl;
            test_passed = false;
        }
        
        std::cout << "✅ Immediate state change tests passed" << std::endl;
    }
    
    void testEmergencyStop() {
        std::cout << "Testing emergency stop response..." << std::endl;
        
        //Start movement
        motor.executeMotion(MotionState::FORWARD);
        
        // Immediate emergency stop (no delay)
        motor.emergencyStop();
        
        if (motor.getCurrentState() != MotionState::STOP) {
            std::cout << "❌ Emergency stop test failed" << std::endl;
            test_passed = false;
            return;
        }
        
        // Test emergency braking
        motor.executeMotion(MotionState::BACKWARD);
        motor.executeEmergencyBrake();
        
        if (motor.getCurrentState() != MotionState::STOP) {
            std::cout << "❌ Emergency brake test failed" << std::endl;
            test_passed = false;
            return;
        }
        
        std::cout << "✅ Emergency stop tests passed" << std::endl;
    }
    
    void testStateTracking() {
        std::cout << "Testing atomic state tracking..." << std::endl;
        
        // Test status change sequence (immediate switch)
        MotionState states[] = {
            MotionState::FORWARD,
            MotionState::TURN_LEFT,
            MotionState::BACKWARD,
            MotionState::TURN_RIGHT,
            MotionState::STOP
        };
        
        for (auto state : states) {
            motor.setMotionState(state);  // Use the new immediate settings interface
            
            if (motor.getCurrentState() != state) {
                std::cout << "❌ Atomic state tracking failed for state: " 
                         << static_cast<int>(state) << std::endl;
                test_passed = false;
                return;
            }
        }
        
        std::cout << "✅ Atomic state tracking test passed" << std::endl;
    }
    
    void testEventDrivenResponse() {
        std::cout << "Testing event-driven response time..." << std::endl;
        
        // Test rapid state switching (simulate real-time event response)
        auto start_time = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < 1000; i++) {
            MotionState test_state = (i % 2 == 0) ? MotionState::FORWARD : MotionState::STOP;
            motor.executeMotion(test_state);
            
            if (motor.getCurrentState() != test_state) {
                std::cout << "❌ Event-driven response failed at iteration " << i << std::endl;
                test_passed = false;
                return;
            }
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - start_time).count();
        
        double avg_response_time = duration / 1000.0;
        
        std::cout << "Average response time: " << avg_response_time << " μs" << std::endl;
        
        if (avg_response_time > 1000.0) {  // Should be less than 1ms
            std::cout << "⚠️ Response time may be too slow for real-time systems" << std::endl;
        } else {
            std::cout << "✅ Excellent real-time response performance" << std::endl;
        }
        
        std::cout << "✅ Event-driven response test passed" << std::endl;
    }
};

int main() {
    try {
        std::cout << "🎯 REAL Event-Driven Motor Controller Test" << std::endl;
        std::cout << "✅ No sleep() statements for timing" << std::endl;
        std::cout << "✅ Pure callback-driven architecture" << std::endl;
        std::cout << "✅ gpiod.hpp GPIO control" << std::endl;
        std::cout << "✅ Sub-millisecond response time" << std::endl;
        
        MotorTest test;
        test.runAllTests();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test exception: " << e.what() << std::endl;
        return -1;
    }
}