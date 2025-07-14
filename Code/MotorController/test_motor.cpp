#include "MotorController.hpp"
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Smart Medical Car - Motor Controller Test" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // Create a motor controller
    MotorController motor;
    
    // Initialization test
    std::cout << "\n1. Testing Motor Initialization..." << std::endl;
    if (!motor.initialize()) {
        std::cerr << "❌ Motor initialization failed!" << std::endl;
        return -1;
    }
    std::cout << "✅ Motor initialized successfully" << std::endl;
    
    // self-testing
    std::cout << "\n2. Running Motor Self-Test..." << std::endl;
    if (!motor.selfTest()) {
        std::cerr << "❌ Motor self-test failed!" << std::endl;
        return -1;
    }
    
    // Interactive testing
    std::cout << "\n3. Interactive Motor Control Test" << std::endl;
    std::cout << "Commands:" << std::endl;
    std::cout << "  f : Move Forward (speed 100)" << std::endl;
    std::cout << "  b : Move Backward (speed 100)" << std::endl;
    std::cout << "  l : Turn Left (speed 80)" << std::endl;
    std::cout << "  r : Turn Right (speed 80)" << std::endl;
    std::cout << "  s : Stop" << std::endl;
    std::cout << "  e : Emergency Stop Test" << std::endl;
    std::cout << "  t : Smooth Turn Test" << std::endl;
    std::cout << "  d : Differential Speed Test" << std::endl;
    std::cout << "  q : Quit" << std::endl;
    std::cout << "Enter command: ";

    char command;
    while (std::cin >> command && command != 'q') {
        switch(command) {
            case 'f':
                std::cout << "Moving forward..." << std::endl;
                motor.moveForward(100);
                break;
            case 'b':
                std::cout << "Moving backward..." << std::endl;
                motor.moveBackward(100);
                break;
            case 'l':
                std::cout << "Turning left..." << std::endl;
                motor.turnLeft(80);
                break;
            case 'r':
                std::cout << "Turning right..." << std::endl;
                motor.turnRight(80);
                break;
            case 's':
                std::cout << "Stopping..." << std::endl;
                motor.stop();
                break;
            case 'e':
                std::cout << "Testing emergency stop..." << std::endl;
                motor.emergencyStop();
                std::this_thread::sleep_for(std::chrono::seconds(2));
                std::cout << "Resuming from emergency..." << std::endl;
                motor.resumeFromEmergency();
                break;
            case 't':
                std::cout << "Testing smooth turns..." << std::endl;
                for (double ratio = -1.0; ratio <= 1.0; ratio += 0.2) {
                    std::cout << "Turn ratio: " << ratio << std::endl;
                    motor.smoothTurn(ratio);
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
                motor.stop();
                break;
            case 'd':
                std::cout << "Testing differential speeds..." << std::endl;
                motor.setDifferentialSpeed(100, 50);
                std::this_thread::sleep_for(std::chrono::seconds(2));
                motor.setDifferentialSpeed(50, 100);
                std::this_thread::sleep_for(std::chrono::seconds(2));
                motor.stop();
                break;
            default:
                std::cout << "Unknown command" << std::endl;
        }
        
        // Show current status
        std::cout << "Motor status: " 
                  << (motor.isEmergencyStopped() ? "EMERGENCY STOPPED" : "NORMAL")
                  << " | Running: " << (motor.isRunning() ? "YES" : "NO") << std::endl;
        std::cout << "Enter command: ";
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // clean up
    motor.shutdown();
    std::cout << "\n✅ Motor test completed successfully!" << std::endl;
    return 0;
}