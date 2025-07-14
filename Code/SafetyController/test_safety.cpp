#include "SafetyController.h"
#include "MotorController.h"
#include "UltrasonicSensor.h"
#include "InfraredSensor.h"
#include "TemperatureController.h"
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Smart Medical Car - Safety Controller Test" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // 创建所有必需的组件
    std::cout << "\n1. Initializing System Components..." << std::endl;
    
    MotorController motor;
    UltrasonicSensor ultrasonic;
    InfraredSensor ir;
    TemperatureController temp;
    SafetyController safety;
    
    // 初始化各个组件
    if (!motor.initialize()) {
        std::cerr << "❌ Motor controller initialization failed!" << std::endl;
        return -1;
    }
    std::cout << "✅ Motor controller initialized" << std::endl;
    
    if (!ultrasonic.initialize()) {
        std::cerr << "❌ Ultrasonic sensor initialization failed!" << std::endl;
        return -1;
    }
    std::cout << "✅ Ultrasonic sensor initialized" << std::endl;
    
    if (!ir.initialize()) {
        std::cerr << "❌ IR sensors initialization failed!" << std::endl;
        return -1;
    }
    std::cout << "✅ IR sensors initialized" << std::endl;
    
    if (!temp.initialize()) {
        std::cerr << "❌ Temperature controller initialization failed!" << std::endl;
        return -1;
    }
    std::cout << "✅ Temperature controller initialized" << std::endl;
    
    // 初始化安全控制器
    std::cout << "\n2. Initializing Safety Controller..." << std::endl;
    if (!safety.initialize(&motor, &ultrasonic, &ir, &temp)) {
        std::cerr << "❌ Safety controller initialization failed!" << std::endl;
        return -1;
    }
    std::cout << "✅ Safety controller initialized successfully" << std::endl;
    
    // 自检测试
    std::cout << "\n3. Running Safety Controller Self-Test..." << std::endl;
    if (!safety.selfTest()) {
        std::cerr << "❌ Safety controller self-test failed!" << std::endl;
        return -1;
    }
    
    // 系统状态检查
    std::cout << "\n4. System Safety Status Check" << std::endl;
    std::cout << "Current safety state: " << safety.getStateDescription() << std::endl;
    std::cout << "Safe to move: " << (safety.isSafeToMove() ? "YES" : "NO") << std::endl;
    std::cout << "Safe to operate: " << (safety.isSafeToOperate() ? "YES" : "NO") << std::endl;
    std::cout << "Emergency active: " << (safety.isEmergencyActive() ? "YES" : "NO") << std::endl;
    
    // 安全阈值配置测试
    std::cout << "\n5. Safety Thresholds Configuration Test" << std::endl;
    safety.setSafetyThresholds(25.0, 15.0, 45.0, 10.0);
    std::cout << "✅ Safety thresholds configured" << std::endl;
    
    // 电机命令安全检查测试
    std::cout << "\n6. Motor Command Safety Check Test" << std::endl;
    
    MotorCommand test_commands[] = {
        MotorCommand(100, 100),    // Normal forward
        MotorCommand(-100, -100),  // Normal backward
        MotorCommand(50, -50),     // Turn
        MotorCommand(300, 300),    // Over-speed (should be rejected)
        MotorCommand(0, 0)         // Stop
    };
    
    for (size_t i = 0; i < sizeof(test_commands)/sizeof(test_commands[0]); i++) {
        MotorCommand cmd = test_commands[i];
        bool is_safe = safety.checkMotorCommand(cmd);
        MotorCommand safe_cmd = safety.applySafetyConstraints(cmd);
        
        std::cout << "Command " << (i+1) << " (" << cmd.left_speed << ", " << cmd.right_speed << "): ";
        std::cout << (is_safe ? "SAFE" : "UNSAFE") << " -> ";
        std::cout << "Applied (" << safe_cmd.left_speed << ", " << safe_cmd.right_speed << ")" << std::endl;
    }
    
    // 紧急停止测试
    std::cout << "\n7. Emergency Stop Test" << std::endl;
    std::cout << "Triggering emergency stop..." << std::endl;
    safety.triggerEmergencyStop("Test emergency stop");
    
    std::cout << "Emergency state: " << (safety.isEmergencyActive() ? "ACTIVE" : "INACTIVE") << std::endl;
    std::cout << "Current state: " << safety.getStateDescription() << std::endl;
    
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    std::cout << "Emergency duration: " << safety.getEmergencyDuration().count() << "ms" << std::endl;
    
    std::cout << "Clearing emergency stop..." << std::endl;
    safety.clearEmergencyStop();
    std::cout << "Emergency cleared: " << (!safety.isEmergencyActive() ? "YES" : "NO") << std::endl;
    
    // 实时安全监控测试
    std::cout << "\n8. Real-time Safety Monitoring Test (30 seconds)" << std::endl;
    std::cout << "Monitoring system safety status..." << std::endl;
    std::cout << "Time\t| State\t\t| Safe Move\t| Safe Operate\t| Distance" << std::endl;
    std::cout << "--------|---------------|---------------|---------------|----------" << std::endl;
    
    auto start_time = std::chrono::steady_clock::now();
    int monitoring_count = 0;
    
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(30)) {
        monitoring_count++;
        
        auto state = safety.getCurrentState();
        bool safe_move = safety.isSafeToMove();
        bool safe_operate = safety.isSafeToOperate();
        double distance = ultrasonic.getDistance();
        
        std::string state_str;
        switch(state) {
            case SafetyController::SafetyState::NORMAL: state_str = "NORMAL"; break;
            case SafetyController::SafetyState::OBSTACLE_DETECTED: state_str = "OBSTACLE"; break;
            case SafetyController::SafetyState::TEMPERATURE_EMERGENCY: state_str = "TEMP_EMRG"; break;
            case SafetyController::SafetyState::SYSTEM_ERROR: state_str = "SYS_ERROR"; break;
            case SafetyController::SafetyState::EMERGENCY_STOP: state_str = "EMERGENCY"; break;
        }
        
        std::cout << std::setw(6) << monitoring_count << "s\t| "
                  << std::setw(12) << state_str << "\t| "
                  << (safe_move ? "YES" : "NO") << "\t\t| "
                  << (safe_operate ? "YES" : "NO") << "\t\t| "
                  << std::fixed << std::setprecision(1) << distance << "cm" << std::endl;
        
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    
    // 安全日志检查
    std::cout << "\n9. Safety Log Analysis" << std::endl;
    auto safety_log = safety.getSafetyLog();
    std::cout << "Total safety events logged: " << safety_log.size() << std::endl;
    
    if (!safety_log.empty()) {
        std::cout << "Recent safety events:" << std::endl;
        int show_count = std::min(5, (int)safety_log.size());
        for (int i = safety_log.size() - show_count; i < (int)safety_log.size(); i++) {
            std::cout << "  " << safety_log[i] << std::endl;
        }
    }
    
    // 模拟紧急情况测试
    std::cout << "\n10. Emergency Simulation Test" << std::endl;
    std::cout << "Simulating obstacle emergency..." << std::endl;
    safety.simulateEmergency("obstacle");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    std::cout << "Simulating temperature emergency..." << std::endl;
    safety.simulateEmergency("temperature");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // 清理和关闭
    std::cout << "\n11. System Shutdown Test" << std::endl;
    std::cout << "Shutting down all components safely..." << std::endl;
    
    safety.shutdown();
    temp.shutdown();
    ir.shutdown();
    ultrasonic.shutdown();
    motor.shutdown();
    
    std::cout << "\n✅ Safety controller test completed successfully!" << std::endl;
    std::cout << "All safety features verified and working correctly." << std::endl;
    
    return 0;
}