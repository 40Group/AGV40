#include "InfraredSensor.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Smart Medical Car - Infrared Sensor Test" << std::endl;
    std::cout << "========================================" << std::endl;
    
    InfraredSensor irSensor;
    
    // 初始化测试
    std::cout << "\n1. Testing Infrared Sensor Initialization..." << std::endl;
    if (!irSensor.initialize()) {
        std::cerr << "❌ IR sensor initialization failed!" << std::endl;
        return -1;
    }
    std::cout << "✅ IR sensors initialized successfully" << std::endl;
    
    // 传感器状态检查
    std::cout << "Sensor running: " << (irSensor.isRunning() ? "YES" : "NO") << std::endl;
    
    // 自检测试
    std::cout << "\n2. Running IR Sensor Self-Test..." << std::endl;
    if (!irSensor.selfTest()) {
        std::cerr << "❌ IR sensor self-test failed!" << std::endl;
        return -1;
    }
    
    // 轮询间隔设置测试
    std::cout << "\n3. Polling Interval Configuration Test" << std::endl;
    std::cout << "Setting polling interval to 100ms..." << std::endl;
    irSensor.setPollingInterval(std::chrono::milliseconds(100));
    
    // 实时障碍物检测测试
    std::cout << "\n4. Real-time Obstacle Detection Test (30 seconds)" << std::endl;
    std::cout << "Move objects near the sensors to test detection..." << std::endl;
    std::cout << "Time\t| Left\t| Right\t| Front\t| Any Obstacle" << std::endl;
    std::cout << "--------|-------|-------|-------|-------------" << std::endl;
    
    auto start_time = std::chrono::steady_clock::now();
    int reading_count = 0;
    int left_detections = 0, right_detections = 0, front_detections = 0;
    
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(30)) {
        reading_count++;
        
        // 获取所有传感器状态
        auto state = irSensor.getAllStates();
        
        // 统计检测次数
        if (state.left) left_detections++;
        if (state.right) right_detections++;
        if (state.front) front_detections++;
        
        // 显示当前状态
        std::cout << std::setw(6) << reading_count << "s\t| "
                  << (state.left ? "DETECT" : "CLEAR") << "\t| "
                  << (state.right ? "DETECT" : "CLEAR") << "\t| "
                  << (state.front ? "DETECT" : "CLEAR") << "\t| "
                  << (irSensor.isAnyObstacle() ? "YES" : "NO") << std::endl;
        
        // 特殊提示
        if (irSensor.isAnyObstacle()) {
            std::cout << "⚠️  OBSTACLE DETECTED - Vehicle should stop!" << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    // 传感器组合测试
    std::cout << "\n5. Sensor Combination Test" << std::endl;
    auto final_state = irSensor.getAllStates();
    
    std::cout << "Individual sensor states:" << std::endl;
    std::cout << "  Left sensor: " << (final_state.left ? "OBSTACLE" : "CLEAR") << std::endl;
    std::cout << "  Right sensor: " << (final_state.right ? "OBSTACLE" : "CLEAR") << std::endl;
    std::cout << "  Front sensor: " << (final_state.front ? "OBSTACLE" : "CLEAR") << std::endl;
    
    std::cout << "Combined detection: " << (irSensor.isAnyObstacle() ? "OBSTACLE PRESENT" : "PATH CLEAR") << std::endl;
    
    // 性能测试
    std::cout << "\n6. Performance and Timing Test" << std::endl;
    std::cout << "Testing sensor response time..." << std::endl;
    
    auto perf_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 100; i++) {
        irSensor.getAllStates();
    }
    auto perf_end = std::chrono::high_resolution_clock::now();
    
    auto avg_time = std::chrono::duration_cast<std::chrono::microseconds>(
        perf_end - perf_start).count() / 100.0;
    
    std::cout << "Average sensor read time: " << avg_time << " μs" << std::endl;
    std::cout << "Performance: " << (avg_time < 1000 ? "EXCELLENT" : "ACCEPTABLE") << std::endl;
    
    // 高频轮询测试
    std::cout << "\n7. High-Frequency Polling Test" << std::endl;
    std::cout << "Setting high-frequency polling (20ms)..." << std::endl;
    irSensor.setPollingInterval(std::chrono::milliseconds(20));
    
    std::cout << "Monitoring for 10 seconds..." << std::endl;
    start_time = std::chrono::steady_clock::now();
    int rapid_readings = 0;
    
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(10)) {
        bool any_obstacle = irSensor.isAnyObstacle();
        rapid_readings++;
        
        if (rapid_readings % 50 == 0) {  // 每秒输出一次
            std::cout << "Rapid reading " << rapid_readings 
                      << ": " << (any_obstacle ? "OBSTACLE" : "CLEAR") << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    std::cout << "Total rapid readings: " << rapid_readings << std::endl;
    std::cout << "Effective sampling rate: " << rapid_readings / 10.0 << " Hz" << std::endl;
    
    // 统计结果
    std::cout << "\n8. Detection Statistics" << std::endl;
    std::cout << "Total test duration: 30 seconds" << std::endl;
    std::cout << "Total readings: " << reading_count << std::endl;
    std::cout << "Left sensor detections: " << left_detections 
              << " (" << (100.0 * left_detections / reading_count) << "%)" << std::endl;
    std::cout << "Right sensor detections: " << right_detections 
              << " (" << (100.0 * right_detections / reading_count) << "%)" << std::endl;
    std::cout << "Front sensor detections: " << front_detections 
              << " (" << (100.0 * front_detections / reading_count) << "%)" << std::endl;
    
    // 清理
    irSensor.shutdown();
    std::cout << "\n✅ IR sensor test completed successfully!" << std::endl;
    return 0;
}