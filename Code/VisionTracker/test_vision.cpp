#include "VisionTracker.hpp"
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Smart Medical Car - Vision Tracker Test" << std::endl;
    std::cout << "========================================" << std::endl;
    
    VisionTracker vision;
    
    // 初始化测试
    std::cout << "\n1. Testing Vision Initialization..." << std::endl;
    if (!vision.initialize(0)) {
        std::cerr << "❌ Vision initialization failed!" << std::endl;
        std::cerr << "Please check camera connection" << std::endl;
        return -1;
    }
    std::cout << "✅ Vision tracker initialized successfully" << std::endl;
    
    // 摄像头状态检查
    std::cout << "Camera ready: " << (vision.isCameraReady() ? "YES" : "NO") << std::endl;
    std::cout << "Sensor type: " << vision.getSensorType() << std::endl;
    std::cout << "Status: " << vision.getSensorStatus() << std::endl;
    
    // 自检测试
    std::cout << "\n2. Running Vision Self-Test..." << std::endl;
    if (!vision.selfTest()) {
        std::cerr << "❌ Vision self-test failed!" << std::endl;
        return -1;
    }
    
    // 实时处理测试
    std::cout << "\n3. Real-time Line Detection Test" << std::endl;
    std::cout << "Processing frames for 10 seconds..." << std::endl;
    std::cout << "Commands during test:" << std::endl;
    std::cout << "  s : Save debug frame" << std::endl;
    std::cout << "  q : Quit early" << std::endl;
    
    auto start_time = std::chrono::steady_clock::now();
    int frame_count = 0;
    
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(10)) {
        // 处理一帧
        auto frame_start = std::chrono::steady_clock::now();
        VisionResult result = vision.processFrame();
        auto frame_end = std::chrono::steady_clock::now();
        
        auto processing_time = std::chrono::duration_cast<std::chrono::microseconds>(
            frame_end - frame_start).count();
        
        frame_count++;
        
        // 显示检测结果
        std::cout << "Frame " << frame_count << ": ";
        if (result.line_detected) {
            std::cout << "Line detected | "
                      << "Offset: " << std::fixed << std::setprecision(3) << result.line_offset
                      << " | Angle: " << result.line_angle
                      << " | Confidence: " << result.confidence
                      << " | Processing: " << processing_time << "μs";
        } else {
            std::cout << "No line detected | Processing: " << processing_time << "μs";
        }
        std::cout << std::endl;
        
        // 性能检查
        if (processing_time > Constants::MAX_VISION_LATENCY_US) {
            std::cout << "⚠️  WARNING: Processing time exceeded real-time threshold!" << std::endl;
        }
        
        // 检查用户输入
        // 简化版本，实际可以用非阻塞输入
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // 性能统计
    std::cout << "\n4. Performance Analysis" << std::endl;
    std::cout << "Total frames processed: " << frame_count << std::endl;
    std::cout << "Average FPS: " << frame_count / 10.0 << std::endl;
    
    // 保存最后一帧用于调试
    std::cout << "\n5. Saving Debug Frame..." << std::endl;
    vision.saveDebugFrame("debug_frame.jpg");
    
    // 清理
    vision.shutdown();
    std::cout << "\n✅ Vision test completed successfully!" << std::endl;
    return 0;
}