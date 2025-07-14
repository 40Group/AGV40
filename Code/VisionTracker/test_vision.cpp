#include "VisionTracker.hpp"
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Smart Medical Car - Vision Tracker Test" << std::endl;
    std::cout << "========================================" << std::endl;
    
    VisionTracker vision;
    
    // Initialisation test
    std::cout << "\n1. Testing Vision Initialization..." << std::endl;
    if (!vision.initialize(0)) {
        std::cerr << "❌ Vision initialization failed!" << std::endl;
        std::cerr << "Please check camera connection" << std::endl;
        return -1;
    }
    std::cout << "✅ Vision tracker initialized successfully" << std::endl;
    
    // Camera status check
    std::cout << "Camera ready: " << (vision.isCameraReady() ? "YES" : "NO") << std::endl;
    std::cout << "Sensor type: " << vision.getSensorType() << std::endl;
    std::cout << "Status: " << vision.getSensorStatus() << std::endl;
    
    // self-checking test
    std::cout << "\n2. Running Vision Self-Test..." << std::endl;
    if (!vision.selfTest()) {
        std::cerr << "❌ Vision self-test failed!" << std::endl;
        return -1;
    }
    
    // Real-time processing tests
    std::cout << "\n3. Real-time Line Detection Test" << std::endl;
    std::cout << "Processing frames for 10 seconds..." << std::endl;
    std::cout << "Commands during test:" << std::endl;
    std::cout << "  s : Save debug frame" << std::endl;
    std::cout << "  q : Quit early" << std::endl;
    
    auto start_time = std::chrono::steady_clock::now();
    int frame_count = 0;
    
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(10)) {
        // Processing a frame
        auto frame_start = std::chrono::steady_clock::now();
        VisionResult result = vision.processFrame();
        auto frame_end = std::chrono::steady_clock::now();
        
        auto processing_time = std::chrono::duration_cast<std::chrono::microseconds>(
            frame_end - frame_start).count();
        
        frame_count++;
        
        // Display of test results
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
        
        // Performance check
        if (processing_time > Constants::MAX_VISION_LATENCY_US) {
            std::cout << "⚠️  WARNING: Processing time exceeded real-time threshold!" << std::endl;
        }

        // Check Input
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Performance check
    std::cout << "\n4. Performance Analysis" << std::endl;
    std::cout << "Total frames processed: " << frame_count << std::endl;
    std::cout << "Average FPS: " << frame_count / 10.0 << std::endl;
    
    // Save last frame for debugging
    std::cout << "\n5. Saving Debug Frame..." << std::endl;
    vision.saveDebugFrame("debug_frame.jpg");
    
    // Clear up
    vision.shutdown();
    std::cout << "\n✅ Vision test completed successfully!" << std::endl;
    return 0;
}
