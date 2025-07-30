#include "VisionTracker.h"
#include "../timer/TimerManager.h"
#include "../common/Common.h"
#include <iostream>
#include <atomic>

class EventDrivenVisionTest {
private:
    TimerManager timer_manager_;
    VisionTracker tracker_;
    std::atomic<bool> test_passed_;
    std::atomic<bool> line_detected_;
    std::atomic<double> last_deviation_;
    std::atomic<int> callback_count_;
    
public:
    EventDrivenVisionTest() 
        : test_passed_(true), line_detected_(false), 
          last_deviation_(0.0), callback_count_(0) {}
    
    void runAllTests() {
        std::cout << "=== Event-Driven Vision Tracker Tests ===" << std::endl;
        
        testInitialization();
        testEventDrivenCapture();
        testLineDetection();
        testColorConfiguration();
        testCallbackSystem();
        testPerformance();
        
        tracker_.stopEventDriven();
        timer_manager_.stop();
        
        if (test_passed_.load()) {
            std::cout << "✅ ALL EVENT-DRIVEN VISION TESTS PASSED!" << std::endl;
        } else {
            std::cout << "❌ SOME EVENT-DRIVEN VISION TESTS FAILED!" << std::endl;
        }
    }
    
private:
    void testInitialization() {
        std::cout << "Testing event-driven vision initialization..." << std::endl;
        
        // 启动TimerManager
        if (!timer_manager_.initialize()) {
            std::cout << "❌ TimerManager initialization failed" << std::endl;
            test_passed_ = false;
            return;
        }
        timer_manager_.start();
        
        // 初始化视觉跟踪器
        bool init_result = tracker_.initialize(&timer_manager_);
        if (!init_result) {
            std::cout << "❌ Event-driven vision tracker initialization failed" << std::endl;
            std::cout << "    This may be due to missing camera or OpenCV issues" << std::endl;
            test_passed_ = false;
            return;
        }
        
        // 检查初始状态
        bool initial_line = tracker_.isLineDetected();
        double initial_deviation = tracker_.getLineDeviation();
        
        if (initial_deviation < -1.0 || initial_deviation > 1.0) {
            std::cout << "❌ Initial deviation out of range: " << initial_deviation << std::endl;
            test_passed_ = false;
            return;
        }
        
        std::cout << "✅ Event-driven vision initialization test passed" << std::endl;
        std::cout << "    Initial line detected: " << (initial_line ? "Yes" : "No") << std::endl;
        std::cout << "    Initial deviation: " << initial_deviation << std::endl;
    }
    
    void testEventDrivenCapture() {
        std::cout << "Testing event-driven frame capture..." << std::endl;
        
        // 启动事件驱动模式
        tracker_.startEventDriven();
        
        // 等待自动捕获和处理
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        // 验证处理状态
        bool line_status = tracker_.isLineDetected();
        double deviation = tracker_.getLineDeviation();
        bool processing = tracker_.isProcessingActive();
        
        if (deviation < -1.0 || deviation > 1.0) {
            std::cout << "❌ Event-driven capture test failed - invalid deviation: " 
                     << deviation << std::endl;
            test_passed_ = false;
            return;
        }
        
        std::cout << "✅ Event-driven capture test passed" << std::endl;
        std::cout << "    Line status: " << (line_status ? "Detected" : "Not detected") << std::endl;
        std::cout << "    Deviation: " << deviation << std::endl;
        std::cout << "    Processing active: " << (processing ? "Yes" : "No") << std::endl;
    }
    
    void testLineDetection() {
        std::cout << "Testing event-driven line detection..." << std::endl;
        
        // 收集多个检测样本
        int detection_count = 0;
        std::vector<double> deviations;
        
        for (int i = 0; i < 10; i++) {
            // 手动请求帧捕获测试
            tracker_.requestFrameCapture();
            std::this_thread::sleep_for(std::chrono::milliseconds(150));
            
            bool detected = tracker_.isLineDetected();
            double dev = tracker_.getLineDeviation();
            
            if (detected) {
                detection_count++;
            }
            deviations.push_back(dev);
        }
        
        // 分析检测结果
        double avg_deviation = 0.0;
        for (double dev : deviations) {
            avg_deviation += dev;
        }
        avg_deviation /= deviations.size();
        
        std::cout << "✅ Event-driven line detection test completed" << std::endl;
        std::cout << "    Detection rate: " << detection_count << "/10 samples" << std::endl;
        std::cout << "    Average deviation: " << avg_deviation << std::endl;
        
        if (detection_count == 0) {
            std::cout << "⚠️ No lines detected - ensure proper test environment with visible lines" << std::endl;
        }
    }
    
    void testColorConfiguration() {
        std::cout << "Testing color configuration..." << std::endl;
        
        // 测试白线检测
        tracker_.setLineColor(cv::Scalar(0, 0, 100), cv::Scalar(180, 30, 255));
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        double white_deviation = tracker_.getLineDeviation();
        
        // 测试蓝线检测
        tracker_.setLineColor(cv::Scalar(100, 50, 50), cv::Scalar(130, 255, 255));
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        double blue_deviation = tracker_.getLineDeviation();
        
        // 测试Canny阈值调整
        tracker_.setCannyThresholds(30, 100);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        tracker_.setCannyThresholds(50, 150);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        std::cout << "✅ Color configuration test completed" << std::endl;
        std::cout << "    White line deviation: " << white_deviation << std::endl;
        std::cout << "    Blue line deviation: " << blue_deviation << std::endl;
    }
    
    void testCallbackSystem() {
        std::cout << "Testing event-driven callback system..." << std::endl;
        
        callback_count_ = 0;
        line_detected_ = false;
        last_deviation_ = 0.0;
        
        // 注册回调
        tracker_.registerCallback([this](bool detected, double deviation) {
            callback_count_++;
            line_detected_ = detected;
            last_deviation_ = deviation;
            std::cout << "    Vision callback: Line=" << (detected ? "Yes" : "No") 
                     << ", Deviation=" << deviation << std::endl;
        });
        
        // 手动触发几次处理
        for (int i = 0; i < 5; i++) {
            tracker_.requestFrameCapture();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        
        int count = callback_count_.load();
        double deviation = last_deviation_.load();
        
        if (deviation < -1.0 || deviation > 1.0) {
            std::cout << "❌ Invalid deviation in callback: " << deviation << std::endl;
            test_passed_ = false;
            return;
        }
        
        std::cout << "✅ Event-driven callback system test passed" << std::endl;
        std::cout << "    Callbacks received: " << count << std::endl;
        std::cout << "    Last line detected: " << (line_detected_.load() ? "Yes" : "No") << std::endl;
        std::cout << "    Last deviation: " << deviation << std::endl;
    }
    
    void testPerformance() {
        std::cout << "Testing event-driven performance..." << std::endl;
        
        auto start_time = std::chrono::steady_clock::now();
        
        // 测试连续处理性能
        for (int i = 0; i < 10; i++) {
            tracker_.requestFrameCapture();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time);
        
        // 验证处理延迟
        bool processing = tracker_.isProcessingActive();
        
        std::cout << "✅ Event-driven performance test completed" << std::endl;
        std::cout << "    Processing duration: " << duration.count() << "ms" << std::endl;
        std::cout << "    Processing active: " << (processing ? "Yes" : "No") << std::endl;
        std::cout << "    Average frame time: " << (duration.count() / 10.0) << "ms" << std::endl;
    }
};

int main() {
    try {
        EventDrivenVisionTest test;
        test.runAllTests();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Event-driven vision test exception: " << e.what() << std::endl;
        return -1;
    }
}