#include "../include/VisionTracker.h"

VisionTracker::VisionTracker() 
    : running_(false), image_size_(Constants::CAMERA_WIDTH, Constants::CAMERA_HEIGHT),
      binary_threshold_(100), min_line_length_(50), max_line_gap_(10),
      rho_resolution_(1), theta_resolution_(CV_PI/180), hough_threshold_(50) {
    
    // 设置ROI (下半部分图像)
    roi_ = cv::Rect(0, image_size_.height * 0.6, image_size_.width, image_size_.height * 0.4);
    
    // 创建形态学操作核
    kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
}

VisionTracker::~VisionTracker() {
    shutdown();
}

bool VisionTracker::initialize(int camera_id) {
    camera_.open(camera_id);
    
    if (!camera_.isOpened()) {
        std::cerr << "Failed to open camera " << camera_id << std::endl;
        return false;
    }
    
    // 设置摄像头参数
    camera_.set(cv::CAP_PROP_FRAME_WIDTH, image_size_.width);
    camera_.set(cv::CAP_PROP_FRAME_HEIGHT, image_size_.height);
    camera_.set(cv::CAP_PROP_FPS, 30);
    
    // 预热摄像头
    cv::Mat dummy_frame;
    for (int i = 0; i < 10; ++i) {
        camera_ >> dummy_frame;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    running_ = true;
    std::cout << "VisionTracker initialized successfully" << std::endl;
    return true;
}

void VisionTracker::shutdown() {
    running_ = false;
    if (camera_.isOpened()) {
        camera_.release();
    }
    
    // 打印视觉处理性能统计
    PerformanceMonitors::vision_monitor.printStatistics();
    
    std::cout << "VisionTracker shutdown" << std::endl;
}

// ✅ 这里是更新的processFrame方法 - 添加了延迟测量
VisionResult VisionTracker::processFrame() {
    // 使用全局视觉监控器进行延迟测量
    PerformanceMonitors::vision_monitor.startMeasurement("Vision Processing");
    
    if (!running_.load() || !camera_.isOpened()) {
        return VisionResult();
    }
    
    cv::Mat frame;
    
    // 阻塞式获取新帧 - 这是正确的事件驱动方式
    auto frame_start = std::chrono::steady_clock::now();
    camera_ >> frame;
    auto frame_end = std::chrono::steady_clock::now();
    
    if (frame.empty()) {
        std::cerr << "Failed to capture frame" << std::endl;
        return VisionResult();
    }
    
    // 测量帧获取时间
    auto frame_capture_time = std::chrono::duration_cast<std::chrono::microseconds>(
        frame_end - frame_start).count();
    
    // 图像处理开始
    auto processing_start = std::chrono::steady_clock::now();
    
    // 图像预处理
    cv::Mat processed = preprocessImage(frame);
    
    // 应用ROI
    cv::Mat roi_image = applyROI(processed);
    
    // 线检测
    std::vector<cv::Vec4i> lines = detectLines(roi_image);
    
    // 分析结果
    VisionResult result = analyzeLines(lines, frame);
    
    // 测量图像处理时间
    auto processing_end = std::chrono::steady_clock::now();
    auto processing_time = std::chrono::duration_cast<std::chrono::microseconds>(
        processing_end - processing_start).count();
    
    // 设置结果中的处理时间
    result.processing_time_us = static_cast<int>(processing_time);
    
    // 更新最新结果
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        latest_result_ = result;
    }
    
    // 结束延迟测量并记录
    double total_latency = PerformanceMonitors::vision_monitor.endMeasurement();
    
    // 详细性能日志（可选）
    if (total_latency > Constants::MAX_VISION_LATENCY_US * 0.8) { // 超过80%阈值时警告
        std::cout << "Vision performance details:" << std::endl;
        std::cout << "  Frame capture: " << frame_capture_time << "μs" << std::endl;
        std::cout << "  Image processing: " << processing_time << "μs" << std::endl;
        std::cout << "  Total latency: " << total_latency << "μs" << std::endl;
    }
    
    return result;
}

cv::Mat VisionTracker::preprocessImage(const cv::Mat& image) {
    cv::Mat gray, binary, processed;
    
    // 转换为灰度图
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    
    // 高斯模糊减少噪声
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);
    
    // 自适应阈值二值化
    cv::adaptiveThreshold(gray, binary, 255, cv::ADAPTIVE_THRESH_MEAN_C, 
                         cv::THRESH_BINARY, 11, 2);
    
    // 形态学操作去除噪声
    cv::morphologyEx(binary, processed, cv::MORPH_CLOSE, kernel_);
    cv::morphologyEx(processed, processed, cv::MORPH_OPEN, kernel_);
    
    return processed;
}

cv::Mat VisionTracker::applyROI(const cv::Mat& image) {
    cv::Mat roi_image = cv::Mat::zeros(image.size(), image.type());
    image(roi_).copyTo(roi_image(roi_));
    return roi_image;
}

std::vector<cv::Vec4i> VisionTracker::detectLines(const cv::Mat& binary_image) {
    std::vector<cv::Vec4i> lines;
    
    // 使用霍夫变换检测直线
    cv::HoughLinesP(binary_image, lines, rho_resolution_, theta_resolution_, 
                    hough_threshold_, min_line_length_, max_line_gap_);
    
    // 过滤掉过于水平的线
    std::vector<cv::Vec4i> filtered_lines;
    for (const auto& line : lines) {
        int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
        double angle = std::atan2(y2 - y1, x2 - x1) * 180.0 / CV_PI;
        
        // 保留角度在 30-150 度范围内的线
        if (std::abs(angle) > 30 && std::abs(angle) < 150) {
            filtered_lines.push_back(line);
        }
    }
    
    return filtered_lines;
}

VisionResult VisionTracker::analyzeLines(const std::vector<cv::Vec4i>& lines, const cv::Mat& image) {
    VisionResult result;
    result.timestamp = std::chrono::steady_clock::now();
    
    if (lines.empty()) {
        result.line_detected = false;
        result.confidence = 0.0;
        return result;
    }
    
    result.line_detected = true;
    result.line_center = calculateLineCenter(lines);
    result.line_offset = calculateLineOffset(result.line_center);
    result.line_angle = calculateLineAngle(lines);
    
    // 计算置信度 (基于检测到的线条数量和质量)
    result.confidence = std::min(1.0, static_cast<double>(lines.size()) / 10.0);
    
    return result;
}

cv::Point2f VisionTracker::calculateLineCenter(const std::vector<cv::Vec4i>& lines) {
    cv::Point2f center(0, 0);
    int count = 0;
    
    for (const auto& line : lines) {
        cv::Point2f line_center((line[0] + line[2]) / 2.0f, (line[1] + line[3]) / 2.0f);
        center += line_center;
        count++;
    }
    
    if (count > 0) {
        center.x /= count;
        center.y /= count;
    }
    
    return center;
}

double VisionTracker::calculateLineOffset(const cv::Point2f& line_center) {
    double image_center_x = image_size_.width / 2.0;
    double offset = (line_center.x - image_center_x) / image_center_x;
    return std::max(-1.0, std::min(1.0, offset));
}

double VisionTracker::calculateLineAngle(const std::vector<cv::Vec4i>& lines) {
    if (lines.empty()) return 0.0;
    
    double total_angle = 0.0;
    int count = 0;
    
    for (const auto& line : lines) {
        int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
        double angle = std::atan2(y2 - y1, x2 - x1) * 180.0 / CV_PI;
        total_angle += angle;
        count++;
    }
    
    return count > 0 ? total_angle / count : 0.0;
}

VisionResult VisionTracker::getLatestResult() {
    std::lock_guard<std::mutex> lock(result_mutex_);
    return latest_result_;
}

void VisionTracker::setHoughParameters(int threshold, int min_length, int max_gap) {
    hough_threshold_ = threshold;
    min_line_length_ = min_length;
    max_line_gap_ = max_gap;
}

cv::Mat VisionTracker::getLastProcessedFrame() {
    // 实现获取最后处理的帧（用于调试）
    cv::Mat frame;
    if (camera_.isOpened()) {
        camera_ >> frame;
    }
    return frame;
}

void VisionTracker::saveDebugFrame(const std::string& filename) {
    cv::Mat frame = getLastProcessedFrame();
    if (!frame.empty()) {
        cv::imwrite(filename, frame);
        std::cout << "Debug frame saved to: " << filename << std::endl;
    }
}

bool VisionTracker::selfTest() {
    std::cout << "Starting vision tracker self-test with performance monitoring..." << std::endl;
    
    if (!camera_.isOpened()) {
        std::cout << "Camera not initialized" << std::endl;
        return false;
    }
    
    // 性能测试 - 处理多帧并测量性能
    std::cout << "Performance testing - processing 10 frames..." << std::endl;
    
    for (int i = 0; i < 10; ++i) {
        VisionResult result = processFrame();
        std::cout << "Frame " << i + 1 << ": Line detected = " 
                  << (result.line_detected ? "Yes" : "No");
        
        if (result.line_detected) {
            std::cout << ", Offset: " << result.line_offset 
                      << ", Angle: " << result.line_angle 
                      << ", Confidence: " << result.confidence
                      << ", Processing time: " << result.processing_time_us << "μs";
        }
        std::cout << std::endl;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // 打印性能统计
    std::cout << "\nVision processing performance analysis:" << std::endl;
    PerformanceMonitors::vision_monitor.printStatistics();
    
    // 检查实时性能合规性
    bool performance_ok = PerformanceMonitors::vision_monitor.isRealTimeCompliant();
    std::cout << "Real-time performance: " 
              << (performance_ok ? "✅ COMPLIANT" : "❌ NON-COMPLIANT") << std::endl;
    
    std::cout << "Vision tracker self-test completed" << std::endl;
    return performance_ok;
}