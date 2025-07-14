#include "../include/VisionTracker.h"

VisionTracker::VisionTracker() 
    : running_(false), image_size_(Constants::CAMERA_WIDTH, Constants::CAMERA_HEIGHT),
      binary_threshold_(100), min_line_length_(50), max_line_gap_(10),
      rho_resolution_(1), theta_resolution_(CV_PI/180), hough_threshold_(50) {
    
    // Setting the ROI (lower half of the image)
    roi_ = cv::Rect(0, image_size_.height * 0.6, image_size_.width, image_size_.height * 0.4);
    
    // Creation of morphological manipulation kernels
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
    
    // Setting the camera parameters
    camera_.set(cv::CAP_PROP_FRAME_WIDTH, image_size_.width);
    camera_.set(cv::CAP_PROP_FRAME_HEIGHT, image_size_.height);
    camera_.set(cv::CAP_PROP_FPS, 30);
    
    // Warm-up camera
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
    
    // Print Vision Processing Performance Statistics
    PerformanceMonitors::vision_monitor.printStatistics();
    
    std::cout << "VisionTracker shutdown" << std::endl;
}

// Here is the updated processFrame method - added delay measurement!!!
VisionResult VisionTracker::processFrame() {
    // Use the global vision performance monitor for latency measurement.
    PerformanceMonitors::vision_monitor.startMeasurement("Vision Processing");
    
    if (!running_.load() || !camera_.isOpened()) {
        return VisionResult();
    }
    
    cv::Mat frame;
    
    // Blocking acquisition of a new frame.
    auto frame_start = std::chrono::steady_clock::now();
    camera_ >> frame;
    auto frame_end = std::chrono::steady_clock::now();
    
    if (frame.empty()) {
        std::cerr << "Failed to capture frame" << std::endl;
        return VisionResult();
    }
    
    // Measurement of frame acquisition time
    auto frame_capture_time = std::chrono::duration_cast<std::chrono::microseconds>(
        frame_end - frame_start).count();
    
    // Image processing started
    auto processing_start = std::chrono::steady_clock::now();
    
    // Image pre-processing
    cv::Mat processed = preprocessImage(frame);
    
    // Apply ROI
    cv::Mat roi_image = applyROI(processed);
    
    // Line detection
    std::vector<cv::Vec4i> lines = detectLines(roi_image);
    
    // Analysed result
    VisionResult result = analyzeLines(lines, frame);
    
    // Measurement of image processing time
    auto processing_end = std::chrono::steady_clock::now();
    auto processing_time = std::chrono::duration_cast<std::chrono::microseconds>(
        processing_end - processing_start).count();
    
    // Setting the processing time in the result
    result.processing_time_us = static_cast<int>(processing_time);
    
    // Updating the latest results
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        latest_result_ = result;
    }
    
    // End delay measurement and record
    double total_latency = PerformanceMonitors::vision_monitor.endMeasurement();
    
    // Detailed Performance Log
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
    
    // Convert to greyscale
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    
    // Gaussian blur reduces noise
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);
    
    // Adaptive threshold binarisation
    cv::adaptiveThreshold(gray, binary, 255, cv::ADAPTIVE_THRESH_MEAN_C, 
                         cv::THRESH_BINARY, 11, 2);
    
    // Morphological operations to remove noise
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
    
    // Detecting Straight Lines Using the Hough Transform
    cv::HoughLinesP(binary_image, lines, rho_resolution_, theta_resolution_, 
                    hough_threshold_, min_line_length_, max_line_gap_);
    
    // Filter out overly horizontal lines
    std::vector<cv::Vec4i> filtered_lines;
    for (const auto& line : lines) {
        int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
        double angle = std::atan2(y2 - y1, x2 - x1) * 180.0 / CV_PI;
        
        // Retaining lines with angles in the range of 30-150 degrees
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
    
    // Calculate the confidence level (based on the number and quality of lines detected)
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
    // Implementation to get the last processed frame (for debugging)
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
    
    // Performance testing - process multiple frames and measure performance
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
    
    // Printing Performance Statistics
    std::cout << "\nVision processing performance analysis:" << std::endl;
    PerformanceMonitors::vision_monitor.printStatistics();
    
    // Checking real-time performance compliance
    bool performance_ok = PerformanceMonitors::vision_monitor.isRealTimeCompliant();
    std::cout << "Real-time performance: " 
              << (performance_ok ? "✅ COMPLIANT" : "❌ NON-COMPLIANT") << std::endl;
    
    std::cout << "Vision tracker self-test completed" << std::endl;
    return performance_ok;
}
