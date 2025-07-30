#include "VisionTracker.h"
#include "../timer/TimerManager.h"
#include <stdexcept>

VisionTracker::VisionTracker(int camera_id, int trigger_pin, const std::string& chip_path)
    : camera_id_(camera_id), trigger_pin_(trigger_pin), chip_path_(chip_path),
      line_detected_(false), line_deviation_(0.0), processing_active_(false),
      hsv_lower_(cv::Scalar(0, 0, 100)),     // 默认检测白线
      hsv_upper_(cv::Scalar(180, 30, 255)),
      canny_low_(50), canny_high_(150),
      timer_manager_(nullptr) {
}

VisionTracker::~VisionTracker() {
    stopEventDriven();
}

bool VisionTracker::initialize(TimerManager* timer_manager) {
    try {
        if (!timer_manager) {
            LOG_ERROR("TimerManager is required for event-driven vision tracker");
            return false;
        }
        
        timer_manager_ = timer_manager;
        
        // 初始化摄像头
        camera_.open(camera_id_);
        if (!camera_.isOpened()) {
            LOG_ERROR("Failed to open camera " + std::to_string(camera_id_));
            return false;
        }
        
        // 设置摄像头参数
        camera_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        camera_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        camera_.set(cv::CAP_PROP_FPS, 30);
        camera_.set(cv::CAP_PROP_BUFFERSIZE, 1); // 减少缓冲延迟
        
        // 初始化GPIO
        setupGPIO();
        
        LOG_INFO("VisionTracker initialized successfully (Event-Driven + gpiod)");
        return true;
        
    } catch (const std::exception& e) {
        LOG_ERROR("VisionTracker initialization failed: " + std::string(e.what()));
        return false;
    }
}

void VisionTracker::setupGPIO() {
    try {
        // 初始化GPIO芯片
        gpio_chip_ = std::make_unique<gpiod::chip>(chip_path_);
        
        // 配置触发引脚为输入，启用边沿中断
        trigger_line_ = std::make_unique<gpiod::line>(gpio_chip_->get_line(trigger_pin_));
        gpiod::line_request trigger_config = {
            "vision_trigger",
            gpiod::line_request::EVENT_RISING_EDGE,
            0
        };
        trigger_line_->request(trigger_config);
        
        LOG_INFO("GPIO configured - Trigger: GPIO" + std::to_string(trigger_pin_));
        
    } catch (const std::exception& e) {
        LOG_WARNING("GPIO setup failed, continuing without hardware trigger: " + 
                   std::string(e.what()));
        // 可选择性继续运行，只使用定时器触发
    }
}

void VisionTracker::startEventDriven() {
    if (!timer_manager_) {
        LOG_ERROR("Cannot start event-driven mode without TimerManager");
        return;
    }
    
    // 启动GPIO中断监听（如果可用）
    if (trigger_line_) {
        std::thread gpio_monitor([this]() {
            while (!g_emergency_stop.load()) {
                try {
                    if (trigger_line_->event_wait(std::chrono::milliseconds(100))) {
                        auto event = trigger_line_->event_read();
                        if (event.event_type == gpiod::line_event::RISING_EDGE) {
                            handleExternalTrigger();
                        }
                    }
                } catch (const std::exception& e) {
                    LOG_ERROR("GPIO interrupt error: " + std::string(e.what()));
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
        });
        gpio_monitor.detach();
    }
    
    // 注册定时帧捕获事件（10Hz频率）
    timer_manager_->scheduleRepeating(100, [this]() {
        requestFrameCapture();
    });
    
    LOG_INFO("VisionTracker started in event-driven mode");
}

void VisionTracker::stopEventDriven() {
    camera_.release();
    LOG_INFO("VisionTracker stopped");
}

void VisionTracker::requestFrameCapture() {
    if (processing_active_.load()) {
        return; // 上次处理还未完成
    }
    
    // 异步捕获帧
    timer_manager_->scheduleOnce(0, [this]() {
        captureFrame();
    });
}

void VisionTracker::captureFrame() {
    try {
        cv::Mat frame;
        
        if (!camera_.read(frame)) {
            LOG_WARNING("Failed to capture frame from camera");
            return;
        }
        
        if (frame.empty()) {
            LOG_WARNING("Captured empty frame");
            return;
        }
        
        // 存储帧并触发处理
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            current_frame_ = frame.clone();
        }
        
        // 异步处理帧
        timer_manager_->scheduleOnce(0, [this]() {
            processFrame();
        });
        
    } catch (const std::exception& e) {
        LOG_ERROR("Frame capture error: " + std::string(e.what()));
    }
}

void VisionTracker::processFrame() {
    if (processing_active_.exchange(true)) {
        return; // 避免并发处理
    }
    
    try {
        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (current_frame_.empty()) {
                processing_active_ = false;
                return;
            }
            frame = current_frame_.clone();
        }
        
        // 图像预处理
        cv::Mat processed = preprocessImage(frame);
        
        // 线条检测
        double deviation = 0.0;
        bool line_found = detectLine(processed, deviation);
        
        // 处理结果
        handleVisionResult(line_found, deviation);
        
    } catch (const std::exception& e) {
        LOG_ERROR("Frame processing error: " + std::string(e.what()));
    }
    
    processing_active_ = false;
}

cv::Mat VisionTracker::preprocessImage(const cv::Mat& frame) {
    cv::Mat hsv, mask, gray, blur, edges;
    
    // HSV颜色空间转换
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    
    // 颜色过滤
    cv::inRange(hsv, hsv_lower_, hsv_upper_, mask);
    
    // 形态学操作去噪
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    
    // 转换为灰度图
    cv::cvtColor(mask, gray, cv::COLOR_GRAY2BGR);
    cv::cvtColor(gray, gray, cv::COLOR_BGR2GRAY);
    
    // 高斯模糊
    cv::GaussianBlur(gray, blur, cv::Size(5, 5), 1.5);
    
    // 边缘检测
    cv::Canny(blur, edges, canny_low_, canny_high_);
    
    return edges;
}

bool VisionTracker::detectLine(const cv::Mat& processed, double& deviation) {
    // 只处理图像下半部分（ROI优化）
    cv::Rect roi(0, processed.rows / 2, processed.cols, processed.rows / 2);
    cv::Mat roi_image = processed(roi);
    
    // 霍夫直线检测
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(roi_image, lines, 1, CV_PI/180, 30, 20, 10);
    
    if (lines.empty()) {
        deviation = 0.0;
        return false;
    }
    
    // 过滤和选择最佳线条
    std::vector<cv::Vec4i> valid_lines;
    for (const auto& line : lines) {
        int x1 = line[0], y1 = line[1];
        int x2 = line[2], y2 = line[3];
        
        // 计算线条角度
        double angle = atan2(y2 - y1, x2 - x1) * 180.0 / CV_PI;
        
        // 过滤近似水平的线条（允许±30度）
        if (abs(angle) < 30 || abs(angle) > 150) {
            valid_lines.push_back(line);
        }
    }
    
    if (valid_lines.empty()) {
        deviation = 0.0;
        return false;
    }
    
    // 计算线条中心相对于图像中心的偏移
    int image_center = processed.cols / 2;
    double total_center = 0.0;
    
    for (const auto& line : valid_lines) {
        int x1 = line[0], y1 = line[1];
        int x2 = line[2], y2 = line[3];
        
        // 计算线条中点
        int line_center = (x1 + x2) / 2;
        total_center += line_center;
    }
    
    double avg_center = total_center / valid_lines.size();
    deviation = (avg_center - image_center) / (double)image_center;
    deviation = std::max(-1.0, std::min(1.0, deviation)); // 限制在 [-1, 1]
    
    return true;
}

void VisionTracker::handleVisionResult(bool detected, double deviation) {
    // 更新状态
    line_detected_ = detected;
    line_deviation_ = deviation;
    
    // 触发回调
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (vision_callback_) {
        // 在TimerManager的事件线程中执行回调
        timer_manager_->scheduleOnce(0, [this, detected, deviation]() {
            vision_callback_(detected, deviation);
        });
    }
    
    LOG_DEBUG("Vision: Line " + (detected ? "detected" : "not detected") + 
              ", Deviation: " + std::to_string(deviation));
}

void VisionTracker::handleExternalTrigger() {
    // GPIO触发的帧捕获
    LOG_DEBUG("External trigger received");
    requestFrameCapture();
}

void VisionTracker::registerCallback(VisionCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    vision_callback_ = callback;
    LOG_INFO("Vision callback registered");
}

bool VisionTracker::isLineDetected() const {
    return line_detected_.load();
}

double VisionTracker::getLineDeviation() const {
    return line_deviation_.load();
}

bool VisionTracker::isProcessingActive() const {
    return processing_active_.load();
}

void VisionTracker::setLineColor(const cv::Scalar& hsv_lower, const cv::Scalar& hsv_upper) {
    hsv_lower_ = hsv_lower;
    hsv_upper_ = hsv_upper;
    LOG_INFO("Line color thresholds updated");
}

void VisionTracker::setCannyThresholds(int low_threshold, int high_threshold) {
    if (low_threshold > 0 && high_threshold > low_threshold) {
        canny_low_ = low_threshold;
        canny_high_ = high_threshold;
        LOG_INFO("Canny thresholds set to " + std::to_string(low_threshold) + 
                 "-" + std::to_string(high_threshold));
    } else {
        LOG_ERROR("Invalid Canny threshold values");
    }
}