#ifndef VISIONTRACKER_H
#define VISIONTRACKER_H

#include "../common/Common.h"
#include <opencv2/opencv.hpp>
#include <gpiod.hpp>
#include <memory>

class TimerManager; // 前向声明

class VisionTracker {
private:
    // 摄像头配置
    cv::VideoCapture camera_;
    int camera_id_;
    
    // GPIO集成（用于外部触发）
    std::unique_ptr<gpiod::chip> gpio_chip_;
    std::unique_ptr<gpiod::line> trigger_line_;
    int trigger_pin_;
    std::string chip_path_;
    
    // 状态数据
    std::atomic<bool> line_detected_;
    std::atomic<double> line_deviation_;
    std::atomic<bool> processing_active_;
    
    // 回调系统
    VisionCallback vision_callback_;
    std::mutex callback_mutex_;
    
    // 图像处理参数
    cv::Scalar hsv_lower_;
    cv::Scalar hsv_upper_;
    int canny_low_;
    int canny_high_;
    
    // TimerManager引用
    TimerManager* timer_manager_;
    
    // 图像处理缓冲区
    cv::Mat current_frame_;
    std::mutex frame_mutex_;
    
    // 事件处理方法
    void captureFrame();
    void processFrame();
    cv::Mat preprocessImage(const cv::Mat& frame);
    bool detectLine(const cv::Mat& processed, double& deviation);
    void handleVisionResult(bool detected, double deviation);
    
    // GPIO触发处理
    void handleExternalTrigger();
    void setupGPIO();
    
public:
    VisionTracker(int camera_id = 0, int trigger_pin = 25, 
                  const std::string& chip_path = "/dev/gpiochip0");
    ~VisionTracker();
    
    // 初始化
    bool initialize(TimerManager* timer_manager);
    
    // 事件驱动启动
    void startEventDriven();
    void stopEventDriven();
    
    // 回调注册
    void registerCallback(VisionCallback callback);
    
    // 状态查询
    bool isLineDetected() const;
    double getLineDeviation() const;
    bool isProcessingActive() const;
    
    // 配置
    void setLineColor(const cv::Scalar& hsv_lower, const cv::Scalar& hsv_upper);
    void setCannyThresholds(int low_threshold, int high_threshold);
    
    // 手动触发处理（用于测试）
    void requestFrameCapture();
};

#endif // VISIONTRACKER_H