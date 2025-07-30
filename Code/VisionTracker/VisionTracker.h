#ifndef VISIONTRACKER_H
#define VISIONTRACKER_H

#include "../common/Common.h"
#include <opencv2/opencv.hpp>
#include <gpiod.hpp>
#include <memory>

class TimerManager; // Forward statement

class VisionTracker {
private:
    // Camera Configuration
    cv::VideoCapture camera_;F
    int camera_id_;
    
    // GPIO integration (for external triggering)
    std::unique_ptr<gpiod::chip> gpio_chip_;
    std::unique_ptr<gpiod::line> trigger_line_;
    int trigger_pin_;
    std::string chip_path_;
    
    // Status data
    std::atomic<bool> line_detected_;
    std::atomic<double> line_deviation_;
    std::atomic<bool> processing_active_;
    
    // Callback system
    VisionCallback vision_callback_;
    std::mutex callback_mutex_;
    
    // Image processing parameters
    cv::Scalar hsv_lower_;
    cv::Scalar hsv_upper_;
    int canny_low_;
    int canny_high_;
    
    // TimerManager reference
    TimerManager* timer_manager_;
    
    // Image processing buffer
    cv::Mat current_frame_;
    std::mutex frame_mutex_;
    
    // Event handler
    void captureFrame();
    void processFrame();
    cv::Mat preprocessImage(const cv::Mat& frame);
    bool detectLine(const cv::Mat& processed, double& deviation);
    void handleVisionResult(bool detected, double deviation);
    
    // GPIO Trigger Handling
    void handleExternalTrigger();
    void setupGPIO();
    
public:
    VisionTracker(int camera_id = 0, int trigger_pin = 25, 
                  const std::string& chip_path = "/dev/gpiochip0");
    ~VisionTracker();
    
    // Initialisation
    bool initialize(TimerManager* timer_manager);
    
    // Event-driven startup (computing)
    void startEventDriven();
    void stopEventDriven();
    
    // Callback registration
    void registerCallback(VisionCallback callback);
    
    // Status Enquiry
    bool isLineDetected() const;
    double getLineDeviation() const;
    bool isProcessingActive() const;
    
    // Configure
    void setLineColor(const cv::Scalar& hsv_lower, const cv::Scalar& hsv_upper);
    void setCannyThresholds(int low_threshold, int high_threshold);
    
    // Manual trigger processing (for testing)
    void requestFrameCapture();
};

#endif // VISIONTRACKER_H
