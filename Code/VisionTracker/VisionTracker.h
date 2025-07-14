#include "Common.h"
// #ifndef VISIONTRACKER_H
// #define VISIONTRACKER_H
#ifndef VISION_TRACKER_H
#define VISION_TRACKER_H


class VisionTracker {
private:
    cv::VideoCapture camera_;
    std::atomic<bool> running_;
    std::mutex result_mutex_;
    VisionResult latest_result_;
    
    // Image processing parameters
    cv::Size image_size_;
    cv::Mat kernel_;
    
    // Line Detection Parameters
    int binary_threshold_;
    int min_line_length_;
    int max_line_gap_;
    double rho_resolution_;
    double theta_resolution_;
    int hough_threshold_;
    
    // Setting ROI
    cv::Rect roi_;
    
    // Image Processing Methods
    cv::Mat preprocessImage(const cv::Mat& image);
    cv::Mat applyROI(const cv::Mat& image);
    std::vector<cv::Vec4i> detectLines(const cv::Mat& binary_image);
    VisionResult analyzeLines(const std::vector<cv::Vec4i>& lines, const cv::Mat& image);
    cv::Point2f calculateLineCenter(const std::vector<cv::Vec4i>& lines);
    double calculateLineOffset(const cv::Point2f& line_center);
    double calculateLineAngle(const std::vector<cv::Vec4i>& lines);
    
    // Debugging and Visualisation
    void drawDebugInfo(cv::Mat& image, const VisionResult& result, 
                      const std::vector<cv::Vec4i>& lines);

public:
    VisionTracker();
    ~VisionTracker();
    
    bool initialize(int camera_id = 0);
    void shutdown();
    
    // Blocking Acquisition of New Frames
    VisionResult processFrame();
    VisionResult getLatestResult();
    
    // parameter setting
    void setBinaryThreshold(int threshold) { binary_threshold_ = threshold; }
    void setROI(const cv::Rect& roi) { roi_ = roi; }
    void setHoughParameters(int threshold, int min_length, int max_gap);
    
    // Status Enquiry
    bool isRunning() const { return running_.load(); }
    bool isCameraReady() const { return camera_.isOpened(); }
    
    // Commissioning method
    cv::Mat getLastProcessedFrame();
    void saveDebugFrame(const std::string& filename);
    bool selfTest();
};

#endif // VISIONTRACKER_H
