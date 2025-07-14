#include "Common.h"
// #ifndef VISIONTRACKER_H
// #define VISIONTRACKER_H
#ifndef VISION_TRACKER_H  // 建议用这个  防护宏的作用就是防止头文件被重复包含！
#define VISION_TRACKER_H


class VisionTracker {
private:
    cv::VideoCapture camera_;
    std::atomic<bool> running_;
    std::mutex result_mutex_;
    VisionResult latest_result_;
    
    // 图像处理参数
    cv::Size image_size_;
    cv::Mat kernel_;
    
    // 线检测参数
    int binary_threshold_;
    int min_line_length_;
    int max_line_gap_;
    double rho_resolution_;
    double theta_resolution_;
    int hough_threshold_;
    
    // ROI设置
    cv::Rect roi_;
    
    // 图像处理方法
    cv::Mat preprocessImage(const cv::Mat& image);
    cv::Mat applyROI(const cv::Mat& image);
    std::vector<cv::Vec4i> detectLines(const cv::Mat& binary_image);
    VisionResult analyzeLines(const std::vector<cv::Vec4i>& lines, const cv::Mat& image);
    cv::Point2f calculateLineCenter(const std::vector<cv::Vec4i>& lines);
    double calculateLineOffset(const cv::Point2f& line_center);
    double calculateLineAngle(const std::vector<cv::Vec4i>& lines);
    
    // 调试和可视化
    void drawDebugInfo(cv::Mat& image, const VisionResult& result, 
                      const std::vector<cv::Vec4i>& lines);

public:
    VisionTracker();
    ~VisionTracker();
    
    bool initialize(int camera_id = 0);
    void shutdown();
    
    // 主要处理方法 - 阻塞式获取新帧
    VisionResult processFrame();
    VisionResult getLatestResult();
    
    // 参数设置
    void setBinaryThreshold(int threshold) { binary_threshold_ = threshold; }
    void setROI(const cv::Rect& roi) { roi_ = roi; }
    void setHoughParameters(int threshold, int min_length, int max_gap);
    
    // 状态查询
    bool isRunning() const { return running_.load(); }
    bool isCameraReady() const { return camera_.isOpened(); }
    
    // 调试方法
    cv::Mat getLastProcessedFrame();
    void saveDebugFrame(const std::string& filename);
    bool selfTest();
};

#endif // VISIONTRACKER_H