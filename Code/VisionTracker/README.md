Vision Tracker Module

Purpose: Real-time line detection for medical car navigation

Files:
- VisionTracker.hpp: Class declaration
- VisionTracker.cpp: Implementation  
- test_vision.cpp: Unit test program
- readme.txt: This file

Hardware Requirements:
- Raspberry Pi Camera Module v2
- OpenCV 4.x library
- Sufficient lighting for line detection

Compilation:
g++ -std=c++17 test_vision.cpp VisionTracker.cpp -o vision_test -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_videoio -lopencv_highgui -lwiringPi -lpthread

Run Test:
./vision_test

Features Tested:
- Camera initialization and configuration
- Real-time frame processing
- Line detection using Hough transform
- Offset and angle calculation
- Performance monitoring
- Debug frame saving

Performance Requirements:
- Target processing time: <50ms per frame
- Real-time compliance: >95%
- Minimum detection confidence: 0.7

Algorithm Details:
- Grayscale conversion
- Gaussian blur for noise reduction
- Adaptive thresholding
- Morphological operations
- Hough line detection
- Line filtering and analysis

Medical Navigation Features:
- ROI-based processing for efficiency
- Confidence-based reliability
- Angle-based navigation guidance
- Real-time performance monitoring

All in all, This vision tracking module is built on OpenCV and performs image preprocessing through grayscale conversion, filtering, adaptive thresholding, and morphological operations. It applies ROI-based masking and probabilistic Hough transform for efficient line detection, then extracts key navigation metrics such as lateral offset, orientation angle, and detection confidence. Additionally, integrated performance monitoring ensures that frame rate and latency meet the real-time and stability requirements of smart medical vehicle navigation.
