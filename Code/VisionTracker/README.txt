Vision Tracker Module
=====================

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
