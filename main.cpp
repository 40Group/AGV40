#include <fstream>
#include <iostream>
#include <thread>
#include <opencv2/opencv.hpp>
#include "libs/httplib.h"
#include "libs/json.hpp"
#include <termios.h>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <opencv2/opencv.hpp>
#include <linux/i2c-dev.h>


//zihan wei
// === Main Entry Point ===

int main(void) {
    init();

    // Start the thread to capture camera frames
    std::thread capture_thread(capture_frames);
    // Detach the thread so that it can run independently
    capture_thread.detach();

    std::thread temp_control_thread(temp_control);
    temp_control_thread.detach();

    std::thread distance_thread(get_distance);
    distance_thread.detach();
 Server svr;

    svr.set_base_dir("/home/pi/program/dist");

    svr.Get("/", [](const Request &req, Response &res) {
        try {
            std::ifstream file("/home/pi/program/dist/index.html");
            if (file.is_open()) {
                std::stringstream buffer;
                buffer << file.rdbuf();
                std::string content = buffer.str();
                res.set_content(content, "text/html");
            } else {
                res.status = 404;
                res.set_content("File not found", "text/plain");
            }
        } catch (...) {
            res.status = 500;
            res.set_content("Internal server error", "text/plain");
        }
    });
    // Define a route to handle video stream requests
    svr.Get("/video_feed", [](const Request &req, Response &res) {
        res.set_content(generate_frames(), "multipart/x-mixed-replace; boundary=frame");
    });
 // New route to receive the switch status sent from the front - end
    svr.Post("/api/curtainSwitch", [](const Request &req, Response &res) {
        try {
            // Parse the JSON data from the request body
            auto json = nlohmann::json::parse(req.body);
            // Update the running status of the car
            carRunStatus = json["status"];
            std::cout << "Modified status: " << carRunStatus << std::endl;
            res.set_content("Success", "text/plain");
        } catch (...) {
            res.set_content("Error", "text/plain");
        }
    });
// Define a route to handle video stream requests
    svr.Get("/video_feed", [](const Request &req, Response &res) {
        res.set_content(generate_frames(), "multipart/x-mixed-replace; boundary=frame");
    });
// New route to receive the switch status sent from the front - end
    svr.Post("/api/curtainSwitch", [](const Request &req, Response &res) {
        try {
            // Parse the JSON data from the request body
            auto json = nlohmann::json::parse(req.body);
            // Update the running status of the car
            carRunStatus = json["status"];
            std::cout << "Modified status: " << carRunStatus << std::endl;
            res.set_content("Success", "text/plain");
        } catch (...) {
            res.set_content("Error", "text/plain");
        }
    });













// === Hardware Gpio Temp ===


// Yixuan ding
void initPWM() {
    // Set the pins to output mode
    gpioSetMode(PIN_17, PI_OUTPUT);
    gpioSetMode(PIN_27, PI_OUTPUT);

    // Set the PWM frequency
    gpioSetPWMfrequency(PIN_17, PWM_FREQUENCY);
    gpioSetPWMfrequency(PIN_27, PWM_FREQUENCY);

    // Set the PWM range from 0 to 1000
    gpioSetPWMrange(PIN_17, 1000);
    gpioSetPWMrange(PIN_27, 1000);
}

void initmeasureDistance() {
    if (gpioInitialise() < 0) {
        std::cerr << "Failed to initialize pigpio library." << std::endl;
        exit(1);
    }

    // Set the Trig pin as an output mode and the Echo pin as an input mode
    gpioSetMode(TRIG_PIN, PI_OUTPUT);
    gpioSetMode(ECHO_PIN, PI_INPUT);
}









//KUN MAO first part

// === Car Control Serial ===


// === 函数：car_run ===
void car_run() {
    const char *message1 = "{#008P2000T0000!#009P1000T0000!}";
    // Write the message to the serial port
    ssize_t bytesWritten = write(serialFd, message1, strlen(message1));
    if (bytesWritten == -1) {
        std::cerr << "Failed to send message via serial port" << std::endl;
    }
}    



//YANSHI WANG
//state_machine_threads

// === timer_handler ===
void timer_handler(const boost::system::error_code & /*e*/,
                   boost::asio::steady_timer *timer) {
    // std::cout << "Timer triggered, current time: " << std::chrono::steady_clock::now().time_since_epoch().count() << std::endl;

    // Check if the running status of the car has changed
    if (carRunStatus != precarRunStatus) {
        if (carRunStatus) {
            // If the car should run, call the car_run function
            car_run();
            currentState = STATE_LINE_FOLLOWING;
        } else {
            // If the car should stop, call the car_stop function
            car_stop();
            currentState = STATE_STOPPED;
        }
        // Update the previous running status
        precarRunStatus = carRunStatus;
    }
 Mat frame;
    // Read a frame from the camera
    cap.read(frame);

    if (!frame.empty()) {
        // Check the running status of the car
        if (carRunStatus) {
            if (is_turning_right || is_turning_left) {
                auto current_time = std::chrono::steady_clock::now();
                auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                    current_time - turn_start_time).count();
                std::cout << elapsed_time << std::endl;

                if (is_turning_right) {
                    if (elapsed_time >= 1000) {
                        // Turn right for two seconds
                        car_turn_left();
                        is_turning_right = false;
                        is_turning_left = true;
                        turn_start_time = std::chrono::steady_clock::now();
                    }
                } else if (is_turning_left) {
                    if (elapsed_time >= 1000) {
                        // After two seconds left turn
                        is_turning_left = false;
                        currentState = STATE_LINE_FOLLOWING;
                    }
                }
                latest_frame = frame.clone();
            } else {
                switch (currentState) {
                    case STATE_LINE_FOLLOWING: {
                        // std::cout << distance << std::endl;
                        // Check the distance
                        if (distance < 35) {
                            // An obstacle is considered if the distance is less than 20cm
                            // car_stop();
                            currentState = STATE_OBSTACLE_AVOIDANCE;
                        } else {
                            cv::Mat gray;
                            // Convert the frame to grayscale
                            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

                            // Color filtering to identify black lines
                            cv::Mat binary;
                            // Threshold the grayscale image to get a binary image
                            threshold(gray, binary, 50, 255, THRESH_BINARY_INV);

                            // Morphological operations
                            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
                            // Perform an opening operation to remove small noise
                            cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
                            // Perform a closing operation to connect broken trajectories
                            cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);

                            // Contour detection
                            std::vector<std::vector<cv::Point> > contours;
                            std::vector<cv::Vec4i> hierarchy;
                            // Find contours in the binary image
                            cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

                            // Select the trajectory
                            std::vector<cv::Point> selectedContour;
                            for (const auto &contour: contours) {
                                double area = cv::contourArea(contour);
                                if (area > 100) {
                                    // Select the contour with an area greater than 100
                                    selectedContour = contour;
                                    break;
                                }
                            }

                            if (!selectedContour.empty()) {
                                // Calculate the center of gravity of the line
                                cv::Point center = getLineCenter(binary);

                                // Generate control commands
                                int command = generateControlCommand(center, binary.cols);
                                const char *message_template = "{#010P%dT0000!}";
                                char message[50];
                                // Format the control command into a message
                                snprintf(message, sizeof(message), message_template, command);
                                // Write the message to the serial port
                                ssize_t bytesWritten = write(serialFd, message, strlen(message));
                                if (bytesWritten == -1) {
                                    std::cerr << "Failed to send message via serial port" << std::endl;
                                }
                                // std::cout << "Output: " << command << std::endl;
                                // std::cout << "Concatenated message: " << message << std::endl;

                                // Find the centerline contour points of the trajectory line
                                std::vector<cv::Point> centerlinePoints = findCenterlinePoints(binary, selectedContour);

                                // Draw the centerline contour on the color image
                                cv::Mat colorImageWithCenterline = frame.clone();
                                drawCenterlineOnColorImage(colorImageWithCenterline, centerlinePoints);

                                // Update the latest frame
                                latest_frame = colorImageWithCenterline.clone();
                            } else {
                                // If no contour is selected, use the original frame
                                latest_frame = frame.clone();
                            }
                        }
                        break;
                    }
                    case STATE_OBSTACLE_AVOIDANCE: {
                        car_turn_right();
                        turn_start_time = std::chrono::steady_clock::now();
                        is_turning_right = true;
                        latest_frame = frame.clone();
                        break;
                    }
                    case STATE_STOPPED: {
                        latest_frame = frame.clone();
                        break;
                    }
                }
            }
        } else {
            // If the car is not running, use the original frame
            latest_frame = frame.clone();
        }
    } else {
        // If the frame is empty, use the original frame
        latest_frame = frame.clone();
    }

    // Reset the timer to trigger again after 30 milliseconds
    timer->expires_at(timer->expiry() + boost::asio::chrono::milliseconds(30));
    // Asynchronously wait for the timer to expire and call the timer_handler function
    timer->async_wait(boost::bind(timer_handler,
                                  boost::asio::placeholders::error,
                                  timer));
}









//zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzqqqqqqqq
// === 图像模块处理（摄像头帧的处理、轨迹线检测、中心点计算以及视频帧生成） ===


// Calculate the center of gravity of the line
cv::Point getLineCenter(const cv::Mat &binary) {
    // Calculate the moments of the binary image
    cv::Moments m = cv::moments(binary, true);
    cv::Point center;
    // If the zero - order moment is not zero, calculate the center coordinates
    if (m.m00 != 0) {
        center.x = static_cast<int>(m.m10 / m.m00);
        center.y = static_cast<int>(m.m01 / m.m00);
    } else {
        // If the zero - order moment is zero, set the center coordinates to (0, 0)
        center.x = 0;
        center.y = 0;
    }
    return center;
}

// Generate control commands based on the center point and frame width
int generateControlCommand(const cv::Point &center, int frameWidth) {
    int centerX = center.x;
    int halfWidth = frameWidth / 2;

    // Define PID parameters, which need to be adjusted according to the actual situation
    double Kp = 1.5625;
    double Ki = 0;
    double Kd = 0;

    // Initialize the integral term and the previous error
    static double integral = 0.0;
    static double previousError = 0.0;

    // Calculate the current error, modify the error calculation method so that the error is positive when the target is on the right
    double error = centerX - halfWidth;

    // Check if the error is within the range of 5
    if (std::abs(error) <= 5) {
        // If the error is within the range, reset the integral term to avoid integral accumulation
        integral = 0.0;
        previousError = 0.0;
        return 1500;
    }

    // Update the integral term
    integral += error;

    // Calculate the derivative term
    double derivative = error - previousError;

    // Calculate the PID output
    double pidOutput = Kp * error + Ki * integral + Kd * derivative;

    // Update the previous error
    previousError = error;

    // The basic output value is 1500
    int baseOutput = 1500;

    // The final output
    int finalOutput = baseOutput + static_cast<int>(pidOutput);

    // Limit the output range to 1000 - 2000
    finalOutput = std::max(1000, std::min(2000, finalOutput));

    return finalOutput;
}

// Find the centerline contour points of the trajectory line
std::vector<cv::Point> findCenterlinePoints(const cv::Mat &binary, const std::vector<cv::Point> &contour) {
    std::vector<cv::Point> centerlinePoints;
    // Calculate the bounding rectangle of the contour
    cv::Rect boundingRect = cv::boundingRect(contour);
    for (int y = boundingRect.y; y < boundingRect.y + boundingRect.height; ++y) {
        // Get a single row of the binary image
        cv::Mat row = binary.row(y);
        // Calculate the moments of the row
        cv::Moments m = cv::moments(row, true);
        if (m.m00 != 0) {
            // If the zero - order moment is not zero, calculate the center x - coordinate of the row
            int centerX = static_cast<int>(m.m10 / m.m00);
            centerlinePoints.emplace_back(centerX, y);
        }
    }
    return centerlinePoints;
}

// Draw the centerline contour on the color image
void drawCenterlineOnColorImage(cv::Mat &colorImage, const std::vector<cv::Point> &centerlinePoints) {
    for (size_t i = 0; i < centerlinePoints.size() - 1; ++i) {
        // Draw a line between adjacent centerline points
        cv::line(colorImage, centerlinePoints[i], centerlinePoints[i + 1], cv::Scalar(0, 255, 0), 2);
    }
}



// KUN  MAO second part


// Function to generate video streams
std::string generate_frames() {
    if (latest_frame.empty()) {
        return "";
    }
    std::vector<uchar> buffer;
    // Encode the frame as a JPEG image
    imencode(".jpg", latest_frame, buffer);

    std::string frame_str(buffer.begin(), buffer.end());
    std::string boundary = "frame";
    std::stringstream ss;
    // Format the frame data as a multipart response
    ss << "--" << boundary << "\r\n";
    ss << "Content-Type: image/jpeg\r\n\r\n";
    ss << frame_str << "\r\n";
    return ss.str();
}

// KUN MAO










