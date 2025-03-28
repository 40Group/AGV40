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









//KUN MAO
// === Web Api Server ===


// === Function：generate_frames ===
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








//zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzqqqqqqqq
// === Vision Line Pid ===







//zhelishi
void timer2_handler(const boost::system::error_code & /*e*/,
                    boost::asio::steady_timer *timer2) {
    // PID controller parameters
    double kp = 25;
    double ki = 0;
    double kd = 0;

    // PID controller related variables
    static double integral = 0;
    static double previous_error = 0;

    double temp = get_temp();

    // Calculate the PID output
    double error = setpoint - temp;
    integral += error;
    double derivative = error - previous_error;
    double pid_output = kp * error + ki * integral + kd * derivative;
    previous_error = error;

    if (std::abs(error) <= 0.4) {
        // If the error is within the range, reset the integral term to avoid integral accumulation
        integral = 0.0;
        previous_error = 0.0;
    }else {
        // Control the cooling plate or heating plate according to the PID output
        if (pid_output > 0) {
            // Heating
            double dutyCycle = std::min(1.0, std::max(0.0, pid_output));
            setPWM_DutyCycle(27, dutyCycle);
            setPWM_DutyCycle(17, 0.0); // Turn off the cooling plate
        } else {
            // Cooling
            double dutyCycle = std::min(1.0, std::max(0.0, -pid_output));
            setPWM_DutyCycle(17, dutyCycle);
            setPWM_DutyCycle(27, 0.0); // Turn off the heating plate
        }
    }

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

void timer1_handler(const boost::system::error_code & /*e*/,
                    boost::asio::steady_timer *timer1) {
    distance = measureDistance();
    // std::cout << "Distance: " << distance << " cm" << std::endl;

    // Reset the timer to trigger again after 600 milliseconds
    timer1->expires_at(timer1->expiry() + boost::asio::chrono::milliseconds(100));
    // Asynchronously wait for the timer to expire and call the timer_handler function
    timer1->async_wait(boost::bind(timer1_handler,
                                   boost::asio::placeholders::error,
                                   timer1));
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

//daozheli

//zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqq











