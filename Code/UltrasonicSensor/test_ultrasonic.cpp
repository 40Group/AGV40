#include "UltrasonicSensor.hpp"
#include <iostream>
#include <iomanip>

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Smart Medical Car - Ultrasonic Sensor Test" << std::endl;
    std::cout << "========================================" << std::endl;
    
    UltrasonicSensor ultrasonic;
    
    // Initialization test
    std::cout << "\n1. Testing Ultrasonic Initialization..." << std::endl;
    if (!ultrasonic.initialize()) {
        std::cerr << "❌ Ultrasonic sensor initialization failed!" << std::endl;
        return -1;
    }
    std::cout << "✅ Ultrasonic sensor initialized successfully" << std::endl;
    
    // Sensor information
    std::cout << "Sensor type: " << ultrasonic.getSensorType() << std::endl;
    std::cout << "Status: " << ultrasonic.getSensorStatus() << std::endl;
    
    // Self-check test
    std::cout << "\n2. Running Ultrasonic Self-Test..." << std::endl;
    if (!ultrasonic.selfTest()) {
        std::cerr << "❌ Ultrasonic self-test failed!" << std::endl;
        return -1;
    }
    
    // Continuous distance measurement test
    std::cout << "\n3. Continuous Distance Measurement (30 seconds)" << std::endl;
    std::cout << "Distance readings every 500ms:" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    
    auto start_time = std::chrono::steady_clock::now();
    int reading_count = 0;
    double min_distance = 999.0, max_distance = 0.0, total_distance = 0.0;
    
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(30)) {
        double distance = ultrasonic.getDistance();
        reading_count++;
        
        // statistical data
        if (distance < 400.0) { // Valid reading
            if (distance < min_distance) min_distance = distance;
            if (distance > max_distance) max_distance = distance;
            total_distance += distance;
        }
        
        // Display reading
        std::cout << "Reading " << std::setw(3) << reading_count << ": ";
        if (distance >= 999.0) {
            std::cout << "OUT OF RANGE";
        } else {
            std::cout << std::fixed << std::setprecision(2) << distance << " cm";
            
            // Obstacle detection alert
            if (ultrasonic.isObstacleDetected(Constants::MIN_SAFE_DISTANCE)) {
                std::cout << " [OBSTACLE ALERT]";
            } else if (ultrasonic.isObstacleDetected(Constants::MAX_SAFE_DISTANCE)) {
                std::cout << " [CAUTION]";
            }
        }
        std::cout << std::endl;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    // Stable distance test
    std::cout << "\n4. Stable Distance Measurement Test" << std::endl;
    double stable_distance = ultrasonic.getStableDistance();
    std::cout << "Stable distance (3 readings average): " 
              << std::fixed << std::setprecision(2) << stable_distance << " cm" << std::endl;
    
    // Obstacle detection test
    std::cout << "\n5. Obstacle Detection Test" << std::endl;
    std::cout << "Safe distance threshold: " << Constants::MAX_SAFE_DISTANCE << " cm" << std::endl;
    std::cout << "Critical distance threshold: " << Constants::MIN_SAFE_DISTANCE << " cm" << std::endl;
    
    bool obstacle_safe = !ultrasonic.isObstacleDetected(Constants::MAX_SAFE_DISTANCE);
    bool obstacle_critical = ultrasonic.isObstacleDetected(Constants::MIN_SAFE_DISTANCE);
    
    std::cout << "Current distance: " << ultrasonic.getDistance() << " cm" << std::endl;
    std::cout << "Safe to move: " << (obstacle_safe ? "YES" : "NO") << std::endl;
    std::cout << "Critical obstacle: " << (obstacle_critical ? "YES" : "NO") << std::endl;
    
    // statistical result
    std::cout << "\n6. Measurement Statistics" << std::endl;
    std::cout << "Total readings: " << reading_count << std::endl;
    if (reading_count > 0) {
        std::cout << "Average distance: " << std::fixed << std::setprecision(2) 
                  << total_distance / reading_count << " cm" << std::endl;
        std::cout << "Minimum distance: " << min_distance << " cm" << std::endl;
        std::cout << "Maximum distance: " << max_distance << " cm" << std::endl;
    }
    
    // clear
    ultrasonic.shutdown();
    std::cout << "\n✅ Ultrasonic test completed successfully!" << std::endl;
    return 0;
}
