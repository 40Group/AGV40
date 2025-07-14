#include "TemperatureController.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Smart Medical Car - Temperature Controller Test" << std::endl;
    std::cout << "========================================" << std::endl;
    
    TemperatureController tempCtrl;
    
    // Initialization test
    std::cout << "\n1. Testing Temperature Controller Initialization..." << std::endl;
    if (!tempCtrl.initialize()) {
        std::cerr << "❌ Temperature controller initialization failed!" << std::endl;
        std::cerr << "Please check DS18B20 sensor connection" << std::endl;
        return -1;
    }
    std::cout << "✅ Temperature controller initialized successfully" << std::endl;
    
    // Sensor status check
    std::cout << "Current temperature: " << std::fixed << std::setprecision(2) 
              << tempCtrl.getCurrentTemperature() << "°C" << std::endl;
    std::cout << "Target temperature: " << tempCtrl.getTargetTemperature() << "°C" << std::endl;
    std::cout << "Within safe range: " << (tempCtrl.isWithinSafeRange() ? "YES" : "NO") << std::endl;
    
    // self-testing
    std::cout << "\n2. Running Temperature Controller Self-Test..." << std::endl;
    if (!tempCtrl.selfTest()) {
        std::cerr << "❌ Temperature controller self-test failed!" << std::endl;
        return -1;
    }
    
    // PID parameter setting test
    std::cout << "\n3. PID Parameters Configuration Test" << std::endl;
    std::cout << "Setting PID parameters (Kp=2.0, Ki=0.1, Kd=0.5)..." << std::endl;
    tempCtrl.setPIDParameters(2.0, 0.1, 0.5);
    
    // Temperature control test
    std::cout << "\n4. Temperature Control Test" << std::endl;
    std::cout << "Setting target temperature to 30°C..." << std::endl;
    tempCtrl.setTargetTemperature(30.0);
    
    std::cout << "Monitoring temperature control for 60 seconds..." << std::endl;
    std::cout << "Time\t| Current\t| Target\t| Stable\t| Safe Range" << std::endl;
    std::cout << "--------|---------------|---------------|---------------|-------------" << std::endl;
    
    auto start_time = std::chrono::steady_clock::now();
    int reading_count = 0;
    
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(60)) {
        double current = tempCtrl.getCurrentTemperature();
        double target = tempCtrl.getTargetTemperature();
        bool stable = tempCtrl.isTemperatureStable(1.0);
        bool safe = tempCtrl.isWithinSafeRange();
        
        reading_count++;
        
        std::cout << std::setw(6) << reading_count << "s\t| "
                  << std::fixed << std::setprecision(2) << std::setw(8) << current << "°C\t| "
                  << std::setw(8) << target << "°C\t| "
                  << (stable ? "YES" : "NO") << "\t\t| "
                  << (safe ? "YES" : "NO") << std::endl;
        
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    
    // Manual heating/cooling test
    std::cout << "\n5. Manual Heating/Cooling Test" << std::endl;
    std::cout << "Testing manual heating (20% power for 5 seconds)..." << std::endl;
    tempCtrl.setHeatingPower(0.2);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    std::cout << "Testing manual cooling (20% power for 5 seconds)..." << std::endl;
    tempCtrl.setCoolingPower(0.2);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    std::cout << "Turning off manual control..." << std::endl;
    tempCtrl.turnOffControl();
    
    // Temperature Change Target Test
    std::cout << "\n6. Temperature Target Change Test" << std::endl;
    std::cout << "Changing target to 25°C..." << std::endl;
    tempCtrl.setTargetTemperature(25.0);
    
    std::cout << "Monitoring for 30 seconds..." << std::endl;
    start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(30)) {
        double current = tempCtrl.getCurrentTemperature();
        double target = tempCtrl.getTargetTemperature();
        
        std::cout << "Temperature: " << std::fixed << std::setprecision(2) << current 
                  << "°C (Target: " << target << "°C)" << std::endl;
        
        if (tempCtrl.isTemperatureStable(0.5)) {
            std::cout << "✅ Temperature stabilized!" << std::endl;
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }
    
    // temperature statistics
    std::cout << "\n7. Temperature Statistics" << std::endl;
    auto stats = tempCtrl.getTemperatureStats();
    std::cout << "Min temperature: " << stats.min_temp << "°C" << std::endl;
    std::cout << "Max temperature: " << stats.max_temp << "°C" << std::endl;
    std::cout << "Average temperature: " << stats.avg_temp << "°C" << std::endl;
    std::cout << "Total readings: " << stats.readings_count << std::endl;
    
    // Safety margin testing
    std::cout << "\n8. Safety Range Test" << std::endl;
    std::cout << "Current temperature safety: " << (tempCtrl.isWithinSafeRange() ? "SAFE" : "UNSAFE") << std::endl;
    std::cout << "Temperature stable (±1°C): " << (tempCtrl.isTemperatureStable(1.0) ? "YES" : "NO") << std::endl;
    
    // Clean up
    tempCtrl.shutdown();
    std::cout << "\n✅ Temperature controller test completed successfully!" << std::endl;
    return 0;
}