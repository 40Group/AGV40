cmake_minimum_required(VERSION 3.10)
project(SmartMedicalCar LANGUAGES C CXX)

# Require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)

# libgpiod
pkg_check_modules(GPIO_CXX REQUIRED libgpiodcxx)
pkg_check_modules(GPIO_C REQUIRED libgpiod)

# Include directories
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${GPIO_CXX_INCLUDE_DIRS}
    ${GPIO_C_INCLUDE_DIRS}
    code/common
)

# GPIO libraries
set(GPIOD_LIBS ${GPIO_CXX_LIBRARIES} ${GPIO_C_LIBRARIES})

# Main executable
add_executable(smart_medical_car
    code/common/Common.cpp
    code/TimerManager/TimerManager.cpp
    code/MotorController/MotorController.cpp
    code/VisionTracker/VisionTracker.cpp
    code/UltrasonicSensor/UltrasonicSensor.cpp
    code/InfraredSensor/InfraredSensor.cpp
    code/TemperatureController/TemperatureController.cpp
    code/SafetyController/SafetyController.cpp
    code/main/main.cpp
)

target_link_libraries(smart_medical_car
    ${OpenCV_LIBS}
    ${GPIOD_LIBS}
    Threads::Threads
)

# Tests
add_executable(test_timer
    code/common/Common.cpp
    code/TimerManager/TimerManager.cpp
    code/TimerManager/test_timer.cpp
)
target_link_libraries(test_timer Threads::Threads)

add_executable(test_motor
    code/common/Common.cpp
    code/TimerManager/TimerManager.cpp
    code/MotorController/MotorController.cpp
    code/MotorController/test_motor.cpp
)
target_link_libraries(test_motor ${GPIOD_LIBS} Threads::Threads)

add_executable(test_integration
    code/common/Common.cpp
    code/TimerManager/TimerManager.cpp
    code/MotorController/MotorController.cpp
    code/UltrasonicSensor/UltrasonicSensor.cpp
    code/TemperatureController/TemperatureController.cpp
    code/SafetyController/SafetyController.cpp
    code/main/test_integration.cpp
)
target_link_libraries(test_integration ${GPIOD_LIBS} Threads::Threads)

# Install
install(TARGETS smart_medical_car DESTINATION bin)