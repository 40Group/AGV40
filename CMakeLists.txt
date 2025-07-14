cmake_minimum_required(VERSION 3.16)
project(SmartMedicalCar VERSION 1.0.0)

# Require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required libraries
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)

# WiringPi
pkg_check_modules(WIRINGPI REQUIRED wiringpi)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${WIRINGPI_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/code/main
    ${CMAKE_SOURCE_DIR}/code/MotorController
    ${CMAKE_SOURCE_DIR}/code/VisionTracker
    ${CMAKE_SOURCE_DIR}/code/UltrasonicSensor
    ${CMAKE_SOURCE_DIR}/code/InfraredSensor
    ${CMAKE_SOURCE_DIR}/code/TemperatureController
    ${CMAKE_SOURCE_DIR}/code/SafetyController
    ${CMAKE_SOURCE_DIR}/code/TimerManager
)

# library catalogue
link_directories(
    ${WIRINGPI_LIBRARY_DIRS}
)

# library variable
set(ALL_LIBS
    ${OpenCV_LIBS}
    ${WIRINGPI_LIBRARIES}
    Threads::Threads
)

# Module library
add_library(MotorController STATIC
    code/MotorController/MotorController.cpp
    code/MotorController/MotorController.h
)
target_link_libraries(MotorController PRIVATE ${ALL_LIBS})

add_library(VisionTracker STATIC
    code/VisionTracker/VisionTracker.cpp
    code/VisionTracker/VisionTracker.h
)
target_link_libraries(VisionTracker PRIVATE ${ALL_LIBS})

add_library(UltrasonicSensor STATIC
    code/UltrasonicSensor/UltrasonicSensor.cpp
    code/UltrasonicSensor/UltrasonicSensor.h
)
target_link_libraries(UltrasonicSensor PRIVATE ${ALL_LIBS})

add_library(InfraredSensor STATIC
    code/InfraredSensor/InfraredSensor.cpp
    code/InfraredSensor/InfraredSensor.h
)
target_link_libraries(InfraredSensor PRIVATE ${ALL_LIBS})

add_library(TemperatureController STATIC
    code/TemperatureController/TemperatureController.cpp
    code/TemperatureController/TemperatureController.h
)
target_link_libraries(TemperatureController PRIVATE ${ALL_LIBS})

add_library(SafetyController STATIC
    code/SafetyController/SafetyController.cpp
    code/SafetyController/SafetyController.h
)
target_link_libraries(SafetyController PRIVATE ${ALL_LIBS})

add_library(TimerManager STATIC
    code/TimerManager/TimerManager.cpp
    code/TimerManager/TimerManager.h
)
target_link_libraries(TimerManager PRIVATE ${ALL_LIBS})

add_library(Common STATIC
    code/main/Common.cpp
    code/main/Common.h
)
target_link_libraries(Common PRIVATE ${ALL_LIBS})

# Main programme
add_executable(smart_car
    code/main/main.cpp
)
target_link_libraries(smart_car PRIVATE
    MotorController VisionTracker UltrasonicSensor
    InfraredSensor TemperatureController SafetyController
    TimerManager Common ${ALL_LIBS}
)

# Test procedures
enable_testing()

add_executable(test_motor
    code/MotorController/test_motor.cpp
)T
target_link_libraries(test_motor PRIVATE MotorController Common ${ALL_LIBS})
add_test(NAME MotorTest COMMAND test_motor)

add_executable(test_vision
    code/VisionTracker/test_vision.cpp
)
target_link_libraries(test_vision PRIVATE VisionTracker Common ${ALL_LIBS})
add_test(NAME VisionTest COMMAND test_vision)

add_executable(test_ultrasonic
    code/UltrasonicSensor/test_ultrasonic.cpp
)
target_link_libraries(test_ultrasonic PRIVATE UltrasonicSensor Common ${ALL_LIBS})
add_test(NAME UltrasonicTest COMMAND test_ultrasonic)

add_executable(test_infrared
    code/InfraredSensor/test_infrared.cpp
)
target_link_libraries(test_infrared PRIVATE InfraredSensor Common ${ALL_LIBS})
add_test(NAME InfraredTest COMMAND test_infrared)

add_executable(test_temperature
    code/TemperatureController/test_temperature.cpp
)
target_link_libraries(test_temperature PRIVATE TemperatureController Common ${ALL_LIBS})
add_test(NAME TemperatureTest COMMAND test_temperature)

add_executable(test_safety
    code/SafetyController/test_safety.cpp
)
target_link_libraries(test_safety PRIVATE
    SafetyController MotorController UltrasonicSensor
    InfraredSensor TemperatureController Common ${ALL_LIBS}
)
add_test(NAME SafetyTest COMMAND test_safety)

add_executable(test_timer
    code/TimerManager/test_timer.cpp
)
target_link_libraries(test_timer PRIVATE TimerManager Common ${ALL_LIBS})
add_test(NAME TimerTest COMMAND test_timer)
