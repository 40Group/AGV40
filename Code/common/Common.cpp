// #include "../include/Common.h"

// // Global Latency Monitor instance definition
// namespace PerformanceMonitors {
//     LatencyMonitor vision_monitor("Vision Processing", Constants::MAX_VISION_LATENCY_US);
//     LatencyMonitor sensor_monitor("Sensor Reading", Constants::MAX_SENSOR_LATENCY_US);
//     LatencyMonitor control_monitor("Control Loop", Constants::MAX_CONTROL_LATENCY_US);
//     LatencyMonitor safety_monitor("Safety Check", Constants::MAX_SAFETY_LATENCY_US);
// }


// #include "Common.h"

// // Global variable definition (thread-safe atomic variables)
// std::atomic<SystemState> g_system_state{SystemState::INITIALIZING};
// std::atomic<bool> g_emergency_stop{false};
// std::atomic<bool> g_running{false};  // 新增：全局运行控制

// The commented out monitoring code is left
// #include "../include/Common.h"

// // Global Latency Monitor instance definition
// namespace PerformanceMonitors {
//     LatencyMonitor vision_monitor("Vision Processing", Constants::MAX_VISION_LATENCY_US);
//     LatencyMonitor sensor_monitor("Sensor Reading", Constants::MAX_SENSOR_LATENCY_US);
//     LatencyMonitor control_monitor("Control Loop", Constants::MAX_CONTROL_LATENCY_US);
//     LatencyMonitor safety_monitor("Safety Check", Constants::MAX_SAFETY_LATENCY_US);
// }




#include "Common.h"

// Global state initialization
std::atomic<SystemState> g_system_state{SystemState::INITIALIZING};
std::atomic<bool> g_emergency_stop{false};
std::atomic<bool> g_running{false};

// Enumerate the conversion function
// const char* systemStateToString(SystemState state) {
//     switch(state) {
//         case SystemState::INITIALIZING: return "INITIALIZING";
//         case SystemState::RUNNING: return "RUNNING";
//         case SystemState::EMERGENCY_STOP: return "EMERGENCY_STOP";
//         case SystemState::SHUTDOWN: return "SHUTDOWN";
//         case SystemState::DIAGNOSTIC: return "DIAGNOSTIC";
//         default: return "UNKNOWN";
//     }
// }
