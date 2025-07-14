#include "../include/Common.h"

// 全局延迟监控器实例定义
namespace PerformanceMonitors {
    LatencyMonitor vision_monitor("Vision Processing", Constants::MAX_VISION_LATENCY_US);
    LatencyMonitor sensor_monitor("Sensor Reading", Constants::MAX_SENSOR_LATENCY_US);
    LatencyMonitor control_monitor("Control Loop", Constants::MAX_CONTROL_LATENCY_US);
    LatencyMonitor safety_monitor("Safety Check", Constants::MAX_SAFETY_LATENCY_US);
}