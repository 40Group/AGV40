#include "TimerManager.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Smart Medical Car - Timer Manager Test" << std::endl;
    std::cout << "========================================" << std::endl;
    
    TimerManager timerMgr;
    
    // 初始化测试
    std::cout << "\n1. Testing Timer Manager Initialization..." << std::endl;
    if (!timerMgr.initialize()) {
        std::cerr << "❌ Timer manager initialization failed!" << std::endl;
        return -1;
    }
    std::cout << "✅ Timer manager initialized successfully" << std::endl;
    
    // 自检测试
    std::cout << "\n2. Running Timer Manager Self-Test..." << std::endl;
    if (!timerMgr.selfTest()) {
        std::cerr << "❌ Timer manager self-test failed!" << std::endl;
        return -1;
    }
    
    // 基本定时器测试
    std::cout << "\n3. Basic Timer Functionality Test" << std::endl;
    
    std::atomic<int> timer1_count(0);
    std::atomic<int> timer2_count(0);
    std::atomic<bool> oneshot_executed(false);
    
    // 创建重复定时器
    uint32_t timer1 = timerMgr.createTimer(
        std::chrono::milliseconds(500),
        [&timer1_count]() {
            timer1_count++;
            std::cout << "Timer1 executed (count: " << timer1_count.load() << ")" << std::endl;
        },
        true
    );
    
    uint32_t timer2 = timerMgr.createTimer(
        std::chrono::milliseconds(1000),
        [&timer2_count]() {
            timer2_count++;
            std::cout << "Timer2 executed (count: " << timer2_count.load() << ")" << std::endl;
        },
        true
    );
    
    // 创建一次性定时器
    uint32_t oneshot = timerMgr.createOneShotTimer(
        std::chrono::milliseconds(2500),
        [&oneshot_executed]() {
            oneshot_executed = true;
            std::cout << "One-shot timer executed!" << std::endl;
        }
    );
    
    std::cout << "Created timers - Timer1: " << timer1 << ", Timer2: " << timer2 
              << ", OneShot: " << oneshot << std::endl;
    
    // 监控定时器执行
    std::cout << "\n4. Timer Execution Monitoring (10 seconds)" << std::endl;
    auto start_time = std::chrono::steady_clock::now();
    
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(10)) {
        auto stats = timerMgr.getStats();
        
        std::cout << "Active timers: " << stats.active_timers 
                  << " | Total: " << stats.total_timers
                  << " | Timer1: " << timer1_count.load()
                  << " | Timer2: " << timer2_count.load()
                  << " | OneShot: " << (oneshot_executed ? "DONE" : "WAITING") << std::endl;
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    // 定时器控制测试
    std::cout << "\n5. Timer Control Test" << std::endl;
    
    std::cout << "Pausing Timer1..." << std::endl;
    timerMgr.pauseTimer(timer1);
    
    std::cout << "Timer1 active: " << (timerMgr.isTimerActive(timer1) ? "YES" : "NO") << std::endl;
    std::cout << "Timer2 active: " << (timerMgr.isTimerActive(timer2) ? "YES" : "NO") << std::endl;
    
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    std::cout << "Resuming Timer1..." << std::endl;
    timerMgr.resumeTimer(timer1);
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // 定时器剩余时间测试
    std::cout << "\n6. Timer Remaining Time Test" << std::endl;
    auto remaining1 = timerMgr.getTimerRemaining(timer1);
    auto remaining2 = timerMgr.getTimerRemaining(timer2);
    
    std::cout << "Timer1 remaining: " << remaining1.count() << "ms" << std::endl;
    std::cout << "Timer2 remaining: " << remaining2.count() << "ms" << std::endl;
    
    // 高频定时器测试
    std::cout << "\n7. High-Frequency Timer Test" << std::endl;
    
    std::atomic<int> high_freq_count(0);
    uint32_t high_freq_timer = timerMgr.createTimer(
        std::chrono::milliseconds(10),  // 10ms = 100Hz
        [&high_freq_count]() {
            high_freq_count++;
        },
        true
    );
    
    std::cout << "Running 100Hz timer for 5 seconds..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    int expected_count = 500;  // 5 seconds * 100Hz
    int actual_count = high_freq_count.load();
    double accuracy = (double)actual_count / expected_count * 100.0;
    
    std::cout << "Expected executions: " << expected_count << std::endl;
    std::cout << "Actual executions: " << actual_count << std::endl;
    std::cout << "Timing accuracy: " << std::fixed << std::setprecision(1) << accuracy << "%" << std::endl;
    
    timerMgr.cancelTimer(high_freq_timer);
    
    // 批量操作测试
    std::cout << "\n8. Batch Operations Test" << std::endl;
    
    // 创建多个定时器
    std::vector<uint32_t> batch_timers;
    for (int i = 0; i < 5; i++) {
        uint32_t timer_id = timerMgr.createTimer(
            std::chrono::milliseconds(100 + i * 50),
            [i]() { std::cout << "Batch timer " << i << " executed" << std::endl; },
            true
        );
        batch_timers.push_back(timer_id);
    }
    
    std::cout << "Created 5 batch timers" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    std::cout << "Pausing all timers..." << std::endl;
    timerMgr.pauseAllTimers();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    std::cout << "Resuming all timers..." << std::endl;
    timerMgr.resumeAllTimers();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    std::cout << "Cancelling all timers..." << std::endl;
    timerMgr.cancelAllTimers();
    
    // 性能测试
    std::cout << "\n9. Performance Test" << std::endl;
    
    auto perf_start = std::chrono::high_resolution_clock::now();
    
    // 创建大量定时器
    std::vector<uint32_t> perf_timers;
    for (int i = 0; i < 100; i++) {
        uint32_t timer_id = timerMgr.createTimer(
            std::chrono::milliseconds(1000),
            []() {},
            false
        );
        perf_timers.push_back(timer_id);
    }
    
    auto perf_end = std::chrono::high_resolution_clock::now();
    auto creation_time = std::chrono::duration_cast<std::chrono::microseconds>(
        perf_end - perf_start).count();
    
    std::cout << "Created 100 timers in " << creation_time << " μs" << std::endl;
    std::cout << "Average creation time: " << creation_time / 100.0 << " μs per timer" << std::endl;
    
    // 清理性能测试定时器
    for (auto timer_id : perf_timers) {
        timerMgr.cancelTimer(timer_id);
    }
    
    // 最终统计
    std::cout << "\n10. Final Statistics" << std::endl;
    auto final_stats = timerMgr.getStats();
    
    std::cout << "Total timers created: " << final_stats.total_timers << std::endl;
    std::cout << "Active timers: " << final_stats.active_timers << std::endl;
    std::cout << "Paused timers: " << final_stats.paused_timers << std::endl;
    
    std::cout << "Timer execution counts:" << std::endl;
    std::cout << "  Timer1 (500ms): " << timer1_count.load() << " executions" << std::endl;
    std::cout << "  Timer2 (1000ms): " << timer2_count.load() << " executions" << std::endl;
    std::cout << "  OneShot: " << (oneshot_executed ? "Executed" : "Not executed") << std::endl;
    std::cout << "  High-frequency: " << actual_count << " executions" << std::endl;
    
    // 清理
    timerMgr.shutdown();
    std::cout << "\n✅ Timer manager test completed successfully!" << std::endl;
    return 0;
}