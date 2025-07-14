#ifndef TIMERMANAGER_H
#define TIMERMANAGER_H

#include "Common.h"

class TimerManager {
public:
    using TimerCallback = std::function<void()>;
    
    struct TimerInfo {
        uint32_t id;
        std::chrono::milliseconds interval;
        TimerCallback callback;
        bool repeating;
        bool active;
        std::chrono::steady_clock::time_point next_execution;
        std::chrono::steady_clock::time_point created_time;
    };

private:
    std::atomic<bool> running_;
    std::atomic<uint32_t> next_timer_id_;
    
    std::vector<std::shared_ptr<TimerInfo>> timers_;
    std::mutex timers_mutex_;
    
    std::thread timer_thread_;
    std::condition_variable timer_cv_;
    
    // 定时器管理主循环
    void timerLoop();
    
    // 获取下一个需要执行的定时器
    std::shared_ptr<TimerInfo> getNextTimer();

public:
    TimerManager();
    ~TimerManager();
    
    bool initialize();
    void shutdown();
    
    // 定时器创建和管理
    uint32_t createTimer(std::chrono::milliseconds interval, 
                        TimerCallback callback, 
                        bool repeating = true);
    
    uint32_t createOneShotTimer(std::chrono::milliseconds delay, 
                               TimerCallback callback);
    
    bool cancelTimer(uint32_t timer_id);
    bool pauseTimer(uint32_t timer_id);
    bool resumeTimer(uint32_t timer_id);
    
    // 定时器状态查询
    bool isTimerActive(uint32_t timer_id);
    std::chrono::milliseconds getTimerRemaining(uint32_t timer_id);
    size_t getActiveTimerCount();
    
    // 批量操作
    void cancelAllTimers();
    void pauseAllTimers();
    void resumeAllTimers();
    
    // 状态查询
    bool isRunning() const { return running_.load(); }
    
    // 统计信息
    struct TimerStats {
        size_t total_timers;
        size_t active_timers;
        size_t paused_timers;
        std::chrono::steady_clock::time_point uptime;
    };
    TimerStats getStats() const;
    
    // 测试方法
    bool selfTest();
};

#endif // TIMERMANAGER_H