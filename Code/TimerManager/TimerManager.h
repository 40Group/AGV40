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
    
    // The timer manages the main loop
    void timerLoop();
    
    // Get the next timer that needs to be executed
    std::shared_ptr<TimerInfo> getNextTimer();

public:
    TimerManager();
    ~TimerManager();
    
    bool initialize();
    void shutdown();
    
    // Timer creation and management
    uint32_t createTimer(std::chrono::milliseconds interval, 
                        TimerCallback callback, 
                        bool repeating = true);
    
    uint32_t createOneShotTimer(std::chrono::milliseconds delay, 
                               TimerCallback callback);
    
    bool cancelTimer(uint32_t timer_id);
    bool pauseTimer(uint32_t timer_id);
    bool resumeTimer(uint32_t timer_id);
    
    // Timer status query
    bool isTimerActive(uint32_t timer_id);
    std::chrono::milliseconds getTimerRemaining(uint32_t timer_id);
    size_t getActiveTimerCount();
    
    // Bulk operations
    void cancelAllTimers();
    void pauseAllTimers();
    void resumeAllTimers();
    
    // Status inquiry
    bool isRunning() const { return running_.load(); }
    
    // Statistics
    struct TimerStats {
        size_t total_timers;
        size_t active_timers;
        size_t paused_timers;
        std::chrono::steady_clock::time_point uptime;
    };
    TimerStats getStats() const;
    
    // Test Method:
    bool selfTest();
};

#endif // TIMERMANAGER_H
