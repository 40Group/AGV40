#ifndef TIMERMANAGER_H
#define TIMERMANAGER_H

#include "../common/Common.h"
#include <map>
#include <queue>
#include <condition_variable>

class TimerManager {
private:
    struct TimerEvent {
        int timer_id;
        std::chrono::steady_clock::time_point trigger_time;
        std::chrono::milliseconds interval;
        TimerCallback callback;
        bool repeat;
        bool active;
        
        TimerEvent(int id, std::chrono::steady_clock::time_point trigger, 
                  std::chrono::milliseconds int_val, TimerCallback cb, bool rep)
            : timer_id(id), trigger_time(trigger), interval(int_val), 
              callback(cb), repeat(rep), active(true) {}
        
        // Priority queue sorting: The earliest triggered event takes precedence
        bool operator<(const TimerEvent& other) const {
            return trigger_time > other.trigger_time;  
        }
    };
    
    // Real-world event-driven components
    std::thread event_thread_;
    std::atomic<bool> running_;
    
    // Event-driven priority queues
    std::priority_queue<TimerEvent> event_queue_;
    std::mutex queue_mutex_;
    std::condition_variable event_condition_;
    
    // Timer management
    std::map<int, bool> active_timers_;  
    std::mutex timers_mutex_;
    std::atomic<int> next_timer_id_;
    
    // Real-world event-driven approach
    void realEventDrivenLoop();
    void scheduleNextTimerEvent(const TimerEvent& event);
    void processTimerEvent(const TimerEvent& event);
    
public:
    TimerManager();
    ~TimerManager();
    
    // Initialization and control
    bool initialize();
    void start();
    void stop();
    
    // Event-driven timer management
    int createTimer(std::chrono::milliseconds interval, TimerCallback callback, bool repeat = true);
    int createOneShotTimer(std::chrono::milliseconds delay, TimerCallback callback);
    void removeTimer(int timer_id);
    void pauseTimer(int timer_id);
    void resumeTimer(int timer_id);
    
    // Convenient way
    int scheduleRepeating(int interval_ms, TimerCallback callback);
    int scheduleOnce(int delay_ms, TimerCallback callback);
    
    // Trigger an event immediately
    void triggerImmediateEvent(TimerCallback callback);
    void scheduleDelayedEvent(TimerCallback callback, std::chrono::milliseconds delay);
    
    // Status inquiry
    bool isTimerActive(int timer_id);
    int getActiveTimerCount();
    int getPendingEventCount();
};

#endif // TIMERMANAGER_H
