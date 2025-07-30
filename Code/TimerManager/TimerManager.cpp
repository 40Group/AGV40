#include "TimerManager.h"

TimerManager::TimerManager() : running_(false), next_timer_id_(1) {
}

TimerManager::~TimerManager() {
    stop();
}

bool TimerManager::initialize() {
    try {
        LOG_INFO("TimerManager initialized with REAL event-driven architecture");
        return true;
    } catch (const std::exception& e) {
        LOG_ERROR("TimerManager initialization failed: " + std::string(e.what()));
        return false;
    }
}

void TimerManager::start() {
    if (running_.load()) {
        return;
    }
    
    running_ = true;
    
    // Start a real event-driven thread
    event_thread_ = std::thread(&TimerManager::realEventDrivenLoop, this);
    
    LOG_INFO("TimerManager started with REAL event-driven timing");
}

void TimerManager::stop() {
    running_ = false;
    
    // Notify the event thread to exit
    event_condition_.notify_all();
    
    if (event_thread_.joinable()) {
        event_thread_.join();
    }
    
    // Clean up all timers
    {
        std::lock_guard<std::mutex> lock1(queue_mutex_);
        std::lock_guard<std::mutex> lock2(timers_mutex_);
        
        // Clear the incident queue
        std::priority_queue<TimerEvent> empty_queue;
        event_queue_.swap(empty_queue);
        
        active_timers_.clear();
    }
    
    LOG_INFO("TimerManager stopped");
}

void TimerManager::realEventDrivenLoop() {
    LOG_INFO("Real event-driven timer loop started - NO POLLING!");
    
    while (running_.load()) {
        try {
            TimerEvent next_event{0, std::chrono::steady_clock::time_point::max(), 
                                 std::chrono::milliseconds(0), nullptr, false};
            bool has_event = false;
            
            // Get the next event to trigger
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                
                if (!event_queue_.empty()) {
                    next_event = event_queue_.top();
                    event_queue_.pop();
                    has_event = true;
                }
            }
            
            if (!has_event) {
                // When there are no events, wait for a new event or exit signal
                std::unique_lock<std::mutex> lock(queue_mutex_);
                event_condition_.wait(lock, [this] { 
                    return !running_.load() || !event_queue_.empty(); 
                });
                continue;
            }
            
            // Wait until the event triggers time
            auto now = std::chrono::steady_clock::now();
            if (next_event.trigger_time > now) {
                std::this_thread::sleep_until(next_event.trigger_time);
            }
            
            // Check if the timer is still active
            bool timer_active = false;
            {
                std::lock_guard<std::mutex> lock(timers_mutex_);
                auto it = active_timers_.find(next_event.timer_id);
                timer_active = (it != active_timers_.end() && it->second);
            }
            
            if (timer_active && running_.load()) {
                // Handling Events
                processTimerEvent(next_event);
            
                if (next_event.repeat) {
                    scheduleNextTimerEvent(next_event);
                }
            }
            
        } catch (const std::exception& e) {
            LOG_ERROR("Real event-driven timer error: " + std::string(e.what()));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    LOG_INFO("Real event-driven timer loop terminated");
}

void TimerManager::processTimerEvent(const TimerEvent& event) {
    try {
        if (event.callback && running_.load() && !g_emergency_stop.load()) {
            // Immediate Execution of Callbacks
            event.callback();
            LOG_DEBUG("Timer event " + std::to_string(event.timer_id) + " executed");
        }
    } catch (const std::exception& e) {
        LOG_ERROR("Timer callback error: " + std::string(e.what()));
    }
}

void TimerManager::scheduleNextTimerEvent(const TimerEvent& event) {
    // Calculate the next trigger time
    auto next_trigger = std::chrono::steady_clock::now() + event.interval;
    
    TimerEvent next_event{event.timer_id, next_trigger, event.interval, 
                         event.callback, event.repeat};
    
    // Queue the next event
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        event_queue_.push(next_event);
    }
    
    // Notify the event thread that there is a new event
    event_condition_.notify_one();
}

int TimerManager::createTimer(std::chrono::milliseconds interval, TimerCallback callback, bool repeat) {
    if (!callback) {
        LOG_ERROR("Timer callback cannot be null");
        return -1;
    }
    
    int timer_id = next_timer_id_.fetch_add(1);
    auto trigger_time = std::chrono::steady_clock::now() + interval;
    
    // Create a timer event
    TimerEvent event{timer_id, trigger_time, interval, callback, repeat};
    
    // Register the timer
    {
        std::lock_guard<std::mutex> lock(timers_mutex_);
        active_timers_[timer_id] = true;
    }
    
    // Schedule events
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        event_queue_.push(event);
    }
    
    // Notify the event thread
    event_condition_.notify_one();
    
    LOG_INFO("Created event-driven timer " + std::to_string(timer_id) + 
             " with interval " + std::to_string(interval.count()) + "ms");
    
    return timer_id;
}

int TimerManager::createOneShotTimer(std::chrono::milliseconds delay, TimerCallback callback) {
    return createTimer(delay, callback, false);
}

void TimerManager::removeTimer(int timer_id) {
    std::lock_guard<std::mutex> lock(timers_mutex_);
    
    auto it = active_timers_.find(timer_id);
    if (it != active_timers_.end()) {
        it->second = false;  
        active_timers_.erase(it);
        LOG_INFO("Removed timer " + std::to_string(timer_id));
    }
}

void TimerManager::pauseTimer(int timer_id) {
    std::lock_guard<std::mutex> lock(timers_mutex_);
    
    auto it = active_timers_.find(timer_id);
    if (it != active_timers_.end()) {
        it->second = false;
        LOG_INFO("Paused timer " + std::to_string(timer_id));
    }
}

void TimerManager::resumeTimer(int timer_id) {
    std::lock_guard<std::mutex> lock(timers_mutex_);
    
    auto it = active_timers_.find(timer_id);
    if (it != active_timers_.end()) {
        it->second = true;
        LOG_INFO("Resumed timer " + std::to_string(timer_id));
    }
}

int TimerManager::scheduleRepeating(int interval_ms, TimerCallback callback) {
    return createTimer(std::chrono::milliseconds(interval_ms), callback, true);
}

int TimerManager::scheduleOnce(int delay_ms, TimerCallback callback) {
    return createTimer(std::chrono::milliseconds(delay_ms), callback, false);
}

void TimerManager::triggerImmediateEvent(TimerCallback callback) {
    if (!callback) return;
    
    // Trigger an event immediately 
    try {
        if (running_.load() && !g_emergency_stop.load()) {
            callback();
            LOG_DEBUG("Immediate event triggered");
        }
    } catch (const std::exception& e) {
        LOG_ERROR("Immediate event error: " + std::string(e.what()));
    }
}

void TimerManager::scheduleDelayedEvent(TimerCallback callback, std::chrono::milliseconds delay) {
    // Create a one-time timer
    createOneShotTimer(delay, callback);
}

bool TimerManager::isTimerActive(int timer_id) {
    std::lock_guard<std::mutex> lock(timers_mutex_);
    
    auto it = active_timers_.find(timer_id);
    return (it != active_timers_.end()) && it->second;
}

int TimerManager::getActiveTimerCount() {
    std::lock_guard<std::mutex> lock(timers_mutex_);
    
    int count = 0;
    for (const auto& [id, active] : active_timers_) {
        if (active) count++;
    }
    return count;
}

int TimerManager::getPendingEventCount() {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return static_cast<int>(event_queue_.size());
}
