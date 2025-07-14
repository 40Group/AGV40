#include "../include/TimerManager.h"
#include <algorithm>

TimerManager::TimerManager() : running_(false), next_timer_id_(1) {
}

TimerManager::~TimerManager() {
    shutdown();
}

bool TimerManager::initialize() {
    if (running_.load()) {
        std::cout << "TimerManager already initialized" << std::endl;
        return true;
    }
    
    running_ = true;
    timer_thread_ = std::thread(&TimerManager::timerLoop, this);
    
    std::cout << "TimerManager initialized successfully" << std::endl;
    return true;
}

void TimerManager::shutdown() {
    if (!running_.load()) {
        return;
    }
    
    running_ = false;
    timer_cv_.notify_all();
    
    if (timer_thread_.joinable()) {
        timer_thread_.join();
    }
    
    // Clear all timers
    {
        std::lock_guard<std::mutex> lock(timers_mutex_);
        timers_.clear();
    }
    
    std::cout << "TimerManager shutdown" << std::endl;
}

void TimerManager::timerLoop() {
    while (running_.load()) {
        try {
            auto next_timer = getNextTimer();
            
            if (!next_timer) {
                // There is no active timer, wait for a new timer or exit signal
                std::unique_lock<std::mutex> lock(timers_mutex_);
                timer_cv_.wait_for(lock, std::chrono::milliseconds(1000));
                continue;
            }
            
            auto now = std::chrono::steady_clock::now();
            
            if (now >= next_timer->next_execution) {
                // Execute a timer callback
                try {
                    if (next_timer->callback) {
                        next_timer->callback();
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Timer callback exception: " << e.what() << std::endl;
                }
                
                // Update the timer status
                {
                    std::lock_guard<std::mutex> lock(timers_mutex_);
                    
                    if (next_timer->repeating && next_timer->active) {
                        // Repeat the timer to calculate the next execution time
                        next_timer->next_execution = now + next_timer->interval;
                    } else {
                        // One-time timer or deactivated, marked as inactive
                        next_timer->active = false;
                    }
                }
            } else {
                // Wait until the next execution time
                auto sleep_duration = next_timer->next_execution - now;
                std::this_thread::sleep_for(sleep_duration);
            }
            
        } catch (const std::exception& e) {
            std::cerr << "TimerManager loop exception: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

std::shared_ptr<TimerManager::TimerInfo> TimerManager::getNextTimer() {
    std::lock_guard<std::mutex> lock(timers_mutex_);
    
    std::shared_ptr<TimerInfo> next_timer = nullptr;
    
    // Clean up deactivated timers
    timers_.erase(
        std::remove_if(timers_.begin(), timers_.end(),
                      [](const std::shared_ptr<TimerInfo>& timer) {
                          return !timer->active;
                      }),
        timers_.end()
    );
    
    // Find the next timer that needs to be executed
    for (auto& timer : timers_) {
        if (timer->active) {
            if (!next_timer || timer->next_execution < next_timer->next_execution) {
                next_timer = timer;
            }
        }
    }
    
    return next_timer;
}

uint32_t TimerManager::createTimer(std::chrono::milliseconds interval, 
                                  TimerCallback callback, 
                                  bool repeating) {
    if (!callback) {
        std::cerr << "TimerManager: Invalid callback" << std::endl;
        return 0;
    }
    
    std::lock_guard<std::mutex> lock(timers_mutex_);
    
    auto timer = std::make_shared<TimerInfo>();
    timer->id = next_timer_id_++;
    timer->interval = interval;
    timer->callback = callback;
    timer->repeating = repeating;
    timer->active = true;
    timer->created_time = std::chrono::steady_clock::now();
    timer->next_execution = timer->created_time + interval;
    
    timers_.push_back(timer);
    
    // Notifies the timer thread that there is a new timer
    timer_cv_.notify_one();
    
    std::cout << "Created timer " << timer->id << " with interval " 
              << interval.count() << "ms" << std::endl;
    
    return timer->id;
}

uint32_t TimerManager::createOneShotTimer(std::chrono::milliseconds delay, 
                                         TimerCallback callback) {
    return createTimer(delay, callback, false);
}

bool TimerManager::cancelTimer(uint32_t timer_id) {
    std::lock_guard<std::mutex> lock(timers_mutex_);
    
    for (auto& timer : timers_) {
        if (timer->id == timer_id) {
            timer->active = false;
            std::cout << "Timer " << timer_id << " cancelled" << std::endl;
            return true;
        }
    }
    
    return false;
}

bool TimerManager::pauseTimer(uint32_t timer_id) {
    std::lock_guard<std::mutex> lock(timers_mutex_);
    
    for (auto& timer : timers_) {
        if (timer->id == timer_id && timer->active) {
            timer->active = false;
            std::cout << "Timer " << timer_id << " paused" << std::endl;
            return true;
        }
    }
    
    return false;
}

bool TimerManager::resumeTimer(uint32_t timer_id) {
    std::lock_guard<std::mutex> lock(timers_mutex_);
    
    for (auto& timer : timers_) {
        if (timer->id == timer_id && !timer->active) {
            timer->active = true;
            timer->next_execution = std::chrono::steady_clock::now() + timer->interval;
            timer_cv_.notify_one();
            std::cout << "Timer " << timer_id << " resumed" << std::endl;
            return true;
        }
    }
    
    return false;
}

bool TimerManager::isTimerActive(uint32_t timer_id) {
    std::lock_guard<std::mutex> lock(timers_mutex_);
    
    for (const auto& timer : timers_) {
        if (timer->id == timer_id) {
            return timer->active;
        }
    }
    
    return false;
}

std::chrono::milliseconds TimerManager::getTimerRemaining(uint32_t timer_id) {
    std::lock_guard<std::mutex> lock(timers_mutex_);
    
    auto now = std::chrono::steady_clock::now();
    
    for (const auto& timer : timers_) {
        if (timer->id == timer_id && timer->active) {
            auto remaining = timer->next_execution - now;
            return std::chrono::duration_cast<std::chrono::milliseconds>(remaining);
        }
    }
    
    return std::chrono::milliseconds(0);
}

size_t TimerManager::getActiveTimerCount() {
    std::lock_guard<std::mutex> lock(timers_mutex_);
    
    size_t count = 0;
    for (const auto& timer : timers_) {
        if (timer->active) {
            count++;
        }
    }
    
    return count;
}

void TimerManager::cancelAllTimers() {
    std::lock_guard<std::mutex> lock(timers_mutex_);
    
    for (auto& timer : timers_) {
        timer->active = false;
    }
    
    std::cout << "All timers cancelled" << std::endl;
}

void TimerManager::pauseAllTimers() {
    std::lock_guard<std::mutex> lock(timers_mutex_);
    
    for (auto& timer : timers_) {
        timer->active = false;
    }
    
    std::cout << "All timers paused" << std::endl;
}

void TimerManager::resumeAllTimers() {
    std::lock_guard<std::mutex> lock(timers_mutex_);
    
    auto now = std::chrono::steady_clock::now();
    
    for (auto& timer : timers_) {
        timer->active = true;
        timer->next_execution = now + timer->interval;
    }
    
    timer_cv_.notify_all();
    std::cout << "All timers resumed" << std::endl;
}

TimerManager::TimerStats TimerManager::getStats() const {
    std::lock_guard<std::mutex> lock(timers_mutex_);
    
    TimerStats stats;
    stats.total_timers = timers_.size();
    stats.active_timers = 0;
    stats.paused_timers = 0;
    stats.uptime = std::chrono::steady_clock::now();
    
    for (const auto& timer : timers_) {
        if (timer->active) {
            stats.active_timers++;
        } else {
            stats.paused_timers++;
        }
    }
    
    return stats;
}

bool TimerManager::selfTest() {
    std::cout << "Starting timer manager self-test..." << std::endl;
    
    if (!running_.load()) {
        std::cout << "Timer manager not initialized" << std::endl;
        return false;
    }
    
    // Test the one-time timer
    bool test1_executed = false;
    uint32_t timer1 = createOneShotTimer(
        std::chrono::milliseconds(100),
        [&test1_executed]() {
            test1_executed = true;
            std::cout << "One-shot timer executed" << std::endl;
        }
    );
    
    // Test the repeat timer
    int test2_count = 0;
    uint32_t timer2 = createTimer(
        std::chrono::milliseconds(50),
        [&test2_count]() {
            test2_count++;
            std::cout << "Repeating timer executed: " << test2_count << std::endl;
        },
        true
    );
    
    // Wait for execution
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    
    // Stop repeating the timer
    cancelTimer(timer2);
    
    // Check the results
    bool test_passed = test1_executed && test2_count >= 3;
    
    std::cout << "Timer manager self-test " 
              << (test_passed ? "PASSED" : "FAILED") << std::endl;
    std::cout << "Active timers: " << getActiveTimerCount() << std::endl;
    
    return test_passed;
}
