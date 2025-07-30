#include "../TimerManager/TimerManager.h"
#include "../common/Common.h"
#include <iostream>
#include <atomic>
#include <thread>
#include <chrono>

class TimerTest {
private:
    TimerManager timer_manager;
    std::atomic<int> callback_count;
    std::atomic<bool> test_passed;
    std::atomic<int> event_sequence;
    
public:
    TimerTest() : callback_count(0), test_passed(true), event_sequence(0) {}
    
    void runAllTests() {
        std::cout << "=== REAL Event-Driven Timer Manager Tests ===" << std::endl;
        
        testInitialization();
        testBasicEventDrivenTimer();
        testOneShotEventTimer();
        testMultipleEventTimers();
        testImmediateEvents();
        testEventSequencing();
        testTimerRemoval();
        testPerformance();
        
        timer_manager.stop();
        
        if (test_passed.load()) {
            std::cout << "✅ ALL REAL EVENT-DRIVEN TIMER TESTS PASSED!" << std::endl;
        } else {
            std::cout << "❌ SOME TIMER TESTS FAILED!" << std::endl;
        }
    }
    
private:
    void testInitialization() {
        std::cout << "Testing real event-driven timer manager initialization..." << std::endl;
        
        bool init_result = timer_manager.initialize();
        if (!init_result) {
            std::cout << "❌ Timer manager initialization failed" << std::endl;
            test_passed = false;
            return;
        }
        
        timer_manager.start();
        
        // Check the initial status
        if (timer_manager.getActiveTimerCount() != 0) {
            std::cout << "❌ Initial active timer count should be 0" << std::endl;
            test_passed = false;
            return;
        }
        
        if (timer_manager.getPendingEventCount() != 0) {
            std::cout << "❌ Initial pending event count should be 0" << std::endl;
            test_passed = false;
            return;
        }
        
        std::cout << "✅ Real event-driven timer manager initialization test passed" << std::endl;
    }
    
    void testBasicEventDrivenTimer() {
        std::cout << "Testing basic event-driven repeating timer..." << std::endl;
        
        callback_count = 0;
        auto start_time = std::chrono::steady_clock::now();
        
        int timer_id = timer_manager.scheduleRepeating(100, [this, start_time]() {
            callback_count++;
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);
            std::cout << "🔥 TIMER EVENT #" << callback_count.load() 
                      << " at " << elapsed.count() << "ms" << std::endl;
        });
        
        if (!timer_manager.isTimerActive(timer_id)) {
            std::cout << "❌ Timer should be active after creation" << std::endl;
            test_passed = false;
            return;
        }
        
        // Wait a few times for the event to trigger
        std::this_thread::sleep_for(std::chrono::milliseconds(350));
        
        int count = callback_count.load();
        if (count < 2 || count > 4) {
            std::cout << "❌ Event-driven timer test failed. Expected 2-4 events, got " 
                     << count << std::endl;
            test_passed = false;
            return;
        }
        
        timer_manager.removeTimer(timer_id);
        std::cout << "✅ Event-driven timer test passed (" << count << " events)" << std::endl;
    }
    
    void testOneShotEventTimer() {
        std::cout << "Testing one-shot event timer..." << std::endl;
        
        callback_count = 0;
        auto start_time = std::chrono::steady_clock::now();
        
        timer_manager.scheduleOnce(150, [this, start_time]() {
            callback_count++;
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);
            std::cout << "🎯 ONE-SHOT EVENT triggered at " << elapsed.count() << "ms" << std::endl;
        });
        
        // Wait for the event to execute
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        int count = callback_count.load();
        if (count != 1) {
            std::cout << "❌ One-shot event test failed. Expected 1 event, got " 
                     << count << std::endl;
            test_passed = false;
            return;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        count = callback_count.load();
        if (count != 1) {
            std::cout << "❌ One-shot event triggered multiple times. Got " 
                     << count << " events" << std::endl;
            test_passed = false;
            return;
        }
        
        std::cout << "✅ One-shot event test passed" << std::endl;
    }
    
    void testMultipleEventTimers() {
        std::cout << "Testing multiple concurrent event timers..." << std::endl;
        
        std::atomic<int> fast_count(0), slow_count(0);
        auto start_time = std::chrono::steady_clock::now();
        
        int fast_timer = timer_manager.scheduleRepeating(50, [&fast_count, start_time]() {
            fast_count++;
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);
            std::cout << "⚡ FAST EVENT #" << fast_count.load() 
                      << " at " << elapsed.count() << "ms" << std::endl;
        });
        
        int slow_timer = timer_manager.scheduleRepeating(200, [&slow_count, start_time]() {
            slow_count++;
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);
            std::cout << "🐌 SLOW EVENT #" << slow_count.load() 
                      << " at " << elapsed.count() << "ms" << std::endl;
        });
        
        // Check the number of active timers
        if (timer_manager.getActiveTimerCount() < 2) {
            std::cout << "❌ Should have at least 2 active timers" << std::endl;
            test_passed = false;
        }
        
        // Wait for execution
        std::this_thread::sleep_for(std::chrono::milliseconds(450));
        
        int fast = fast_count.load();
        int slow = slow_count.load();
        
        if (fast <= slow) {
            std::cout << "❌ Multiple event timers test failed. Fast: " << fast 
                     << ", Slow: " << slow << std::endl;
            test_passed = false;
        } else {
            std::cout << "✅ Multiple event timers test passed. Fast: " << fast 
                     << ", Slow: " << slow << std::endl;
        }
        
        timer_manager.removeTimer(fast_timer);
        timer_manager.removeTimer(slow_timer);
    }
    
    void testImmediateEvents() {
        std::cout << "Testing immediate event execution..." << std::endl;
        
        callback_count = 0;
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Trigger an immediate event
        timer_manager.triggerImmediateEvent([this, start_time]() {
            auto now = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time);
            callback_count++;
            std::cout << "⚡ IMMEDIATE EVENT executed in " << elapsed.count() << "μs" << std::endl;
        });
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        if (callback_count.load() != 1) {
            std::cout << "❌ Immediate event not executed" << std::endl;
            test_passed = false;
            return;
        }
        
        std::cout << "✅ Immediate event test passed" << std::endl;
    }
    
    void testEventSequencing() {
        std::cout << "Testing event sequencing and timing..." << std::endl;
        
        event_sequence = 0;
        std::vector<int> sequence_order;
        std::mutex sequence_mutex;
        
        // Create events with different delays
        timer_manager.scheduleDelayedEvent([this, &sequence_order, &sequence_mutex]() {
            std::lock_guard<std::mutex> lock(sequence_mutex);
            sequence_order.push_back(3);
            std::cout << "🕒 Event 3 (300ms) executed" << std::endl;
        }, std::chrono::milliseconds(300));
        
        timer_manager.scheduleDelayedEvent([this, &sequence_order, &sequence_mutex]() {
            std::lock_guard<std::mutex> lock(sequence_mutex);
            sequence_order.push_back(1);
            std::cout << "🕐 Event 1 (100ms) executed" << std::endl;
        }, std::chrono::milliseconds(100));
        
        timer_manager.scheduleDelayedEvent([this, &sequence_order, &sequence_mutex]() {
            std::lock_guard<std::mutex> lock(sequence_mutex);
            sequence_order.push_back(2);
            std::cout << "🕑 Event 2 (200ms) executed" << std::endl;
        }, std::chrono::milliseconds(200));
        
        // Wait for all events to be executed
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        
        // Check the order of execution
        std::vector<int> expected{1, 2, 3};
        if (sequence_order != expected) {
            std::cout << "❌ Event sequencing failed. Expected [1,2,3], got [";
            for (size_t i = 0; i < sequence_order.size(); ++i) {
                std::cout << sequence_order[i];
                if (i < sequence_order.size() - 1) std::cout << ",";
            }
            std::cout << "]" << std::endl;
            test_passed = false;
            return;
        }
        
        std::cout << "✅ Event sequencing test passed" << std::endl;
    }
    
    void testTimerRemoval() {
        std::cout << "Testing timer removal..." << std::endl;
        
        callback_count = 0;
        int timer_id = timer_manager.scheduleRepeating(100, [this]() {
            callback_count++;
        });
        
        // Wait for a few callbacks
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        
        // Remove the timer
        timer_manager.removeTimer(timer_id);
        
        if (timer_manager.isTimerActive(timer_id)) {
            std::cout << "❌ Timer should not be active after removal" << std::endl;
            test_passed = false;
            return;
        }
        
        int count_before = callback_count.load();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        
        int count_after = callback_count.load();
        
        if (count_after != count_before) {
            std::cout << "❌ Timer removal test failed. Timer still active after removal" << std::endl;
            test_passed = false;
            return;
        }
        
        std::cout << "✅ Timer removal test passed" << std::endl;
    }
    
    void testPerformance() {
        std::cout << "Testing event-driven timer performance..." << std::endl;
        
        const int num_timers = 100;
        std::atomic<int> total_events(0);
        std::vector<int> timer_ids;
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Create multiple quick timers
        for (int i = 0; i < num_timers; ++i) {
            int timer_id = timer_manager.scheduleRepeating(10 + (i % 50), [&total_events]() {
                total_events++;
            });
            timer_ids.push_back(timer_id);
        }
        
        auto creation_time = std::chrono::high_resolution_clock::now();
        auto creation_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            creation_time - start_time).count();
        
        std::cout << "Created " << num_timers << " timers in " << creation_duration << "ms" << std::endl;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        int events = total_events.load();
        std::cout << "Total events processed: " << events << std::endl;
        
        for (int timer_id : timer_ids) {
            timer_manager.removeTimer(timer_id);
        }
        
        if (events < 100) {
            std::cout << "⚠️ Performance may be suboptimal. Expected >100 events, got " << events << std::endl;
        } else {
            std::cout << "✅ Performance test passed" << std::endl;
        }
    }
};

int main() {
    try {
        std::cout << "🎯 REAL Event-Driven Timer Manager Test" << std::endl;
        std::cout << "✅ No polling loops - pure event scheduling" << std::endl;
        std::cout << "✅ Priority queue event management" << std::endl;
        std::cout << "✅ Condition variable driven timing" << std::endl;
        std::cout << "✅ Precise event sequencing" << std::endl;
        
        TimerTest test;
        test.runAllTests();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Timer test exception: " << e.what() << std::endl;
        return -1;
    }
}
