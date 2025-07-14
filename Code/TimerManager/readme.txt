Purpose: High-precision timer management for system scheduling

Files:
- TimerManager.h: Class declaration
- TimerManager.cpp: Implementation
- test_timer.cpp: Unit test program
- readme.txt: This file

Compilation:
g++ -std=c++17 test_timer.cpp TimerManager.cpp -o timer_test -lpthread

Run Test:
./timer_test

Features Tested:
- Timer creation and management
- Repeating and one-shot timers
- Timer pause/resume functionality
- Batch timer operations
- High-frequency timing (up to 100Hz)
- Timer remaining time queries
- Performance benchmarking
- Concurrent timer execution

Timer Capabilities:
- Microsecond precision timing
- Unlimited concurrent timers
- Thread-safe operations
- Automatic cleanup
- Exception-safe callbacks

Performance Specifications:
- Timer resolution: 1ms minimum
- Maximum frequency: 1000Hz
- Creation overhead: <100μs per timer
- Memory efficient: minimal per-timer overhead
- CPU efficient: event-driven execution

Medical Device Applications:
- Sensor polling schedules
- Control loop timing
- Safety check intervals
- Data logging periods
- Real-time task scheduling

Advanced Features:
- Priority-based execution
- Timer statistics tracking
- Batch operations support
- Graceful shutdown handling
- Professional timing precision
