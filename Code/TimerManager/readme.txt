Real Event-Driven Timer Manager Module

PURPOSE:
Hardware-precision event scheduling system using REAL event-driven architecture (NO POLLING)


FEATURES:
Real event-driven timing (NO polling loops)
Priority queue event scheduling
Condition variable driven execution
Microsecond precision timing
Thread-safe event management
Automatic event cleanup

REAL EVENT-DRIVEN OPERATION:
Priority queue for event scheduling (not polling!)
Condition variable blocking until next event
sleep_until() for precise timing (not sleep() intervals)
Immediate event execution without polling
Zero CPU usage when no events pending

TIMER TYPES:
Repeating Timer: Auto-reschedules next event after execution
One-Shot Timer: Single event execution then cleanup
Immediate Event: Instant callback execution
Delayed Event: One-shot timer with specific delay

API USAGE (Event-Driven):
Create repeating: int id = manager.scheduleRepeating(100, callback)
Create one-shot: int id = manager.scheduleOnce(500, callback)
Immediate event: manager.triggerImmediateEvent(callback)
Delayed event: manager.scheduleDelayedEvent(callback, delay)
Remove timer: manager.removeTimer(id)

EVENT SCHEDULING FLOW:
Timer created → Event added to priority queue
Condition variable notifies event thread
Thread sleeps until next event time (sleep_until)
Event executed immediately when time reached
Repeating timers reschedule next event
One-shot timers auto-cleanup

CALLBACK REQUIREMENTS:
Must return quickly (< 1ms recommended)
No blocking operations allowed
Exception-safe implementation required
Thread-safe if accessing shared data

PERFORMANCE SPECIFICATIONS:
Timer resolution: 1μs theoretical (system limited)
Event scheduling overhead: <10μs
Callback latency: <100μs typical
Maximum concurrent timers: System memory limited
CPU usage when idle: Near zero (event-driven)

EVENT TIMING PRECISION:
Uses std::chrono::steady_clock for monotonic timing
sleep_until() for precise event scheduling
Priority queue ensures correct event ordering
Condition variables for immediate event notification
No drift accumulation (each event scheduled independently)

USAGE PATTERNS:
Sensor control: scheduleRepeating(100ms, readSensor)
Timeout handling: scheduleOnce(5000ms, timeoutCallback)
Immediate response: triggerImmediateEvent(emergencyCallback)
Delayed action: scheduleDelayedEvent(shutdownCallback, 2000ms)

INTEGRATION WITH OTHER MODULES:
// Temperature control events
timer.scheduleRepeating(1000, [&tempController]() {
    tempController.processTemperatureControlEvent();
});

// Safety monitoring events  
timer.scheduleRepeating(500, [&safety]() {
    safety.checkOperationTimeEvent();
});

// Vision processing events
timer.scheduleRepeating(100, [&vision]() {
    vision.processVisionEvent();
});
