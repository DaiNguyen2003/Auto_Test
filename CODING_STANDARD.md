# STM32 Senior Coding Standard & Architecture Rules

## 1. Core Philosophy
*   **Minimal main()**: `main()` should only contain chip/HAL init, clock config, system init, and the super-loop dispatcher.
*   **Non-Blocking**: Absolute prohibition of `HAL_Delay()` in runtime. Use tick-based timers or state machines.
*   **Deterministic Timing**: Logic must rely on a scheduler or state machine, not "feeling-based" delays.
*   **Modularization**: One module per task group. No cross-calling chaos.
*   **Hierarchy**: Clear separation between Hardware Layer (BSP) -> Service Layer -> Application Layer.

## 2. Project Structure (Senior Style)
```
/Core           - STM32 auto-generated core files
/App            - Business logic (app_main, app_vehicle, etc.)
/Bsp            - Low-level hardware drivers (gpio, uart, can, pwm)
/Drivers        - Vendor HAL/CMSIS drivers
/Config         - app_config.h, hw_map.h (pin mapping, task periods)
/Common         - Utils, scheduler, state_machine logic
```

## 3. The "Main" Rule
A clean `main()` looks like this:
```c
int main(void) {
    HAL_Init();
    SystemClock_Config();
    BSP_Init();
    APP_Init();
    while (1) {
        APP_Run(); // Dispatches tasks
    }
}
```

## 4. Task Management & Scheduling
*   **Task Cycles**: 
    *   **1ms**: Fast IRQ/Input scanning.
    *   **5ms**: Motor/Servo ramp updates.
    *   **10ms**: Debounce & signal filtering.
    *   **50ms**: Primary State Machine.
    *   **100ms+**: UI, logging, health monitor.
*   **Cooperative Scheduling**: Tasks must execute quickly and return CPU. Never use `while(!done)` in background tasks.

## 5. Data Integrity & Concurrency
*   **Single-Writer Rule**: A variable/state is only writable by its owner module (e.g., only `GearManager` writes to `gear_cmd`).
*   **Read-Only Interface**: Use Getter/Setter functions. Never access internal module variables directly via `extern`.
*   **Volatile Keyword**: Mandatory for interrupt flags and hardware-updated counters.
*   **Atomic Access**: Protect shared multi-byte data by briefly disabling interrupts or using queues.

## 6. Interrupt (ISR) Rules
*   **Fast & Lean**: Clear flags, push to ring buffer, or set a flag.
*   **No Heavy Lifting**: No `printf`, no protocol parsing, no `HAL_Delay`, no `malloc` inside ISR.
*   **Deferred Processing**: Shift complex parsing from ISR to the background/scheduler tasks.

## 7. State Machine Architecture
*   Mandatory for complex sequences (e.g., Brake -> Wait -> Gear Shift -> CAN Confirm).
*   Avoid long nested `if-else` blocks.
*   Each state must have: Entry logic, Running logic, Exit logic, and Timeout handling.

## 8. Safety & Fault Handling
*   **Health Monitor**: Track "Heartbeat" for CAN, UART, and critical tasks.
*   **Safe State**: On timeout or error, actuators (Servos/Motors) must move to a predefined "Safe Position".
*   **Watchdog**: Always enabled. Fed by the main scheduler if all critical tasks are alive.

## 9. Coding Guidelines
*   **Prefixes**: Every function/variable in a module should have a prefix (e.g., `Servo_Init`, `Brake_SetPos`).
*   **Static Scope**: Keep internal variables and functions `static`. Only expose the API.
*   **No Magic Numbers**: Use `enum` or `#define`.
*   **Comments**: Explain *why*, not just *what*.

---
**Standard mandated by Antigravity (Senior AI Coding Assistant). Every future code modification must validate against these entries.**
