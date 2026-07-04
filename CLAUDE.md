# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Smart following suitcase firmware for ESP32-S3. Pure UWB follow mode (no obstacle avoidance). Uses FreeRTOS multi-task architecture with shared snapshot pattern for inter-task communication.

## Build Commands

```bash
idf.py set-target esp32s3
idf.py menuconfig   # Configure pins, PID, follow parameters under "Follow-only suitcase"
idf.py build
idf.py flash monitor
```

All configuration is done via Kconfig under `Follow-only suitcase` menu—no code changes needed for parameter tuning.

## Architecture

### Task Structure
- `uwb_task` (priority 6): UART1 receives UWB target position data, writes to shared struct
- `control_task` (priority 7): 50Hz loop—state machine (IDLE/SEARCH/FOLLOW), IMU heading correction, chassis drive
- IMU read is synchronous within control_task, not a separate task

### Shared State Pattern
Global `g_shared` struct with mutex protects UWB data between tasks:
```c
typedef struct {
    SemaphoreHandle_t lock;
    float tgt_distance_m;
    float tgt_bearing_rad;
    uint64_t tgt_ts_us;
} shared_t;
```

### Component Layout
- `components/sensors/bu_uwb/` - BU03/BU04 UWB driver (UART, TWR JSON parsing)
- `components/sensors/imu_i2c/` - 9-axis IMU driver (I2C, quaternion/euler output)
- `components/control/chassis/` - Closed-loop diff-drive (ESC PWM + encoder PID + feed-forward)
- `examples/follow_only/main/main.c` - Main application with state machine

### Chassis Control Flow
`chassis_set_velocity(v, omega)` → differential drive mix → per-wheel PID → feed-forward + PID correction → ESC pulse with slew limiting

## Key Conventions

- All runtime parameters configurable via Kconfig (PID gains, pins, distances, timeouts)
- Encoder `TICKS_PER_METER` must be calibrated per robot (push 1m, read tick delta)
- `FR_*` defines in main.c control sign inversion for motors/encoders/UWB/IMU—adjust for hardware wiring
- Data logging writes CSV to SD card at `/sdcard/test-data/follow_*.log` when enabled in Kconfig
- Coordinate convention: v > 0 = forward, omega > 0 = turn LEFT (CCW)

## Workspace Rules (from AGENTS.md)

- Treat this folder as the project root
- Keep temporary analysis in `work/`
- Keep user-facing generated files in `outputs/`
- Put durable notes and documentation in `docs/`
- Prefer small, focused changes and verify before calling complete
