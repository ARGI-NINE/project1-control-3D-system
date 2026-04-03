# Master Bench Source

This directory contains the STM32F103C8T6 master-side source code used by the bench firmware.

## Scope

- `User/main.c`
- `User/Timer.c` / `User/Timer.h`
- `User/master_tick.h`
- `Hardware/app/*`
- `Hardware/comm/*`
- `Hardware/common/*`
- `Hardware/control/*`
- `Hardware/diag/*`
- `Hardware/fusion/*`
- `Hardware/imu/*`
- `Hardware/oled/*`

It does not include the Keil project files, StdPeriph library sources, or generated build artifacts.

## Tick Design

The project uses a timer-driven tick:

- `MASTER_TICK_HZ = 400`
- Tick period: `2.5 ms`
- `MASTER_TASK_400HZ_DIV = 1` -> `400 Hz` control cadence
- `MASTER_TASK_CAN_200HZ_DIV = 2` -> `200 Hz`
- `MASTER_TASK_LOG_1HZ_DIV = 400` -> `1 Hz`

This is intentional. The base interrupt rate matches the control loop cadence directly. The ISR remains lightweight (tick accounting and task pending flags), while control/CAN/logic runs in `app_loop()`.

If the interrupt handler starts doing heavy work, the correct fix is to move that work back into task context.

## Key Configuration

`Hardware/common/include/project_config.h` contains the main timing constants:

- `MASTER_CTRL_DT_S = 0.0025f`
- `MASTER_CTRL_DT_MIN_S = 0.001f`
- `MASTER_CTRL_DT_MAX_S = 0.020f`
- `MASTER_DIAG_400HZ_BUDGET_US = 2500u`
- `MASTER_DIAG_STATUS_TIMEOUT_MS = 100u`
- `MASTER_DIAG_ENABLE_OLED = 1u`

## Timer Entry

`User/master_tick.h` exposes the timer ISR entry point:

```c
void master_tick_isr(void);
```

Call it from the timer update interrupt configured at `MASTER_TICK_HZ`.

## Integration Note

- Keep shared STM32 StdPeriph/CMSIS files in your main Keil project. This bench folder only provides project logic.
- Ensure there is only one `TIM2_IRQHandler` in the final firmware image. Keep the implementation in `User/Timer.c` and remove duplicate handlers from other source files.

## Peripheral Node Check

- Timer: `TIM2` update interrupt, no remap requirement.
- CAN: `CAN1` with `PA11(RX)` / `PA12(TX)`, remap disabled in code.
- CAN bit timing follows your CAN examples: `Prescaler=48`, `BS1=2tq`, `BS2=3tq`, `SJW=2tq` (125kbps @ APB1=36MHz).
- IMU bus follows your MPU6050 hardware-I2C example: `I2C2` on `PB10(SCL)` / `PB11(SDA)`.
- MPU6050 DMA RX uses `DMA1_Channel5` (F103 mapping for `I2C2_RX`).

If your board is wired to `I2C1 PB6/PB7` instead, you must switch bus instance, pins, and DMA mapping together.

## OLED Diagnostics

The optional OLED diagnostics page displays:

- last/max execution time for 400 Hz, 200 Hz, and 1 Hz hooks
- 400 Hz budget overrun count
- scheduler overrun and pending counters
- CAN receive health
- axis timeout and missing masks

## Delay Success Criteria

For delay verification on real hardware, treat timing as successful only when all of the following stay true in your observation window:

- `timing_passed == 1` in `perf_snapshot_t`
- `timing_fail_mask == 0`
- `over_budget_400hz == 0`
- `overrun_400hz == 0`, `overrun_can_200hz == 0`, `overrun_log_1hz == 0`
- `pending_400hz == 0`, `pending_can_200hz == 0`, `pending_log_1hz == 0`

OLED quick-check:

- `TY` means pass, `TN` means fail
- `Fxx` shows fail bits in hex

`timing_fail_mask` bit definition:

- bit0: 400Hz hook exceeded budget
- bit1: 400Hz task overrun
- bit2: 200Hz task overrun
- bit3: 1Hz task overrun
- bit4: 400Hz task still pending
- bit5: 200Hz task still pending
- bit6: 1Hz task still pending
