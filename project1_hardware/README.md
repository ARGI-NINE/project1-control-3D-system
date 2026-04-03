# project1_hardware

Two independent STM32 StdPeriph firmware projects:

- `master/` - attitude estimation, control, and CAN command output
- `slave/` - servo position loop, CAN command receive, and PWM output

The shared `Hardware/common` sources are duplicated under each project so `master` and `slave` can be built as separate Keil targets with no cross-project file references.

## Layout

`master/`
- `Hardware/app` - `master_node`, scheduler
- `Hardware/comm` - CAN command/status transport
- `Hardware/control` - axis mapping, controllers A/B, rate limiting
- `Hardware/fusion` - Mahony filter and quaternion to Euler conversion
- `Hardware/imu` - MPU6050 over I2C2 with DMA
- `Hardware/common` - shared protocol, math, types, config
- `User/main.c` - startup and task wiring
- `User/Timer.c`, `User/Timer.h`, `User/master_tick.h` - TIM2 timebase

`slave/`
- `Hardware/app` - servo node state machine
- `Hardware/comm` - CAN receive / status transmit
- `Hardware/config` - axis-specific servo configuration
- `Hardware/driver` - TIM3 PWM output
- `Hardware/common` - shared protocol, math, types, config
- `User/main.c` - startup and task wiring

## Runtime Model

### Master

- `TIM2` update interrupt is the 400 Hz base tick (`MASTER_TICK_HZ = 400`, period = 2500 us).
- `Timer.c` calls `master_tick_isr()` from `TIM2_IRQHandler()`.
- `master_tick_isr()`:
  - advances the microsecond timebase
  - updates CAN time in milliseconds when the accumulated tick crosses 1 ms
  - calls `scheduler_tick_isr()`
- The scheduler splits the 400 Hz tick into:
  - 400 Hz control task: `MASTER_TASK_400HZ_DIV = 1`
  - 200 Hz CAN task: `MASTER_TASK_CAN_200HZ_DIV = 2`
- `app_loop()` services at most one pending 400 Hz task and one pending 200 Hz task per pass. Pending work is collapsed to a single execution and overruns are counted.
- `master_task_400hz()`:
  - polls the MPU6050 DMA sample
  - reuses the last valid IMU sample in `MASTER_RUN` if the current read is missing
  - calls `master_node_step_400hz()`
  - starts the next DMA read
- `master_node_step_400hz()` state flow:
  - `MASTER_INIT` - init MPU6050, zero bias, enter calibration
  - `MASTER_CALIB` - average gyro bias for 2.0 s
  - `MASTER_RUN` - update Mahony attitude estimate and run controller A or B
  - `MASTER_SAFE` - decay stored position references toward zero at 120 deg/s and hold CAN output at zero
- `main.c` selects controller A by default with `master_node_select_controller(false)` and sets the target to 0, 0, 0 deg at startup.
- `master_task_can_200hz()` sends the current command and polls received status frames.

### Slave

- `TIM2` update interrupt is the 200 Hz base tick (`SLAVE_TICK_HZ = 200`, period = 5000 us).
- `TIM2_IRQHandler()` calls `slave_tick_isr()`.
- `slave_tick_isr()` advances the millisecond counter by 5 ms and queues one pending 200 Hz servo task.
- The main loop continuously drains CAN FIFO0 with `can_rx_poll_hw()` and runs one servo update when a pending tick exists.
- `board_read_vbat_v()` is a weak hook in `User/main.c` and defaults to 6.0 V. Replace it in the board layer if real battery sensing is needed.
- `servo_node_init()`:
  - initializes TIM3 PWM on PA6
  - sets the PWM frequency from axis config, currently 50 Hz
  - centers the servo at the configured zero angle
  - publishes an initial status frame
- `servo_node_step_200hz()`:
  - clamps dt to the configured min/max range
  - enters `SLAVE_FAULT_VBAT_LOW` if VBAT drops below 4.7 V
  - times out after 100 ms of link silence since the last valid command
  - optionally returns to center over 0.2 s during timeout
  - applies velocity limiting and acceleration limiting before updating position
  - converts the final position to a servo pulse and publishes status every step
- `SLAVE_AXIS_INDEX` is 1, so the slave currently runs the pitch axis (`axis_id = 2`).

## CAN and PWM Behavior

- CAN uses standard 11-bit identifiers and 8-byte data frames.
- Command frame:
  - ID `0x100`
  - payload layout: `seq`, `yaw_ref_cdeg`, `pitch_ref_cdeg`, `roll_ref_cdeg`
- Status frames:
  - IDs `0x201`, `0x202`, `0x203`
  - payload layout: `seq_echo`, `pulse_us`, `cur_pos_cdeg`, `fault`, `vbat_01v`
- All 16-bit fields are little-endian.
- Master CAN setup:
  - CAN1 on PA11/PA12
  - normal mode
  - 125 kbps timing (`prescaler = 48`, `1 + 2 + 3` time quanta as coded)
  - filter accepts `0x2xx` status frames
- Slave CAN setup:
  - CAN1 on PA11/PA12
  - normal mode
  - 125 kbps timing with the same bit timing
  - filter accepts only standard data frame `0x100`
  - receive path keeps the latest valid command and drops older backlog frames in FIFO
- The PWM driver uses TIM3 CH1 on PA6.
- `servo_pwm_set_pulse_us()` clamps the final pulse to 500 to 2500 us.
- Axis config maps position to 1000 to 2000 us for the current servo range.

## Shared Configuration

- `master/Hardware/common/include/project_config.h`
  - 400 Hz master tick
  - 400 Hz control task
  - 200 Hz CAN task
- `slave/Hardware/common/include/project_config.h`
  - 200 Hz slave tick
  - 50 Hz servo PWM
  - 100 ms CAN timeout
  - 0.2 s timeout return
  - 4.7 V low-battery threshold
- `axis_params.c` defines per-axis mechanical limits and slew rates.
  - Master uses only the angular limits and slew rate.
  - Slave also uses axis ID, sign, zero offset, pulse range, and acceleration limit.

## Integration Notes

- Keep `master` and `slave` as separate build targets.
- Do not compile both `User/main.c` files into one target.
- For the master, wire the TIM2 update interrupt to `master_tick_isr()`.
- For the slave, keep `TIM2_IRQHandler()` calling `slave_tick_isr()`.
- Replace the weak `board_read_vbat_v()` hook in the slave if your hardware provides real battery sensing.
- The CAN status cache on the master is per-axis, and `can_proto_get_latest_status()` also reports the age of the cached sample in milliseconds.
