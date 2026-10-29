# Lidarino Firmware

Firmware for the Lidarino robot, built for an ESP32-S3 with the Arduino framework and PlatformIO.

It controls the drivetrain, reads the wheel encoders and the IMU, measures the battery, and exchanges commands and telemetry with an Orange Pi over a custom binary serial protocol used by the ROS 2 hardware interface (`lidarino_hardware`).

## What it does

- Drives two DC motors through a TB6612FNG motor driver (PWM 9.5 kHz, 12 bit).
- Reads two AS5600 magnetic encoders, each on its own I2C bus.
- Regulates left and right wheel velocity with a PID loop at 100 Hz.
- Reads an MPU6500 IMU, calibrated and expressed in the robot frame (REP 103).
- Measures the battery voltage through a resistive divider.
- Accepts binary velocity commands from the Orange Pi and sends telemetry back at 50 Hz.
- Stops the motors automatically on command timeout or e-stop.
- Shows the robot state on an RGB status LED.
- Resets itself through the task watchdog if the main loop hangs for more than 3 s, and reports the reset reason at the next boot.

## Project layout

- `src/main.cpp` - firmware entry point.
- `include/robot_core.h` - main robot logic: setup, control loop, serial protocol, telemetry, LED, IMU.
- `include/serial_protocol.h` - binary packet definitions shared with the ROS 2 side.
- `include/pins.h` - pin assignments.
- `include/user_config.h` - `DEBUG_MODE`, command timeout and battery threshold.
- `include/tb6612.h` - motor driver wrapper (PWM frequency and resolution).
- `include/wheel_pid.h` - wheel velocity PID controller.
- `include/imu_calibration.h` - IMU axis remap, calibration values and boot gyro calibration settings.
- `test/test_motor_pwm/` - on-board test of the motor PWM configuration.
- `tools/imu_calibrate.py` - reads IMU averages from the serial port, used for the calibration.
- `sync_to_pi.sh` - copies this folder to the Orange Pi to build and flash from there.

Encoders use the [RobTillaart AS5600](https://github.com/RobTillaart/AS5600) library and the IMU uses the [Bolder Flight invensense-imu](https://github.com/bolderflight/invensense-imu) library.

## Hardware

- BPI-Leaf-S3 (ESP32-S3, PlatformIO board `bpi_leaf_s3`)
- TB6612FNG dual motor driver
- Two AS5600 encoders, each on its own I2C bus (they have the same fixed I2C address)
- MPU6500 IMU, on the same I2C bus as the right encoder
- WS2812 (NeoPixel) status LED
- Orange Pi 4 Pro as the high-level controller, connected over the ESP32-S3 native USB port

The TB6612FNG **VCC** (logic supply) must be connected to the ESP32 3.3 V. If it is disconnected, the chip is fed through its input pins and VCC reads about 2.5 V (3.3 V minus a diode drop): below the 2.7 V minimum, the driver does not work.

## Pinout

The pin mapping is defined in `include/pins.h`. GPIO 0, 19, 20, 45 and 46 are avoided (strapping pins and native USB).

| Function | GPIO |
|---|---|
| Right motor (TB6612 channel A) PWM | 6 |
| Right motor IN1 / IN2 | 4 / 5 |
| Left motor (TB6612 channel B) PWM | 15 |
| Left motor IN1 / IN2 | 17 / 16 |
| TB6612 standby (STBY, LOW = motors disabled) | 7 |
| I2C bus 0 SDA / SCL: right encoder + IMU | 9 / 8 |
| I2C bus 1 SDA / SCL: left encoder | 11 / 10 |
| E-stop sense input (pull-up, LOW = pressed) | 13 |
| Battery voltage (resistive divider) | 12 |
| Status LED (WS2812) | 48 |
| USB to Orange Pi (native USB D- / D+) | 19 / 20 |

The e-stop input only reports the state: the primary safety is the physical power cut on the button. `pins.h` also defines GPIO 43 / 44 as UART TX / RX, but they are not used: the Orange Pi link is the native USB port.

## Serial link

- Transport: native USB of the ESP32-S3, using the **USB-Serial/JTAG** peripheral (`ARDUINO_USB_MODE=1` in `platformio.ini`).
  The TinyUSB stack (`ARDUINO_USB_MODE=0`) was abandoned: its transmit path stalled after a while, so commands kept arriving but telemetry stopped.
- USB ID: `303a:1001`. On the Orange Pi a udev rule (`/etc/udev/rules.d/99-lidarino.rules`) creates the `/dev/lidarino` symlink to the `ttyACM` device.
- Nominal baud rate 115200 (on native USB the value is not used, but both sides set it).
- All packets are packed and little-endian, framed with the sync bytes `0xAA 0x55` and ended by `0x0D`.
- Checksum: XOR of every byte before the checksum, **sync bytes included**.

### Command packet, Orange Pi to ESP32

Size: 10 bytes

| Offset | Field | Type | Content |
|---|---|---|---|
| 0 | `sync0` | uint8 | `0xAA` |
| 1 | `sync1` | uint8 | `0x55` |
| 2 | `vel_left` | int16 | left wheel target velocity, rad/s × 1000 |
| 4 | `vel_right` | int16 | right wheel target velocity, rad/s × 1000 |
| 6 | `reserved` | uint16 | always 0 |
| 8 | `checksum` | uint8 | XOR of bytes 0-7 |
| 9 | `terminator` | uint8 | `0x0D` |

### Telemetry packet, ESP32 to Orange Pi

Size: 27 bytes, sent at 50 Hz

| Offset | Field | Type | Content |
|---|---|---|---|
| 0 | `sync0` | uint8 | `0xAA` |
| 1 | `sync1` | uint8 | `0x55` |
| 2 | `pos_left` | int32 | left wheel cumulative position, ticks (4096 per turn) |
| 6 | `pos_right` | int32 | right wheel cumulative position, ticks (4096 per turn) |
| 10 | `accel_x`, `accel_y`, `accel_z` | int16 × 3 | acceleration, m/s² × 1000, robot frame |
| 16 | `gyro_x`, `gyro_y`, `gyro_z` | int16 × 3 | angular velocity, rad/s × 1000, robot frame |
| 22 | `status_flags` | uint8 | bit 0 e-stop pressed, bit 1 left encoder OK, bit 2 right encoder OK, bit 3 IMU OK |
| 23 | `battery_mv` | uint16 | battery voltage, mV (updated at 1 Hz, low-pass filtered) |
| 25 | `checksum` | uint8 | XOR of bytes 0-24 |
| 26 | `terminator` | uint8 | `0x0D` |

IMU values are saturated to the int16 range: accelerations up to ±32.7 m/s², angular velocities up to ±32.7 rad/s.

Telemetry is never blocking: if the USB transmit buffer is full (host not reading), the packet is dropped and the next one is sent 20 ms later.

### Text lines in the serial stream

Besides the binary packets, the firmware sends a few text lines. The ROS 2 hardware interface discards them as packet data and writes them to its log:

- `[BOOT] reset_reason=... last_stage=... gyro_bias=boot|default(x,y,z)`: sent at boot and again 3 s later. `last_stage` is the loop stage reached before a watchdog or panic reset (kept in RTC memory, not meaningful after a power-on). `gyro_bias=boot` means the boot gyro calibration succeeded, `default` that the stored values are used.
- Setup errors: `Error initializing right encoder!`, `Error initializing left encoder!`, `Error initializing IMU!`, `IMU not responding: starting without it, retrying in the loop`, `Error initializing left motor PWM!`, `Error initializing right motor PWM!`.

## Control loop behavior

- Velocity PID control runs at 100 Hz on the encoder speed (low-pass filtered). When the target is 0, the PID is reset and the motor output is 0.
- Telemetry is sent at 50 Hz, the battery is read at 1 Hz.
- If no valid command is received for 500 ms (60 s in `DEBUG_MODE`), the wheel targets are set to 0.
- If the e-stop input is active, the wheel targets are set to 0.
- The main loop never blocks: no `delay()` after setup.

## Status LED

The LED blinks (1 s period, 250 ms when the battery is low). Only the first matching state is shown, in this priority order:

| Color | Meaning |
|---|---|
| Solid blue | Setup in progress. If it stays blue, setup is stuck waiting for an encoder: check the serial output |
| Red, bright / dim | E-stop pressed |
| Magenta, fast blink | Battery low (≤ `BATTERY_MV_THRESHOLD`, 10 V) |
| Yellow | At least one encoder not connected or magnet not detected |
| Purple | IMU not working (the robot still drives, see below) |
| Orange / red alternating | No valid command received within the command timeout (normal when ROS 2 is not running) |
| Green, bright / dim | Everything OK, commands arriving |

## Motor driver PWM

The PWM settings are constants in `include/tb6612.h`: **9.5 kHz, 12 bit** (4096 steps).

On this core the LEDC peripheral is clocked at 40 MHz, so the maximum frequency at 12 bit is 40 MHz / 4096 = 9.77 kHz. A previous setting of 10 kHz / 12 bit could not be achieved: `ledcSetup()` failed, no PWM reached the driver and the motors never moved, while IN1 / IN2 switched normally. `TB6612Motor::begin()` now returns `false` when the PWM cannot be configured, and the firmware prints `Error initializing ... motor PWM!`.

### PWM test

`test/test_motor_pwm/test_motor_pwm.cpp` is a Unity test that runs on the ESP32. It checks, with the constants of `tb6612.h`:

1. `TB6612Motor::begin()` succeeds for both motors.
2. `ledcSetup()` reaches the configured frequency (within 1%).
3. The 50% duty is applied (read back with `ledcRead()`).

IN1 / IN2 stay LOW during the test (driver in brake), so the motors cannot move. After the tests the PWM stays at 50% on GPIO 6 and 15, to check it with a logic analyzer or oscilloscope.

```bash
pio test -e bpi_leaf_s3 -f test_motor_pwm --upload-port /dev/ttyACM0 --test-port /dev/ttyACM0
```

The test replaces the robot firmware on the board: flash the firmware again afterwards (`pio run -t upload`). Run it with the ROS 2 stack stopped, since the serial port is needed.

## IMU

### Configuration

- MPU6500 on I2C bus 0, address `0x68`.
- Output data rate 200 Hz, digital low-pass filter 41 Hz (avoids aliasing of motor vibrations at the 50 Hz telemetry rate).
- Accelerometer range ±4 g (the telemetry saturates at about ±3.3 g anyway).
- Data-ready flag enabled, so a read only returns new samples.

### Failure handling

- At boot the IMU is tried 3 times. If it does not answer, the firmware starts without it: motors and encoder odometry work, the LED is purple and the `IMU OK` status bit is 0 (the ROS 2 side logs a warning).
- After 5 consecutive failed reads (100 ms) the IMU is marked as not working.
- While it is not working, the firmware retries once per second. If the IMU answers on the I2C bus again and the wheels are stopped, it is configured again (this takes about 100 ms, so it is never done while the motors are controlled). This also recovers an IMU that lost its configuration after a loose connection or a voltage drop.

### Calibration

The correction applied to every sample, with the values in `include/imu_calibration.h`:

```
accel = ALIGN * (ACCEL_SCALE * (remap(accel_lib) - ACCEL_OFFSET))
gyro  = ALIGN * (remap(gyro_lib) - gyro_bias)
```

1. **Axis remap**: the library uses NED-style axes (x forward, y right, z down). They are converted to the robot frame of REP 103 (x forward, y left, z up) by flipping y and z.
2. **Accelerometer offset and scale**: per-axis offset (m/s²) and scale factor, so that gravity reads 9.81 m/s² on each axis.
3. **Alignment**: a rotation matrix that compensates the mounting tilt of the IMU (roll -1.64°, pitch +4.20° with respect to the floor), so that with the robot on flat ground gravity is exactly on +z.
4. **Gyro bias**: measured at every boot, see below.

How it was measured (2026-10-28): with the ROS 2 stack stopped and the robot still, `tools/imu_calibrate.py` was run on the Orange Pi in 4 positions: flat, on the right side, on the left side and on the back. The script only reads the telemetry (it sends no commands) and prints mean and standard deviation of each accelerometer and gyro axis, plus the magnitude of the acceleration:

```bash
python3 tools/imu_calibrate.py --seconds 5 --label flat
```

From the gravity readings in the 4 positions the offset, scale and alignment values were computed and written in `imu_calibration.h`. The noise covariances used by ROS 2 (`controllers_real.yaml` in the ROS 2 repository) were measured separately on the ROS 2 side, with the robot still.

**Boot gyro calibration**: right after the IMU setup, the firmware averages 200 gyro samples (about 1 s at 200 Hz) to measure the bias. If the standard deviation of any axis is above 0.01 rad/s (robot moving) or the samples do not arrive within 2 s, the stored `GYRO_BIAS_DEFAULT` values are used. The `[BOOT]` line reports which one is in use. **Keep the robot still for about 2 s after power-on.**

The calibration is specific to this MPU6500 and its mounting: redo it if the IMU is replaced or moved (for example with the planned switch to an LSM6DS3).

## Debug mode

Set `DEBUG_MODE 1` in `include/user_config.h` to drive the motors by hand from a serial monitor. In this mode no telemetry is sent, the command timeout is 60 s and the binary protocol is replaced by single-key commands:

| Key | Action |
|---|---|
| `w` | Both motors forward, PID at 10 rad/s |
| `1` / `2` | Left / right motor only, PID at 10 rad/s |
| `q` / `e` | Left / right motor at raw 50% PWM, PID bypassed |
| `s` | Stop |
| `u` / `j` | Kp +0.01 / -0.01 |
| `i` / `k` | Ki +0.01 / -0.01 |

While a PID command is active, target and measured speeds are printed in Serial Plotter format.

- Raw PWM (`q` / `e`) has no timeout: it runs until `s` is pressed.
- Never start the ROS 2 stack with a `DEBUG_MODE 1` firmware: the binary command bytes would be read as keys.
- Set `DEBUG_MODE 0` again before using the robot with ROS 2.

## Build and flash

This project uses PlatformIO.

```bash
pio run              # build
pio check            # static analysis
pio device monitor   # serial monitor (115200)
```

The board is connected to the Orange Pi, so the firmware is flashed from there:

1. Stop the ROS 2 stack (`robot.launch.py`): `/dev/lidarino` must be free.
2. From Git Bash on the PC, copy this folder to the Pi. Check the preview first:
   ```bash
   ./sync_to_pi.sh --dry-run
   ./sync_to_pi.sh              # add --delete to also remove files deleted locally
   ```
   The default host is set by `LIDARINO_HOST` (an SSH host from `~/.ssh/config`).
3. On the Pi:
   ```bash
   cd ~/Lidarino-firmware
   pio run -t upload --upload-port /dev/ttyACM0
   ```

With USB-Serial/JTAG, esptool resets the chip by itself: the 1200 baud touch is disabled in `platformio.ini`. If the board was still running a TinyUSB firmware (`ARDUINO_USB_MODE=0`, USB ID `303a:80df`), that touch is needed once to enter the bootloader.

## Notes

- The firmware must match the ROS 2 hardware interface (`lidarino_hardware`): any change to `include/serial_protocol.h` has to be applied on both sides.
- Units follow ROS conventions (REP 103): meters, radians, seconds. Conversions are done in the firmware.
