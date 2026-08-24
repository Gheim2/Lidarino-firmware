# Lidarino Firmware

Firmware for the Lidarino robot built for an ESP32-S3 with the Arduino framework and PlatformIO.

It controls the drivetrain, reads the wheel encoders, streams IMU telemetry, and exchanges commands with an Orange Pi over a custom binary serial protocol used by the ROS 2 side of the project.

## What it does

- Drives two DC motors through a TB6612FNG motor driver.
- Reads two AS5600 magnetic encoders on separate I2C buses.
- Uses a PID loop to regulate left and right wheel velocity.
- Reads an MPU6500 IMU and includes accel/gyro data in telemetry.
- Accepts binary velocity commands from the Orange Pi.
- Sends periodic telemetry back with wheel position, status flags, and IMU readings.
- Stops the motors automatically on command timeout or estop.

## Project layout

- `src/main.cpp` - firmware entry point.
- `include/robot_core.h` - main robot control logic.
- `include/serial_protocol.h` - shared binary packet definitions.
- `include/pins.h` - pin assignments and hardware constants.
- `include/tb6612.h` - motor driver wrapper.
- `include/as5600_encoder.h` - encoder wrapper.
- `include/wheel_pid.h` - wheel velocity PID controller.

## Hardware

Designed around:

- ESP32-S3 DevKitC-1 N16R8V
- TB6612FNG dual motor driver
- Two AS5600 encoders, each on its own I2C bus
- MPU6500 IMU
- Orange Pi 4 Pro as the high-level controller

## Pinout

The pin mapping is defined in `include/pins.h`.

Important assignments:

- Left motor PWM: GPIO 4
- Left motor IN1/IN2: GPIO 5 / 6
- Right motor PWM: GPIO 7
- Right motor IN1/IN2: GPIO 15 / 16
- TB6612 standby: GPIO 17
- Left encoder I2C: GPIO 8 / 9
- Right encoder I2C: GPIO 10 / 11
- Estop input: GPIO 12
- UART to Orange Pi: GPIO 43 / 44

## Serial protocol

Communication is binary and framed with sync bytes `0xAA 0x55`.

### Command packet, Orange Pi to ESP32

Size: 10 bytes

- `sync0`, `sync1`: frame header
- `vel_left`, `vel_right`: target wheel velocities in rad/s multiplied by 1000
- `reserved`: reserved for future use
- `checksum`: XOR of all previous bytes in the packet
- `terminator`: `0x0D`

### Telemetry packet, ESP32 to Orange Pi

Size: 25 bytes

- `sync0`, `sync1`: frame header
- `pos_left`, `pos_right`: cumulative encoder positions in ticks
- `accel_x`, `accel_y`, `accel_z`: accelerometer values scaled by 1000
- `gyro_x`, `gyro_y`, `gyro_z`: gyroscope values scaled by 1000
- `status_flags`: estop, encoder OK, and IMU OK bits
- `checksum`: XOR of all previous bytes in the packet
- `terminator`: `0x0D`

## Control loop behavior

- Velocity control runs at 100 Hz.
- Telemetry is transmitted at 50 Hz.
- If no valid command is received for 300 ms, the motors are stopped.
- If the estop input is active, the motors are stopped.

## Build and flash

This project uses PlatformIO.

```bash
pio run
pio run --target upload
pio device monitor
```

If you need a different serial monitor speed, the default is set to 115200 in `platformio.ini`.

## Notes

- The firmware expects the matching ROS 2 hardware interface to use the same packet definitions.
- The packet structs are packed and little-endian.
- `include/serial_protocol.h` should be kept in sync with the host-side implementation.
