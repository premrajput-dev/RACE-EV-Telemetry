# System Architecture

## Controller split
- **Arduino Nano:** MPU6050 sampling and orientation estimates, DS18B20 temperature, TFT dashboard, OLED status display, and buzzer alerts.
- **ESP32:** GPS reception and microSD logging when the final logger firmware is installed and tested.
- **Serial link (optional):** Nano telemetry can be forwarded to the ESP32 for logging. Define and document the exact message format in the ESP32 firmware before relying on it.

## Data and safety notes
- Lean angle and acceleration are IMU-derived estimates; validate them before treating them as measured race telemetry.
- GPS distance should be calculated by summing distances between consecutive valid GPS fixes, not from the geographic origin.
- No wheel-speed sensor is included in the documented parts, so true wheel-slip detection is not supported.
- No battery voltage/current sensor is included, so battery power and energy consumption cannot be calculated.
