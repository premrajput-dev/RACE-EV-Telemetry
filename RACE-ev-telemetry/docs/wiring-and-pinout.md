# Wiring and Pinout

## Arduino Nano
| Module signal | Nano pin | Notes |
|---|---|---|
| TFT CS | D10 | ST7789 |
| TFT DC | D9 | ST7789 |
| TFT RESET | D8 | ST7789 |
| TFT MOSI | D11 | Hardware SPI |
| TFT SCK | D13 | Hardware SPI |
| TFT MISO | D12 | If used by the display module |
| DS18B20 DATA | D2 | Use the pull-up arrangement required by your sensor/wiring |
| Buzzer | D3 | Firmware uses direct Timer2 control |
| MPU6050 SDA | A4 | Shared I²C bus |
| MPU6050 SCL | A5 | Shared I²C bus |
| SSD1306 SDA | A4 | Address 0x3C |
| SSD1306 SCL | A5 | Address 0x3C |

## ESP32 GPS and microSD (proposed pinout)
| Module signal | ESP32 pin |
|---|---|
| GPS TX → ESP32 RX2 | GPIO16 | UART2 RX; 9600 baud is common for NEO-6M defaults |
| GPS RX ← ESP32 TX2 | GPIO17 | Optional if sending commands to GPS |
| microSD SCK | GPIO18 | VSPI |
| microSD MISO | GPIO19 | VSPI |
| microSD MOSI | GPIO23 | VSPI |
| microSD CS | GPIO13 | Set in the firmware |

**Electrical cautions:** all connected boards need a common ground. Never connect a 5 V Nano TX signal directly to an ESP32 input; use a suitable level shifter or a verified divider. Confirm the exact GPS and SD module power/logic requirements from their board markings and documentation. The table above is a proposed wiring reference; use the pinout that matches the tested hardware.
