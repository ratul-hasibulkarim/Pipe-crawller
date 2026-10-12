# Controller pin map

Generated from `firmware/include/pinmap.h` by `tools/gen_pinmap_doc.py` - edit the header, not this file.

| Group | Pin | Peripheral | Function / alternate |
|---|---|---|---|
| Pi link | USB D+/D- | USB CDC | Pi <-> MCU protocol link (default) |
| Pi link | GP0 | UART0 TX | alternate Pi link, 921600 baud (build flag PIPEBOT_LINK_UART) |
| Pi link | GP1 | UART0 RX | alternate Pi link |
| Fast I2C, core 0 @ 1 MHz: magnetic encoders through a TCA9548A mux | GP2 | I2C1 SDA | TCA9548A 0x70 -> AS5600 0x36 on ch0..4 (wheel A/B/C, lever, pan) |
| Fast I2C, core 0 @ 1 MHz: magnetic encoders through a TCA9548A mux | GP3 | I2C1 SCL | encoder bus clock |
| Stepper drivers | GP4 | UART1 TX | TMC2209 single-wire UART, addresses 0..3 (1 k to PDN_UART); velocity via VACTUAL |
| Stepper drivers | GP5 | UART1 RX | TMC2209 UART receive (same wire) |
| Stepper drivers | GP6 | PIO0 SM0 | STEP drive A - fallback pulse train (normal: VACTUAL, no pulses) |
| Stepper drivers | GP7 | PIO0 SM1 | STEP drive B - fallback |
| Stepper drivers | GP8 | PIO0 SM2 | STEP drive C - fallback |
| Stepper drivers | GP9 | PIO0 SM3 | STEP expansion - fallback |
| Stepper drivers | GP10 | PWM5 A | IR 850 nm LED driver DIM |
| Stepper drivers | GP11 | GPIO in | E-stop loop sense (pull-up; low = loop closed = OK). Same loop drops the driver ENABLE relay. |
| Stepper drivers | GP12 | GPIO in | supply path status (LTC4412 STAT; high = running from the battery) |
| Stepper drivers | GP13 | GPIO out | camera/ToF window heater MOSFET (anti-fog) |
| Stepper drivers | GP14 | GPIO out | DRIVE_EN, active low, 3 drive drivers (hardware E-stop relay in series) |
| Stepper drivers | GP15 | GPIO out | EXPAND_EN, active low, expansion driver |
| Stepper drivers | GP16 | SPI0 RX | TMC5160 SDO (size L) / spare |
| Stepper drivers | GP17 | SPI0 CSn | TMC5160 chip select via 74HC138 (size L) / spare |
| Stepper drivers | GP18 | SPI0 SCK | TMC5160 SCK (size L) / spare |
| Stepper drivers | GP19 | SPI0 TX | TMC5160 SDI (size L) / spare |
| Slow I2C, core 1 @ 400 kHz: environment / IMU / power | GP20 | I2C0 SDA | BNO085 0x4A, VL53L1X 0x29 (front) / 0x2A (rear), SHT31 0x44, BMP390 0x77, INA219 0x40/0x41, ADS1115 0x48 |
| Slow I2C, core 1 @ 400 kHz: environment / IMU / power | GP21 | I2C0 SCL | sensor bus clock |
| Slow I2C, core 1 @ 400 kHz: environment / IMU / power | GP22 | GPIO out | VL53L1X rear XSHUT (low at boot so the front sensor is re-addressed first) |
| Gimbal, lights | GP23 | PWM3 B | tilt servo, 50 Hz |
| Gimbal, lights | GP24 | PWM4 A | pan gearmotor DRV8833 IN1 |
| Gimbal, lights | GP25 | PWM4 B | pan gearmotor DRV8833 IN2 |
| Analog | GP26 | ADC0 | grip hall sensor arm A (DRV5055A1; 0.6-2.7 V over the 10 mm slide) |
| Analog | GP27 | ADC1 | grip hall sensor arm B |
| Analog | GP28 | ADC2 | grip hall sensor arm C |
| Analog | GP29 | PWM6 B | white LED driver DIM (4 x 1 W constant-current buck) |

I2C addresses: BNO085 0x4A, VL53L1X 0x29/0x2A, SHT31 0x44, BMP390 0x77, INA219 0x40 (drive) / 0x41 (expansion),
ADS1115 0x48, TCA9548A 0x70 (encoder bus) with AS5600 0x36 on mux channels 0-4.
