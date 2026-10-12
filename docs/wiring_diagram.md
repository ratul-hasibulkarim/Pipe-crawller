# Wiring diagram

Pin-level detail: `docs/pinmap.md` (generated from `firmware/include/pinmap.h`).

```mermaid
flowchart LR
  subgraph Reel["Tether reel (surface)"]
    PSU["48 V SELV supply"] --> SR["slip ring: 2 power + 2 data pairs"]
    AP["Wi-Fi AP / router"] <--> SR
    ES["E-stop button (loop)"] --- SR
  end
  Phone["Phone / tablet dashboard"] <-. Wi-Fi .-> AP
  SR == "tether: 2 x 0.5 mm2 + 2 x STP 100BASE-TX + aramid" ==> Conn["robot connector + strain relief"]
  subgraph Robot
    Conn --> OR["ideal-diode OR (LTC4412)"]
    BAT["Li-ion pack (M 4S1P, L 6S2P; S none)"] --> OR
    OR --> BUCKM["buck: motor rail 12 V (L 24 V)"]
    OR --> BUCK5["buck 5 V"]
    BUCKM --> INA1["INA219 drive rail"] --> DRV["3 x stepper driver (TMC2209; L TMC5160)"] --> DM["3 drive motors"]
    BUCKM --> INA2["INA219 expansion"] --> DRVE["expansion driver"] --> EM["expansion motor"]
    BUCK5 --> PI["Pi (S Zero 2 W, M CM4, L Pi 5) + camera"]
    BUCK5 --> MCU["RP2040 controller"]
    Conn -- "Ethernet" --> PI
    PI <-- "USB CDC (or UART 921600)" --> MCU
    MCU -- "UART1 single-wire / SPI (L)" --> DRV
    MCU -- "UART1 / SPI" --> DRVE
    Conn -- "E-stop loop" --> RLY["enable relay"] --> DRV
    RLY --> DRVE
    MCU -- "I2C1 1 MHz via TCA9548A" --> ENC["5 x AS5600: 3 wheels, lever, pan"]
    MCU -- "I2C0 400 kHz" --> SENS["BNO085, 2 x VL53L1X, SHT31, BMP390, ADS1115"]
    MCU -- "ADC0-2" --> HALL["3 x DRV5055 grip hall"]
    MCU -- "PWM" --> GIM["tilt servo, pan DRV8833"]
    MCU -- "PWM" --> LED["white + IR LED drivers"]
  end
```

## Rules

- 48 V is SELV (< 60 V DC); still fuse it at the robot connector (size per `power.buck_*`).
- The E-stop is a **hardware loop** through the tether: open loop drops the driver enable
  relay, independent of software. The MCU only senses it (GP11).
- Keep the Ethernet pairs away from the motor wiring inside the hull; ground the tether
  shield at the reel only.
- Every through-hull cable passes a potted gland; see the seals list in
  `docs/assembly_manual.md`.
- Thermal: drivers and motors get thermal pads to the aluminium hull (T12 assumes it).
