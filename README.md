# Thermocontroller Firmware

Firmware for the GD32F303RCT6-based thermocontroller / heater-management board.
This is a **baseline scaffold**: structure, interfaces and stubs only — no
control logic is implemented yet (every function body is marked `TODO`).

## Design files

- [Schematic (PDF)](docs/Schematic.pdf)
- [PCB layout (PNG)](docs/PCB.png)

## Hardware

| Subsystem        | Detail                                                        |
|------------------|---------------------------------------------------------------|
| MCU              | GD32F303RCT6 (Cortex-M4, 64-pin)                              |
| Temperature      | 8 thermocouples via 2× ADS1220 (24-bit ADC) on shared SPI1   |
| Heater current   | 8 current-transformer analog inputs → on-chip ADC            |
| Proximity        | 2 digital inputs (meterage counting + speed)                 |
| Analog inputs    | 2 general inputs behind resistor voltage dividers            |
| PWM outputs      | 2 channels, duty driven directly from Modbus registers       |
| Relays           | 16 relays (heater/blower) via ULN2803                        |
| Communication    | USART2 → external MAX485 → Modbus RTU (RS-485 slave)         |

Source of truth for pins: `../altium/modbus-host/MainBoard.SchDoc`.

## Layout

```
program/
├── platformio.ini          Build config (platform-gd32, SPL framework)
├── include/
│   ├── board_config.h      MCU pin map (ports/pins per net)
│   └── app_config.h        Channel counts, Modbus + timing constants
└── src/
    ├── main.c              Entry point → app_init / app_run
    ├── hal/                Vendor-isolating abstraction (GD32 SPL inside)
    │   ├── hal_gpio.*      Digital I/O
    │   ├── hal_spi.*       SPI master (ADS1220 bus)
    │   ├── hal_adc.*       On-chip ADC (CTs + analog inputs)
    │   ├── hal_pwm.*       Timer PWM (2 outputs)
    │   ├── hal_uart.*      USART (RS-485, IRQ RX ring buffer)
    │   └── hal_time.*      SysTick / millis / delay
    ├── drivers/            Hardware device drivers
    │   ├── ads1220/        ADS1220 thermocouple ADC (register-level)
    │   ├── relay/          16-channel relay bank
    │   └── proximity/      Proximity inputs (count + speed)
    ├── app/                Application services
    │   ├── thermocouple.*  8-ch temperature (over 2x ADS1220)
    │   ├── current_monitor.* 8-ch heater current (over ADC)
    │   ├── analog_input.*  2-ch divided analog inputs -> mV
    │   ├── pwm_output.*    2-ch PWM driven from Modbus
    │   └── app.*           Wiring + super-loop
    └── comm/               Communication
        ├── modbus_rtu.*    Modbus RTU slave (wraps nanoMODBUS)
        └── register_map.*  Modbus data model ↔ services/actuators
```

## Third-party libraries

| Library | Use | Why |
|---------|-----|-----|
| [nanoMODBUS](https://github.com/debevv/nanoMODBUS) | Modbus RTU server | Pure C, hardware-agnostic, callback-based — only needs UART read/write hooks. Pulled via `lib_deps`. |
| *(ADS1220 — hand-rolled)* | thermocouple ADC | The good ADS1220 libraries ([ADS1220_WE](https://github.com/wollewald/ADS1220_WE), [hideakitai/ADS1220](https://github.com/hideakitai/ADS1220), [Protocentral](https://github.com/Protocentral/Protocentral_ADS1220)) are Arduino-only and won't build under the SPL framework, so the register logic is implemented on `hal_spi`. |

## Modbus register map

See the header block in `src/comm/register_map.h` for the full address plan.
Summary:

- **Input registers (FC04):** thermocouple temps/validity, heater currents,
  the 2 analog inputs (mV), proximity counts (32-bit) and speeds.
- **Holding registers (FC03/06/16):** relay mask, PWM1 duty, PWM2 duty.
- **Coils (FC01/05/15):** individual relays 0..15.

### Layering

```
main → app → { services: thermocouple, current_monitor, proximity, relay }
           → comm: modbus_rtu → register_map → services
   all sit on → hal → GD32 SPL
```

Drivers and services never call the vendor SPL directly — only through `hal/`.
The Modbus layer never touches sensors directly — only through `register_map`.

## Build

```
cd program
pio run                # build
pio run -t upload      # flash via ST-Link
pio device monitor     # serial @115200
```

The GD32 platform is fetched from
`https://github.com/CommunityGD32Cores/platform-gd32.git` (see `platformio.ini`).

## TODO

The logic is implemented and commented; what remains is board-specific
calibration and pin confirmation (all marked `TODO` in code):

- [ ] **Pins** — confirm every `TODO` in `board_config.h` against the schematic
  netlist (relay pins, ADC channels, SPI/UART/timer peripheral choice,
  PROX EXTI lines, RS-485 DE pin).
- [ ] **ADS1220 → temperature** — replace the placeholder `convert_raw_to_c10()`
  in `thermocouple.c` with real µV→°C (e.g. type-K polynomial) + cold-junction
  compensation; set gain/data-rate/reference defaults in `ads1220_init()`.
- [ ] **Current scaling** — calibrate `CT_SCALE_MA_PER_COUNT` (and add RMS if the
  CT conditioning isn't already rectified/filtered).
- [ ] **Analog dividers** — set `AINx_DIVIDER_RATIO` from the actual resistors.
- [ ] **Proximity EXTI** — finish the EXTI/NVIC setup in `proximity_init()` and
  call `proximity_on_edge()` from the ISR.
- [ ] **SPL framework** — verify the `gd32f30x.h` SPL headers are provided by the
  chosen platform package; adjust includes if the core uses different names.
- [ ] **Bring-up** — flash the board, validate Modbus RTU comms end-to-end, and
  exercise each subsystem against live hardware.

### Schematic design review

- [ ] Verify fuse rating and usage
- [x] Confirm Modbus address selection
- [x] Confirm AMS LED wiring
- [ ] Add open-thermocouple detection
- [ ] Add ADC input protection

### PCB design review

- [ ] Check board height / mechanical clearance
- [ ] Verify all component footprints

### Firmware

- [ ] Implement open-thermocouple detection
