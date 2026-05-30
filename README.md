# Electric-Screw-Driver
Attempt to make a Torque-sensing DC screwdriver using motor current feedback for real-time torque estimation and inertia aware control

# DRV8701 + RP2040 Brushed DC Screwdriver

Firmware for a handheld brushed-DC screwdriver using a Raspberry Pi Pico (RP2040) and a DRV8701 gate driver in PH/EN mode with an external N-MOSFET H-bridge. Features closed-loop current sensing, configurable torque limiting, soft-start, dithering, and full serial telemetry over CMSIS-DAP / Picoprobe.

## Table of Contents

- [Hardware Overview](#hardware-overview)
- [Pin Map](#pin-map)
- [Current Sensing Circuit](#current-sensing-circuit)
- [Building](#building)
- [Flashing](#flashing)
- [Debug / Serial Interface](#debug--serial-interface)
- [Serial Commands](#serial-commands)
- [State Machine](#state-machine)
- [Torque Control](#torque-control)
- [LED Indicator](#led-indicator)
- [Configuration Reference](#configuration-reference)

---

## Hardware Overview

| Component | Part |
|---|---|
| MCU | Raspberry Pi Pico (RP2040) |
| Gate driver | Texas Instruments DRV8701 |
| Bridge topology | External N-MOSFET H-bridge (PH/EN mode) |
| Current sense | 5 mΩ shunt + DRV8701 SO pin + resistor divider |
| Trigger | Two momentary buttons — CW (GP13) and CCW (GP12) |
| Speed control | Fixed 100 % duty with soft-start ramp (no pot fitted) |

---

## Pin Map

| GPIO | Direction | Function |
|---|---|---|
| GP0 | OUT | UART0 TX → Picoprobe GP5 |
| GP1 | IN | UART0 RX → Picoprobe GP4 |
| GP12 | IN (pull-up) | Button CCW — active LOW |
| GP13 | IN (pull-up) | Button CW — active LOW |
| GP16 | OUT (PWM) | DRV8701 EN/PWM — 20 kHz |
| GP17 | OUT | DRV8701 PH/DIR — HIGH = CW |
| GP18 | OUT | DRV8701 nSLEEP — LOW = Hi-Z |
| GP19 | IN (pull-up) | DRV8701 nFAULT — open-drain active LOW |
| GP26 | AIN | ADC CH0 — SO current sense |
| GP27 | AIN | ADC CH1 — pot (wired, reading disabled) |
| GP25 | OUT | Onboard LED |

---

## Current Sensing Circuit

The DRV8701 SO pin outputs a voltage proportional to motor current. A resistor divider scales it to the RP2040's 3.3 V ADC range.

```
SO pin ──┬── 10 kΩ ──── 3.3 V
         │
         ├── GP26 (ADC CH0)
         │
        20 kΩ
         │
        GND
```

| Parameter | Value |
|---|---|
| Divider ratio (V_ADC / V_SO) | 0.6667 (20 / 30) |
| Amplifier gain | 20 V/V |
| Sense resistor | 5 mΩ |
| SO offset voltage | 50 mV |

**Current calculation** (per loop tick):

```
V_SO_real      = V_ADC / 0.6667
V_SO_on_phase  = V_SO_real / duty_fraction      ← duty-cycle compensation
V_across_amp   = V_SO_on_phase − 0.050 V
I              = V_across_amp / (20 × 0.005)
```

Readings are frozen when duty < 10 % to avoid noise during near-zero on-time. A first-order IIR filter (α = 0.25) smooths the result.

---

## Building

### Prerequisites

- [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) (≥ 1.5)
- CMake ≥ 3.13
- `arm-none-eabi-gcc`

### CMakeLists.txt fragment

```cmake
add_executable(screwdriver DRV_ESD.cpp)

target_link_libraries(screwdriver PRIVATE
    pico_stdlib
    hardware_pwm
    hardware_adc
    hardware_gpio
    hardware_uart
    hardware_watchdog
    hardware_clocks
)

pico_enable_stdio_uart(screwdriver 1)
pico_enable_stdio_usb(screwdriver 0)
pico_add_extra_outputs(screwdriver)
```

### Build

```bash
mkdir build && cd build
cmake ..
make -j4
```

This produces `DRV_ESD.uf2`.

---

## Flashing

Hold the **BOOTSEL** button on the Pico while plugging in USB. Drag `DRV_ESD.uf2` onto the `RPI-RP2` mass-storage device that appears. The board resets and starts running immediately.
Above is for flashing without debugger, i usually flash the .elf file through debugger. Read Pico Debug documentation should you wish to flash using a debugger.

---

## Debug / Serial Interface

Connect a second Pico running [Picoprobe](https://github.com/raspberrypi/picoprobe) firmware:

| Target | Picoprobe |
|---|---|
| GP0 (TX) | GP5 |
| GP1 (RX) | GP4 |
| GND | GND |

Open the CDC serial port at **921 600 baud, 8N1**. All log lines are prefixed with a millisecond timestamp:

```
[T+   1234 ms] STATE  IDLE -> STARTING  [trigger engaged]
[T+   1284 ms] TELEM  STARTING  dir=CW   I= 0.12A ...
```

---

## Serial Commands

Send commands surrounded by angle brackets, e.g. `<help>`.

| Command | Description |
|---|---|
| `<help>` | Full status dump + command list |
| `<status>` | Same as `<help>` |
| `<cw>` | Start motor CW (latched until `<stop>` or torque trip) |
| `<ccw>` | Start motor CCW (latched until `<stop>` or torque trip) |
| `<stop>` | Clear any serial motor request |
| `<ith X.X>` | Set current threshold in amps, e.g. `<ith 4.5>` |

Serial requests behave identically to physical button presses. `<cw>` and `<ccw>` are only accepted from **IDLE** or **STOPPED**; they are ignored and logged otherwise.

---

## State Machine

```
         ┌──────────────────────────────────────────────────────────────────┐
         │  nFAULT asserted (any time)                                      ▼
  BOOT ──► IDLE ──trigger──► STARTING ──ramp done──► RUNNING               FAULT
           ▲                    │                      │  │                   │
           │              early release             trip │  I > 80% Ith       │ nFAULT clear
           │                    │                        ▼  ▼              + holdoff
           │                    └───────────────► STOPPING ◄── DITHERING      │
           │                                          │             │          │
           │                                    cooldown done       │ I < 60% Ith
           │                                          ▼             │
           └──────────────── trigger released ── STOPPED            └──► RUNNING
```

| State | Description |
|---|---|
| `BOOT` | One-shot init, transitions immediately to `IDLE` |
| `IDLE` | Waiting for trigger; bridge is in sleep / Hi-Z |
| `STARTING` | 200 ms linear duty ramp from 0 → 100 % |
| `RUNNING` | Full duty, monitoring current |
| `DITHERING` | Near threshold — duty modulated to hold current just below limit |
| `STOPPING` | EN = 0 (brake), 300 ms cooldown before sleep |
| `STOPPED` | Bridge asleep; waiting for trigger release before next run |
| `FAULT` | nFAULT asserted by DRV8701; motor braked, LED solid ON |

**Trigger logic** — either the physical button *or* a latched serial request counts as a trigger. If both CW and CCW are simultaneously active (buttons or serial), no motion occurs until the conflict resolves.

---

## Torque Control

Three layered mechanisms protect the motor and workpiece:

### 1. Threshold current trip (sustained overcurrent)

The firmware trips if `I_filtered ≥ I_threshold` persists for **5 consecutive milliseconds**. Default threshold is **5.0 A**; adjustable at runtime with `<ith X.X>` (range 2–10 A). On trip, the motor brakes and the state machine enters `STOPPING`.

### 2. Hard limit

If filtered current reaches **14 A** at any time — including during soft-start — the motor brakes immediately regardless of threshold setting. This protects the FETs from thermal runaway.

### 3. Rapid dI/dt (stall detection)

A ring buffer of the last 8 current samples is compared oldest-to-newest. If the window rise exceeds **4 A**, the motor trips. This catches stalls that happen before sustained overcurrent would fire.

### Dithering (soft current limiting)

Between **80 %** and **100 %** of the threshold current the firmware enters `DITHERING` and scales the duty cycle proportionally down to a minimum of **30 %**, holding torque steady rather than hard-tripping. The motor returns to `RUNNING` if current drops back below **60 %** of threshold.

---

## LED Indicator

| LED state | Meaning |
|---|---|
| Off | Idle / stopped / boot |
| Blinking (0.5 s period) | Motor energised (STARTING, RUNNING, DITHERING) |
| Solid ON | FAULT |

---

## Configuration Reference

All compile-time tunables are `constexpr` near the top of `DRV_ESD.cpp`.

| Symbol | Default | Description |
|---|---|---|
| `I_THRESHOLD_DEFAULT_A` | 5.0 A | Starting current threshold (no pot) |
| `I_THRESHOLD_MIN_A` | 2.0 A | Lower bound for `<ith>` command |
| `I_THRESHOLD_MAX_A` | 10.0 A | Upper bound for `<ith>` command |
| `I_HARD_LIMIT_A` | 14.0 A | Unconditional FET protection limit |
| `SOFT_START_MS` | 200 ms | Duty ramp duration on trigger press |
| `COOLDOWN_MS` | 300 ms | Brake hold time before bridge sleeps |
| `FAULT_HOLDOFF_MS` | 150 ms | Debounce after nFAULT clears |
| `OVERCURRENT_CONFIRM_MS` | 5 ms | Sustained OC window before trip |
| `DI_DT_TRIP_A_PER_WIN` | 4.0 A | dI/dt trip threshold over 8-sample window |
| `DITHER_START_FRAC` | 0.80 | Dither zone entry — fraction of I_threshold |
| `DITHER_MIN_DUTY_FRAC` | 0.30 | Minimum PWM duty while dithering |
| `IIR_ALPHA` | 0.25 | Current filter coefficient (higher = faster) |
| `PWM_WRAP` | 6249 | PWM counter wrap → 20 kHz at 125 MHz |
| `LOOP_PERIOD_US` | 5000 µs | Main loop period (200 Hz) |
| `WATCHDOG_TIMEOUT_MS` | 250 ms | Watchdog reset if loop stalls |
| `UART_BAUD` | 921 600 | Debug UART baud rate |

---
