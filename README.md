# Anti-collision follower train (STM32 Nucleo-F411RE)

A follower locomotive that keeps a safe distance behind a lead train on the same
track, using only a forward-facing ultrasonic sensor. The controller runs on an
STM32 Nucleo-F411RE and drives the locomotive's motor directly through an
Infineon IFX007T H-bridge shield.

Research assistant (HiWi) project, TU Chemnitz.

**In this folder:** `Core/Src/main.c` (all application code is in its `USER CODE` blocks)
and this README. The full STM32CubeIDE project (`.ioc`, startup files, HAL drivers)
is regenerated from the settings listed below.

**Design goal: crash-proof following, independent of the track layout.** The
follower does not know the track geometry. It must never hit the lead train,
whatever the lead does (speed up, slow down, stop, reverse).

---

## Contents

1. [Background](#background)
2. [Hardware](#hardware)
3. [Pin map](#pin-map)
4. [Firmware architecture](#firmware-architecture)
5. [Control logic](#control-logic)
6. [Telemetry](#telemetry)
7. [Build, flash and run](#build-flash-and-run)
8. [Development log](#development-log)
9. [Problems we hit and how we solved them](#problems-we-hit-and-how-we-solved-them)
10. [Status and roadmap](#status-and-roadmap)

---

## Background

The first version ran on a LEGO MINDSTORMS EV3 brick (MATLAB/Simulink). The
EV3 rode in a wagon and physically turned the knob of the train's remote with a
motor. That worked, but it was slow, indirect and limited by the EV3 toolchain.

This repository is the migration to an STM32:

- **Toolchain:** STM32CubeIDE and STM32CubeMX, plain C, no MATLAB.
- **Actuator:** the remote is no longer used. The locomotive's motor is wired
  directly to an H-bridge controlled by the Nucleo.
- **Sensor:** HC-SR04 ultrasonic sensor, read by timer input capture.

The trains are BlueBrixx locomotives with Mould King 2.4 GHz hubs. Early on we
assumed LEGO Power Functions (infrared) and planned to command the train by IR.
That assumption was wrong (see the problems table), so the IR plan was dropped
and the hub on the follower was disconnected.

---

## Hardware

| Part | Role |
|---|---|
| STM32 Nucleo-F411RE | Controller (84 MHz Cortex-M4) |
| Infineon BLDC Motor Control Shield with IFX007T | Two of its three half-bridges form a full H-bridge for the brushed DC motor |
| HC-SR04 | Forward distance sensor (echo through a 1 kΩ / 2 kΩ divider, 5 V to 3.3 V) |
| Urgenex 7.4 V 2S Li-ion, 2000 mAh | Motor supply on shield J3 (measured 7.96 V full) |
| 130-size brushed DC motor | In the locomotive bogie, on shield J2 (OUT1, OUT2) |
| 10 kΩ resistor | Pull-down on the driver enable line (motor stays off during reset and flashing) |
| Dot board, 103 × 80 mm | Carrier board: Nucleo and shield sit side by side in female sockets |

### Carrier board (dot board)

The Nucleo plugs into four 1×19 female strips on its morpho headers (CN7, CN10).
The Infineon shield plugs into Arduino-layout female sockets next to it. All
connections are soldered wires on the underside:

| Nucleo (morpho) | Shield / sensor | Function |
|---|---|---|
| A0 / PA0 (CN7) | Shield D11 (IN_U) | PWM, forward |
| A1 / PA1 (CN7) | Shield D10 (IN_V) | PWM, reverse |
| A2 / PA4 (CN7) | Shield D6 + D5 (INH_U, INH_V) | Driver enable, 10 kΩ pull-down to GND |
| GND (CN7) | Shield GND | Common ground |
| 5V (CN7) | HC-SR04 VCC | Sensor supply |
| D11 / PA7 (CN10) | HC-SR04 TRIG | 10 µs trigger pulse |
| D12 / PA6 (CN10) | HC-SR04 ECHO via 1 kΩ, with 2 kΩ to GND | Echo, divided to 3.3 V |

The Nucleo's D11 (sonar trigger) and the shield's D11 (motor input) are
different signals and are never connected to each other.

Shield notes, verified with a multimeter:

- The shield's logic inputs have 10 kΩ series resistors; 3.3 V logic drives them fine.
- IFX007T legs (legs up, counted from the right): 1 GND, 2 IN, 3 INH, 4 OUT, 5 SR, 6 IS, 7 VS.
- J2 screw 1 = OUT1, screw 2 = OUT2. To reverse the motor direction, swap the motor
  wires on J2. Never reverse the battery.

---

## Pin map

| Pin | Peripheral | Use |
|---|---|---|
| PA0 | TIM2 CH1 | Motor IN_U PWM (forward) |
| PA1 | TIM2 CH2 | Motor IN_V PWM (reverse) |
| PA4 | GPIO out, pull-down | MOTOR_INH (driver enable) |
| PA6 | TIM3 CH1 input capture | HC-SR04 echo, both edges |
| PA7 | TIM3 CH2 PWM | HC-SR04 trigger, 10 µs every 50 ms |
| PA2 / PA3 | USART2, DMA1 Stream 6 TX | Telemetry over the ST-LINK virtual COM port |
| PA5 | GPIO | LD2 heartbeat |
| PB3 | SWO | Trace output |
| TIM11 | Base timer | 50 ms control tick |

Reserved: PB8/PB9 for a future I2C gyro (GY-521). PA8 and TIM1 are free.
LCD 1602 pins (PA9, PA10, PB4, PB5, PB10, PC7) are configured but not wired.

### Clock and timer settings

| Item | Setting | Result |
|---|---|---|
| SYSCLK | HSI 16 MHz → PLL (M 16, N 336, P 4) | 84 MHz, flash latency 2 WS |
| APB1 / APB2 | 42 MHz / 84 MHz | Timer clocks 84 MHz |
| TIM11 | PSC 8399, ARR 499 | 50 ms tick (20 Hz loop) |
| TIM3 | PSC 83, ARR 49999, CH2 pulse 10 | 1 µs resolution, 50 ms ping period, 10 µs trigger |
| TIM2 | PSC 0, ARR 4199 | 20 kHz PWM, 4200 steps |
| USART2 | 115200 8N1 | CSV telemetry at 20 Hz |

---

## Firmware architecture

**Pattern: flag in the interrupt, work in the main loop.**

- `TIM11` interrupt sets `tick_pending` every 50 ms (and counts an overrun if
  the previous tick was not finished).
- `TIM3` input capture interrupt records the echo's rising and falling edge
  times and publishes `echo_us` with a fresh flag.
- The main loop, on each tick:
  1. Takes a consistent snapshot of the echo value (interrupts briefly disabled).
  2. Filters the distance (`sensor_update`).
  3. Runs the controller (`control_step`).
  4. Sends one CSV telemetry line by DMA.
  5. Measures its own execution time with the DWT cycle counter (`work_us`).

All application code lives inside `USER CODE` blocks, so CubeMX can regenerate
the project without overwriting it.

### Motor driver modes

| Mode | IN_U | IN_V | INH | Effect |
|---|---|---|---|---|
| Drive forward | PWM | low | high | Speed set by duty |
| Drive reverse | low | PWM | high | Speed set by duty |
| Brake | low | low | high | Both motor terminals to GND, active braking |
| Float | low | low | low | Driver off, motor coasts |

`motor_drive(cmd)` takes a command from −1000 to +1000 (per mille of full duty).

### Sensor filtering (`sensor_update`)

- Echo shorter than 116 µs (under 2 cm) is ignored as noise.
- Echo of 23,200 µs or more (4 m) means "nothing in range". With no target, the
  HC-SR04 holds ECHO high for about 48 ms, which would otherwise read as about 8 m.
- A 3-sample median removes single stray echoes.
- The sensor counts as **lost** only after 6 ticks (300 ms) without any echo.
  A single missing echo is normal: with no target, only every second ping completes.

---

## Control logic

`control_step()` is a small state machine. Safety rules are checked first and
always win over following.

| State | Letter | Condition | Action |
|---|---|---|---|
| Lost | `L` | No echo for 300 ms | Brake |
| Retreat | `R` | Gap ≤ 20 cm (or still under 25 cm while retreating) | Reverse, capped at 2 s |
| Stop | `S` | Gap ≤ 25 cm, or stopped and gap still ≤ 30 cm | Brake |
| Blind crawl | `B` | Lead vanished while it was closer than 1 m | Crawl at minimum speed for up to 2 s |
| Wait | `W` | Blind crawl expired | Brake |
| Search | `C` | Nothing in range and lead not recently close | Drive at search speed |
| Follow | `A` | Lead in sight | Speed scales with the gap, full speed beyond `D_FAR` |

Details:

- **Hysteresis:** after a stop, the follower only drives forward again once the
  gap is above 30 cm, so it does not chatter at the 25 cm line.
- **Reverse without a rear sensor** is acceptable under the documented
  assumption that only two trains are on the loop.
- **Ramp:** speed increases are limited to 10 % per tick; decreases are immediate.
- **Never trust stale data:** when the reading is lost, the follower brakes
  instead of continuing on the last command.

### Tuning constants

Set in `Core/Src/main.c` (`USER CODE BEGIN PD`). Values tuned on the desk:

| Constant | Meaning |
|---|---|
| `D_REVERSE_MM` | Retreat at or below this gap (200) |
| `D_STOP_MM` | Brake at or below this gap (250) |
| `D_RELEASE_MM` | Drive forward again only above this gap (300) |
| `D_FAR_MM` | Gap beyond which the follower runs at `CMD_FOLLOW` (450) |
| `CMD_SEARCH` | Speed with nothing in sight (400 = 40 %) |
| `CMD_FOLLOW` | Maximum follow speed (1000 = full throttle) |
| `CMD_MIN` | Slowest crawl (150) |
| `CMD_REVERSE` | Retreat speed (400 = 40 %) |
| `CMD_RAMP` | Maximum speed increase per tick (100) |

Between 25 cm and 45 cm the follow speed scales linearly from `CMD_MIN` up to `CMD_FOLLOW`.

---

## Telemetry

One CSV line per tick (20 Hz) on the ST-LINK virtual COM port, 115200 baud:

```
t_ms,tick,overruns,tx_drops,work_us,echo_us,d_mm,fresh,d_filt,state,mode,cmd
```

| Field | Meaning |
|---|---|
| `overruns` | Ticks where the loop had not finished in time (should stay 0) |
| `tx_drops` | Lines skipped because DMA was still busy (should stay 0) |
| `work_us` | Loop execution time (about 50 µs, budget 50,000 µs) |
| `echo_us`, `d_mm` | Raw echo time and raw distance |
| `fresh` | 1 if a new echo arrived this tick |
| `d_filt` | Filtered distance used by the controller |
| `state` | Controller state letter (see table above) |
| `mode` | Motor mode: D drive, B brake, F float |
| `cmd` | Motor command, −1000 to +1000 |

---

## Build, flash and run

1. Open the project in STM32CubeIDE (File → Import → Existing Projects into Workspace).
2. Build (Ctrl+B). Enable "save automatically before build" to avoid building stale files.
3. Flash with the Run button over the Nucleo's ST-LINK.
4. Open a serial terminal on the ST-LINK COM port at 115200 baud to see telemetry.
5. Power-up order: USB first (LD2 blinks), then the battery on J3.
   Test with the wheels lifted off the track first.

After changing the `.ioc` file, regenerate code (Alt+K) and rebuild.

---

## Development log

### Phase 0: project skeleton (26 Oct 2026)
Clock tree, TIM11 control tick, USART2 telemetry with DMA, heartbeat LED, DWT
timing. **Result:** 50 ms ticks, 0 overruns, 0 dropped lines, about 49 µs of work per tick.

### Phase 1.1: HC-SR04 on TIM3 (26 Oct)
Trigger as a 10 µs PWM pulse on TIM3 CH2, echo measured by input capture on
both edges of CH1. **Result:** continuous distance readings in the telemetry.

### Hardware correction and motor driver (27 Oct)
Discovered the trains use Mould King 2.4 GHz hubs, not LEGO Power Functions.
Dropped the IR approach. Mapped the Infineon IFX007T shield with a multimeter,
wired it to the Nucleo, and added 20 kHz PWM on TIM2. **Result:** motor runs
forward, reverse, brake and float under Nucleo control (2.5 V average at 30 %
duty from a 7.96 V battery).

### Controller (28 Oct)
Sensor filtering, the state machine above, retreat logic and speed tuning.
Verified the logic in a host simulation before flashing (it found a missing
state in the hysteresis rule). **Result:** desk tests pass ("all good").

### Carrier board (29 Oct)
Moved everything from jumper wires and a breadboard onto a soldered dot board:
Nucleo and shield side by side in sockets, sonar header with the divider,
pull-down resistor. Every connection mapped by continuity test before soldering,
and checked again after.

---

## Problems we hit and how we solved them

| Problem | Cause | Fix |
|---|---|---|
| Changes had no effect after building | Wrong project selected (another workspace project), and `main.c` not saved | Select the right project before building; enable save-before-build |
| Control tick stopped after adding the sonar | The new capture callback replaced the timer period callback | Keep both callbacks; dispatch by timer instance inside each |
| Sonar gave one reading per reset | TIM3 CH2 pulse was 0, so no trigger pulse was generated | Set the pulse to 10 (10 µs) in CubeMX |
| Ultrasonic read about 8 m with nothing in front | HC-SR04 holds ECHO about 48 ms when it hears nothing | Treat echo ≥ 23,200 µs as "nothing in range" |
| Target "lost" in open space | Only every second ping completes when nothing reflects | A missing echo alone is not a loss; require 300 ms without any echo |
| COM port error 5 | The port was already open in another terminal | Reuse the existing terminal |
| IR control of the train did not work | Wrong assumption: the trains use 2.4 GHz radio hubs, not LEGO PF infrared | Dropped IR; drive the motor directly with an H-bridge |
| Control tick died after removing TIM1 | TIM1 and TIM11 share one interrupt; unticking TIM1 disabled it | Re-enable the shared interrupt from TIM11's NVIC settings |
| Motor PWM at 238 Hz instead of 20 kHz | TIM2 prescaler left at 83 | Set the prescaler to 0 |
| `undefined reference to MX_TIM1_Init` | Pasted a whole `main.c` generated before TIM1 was removed | Regenerate code (Alt+K); paste only `USER CODE` blocks, never whole generated files |
| Duplicate `USER CODE` markers | Copy and paste error | Removed the duplicate pair |
| 3.2 V on the motor pin after flashing | The old firmware was still running | Reflash and confirm with the multimeter |
| Shield pins confusing during mapping | Probing without a reference point | Use the IC's GND leg as the landmark, map IN and INH by resistance |
| Motor turned the wrong way | Motor wires soldered the other way round inside the loco | Swap the wires on J2 (never reverse the battery) |
| Simulation found a stuck state | The hysteresis rule did not include the Wait state | Added W to the set of stopped states |
| Socket strips would not snap | Female headers cannot be broken like male ones | Pull out one pin, cut through the empty slot |
| Shield header off the 2.54 mm grid | The Arduino 10-pin header is offset 1.5 mm | Bend the socket legs slightly to fit the dot board |
| Underside wiring mirrored | Flipping the board swaps left and right | Mapped every signal by continuity test and marked each joint before wiring |

**Lessons that saved the most time:**

- Change one thing at a time and confirm it in telemetry before the next step.
- Paste code into `USER CODE` blocks only. Whole-file pastes bring stale generated code.
- Measure before trusting assumptions about hardware (the IR detour cost a day).
- Continuity-test every soldered connection before applying power.

---

## Status and roadmap

**Working:** sensor, motor drive, controller on the desk, carrier board soldered.

**Next:**

1. First power-up of the soldered board (USB only, then battery with wheels lifted).
2. Stopping distance at full speed against a soft obstacle.
3. **Speed matching:** use the closing speed (rate of change of the gap, smoothed
   over 0.3 to 0.5 s) to adjust the follower's command so it matches the lead's
   speed, plus a time-to-collision brake (brake if gap / closing speed < about 1 s).
   The existing stop and retreat rules stay on top as the safety layer.
4. **Blind curves:** wheel tachometer (photo-interrupter or Hall sensor) for a
   blind travel budget, and a GY-521 gyro to detect curves.

**Later:** custom PCB (small H-bridge such as TB6612FNG, 5 V regulator), and
possibly a multizone time-of-flight sensor (VL53L7CX) for a wider field of view.
Bluetooth only for telemetry, never in the safety loop.
