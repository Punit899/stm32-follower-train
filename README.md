# Anti-collision follower train (STM32 Nucleo-F411RE)

A follower locomotive that keeps a safe distance behind a lead train on the same
track. The controller runs on an STM32 Nucleo-F411RE and drives the locomotive's
motor directly through an Infineon IFX007T H-bridge shield. An HC-SR04 ultrasonic
sensor measures the gap; a BMI160 gyro detects curves, and an MG90S servo turns
the sensor into the curve so the lead stays in view.

Research assistant (HiWi) project, TU Chemnitz.

**In this folder:** `Core/Src/main.c` (all application code is in its `USER CODE` blocks)
and this README. The full STM32CubeIDE project (`.ioc`, startup files, HAL drivers)
is regenerated from the settings listed below. The gyro, the servo and the flight
recorder are set up in code, so they need **no CubeMX changes**.

**Design goal: crash-proof following.** The follower must never hit the lead
train, whatever the lead does (speed up, slow down, stop, reverse), on straights
and in curves.

---

## Contents

1. [Background](#background)
2. [Hardware](#hardware)
3. [Wiring](#wiring)
4. [Pin map](#pin-map)
5. [Firmware architecture](#firmware-architecture)
6. [Control logic](#control-logic)
7. [Curves: gyro and sensor servo](#curves-gyro-and-sensor-servo)
8. [Telemetry and flight recorder](#telemetry-and-flight-recorder)
9. [Build, flash and run](#build-flash-and-run)
10. [Tuning constants](#tuning-constants)
11. [Development log](#development-log)
12. [Problems we hit and how we solved them](#problems-we-hit-and-how-we-solved-them)
13. [Status and roadmap](#status-and-roadmap)

---

## Background

The first version ran on a LEGO MINDSTORMS EV3 brick (MATLAB/Simulink). The
EV3 rode in a wagon and physically turned the knob of the train's remote with a
motor. That worked, but it was slow, indirect and limited by the EV3 toolchain.

This repository is the migration to an STM32:

- **Toolchain:** STM32CubeIDE and STM32CubeMX, plain C, no MATLAB.
- **Actuator:** the remote is no longer used. The locomotive's motor is wired
  directly to an H-bridge controlled by the Nucleo.
- **Sensors:** HC-SR04 ultrasonic (gap), BMI160 gyro (curves).

The trains are BlueBrixx locomotives with Mould King 2.4 GHz hubs. Early on we
assumed LEGO Power Functions (infrared) and planned to command the train by IR.
That assumption was wrong, so the IR plan was dropped and the hub on the
follower was disconnected.

---

## Hardware

| Part | Role |
|---|---|
| STM32 Nucleo-F411RE | Controller (84 MHz Cortex-M4) |
| Infineon BLDC Motor Control Shield with IFX007T | Two of its three half-bridges form a full H-bridge for the brushed DC motor |
| HC-SR04 | Distance sensor (echo through a 1 kΩ / 2 kΩ divider, 5 V to 3.3 V) |
| DFRobot Gravity BMI160 (6-axis IMU) | Gyro Z axis = turn rate, for curve detection |
| MG90S micro servo | Turns the HC-SR04 towards the inside of a curve |
| Elegoo MB102 power module | Separate 5 V supply for the servo, fed from the train battery |
| 470 µF / 16 V electrolytic capacitor | Across the servo supply, against current spikes |
| Urgenex 7.4 V 2S Li-ion, 2000 mAh | Motor supply on shield J3 (7.96 V full) and input of the MB102 |
| 130-size brushed DC motor | In the locomotive bogie, on shield J2 (OUT1, OUT2) |
| 10 kΩ resistor | Pull-down on the driver enable line (motor stays off during reset and flashing) |
| Dot board, 103 × 80 mm | Carrier board: Nucleo and shield side by side in female sockets |

---

## Wiring

### Carrier board (dot board)

The Nucleo plugs into female strips on its morpho headers. The Infineon shield
plugs into Arduino-layout female sockets next to it. Connections are soldered
wires on the underside (remember: the underside is mirrored).

| Nucleo | Goes to | Function |
|---|---|---|
| A0 / PA0 | Shield D11 (IN_U) | Motor PWM, forward |
| A1 / PA1 | Shield D10 (IN_V) | Motor PWM, reverse |
| A2 / PA4 | Shield D6 + D5 (INH_U, INH_V) | Driver enable, 10 kΩ pull-down to GND |
| GND | Shield GND | Common ground |
| 5V | HC-SR04 VCC | Sensor supply |
| D11 / PA7 | HC-SR04 TRIG | 10 µs trigger pulse |
| D12 / PA6 | HC-SR04 ECHO via 1 kΩ, with 2 kΩ to GND | Echo, divided to 3.3 V |
| 3V3 | BMI160 + | Gyro supply |
| GND | BMI160 − | Gyro ground |
| D15 / PB8 | BMI160 C (SCL) | Software I2C clock |
| D14 / PB9 | BMI160 D (SDA) | Software I2C data |
| D7 / PA8 | Servo orange (signal) | 50 Hz servo pulse, TIM1 CH1 |
| GND | MB102 GND | **Shared ground** for the servo signal |

The Nucleo's D11 (sonar trigger) and the shield's D11 (motor input) are
different signals and are never connected to each other.

### Servo power (separate)

```
 7.4 V battery ──► MB102 DC jack (centre = +)   jumper on 5V
 MB102 +5V ─────► servo red
 MB102 GND ─────► servo brown ──► Nucleo GND (shared ground)
 Nucleo D7 ─────► servo orange
 470 µF across MB102 + / − near the servo (stripe on −)
```

The servo used to run from the Nucleo 5V. Its current spikes pulled the 5 V and
3.3 V rails down and reset the gyro into sleep mode. With its own supply the gyro
stays stable.

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
| PA7 | TIM3 CH2 PWM | HC-SR04 trigger, 10 µs every 65 ms |
| PA8 | TIM1 CH1 PWM (set up in code) | Servo signal, 50 Hz, 1 µs resolution |
| PB8 / PB9 | GPIO open drain (set up in code) | Software I2C to the BMI160, 100 kHz |
| PA2 / PA3 | USART2, DMA1 Stream 6 TX | Telemetry over the ST-LINK virtual COM port |
| PA5 | GPIO | LD2 heartbeat (1 Hz gyro OK, 5 Hz no gyro) |
| PC13 | GPIO in | Blue button: held during reset = print the flight recorder |
| PB3 | SWO | Trace output |
| TIM11 | Base timer | 50 ms control tick |
| Flash sectors 6, 7 | | Flight recorder, 2 × 128 KB |

In CubeMX, **leave PA8, PB8 and PB9 unassigned**: the code configures them itself.

### Clock and timer settings

| Item | Setting | Result |
|---|---|---|
| SYSCLK | HSI 16 MHz → PLL (M 16, N 336, P 4) | 84 MHz, flash latency 2 WS |
| APB1 / APB2 | 42 MHz / 84 MHz | Timer clocks 84 MHz |
| TIM11 | PSC 8399, ARR 499 | 50 ms tick (20 Hz loop) |
| TIM3 | PSC 83, ARR set to 64999 in code, CH2 pulse 10 | 1 µs resolution, 65 ms ping period, 10 µs trigger |
| TIM2 | PSC 0, ARR 4199 | 20 kHz PWM, 4200 steps |
| TIM1 | PSC from PCLK2 in code, ARR 19999 | 50 Hz servo PWM, 1 µs steps |
| USART2 | 115200 8N1 | CSV telemetry at 20 Hz |

---

## Firmware architecture

**Pattern: flag in the interrupt, work in the main loop.**

- `TIM11` interrupt sets `tick_pending` every 50 ms (and counts an overrun if the
  previous tick was not finished).
- `TIM3` input capture interrupt records the echo's rising and falling edge times
  and publishes `echo_us` with a fresh flag.
- The main loop, on each tick:
  1. Takes a consistent snapshot of the echo (interrupts briefly disabled).
  2. Filters the distance (`sensor_update`).
  3. Reads the gyro and decides straight or curve (`curve_update`), including the gyro watchdog.
  4. Points the sonar servo (`servo_update`).
  5. Runs the controller (`control_step`).
  6. Writes one entry to the flash flight recorder (`log_write`).
  7. Sends one CSV telemetry line by DMA.
  8. Measures its own execution time with the DWT cycle counter (`work_us`, about 0.7 ms).

At start-up: reset cause is read, the flight recorder starts (or prints, if the
blue button is held), PWM and sonar start, the servo centres, then the gyro is
found and its zero offset measured. **Keep the train still for about 1.5 s after
power-up or reset.**

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
In a curve, forward commands are capped by `fwd_cap`.

### Sensor filtering (`sensor_update`)

- Echo shorter than 116 µs (under 2 cm) counts as 2 cm.
- Echo of 23,200 µs or more (4 m) means "nothing in range".
- Ping every **65 ms** (not 50 ms): late echoes from far walls die out before the next ping.
- A 3-sample median removes single stray echoes.
- The sensor counts as **lost** only after 6 ticks (300 ms) without any echo.
- **In a curve**, a single reading under 35 cm skips the median and is used at once.

---

## Control logic

`control_step()` is a **gain-scheduled, nonlinear PID controller** on the gap
error, embedded in a supervisory state machine. Safety rules are checked first
and always win over following.

### PID core

```c
e    = d - 250;                       /* + too far, - too close (mm)        */
want = e * kp / 10                    /* P: 4.0 per mm, 4.8 when too close  */
     + kd_v                           /* D: 3 x closing speed (mm/s)        */
     + i_sum / 10;                    /* I: learns the lead's speed         */
```

| Term | Details |
|---|---|
| P | `KP_NUM 40` (4.0 per mm); `KP_REV_NUM 48` when too close, so it reacts harder |
| I | Accumulates only within ±10 cm of the target; reset when on the wrong side (anti-windup); clamped |
| D | Closing speed over 300 ms (derivative on measurement); faded out between 60 and 150 cm, where fast changes are mostly curve noise |

### Around the PID

| Rule | Why |
|---|---|
| Hold with hysteresis: stop within ±2 cm, restart only beyond ±4 cm or > 10 cm/s | No twitching from sensor jitter |
| Never forward when too close, never reverse when not too close | No direction hunting around the target |
| Minimum forward 45 % (`CMD_MIN`), first kick 45 % | The loaded train does not move below about 45 % |
| Top speed 70 %, boost to 100 % only while the lead pulls away | Curve safety and stopping distance |
| Ramp up 30 % per tick, reverse 7.5 % per tick, brake 100 ms before a direction change | Smooth, protects the gearbox |
| Reacquisition guard and appear rule | A target that pops up or jumps closer is not trusted blindly |
| Unsteady mode `U` | Chaotic readings: roll on steadily at most 50 %, unless a close target is confirmed |

### States

| State | Letter | Condition | Action |
|---|---|---|---|
| Lost | `L` | No echo for 300 ms | Brake |
| Follow | `A` | Lead in sight, too far | PID, forward |
| Retreat | `R` | Lead too close | PID, reverse |
| Hold | `H` | At the target gap, calm | Brake |
| Unsteady | `U` | Readings jump around | Roll on steadily |
| Blind crawl | `B` | Lead vanished closer than 1 m | Crawl 2 s |
| Wait | `W` | After the blind crawl | Brake 2 s, then creep on |
| Search | `C` | Nothing in range | 70 % (45 % in a curve) |

In the logs the letter is **lower case while in a curve** (`a`, `c`, `h`, ...).

---

## Curves: gyro and sensor servo

### Curve detection (`curve_update`)

The BMI160 gyro measures the turn rate (yaw) around its Z axis. It must be
mounted **flat**; its rotation on the train does not matter. In a curve the yaw is
speed / curve radius (about 30 to 75 °/s on our track); on a straight it is about 0.

- **Enter curve:** |yaw| above 15 °/s for 100 ms.
- **Leave curve:** |yaw| below 8 °/s for 0.2 s while driving forward.
- **Stopped in a curve:** the yaw drops to 0, but the train is still in the curve.
  The curve is only judged again after 1 s of driving forward.
- **In a curve:** forward capped at 45 %, "nothing ahead" means crawl at 45 %
  instead of searching at 70 %, and one close reading brakes at once.

**Gyro watchdog:** a power or wiring glitch can reset the BMI160 into sleep mode,
where it still answers on I2C but reads 0 forever. Every 0.5 s the code checks
its power mode register, and every tick it checks for frozen data (exactly the
same raw value for 1 s). Either way the gyro is switched back on in about 0.1 s.
The count is in the telemetry column `gyro_rst`.

### Sensor servo (`servo_update`)

In a curve the lead sits off to the inside (about 27 to 35° for our curves), outside
the HC-SR04's ±15° beam. The servo turns the sensor towards it:

- angle = yaw × 0.5, limited to ±35°;
- the servo **keeps the largest angle of the current curve** and does not follow
  the yaw back down when the train slows, stops or restarts in the curve;
- returns to straight ahead quickly (25° per tick) once the curve is over;
- turns in smoothly (8° per tick).

---

## Telemetry and flight recorder

### Live telemetry

One CSV line per tick (20 Hz) on the ST-LINK virtual COM port, 115200 baud:

```
t_ms,tick,overruns,tx_drops,work_us,echo_us,d_mm,fresh,d_filt,state,mode,cmd,yaw,servo,gyro_rst
```

| Field | Meaning |
|---|---|
| `overruns` | Ticks where the loop had not finished in time (should stay 0) |
| `tx_drops` | Lines skipped because DMA was still busy |
| `work_us` | Loop execution time |
| `echo_us`, `d_mm` | Raw echo time and raw distance |
| `fresh` | 1 if a new echo arrived this tick |
| `d_filt` | Filtered distance used by the controller |
| `state` | Controller state letter (lower case in a curve) |
| `mode` | Motor mode: D drive, B brake, F float |
| `cmd` | Motor command, −1000 to +1000 |
| `yaw` | Turn rate in °/s; **−128 = gyro not found** |
| `servo` | Servo angle in degrees (+ = left) |
| `gyro_rst` | How often the gyro watchdog had to wake the gyro |

Start-up lines begin with `#`: recorder status, `# gyro: BMI160 at 0x69, bias ...`
or `# gyro: NOT FOUND, curve logic off`.

### Flight recorder (runs without a laptop)

Every run is recorded into flash (sectors 6 and 7, 128 KB each, about 13.6 min per
run, 8 bytes per tick). It keeps the last two runs and **never overwrites a log
that has not been printed**: if both are unprinted, the next runs are not recorded
(`# recorder: OFF`).

Each log also stores what started the board (power-on, reset pin, brown-out, ...).

**To print the logs:**

1. Close every serial console (CubeIDE, Arduino IDE), or the port is busy.
2. Run the PowerShell script below on the laptop.
3. When it says *Ready*: hold the **blue button**, press **RESET once**, release
   the blue button. LD2 stays still while printing, then blinks fast.
4. Wait until PowerShell says `Saved ... lines`. Do not press anything during the dump.

```powershell
$p = New-Object System.IO.Ports.SerialPort COM7,115200,None,8,One
$p.ReadTimeout = 60000
$p.Open()
$out = New-Object System.Collections.Generic.List[string]
Write-Host "Ready. Now HOLD the blue button and press RESET on the Nucleo..."
try {
  $out.Add($p.ReadLine())
  $p.ReadTimeout = 3000
  while ($true) {
    $out.Add($p.ReadLine())
    if ($out.Count % 500 -eq 0) { Write-Host "$($out.Count) lines" }
  }
} catch [TimeoutException] { }
$p.Close()
$out | Set-Content -Path "$HOME\Desktop\log.txt"
Write-Host "Saved $($out.Count) lines to Desktop\log.txt"
```

Dump format: `t_ms,d_raw,d_filt,state,mode,cmd,yaw`. The most recent run is printed last.

---

## Build, flash and run

1. Open the project in STM32CubeIDE (File → Import → Existing Projects into Workspace).
2. Copy `Core/Src/main.c` from this repository over the project's file.
3. Build (Ctrl+B). Enable "save automatically before build".
4. Flash with the Run button. If it fails with *Error finishing flash operation*:
   Debug Configurations → Debugger → Reset behaviour → **Connect under reset**.
5. Open one serial console on the ST-LINK COM port at 115200 baud.
6. Power-up: USB or power bank on the Nucleo, battery on J3 and the MB102,
   MB102 switched on. **Keep the train still for 1.5 s.**
7. Check: LD2 blinks slowly (1 Hz), the console shows the gyro line, `yaw` is not −128.

LED meanings: LD1 (ST-LINK) red = no PC connected, green = PC connected; this
is normal on a power bank. LD2: 1 Hz = running with gyro, 5 Hz = running without gyro.

---

## Tuning constants

All in `Core/Src/main.c`, `USER CODE BEGIN PD`.

| Constant | Value | Meaning |
|---|---|---|
| `D_HOLD_MM` | 250 | Target gap |
| `KP_NUM` / `KP_REV_NUM` | 40 / 48 | P gain (÷10) normally / when too close |
| `KD_NUM` | 3 | D gain on closing speed |
| `I_ZONE_MM` | 100 | Integral only within ±10 cm |
| `CMD_FOLLOW`, `CMD_SEARCH` | 700 | Top speed 70 % |
| `CMD_BOOST` | 1000 | 100 % while the lead pulls away |
| `CMD_MIN`, `CMD_FWD_START` | 450 | Slowest forward / first kick |
| `CMD_REVERSE` / `CMD_REVERSE_MIN` | 1000 / 350 | Reverse limits |
| `SONAR_PERIOD_US` | 65000 | Ping period |
| `CURVE_ENTER_DPS` / `CURVE_EXIT_DPS` | 150 / 80 | Curve thresholds (0.1 °/s) |
| `CURVE_EXIT_TICKS` | 4 | 0.2 s straight to leave the curve |
| `CURVE_RESTART_TICKS` | 20 | After a stop in a curve: 1 s of driving before judging |
| `CMD_CURVE` | 450 | Forward cap in curves |
| `CURVE_STOP_MM` | 350 | In a curve, one reading closer than this brakes at once |
| `SERVO_CENTER_US` | 1500 | Servo "straight ahead" trim |
| `SERVO_DIR` | 1 | Set to −1 if the servo turns the wrong way |
| `SERVO_GAIN_X10` / `SERVO_MAX_DEG` | 5 / 35 | Angle = yaw × 0.5, max ±35° |
| `SERVO_STEP_DEG` / `SERVO_RETURN_STEP_DEG` | 8 / 25 | Turn-in / return speed per tick |
| `SERVO_TEST` | 0 | 1 = servo swings 0/+30/0/−30° every second (hardware test) |
| `TEST_NO_SONAR` | 0 | 1 = ignore the sonar (gyro laps only, **no collision protection**) |
| `CAL_MODE` | 0 | 1 dead-band sweep, 2 full speed 3 s, 3 stopping distance |

---

## Development log

### Phase 0: project skeleton (26 Oct 2026)
Clock tree, TIM11 control tick, USART2 telemetry with DMA, heartbeat LED, DWT
timing. 50 ms ticks, 0 overruns, 0 dropped lines.

### Phase 1: sensor and motor driver (26 to 27 Oct)
HC-SR04 on TIM3 (trigger PWM, echo input capture). Discovered the trains use
2.4 GHz hubs, not LEGO PF infrared; dropped IR and wired the IFX007T shield
directly, 20 kHz PWM on TIM2.

### Phase 2: controller and carrier board (28 to 29 Oct)
First state machine, host simulation, then a soldered dot board with Nucleo and
shield side by side.

### Phase 3: gap holding and tuning (30 Oct to 3 Nov)
- PID gap holding at 25 cm in both directions, hold hysteresis, anti-windup.
- Gentler, then stronger reverse; never reverse when not too close (57 direction
  flips in a real log down to 6).
- Faster forward response: gain 4.0, 45 % minimum forward (the loaded train does
  not move below that), boost while the lead pulls away.
- Ping every 65 ms against ghost echoes from far walls.
- Flash flight recorder for runs without a laptop.
- Sensor phantoms from the track and floor: sensor tilted up.

### Phase 4: curves (3 to 5 Nov)
- BMI160 gyro on software I2C: curve detection, 45 % cap in curves, crawl instead
  of search when nothing is visible in a curve. Tested in sonar-bypass laps:
  9 of 9 curves detected, no false curves on the straights.
- Immediate brake on one close reading in a curve (a crash log showed the lead
  appearing at only 4.9 cm, after 0.2 s of filter delay).
- MG90S servo turns the sensor into the curve; holds the curve angle while stopped
  and on restart; fast return after the curve.
- Servo moved to its own 5 V supply (MB102) after it disturbed the gyro.
- Gyro watchdog against sleep mode and frozen data.

---

## Problems we hit and how we solved them

| Problem | Cause | Fix |
|---|---|---|
| Changes had no effect after building | Wrong project selected, `main.c` not saved | Select the right project; enable save-before-build |
| Control tick stopped after adding the sonar | The capture callback replaced the period callback | Keep both callbacks; dispatch by timer instance |
| Sonar gave one reading per reset | TIM3 CH2 pulse was 0 | Set the pulse to 10 (10 µs) |
| About 8 m with nothing in front | HC-SR04 holds ECHO about 48 ms when it hears nothing | Echo ≥ 23,200 µs = nothing in range |
| Target "lost" in open space | Only every second ping completes | Lost only after 300 ms without any echo |
| COM port error 5 | Port open in another console (even a 2nd or 3rd CubeIDE console) | Close all consoles, open one |
| IR control did not work | Trains use 2.4 GHz hubs, not LEGO PF | Drive the motor directly with an H-bridge |
| Control tick died after removing TIM1 | TIM1 and TIM11 share one interrupt | Re-enable it from TIM11's NVIC settings |
| Motor PWM at 238 Hz | TIM2 prescaler left at 83 | Prescaler 0 |
| `undefined reference to MX_TIM1_Init` | Pasted an old generated `main.c` | Regenerate; paste only `USER CODE` blocks |
| Motor turned the wrong way | Motor wires swapped in the loco | Swap the wires on J2 |
| Forward at 5.6 cm on the desk | Integral windup | Reset the integral on the wrong side, never forward when too close |
| Train twitched while holding | Sensor jitter woke the hold | Hold hysteresis |
| Forward/back shaking | Phantom readings | Appear rule, speed estimate reset on jumps |
| 57 direction flips in a run | Reversing at 35 to 50 cm | Never reverse unless too close |
| Train sat humming at 31 % | Loaded train needs about 45 % to break away | Minimum forward 45 % |
| Stray 66 to 69 cm readings | Echo from a wall about 9 m away arriving after the next ping | Ping every 65 ms |
| Constant 34 to 38 cm with nothing ahead | The beam's lower edge hit the track and floor | Tilt the sensor up 5 to 8° |
| Crash at the curve | Narrow beam looks past the lead in the curve | Gyro curve mode, servo, instant brake |
| Gyro yaw exactly 0 in a lap | Gyro mounted on its edge | Mount the gyro flat |
| Laps missing from the recorder | Two unprinted logs, recorder off; dumps interrupted by pressing reset | Print before running; wait for `Saved` |
| Board restarts on the power bank | Supply dips | Tested on the laptop cable; separate servo supply |
| Servo did nothing, 0.95 V across it | Servo GND not connected | Resoldered the ground |
| Servo returned to 0° when stopping in a curve | Angle followed the yaw | Hold the curve's largest angle |
| Curve mode ended after a stop | Slow restart looked like a straight | Judge the curve only after 1 s of driving |
| Gyro reads 0 after a while | Servo current spikes reset it into sleep mode | Servo on its own 5 V; gyro watchdog |
| `yaw = −128`, LD2 fast | Gyro not found (loose wire) | Check + 3V3, − GND, C D15, D D14 |
| *Error finishing flash operation* | Running code erases flash for the recorder | Connect under reset |

**Lessons that saved the most time:**

- Change one thing at a time and confirm it in telemetry before the next step.
- Paste code into `USER CODE` blocks only.
- Measure before trusting assumptions about hardware.
- Continuity-test every soldered connection before applying power.
- Log first, then change the code: most fixes came from reading a real log.

---

## Status and roadmap

**Working:** gap holding in both directions, fast forward response, curve
detection with the gyro, sensor servo, flight recorder, separate servo supply.

**Known limit:** a lead train parked right behind a curve can still appear late,
because the HC-SR04 sees only a narrow cone.

**Next:**

1. **VL53L7CX multizone time-of-flight sensor** (8 × 8 zones, 90° field of view)
   to see the lead around the curve.
2. **Lead-to-follower radio:** a micro:bit on the lead reports "moving / stopped"
   (received by a second micro:bit or an ESP32 over Bluetooth).
3. **Worst-case speed limit after losing the lead** ("moving block"): drive only
   as fast as allows stopping before the point where the lead could be standing.
4. **Kalman filter** for the gap and closing speed, later an EKF with servo angle
   and curve geometry; a wheel encoder for the follower's own speed.

**Later:** custom PCB (small H-bridge, 5 V regulator, connectors). Bluetooth only
for telemetry and lead state, never as the only safety layer.
