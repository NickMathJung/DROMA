# DROMA quadcopter simulation

This is the Simulink model of the DROMA quadcopter, plus the firmware that comes
out of it. The flight controller is built in Simulink, generated to C++, and
flashed onto a Teensy 4.1 on the real drone. So the controller you simulate is
literally the controller that flies, which is the whole point of the setup.

DROMA itself is a testbench of several quadcopters tracked by an infrared motion
capture system. This folder covers one drone and the ground station it talks to.

Two things worth knowing before you touch anything: the model frame is z-up, not
NED, and every parameter lives in `scripts/params.m`. If you find a number typed
into a block, that's a bug.

## The two top-level models

`models/quadcop.slx` is the full simulation. It wires five referenced models into
a closed loop and simulates everything, drone included.

`models/bench.slx` is the hardware bench. It's the same ground station, but with
the drone simulation cut out: position data comes in live from OptiTrack via the
`Motive` block, and commands go out over `Serial Send` to the sender Teensy and
from there by radio to the real drone. It has a rotary switch for `estop` and a
toggle for `ack` so you can drive the safety logic by hand.

The split exists for speed. `quadcop.slx` has to run at the 1 ms base rate
because the plant, sensors and MCU are all simulated in it, and it only manages
about 45% of real time (roughly 3700 missed ticks in 6.76 s on the machine this
was measured on). `bench.slx` runs at `Ts_gcs`, 10 ms, because nothing fast is
simulated in it, so it keeps up with the real world.

## How the simulation fits together

`quadcop.slx` doesn't do much itself, it just connects these five:

| Model | What it is | Runs where | In → out |
|-------|------------|------------|----------|
| `plant.slx`   | 6-DOF rigid body and propeller model | host sim | `rotor_cmd` → `Bus_State, accel_body, dω/dt, R` |
| `sensors.slx` | IMU and motion capture model | host sim | plant motion → `Bus_IMU, Bus_Mocap` |
| `mcu.slx`     | the flight controller | on the drone | `Bus_IMU, Bus_Cmd, batt_count, btn_ack` → `rotor_cmd, led, throttle` |
| `link.slx`    | radio link, packing and packet loss | host sim | `Bus_Cmd_In` → `cmd_out` |
| `gcu.slx`     | ground station | on the ground | `Bus_Mocap, Bus_State, estop, ack` → `Bus_Cmd` |

The control is split across two machines. Attitude estimation (Mahony filter) and
the geometric attitude controller run on the drone at 1 kHz. The position
controller and the landing supervisor run on the ground station at 100 Hz. The
radio carries `Bus_Cmd` between them, so a dropped packet costs you setpoints,
not stabilization.

There are two PlantUML diagrams one level up if you want the structure as a
picture: [`DROMA_BDD.puml`](../DROMA_BDD.puml) for what contains what (both
top-level models, down to the individual `.m` functions), and
[`DROMA_IBD.puml`](../DROMA_IBD.puml) for the signal flow, with logical and
physical interfaces drawn differently. Rendering them needs Java and Graphviz.

## Where the logic actually lives

This one catches people out. The MATLAB Function blocks inside the models do not
contain the algorithms. Every one of them is a thin wrapper that calls a `.m`
file in `scripts/functions/`, and that `.m` file is the real source. The wrapper
carries an `_sl` suffix, because naming it identically would shadow the external
function and recurse.

So if you want to change the position controller, edit
`scripts/functions/pos_ctrl.m`, not the block. The reason for the indirection is
that `.slx` is binary: inline code drifts without anyone noticing and never shows
up in a git diff.

## What's where

```
Simulation/
├── DROMA.prj                 MATLAB project, open this first
├── models/                   quadcop.slx (full sim), bench.slx (hardware rig),
│                             plus the five referenced models
├── scripts/
│   ├── params.m              everything starts here
│   ├── setup_buses.m         bus object definitions
│   ├── init/                 init_*.m, one per subsystem, build the param structs
│   ├── functions/            the actual algorithms, called by the model wrappers
│   ├── test/                 verify_*.m for battery, overspeed, quaternion codegen
│   └── sitl/                 host-side C++ tests and the codegen automation
│       ├── SITL_Runbook.md   read this before re-certifying mcu.slx
│       ├── matlab/           configure_mcu_codegen, log_mcu_golden, sil_check_mcu, ...
│       └── include/ src/ test/    CMake + GoogleTest suite
├── hardware/                 Teensy firmware and bench tools
│   ├── drone_hal.cpp         the drone HAL, wraps the generated MCU class
│   ├── gcs_sender.cpp        ground station Teensy, USB in, nRF24 out
│   ├── esc_calibrate.cpp     ESC arming and manual motor test
│   ├── i2c_scan.cpp          finds the MPU on the I2C bus
│   ├── analyze_frequency.cpp motor spin-up with live voltage, for the throttle curve
│   ├── battery_health.cpp    load test for the pack's internal resistance
│   ├── channel_scan.cpp      nRF24 channel occupancy scanner
│   └── mcu_arm/              Cortex-M7 generated controller code
├── data/project.sldd         data dictionary
└── Handover_Drohnenschwarm_Sim_7.md    the long engineering log
```

## What you need

For the simulation: MATLAB/Simulink R2025b with Stateflow, Aerospace Blockset
(the accelerometer, gyro and quaternion blocks come from there), MATLAB Coder,
Simulink Coder, Embedded Coder, and Simulink Desktop Real-Time for the real-time
sync block.

For `bench.slx` you also need the OptiTrack side: Motive streaming over NatNet,
and Instrument Control Toolbox for the serial blocks. Two settings have to match
or you'll get nothing useful. Motive must stream with **Up Axis = Z** (Settings →
Streaming), because the project is z-up and the `Motive` block deliberately does
not transform anything. And `mocap.streaming_id` in `init_sensors.m` has to match
the rigid body's streaming ID in Motive's Assets pane. If Motive and MATLAB run
on the same machine both IPs are `127.0.0.1`, otherwise set the host IP to the
Motive machine.

For the host tests in `scripts/sitl/`: CMake 3.15 or newer and a C++17 compiler.
MSVC is what's been used. The older leaf-codegen tests also want `MATLAB_ROOT`
set so they can find `tmwtypes.h`.

For firmware: `arduino-cli` with the Teensy core `teensy:avr@1.60.0` and the RF24
library 1.6.1. Board is `teensy:avr:teensy41`. One annoyance: the Arduino
toolchain trips over spaces in paths, and this repo sits under "MAS
Versuchsaufbau". Build from a scratch directory, or use `build_sketches.sh` which
handles it.

## Running it

Open `DROMA.prj` in MATLAB. That sets up the paths. Then open either
`models/quadcop.slx` or `models/bench.slx` and hit run.

Opening a top model triggers `params.m` through its PreLoadFcn, which calls all
the `init_*` builders and drops `quadcop`, `imu`, `mocap`, `link_params`,
`controller`, `safety`, `mahony`, `traj` and `supervisor` into the workspace. If
you get "undefined variable" errors, that's what didn't run.

The solver is fixed-step ode4. `quadcop.slx` steps at `Ts_inner = 1e-3`;
everything else in it samples at an integer multiple of that, with mocap, ground
station and link at 100 Hz and battery monitoring much slower. `bench.slx` steps
at `Ts_gcs = 10e-3`.

## Changing parameters

Start at `scripts/params.m` and follow it into `scripts/init/`. The useful ones:

- `init_quadcop.m` for mass, inertia, geometry, `c_T`, and `p_from_omega`, the
  polynomial that turns rotor speed into throttle percent. The drone flies on a
  4S pack, so the throttle is voltage-normalized: `throttle = polyval(p_from_omega,
  omega) * 22.2 / clamp(V_filt)`, with `V_filt` clamped to `[11.0, 17.5]` V. That
  clamp is not cosmetic, `V_filt` is in the denominator and a dead battery sensor
  reading 0 would otherwise slam all four motors to 100%.
- `init_sensors.m` for IMU full-scale ranges, noise, mocap rate, and the Motive
  connection settings
- `init_controller.m` and `init_estimator.m` for the attitude gains and the
  Mahony `ka`/`kE` weighting
- `init_safety.m` for the overspeed limit (`omega_max = 8.5 rad/s`), the tilt
  cutoff (`tilt_max_deg = 80`, debounced over 80 base ticks) and the debounce
  counts
- `init_battery_manag.m` for the battery scale (`batt_k = 0.0166737` V per ADC
  count, measured on hardware) and the 12.0 V floor
- `init_link.m` for the packet layout and the drop probability (`pdrop = 0.02`)

## Re-certifying the flight controller

Whenever you change `mcu.slx`, or any signal crossing the MCU boundary, you have
to prove the generated C++ still behaves like the model. There are two gates and
the order matters. The full procedure is in
[`scripts/sitl/SITL_Runbook.md`](scripts/sitl/SITL_Runbook.md); this is the short
version.

Gate A is a SIL run inside MATLAB, closed loop:

```matlab
clear configure_mcu_codegen
configure_mcu_codegen('mcu')     % pins the class name to MCU, single tasking
slbuild('mcu')                   % writes scripts/sitl/mcu_ert_rtw/
run scripts/sitl/matlab/log_mcu_golden.m   % records scripts/sitl/data/golden_mcu_io.csv
sil_check_mcu
```

You want `rotor_cmd max|d|` around 1e-14 and zero `led` mismatches. Note that the
MCU block's `led` output has to be wired to something in the top model, even just
a Terminator, or `log_mcu_golden.m` will stop on purpose.

Gate B is the strict one. No MATLAB, tick-exact, runs under CTest:

```powershell
cd scripts/sitl
cmake --build build --config Release
ctest --test-dir build -C Release -R McuGolden --output-on-failure
```

When Gate B goes red, don't guess. Build the replay tool, it tells you the first
diverging tick and which input changed there:

```powershell
cmake --build build --config Release --target diag_mcu_replay
./build/Release/diag_mcu_replay.exe
```

If you add or rename a root port on `mcu.slx`, `mcu_io.hpp` and the name lists in
`log_mcu_golden.m` have to be pulled along. That's the only reconcile point, but
forgetting it gives you a confusing counter assert rather than a clear error.

Generating the ARM code for the Teensy is a separate run. It writes into
`hardware/mcu_arm/` and deliberately leaves the SITL build untouched, so Gate B
stays valid:

```matlab
run scripts/sitl/matlab/run_mcu_arm_codegen.m
```

## Firmware

`drone_hal.cpp` is the drone side. It ticks at 1 kHz: reads the MPU-6050 into
`Bus_IMU`, reads the battery voltage on pin 41 as raw 12-bit counts into
`batt_count`, reads the re-arm button on pin 21 into `btn_ack`, unpacks the nRF24
packet into `Bus_Cmd`, steps the generated `MCU` class, and pushes `throttle` out
to the ESCs as OneShot125. Pin 40 reads current, but that's telemetry only and
doesn't go into the model. If no valid packet arrives for 100 ms the watchdog
forces `estop = 2`.

Two IMU corrections happen in the HAL, not in the model. The gyro bias is
subtracted here and only here. Then `apply_mount` rotates gyro and accel by
`R_mount`, which takes out the roughly 2.3° tilt from the IMU sitting slightly
crooked on the board. That rotation is per-drone: it was measured on id=2, and
id=1 needs its own measurement, so don't assume one `R_mount` fits every airframe.

To compile it you need the ARM `mcu.h` from `hardware/mcu_arm/mcu_ert_rtw/` and
`mcu_packet.hpp` from `scripts/sitl/include/` on the include path. That codec
header is shared with the host tests, so don't fork it.

`gcs_sender.cpp` is the ground station Teensy. Simulink sends it a frame over
USB, it parses that, repacks it into the 29-byte over-the-air format and
broadcasts on nRF24 channel 76 with auto-ack off. The link runs at 250 kbps, not
the nRF's default 1 Mbps: the slower rate buys about 10 dB of receiver
sensitivity, which the drone needed once packet loss showed up on the bench.

The rest of `hardware/` is bench tools, and the last three all assume the
propellers are on and the drone is bolted down, so treat them with respect.
`i2c_scan.cpp` finds the MPU (it should answer at 0x68). `esc_calibrate.cpp` arms
the ESCs and lets you spin one motor by hand. `analyze_frequency.cpp` does the
same but prints the battery voltage once a second, which is how the throttle curve
gets measured against a known voltage. `battery_health.cpp` runs a load test to
get the pack's internal resistance. `channel_scan.cpp` sweeps the 126 nRF
channels looking for the quietest one, which is worth doing since channel 76 sits
right in the upper WiFi band. `drone_hal.cpp` also has a `HAL_SELFTEST` build that
prints a live I/O report.

```bash
arduino-cli compile -b teensy:avr:teensy41 <sketch-dir>
```

## Before you power up the drone

A few behaviours will confuse you on the bench if you don't know about them.

`safety_overspeed` latches a kill on four conditions, and it reports which one in
`fault_src`: gyro overspeed (1), hard kill from `estop = 2` (2), tilt past 80°
held for 80 ms (3), and the physical button (4). Once latched it stays latched.

Clearing the latch needs a rising edge on `ack`, which is the button on pin 21 or
an `ack=1` pulse from the ground station, and it only clears if the drone isn't
still overspeeding and `estop` isn't 2. The drone boots into `estop = 2` because
there's no link yet, so it comes up latched: even after the link is good and
`estop` goes to 0, motors stay at zero until you give it that edge. Every link
dropout kills and latches again, so you re-arm every time.

The battery latch is permanent by design. If filtered voltage drops to 12.0 V or
below, `batt_land` latches and forces a descent, and it stays latched until you
power-cycle, even if the voltage recovers. That's deliberate, it stops the drone
oscillating between sinking and hovering. On a bench supply, stay above 12 V
unless you're specifically testing this.

If you're driving the chain from `bench.slx`, set simulation pacing to 1.0x.
Without it Simulink dumps frames as fast as it can, the drone's watchdog sees a
burst it can't keep up with, and it kills immediately.

## Other docs

`Handover_Drohnenschwarm_Sim_7.md` is the running engineering log. It has the
locked design decisions, pin assignments, the OTA packet layout and the current
test gate status. It's long but it's where the reasoning lives.

`scripts/sitl/SITL_Runbook.md` covers codegen re-certification in detail plus the
MCU port contract. `scripts/sitl/README.md` covers the host golden tests and how
to switch them over to generated code.
