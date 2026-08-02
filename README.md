# DROMA — Quadcopter SITL Simulation & Firmware

MATLAB/Simulink **model of a quadcopter, plus the
C++ flight-firmware generated from it**. Part of the **DROMA** testbench (a swarm of
quadcopters verified against an infrared motion-capture system). The same
Simulink flight controller (`mcu.slx` and `mcu_flat.slx`) is code-generated to a **Teensy 4.1**, so
the simulation and the real drone run the same control law.

> Model frame is **z-up** (not NED). `scripts/params.m` is the single source of
> truth for all parameters.

---

## Architecture

`models/quadcop.slx` is the top-level simulation model. It wires
five referenced models into a closed loop:

| Model         | Role                                  | Deployment  | Key inputs → outputs |
|---------------|---------------------------------------|-------------|----------------------|
| `plant.slx`   | 6-DOF rigid-body + propeller dynamics | SITL (host) | `rotor_cmd` → `Bus_State, accel_body, dω/dt, R` |
| `sensors.slx` | IMU + motion-capture model            | SITL (host) | plant motion → `Bus_IMU, Bus_Mocap` |
| `mcu.slx`     | **Flight controller** (→ C++)         | Deployed SW (drone) | `Bus_IMU, Bus_Cmd, batt_count` → `rotor_cmd, led, throttle` |
| `link.slx`    | Radio link (TX/RX codec, packet loss) | SITL (host) | `Bus_Cmd_In` → `cmd_out` |
| `gcu.slx`     | Ground-control station                | Deployed SW (ground) | `Bus_Mocap, Bus_State, estop, ack` → `Bus_Cmd` |

**Control is distributed:** the inner **attitude** loop (Mahony estimator +
geometric attitude controller) runs on the drone MCU at **1 kHz**
(`Ts_inner = 1e-3`); the outer **position** loop + landing supervisor run on the
ground station at **100 Hz**. The radio link carries the setpoint bus
(`Bus_Cmd`) between them.

SysML views of this structure live in the repo root:
[`../DROMA_BDD.puml`](../DROMA_BDD.puml) (composition hierarchy) and
[`../DROMA_IBD.puml`](../DROMA_IBD.puml) (signal flow, logical vs. physical
interfaces). Render with the VS Code *PlantUML* extension (needs Java + Graphviz).

---

## Repository layout

```
Simulation/
├── DROMA.prj                 MATLAB project (open this first — sets up paths)
├── models/                   Simulink models
│   ├── quadcop.slx           top-level SITL harness (run this)
│   ├── mcu.slx  gcu.slx  plant.slx  sensors.slx  link.slx
├── scripts/
│   ├── params.m              central parameter file (runs via PreLoadFcn)
│   ├── setup_buses.m         Simulink bus object definitions
│   ├── init/                 init_*.m — per-subsystem parameter builders
│   ├── functions/            MATLAB Function / Stateflow sources + codec (sm3, link_tx/rx)
│   ├── test/                 verify_*.m (battery, overspeed, quat codegen)
│   └── sitl/                 host-side C++ golden tests + codegen automation
│       ├── SITL_Runbook.md   how to re-certify mcu.slx after a change
│       ├── README.md         golden-test decision table
│       ├── matlab/           configure_mcu_codegen, log_mcu_golden, run_gate_a, …
│       ├── include/ src/ test/   CMake + GoogleTest/CTest suite
├── hardware/                 Teensy firmware (Arduino/PlatformIO)
│   ├── drone_hal.cpp         drone HAL (wraps the ARM-generated MCU class)
│   ├── gcs_sender.cpp        ground-station sender Teensy (USB → nRF24)
│   ├── esc_calibrate.cpp  i2c_scan.cpp   bench tools
│   ├── mcu_arm/              ARM (Cortex-M7) generated flight-controller code
│   └── build_sketches.sh     assembles flashable sketches (+ --compile/--upload)
├── data/project.sldd         Simulink data dictionary
└── Handover_Drohnenschwarm_Sim_7.md   detailed engineering log / decisions
```

---

## Requirements

**Simulation (MATLAB):** MATLAB/Simulink **R2025b** with
- Stateflow, Aerospace Blockset (accelerometer/gyroscope/quaternion blocks),
- MATLAB Coder, Simulink Coder, Embedded Coder (C++ code generation),
- Simulink Desktop Real-Time (the *Real-Time Synchronization* block).

**Host golden tests (`scripts/sitl/`):** CMake ≥ 3.15 and a C++17 compiler (MSVC).
Leaf-codegen tests additionally need `MATLAB_ROOT` (for `tmwtypes.h`).

**Firmware (`hardware/`):** `arduino-cli` with Teensy core `teensy:avr@1.60.0`
and the `RF24` library (1.6.1). Target board: `teensy:avr:teensy41`.
> Build firmware from a path **without spaces** (this repo path contains
> “MAS Versuchsaufbau”) — copy the sketch to a scratch dir or use `build_sketches.sh`.

---

## Getting started (simulation)

1. **Open the project:** launch MATLAB and open `DROMA.prj` (adds `scripts/`,
   `scripts/init/`, `scripts/functions/` to the path). Alternatively add those
   folders to the path manually.
2. **Open the top model:** open `models/quadcop.slx`. Its `PreLoadFcn` runs
   `scripts/params.m`, which populates the workspace via the `init_*` builders:
   `quadcop, imu, mocap, link_params, controller, safety, mahony, traj, supervisor`.
3. **Run.** The model uses a fixed-step **ode4** solver at `Ts_sim = Ts_inner =
   1e-3 s`. Every sample time in the model is an integer multiple of `Ts_inner`
   (mocap/GCS/link at 100 Hz, battery monitor slower).

### Changing parameters
Edit `scripts/params.m` and the `scripts/init/init_*.m` builders — never hard-code
values in the blocks. Examples:
- `init_quadcop.m` — mass, inertia, geometry, `p_from_omega_sq` (ω²→throttle poly)
- `init_sensors.m` — IMU FSR/noise, mocap rate
- `init_controller.m` / `init_estimator.m` — attitude gains, Mahony `ka/kE`
- `init_safety.m` / `init_battery_manag.m` — `omega_max = 8.5 rad/s`, battery
  scale `batt_k = 0.0166737 V/count`, floor `12.0 V`
- `init_link.m` — packet fields, drop probability `pdrop = 0.02`

---

## Re-certifying the flight controller (SITL codegen)

When you change `mcu.slx` (or a signal at the MCU boundary), re-verify that the
**generated C++** still matches the model. Full procedure in
[`scripts/sitl/SITL_Runbook.md`](scripts/sitl/SITL_Runbook.md). Short version —
two gates, always in this order:

**Gate A — SIL (in MATLAB, closed loop):**
```matlab
clear configure_mcu_codegen
configure_mcu_codegen('mcu')     % pins class name MCU + SingleTasking
slbuild('mcu')                   % -> scripts/sitl/mcu_ert_rtw/ (C++ class MCU, packNGo)
run scripts/sitl/matlab/log_mcu_golden.m   % -> scripts/sitl/data/golden_mcu_io.csv
sil_check_mcu                    % expect rotor_cmd max|d| ~1e-14, led 0 mismatches
```
> The MCU block’s `led` output must be wired in the top model (a Terminator is
> enough) or `log_mcu_golden.m` aborts by design.

**Gate B — host golden (MATLAB-free, tick-exact, CTest):**
```powershell
cd scripts/sitl
cmake --build build --config Release
ctest --test-dir build -C Release -R McuGolden --output-on-failure
```
If Gate B fails, diagnose with the replay target:
```powershell
cmake --build build --config Release --target diag_mcu_replay
./build/Release/diag_mcu_replay.exe
```

**ARM code generation** (for the Teensy), which writes to `hardware/mcu_arm/`
without touching the SITL build:
```matlab
run scripts/sitl/matlab/run_mcu_arm_codegen.m   % target = 'arm' (Cortex-M7, double, LE)
```

---

## Firmware (Teensy 4.1)

- `hardware/drone_hal.cpp` — 1 kHz tick: MPU-6050 → `Bus_IMU`, ADC(pin 40) →
  `batt_count`, nRF24 unpack → `Bus_Cmd`, runs the generated `MCU` class,
  `throttle` → OneShot125 ESCs, watchdog → `estop = 2`. Needs the ARM `mcu.h`
  (`hardware/mcu_arm/mcu_ert_rtw/`) and the codec SSOT `mcu_packet.hpp`
  (`scripts/sitl/include/`) on the include path.
- `hardware/gcs_sender.cpp` — ground-station Teensy: USB frame from Simulink →
  `gcs::parse` → `pkt::pack` → nRF24 broadcast (channel 76, 1 Mbps, auto-ack off).
- Bench tools: `i2c_scan.cpp` (find MPU at 0x68), `esc_calibrate.cpp` (ESC learn /
  motor test), and `drone_hal.cpp`’s `HAL_SELFTEST` build.

Compile example:
```bash
arduino-cli compile -b teensy:avr:teensy41 <sketch-dir>
```

### ⚠️ Operational safety (read before any bench/HW test)
- The drone **boots with `estop = 2`** (no link) → hard-kill latch set. Once the
  link is up (`estop = 0`), rotors/throttle stay **0** until a **rising `ack`
  edge** (button on pin 21, or GCS `ack=1` pulse) releases the latch. Any link
  loss re-arms the kill → re-arm again.
- `batt_land` is **permanent**: if `Vf ≤ 12.0 V` the battery latch forces a hard
  descent until **power-cycle**, even if voltage recovers. When testing on a bench
  supply, keep it **> 12 V**.
- Driving the ground-station chain from Simulink requires **Simulation Pacing at
  1.0×** (otherwise a frame burst trips the drone watchdog immediately).

---

## Further reading
- [`Handover_Drohnenschwarm_Sim_7.md`](Handover_Drohnenschwarm_Sim_7.md) —
  detailed session log, locked design decisions, pin assignments, gate status.
- [`scripts/sitl/SITL_Runbook.md`](scripts/sitl/SITL_Runbook.md) — codegen
  re-certification procedure and the MCU port contract.
- [`scripts/sitl/README.md`](scripts/sitl/README.md) — host golden-test decision
  table and codegen switch-over.
```
