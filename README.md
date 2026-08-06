# DROMA — quadcopter testbench (Drohnenversuchsstand)

DROMA is a testbench for small quadcopters flying indoors under an OptiTrack
infrared motion-capture system. This repository contains everything that makes
one drone fly: the Simulink models of the plant and of **two** flight
controllers, the code-generation and verification pipeline, the Teensy firmware
for drone and ground station, the motion-capture toolchain, and the flight-test
documentation.

The core principle is model-based design taken all the way: the flight
controller is built and simulated in Simulink, code-generated to C++, proven
bit-equal against the model by golden tests, and flashed onto a **Teensy 4.1**
on the drone. The controller you simulate is literally the controller that
flies.

Three conventions before you touch anything:

- The model frame is **z-up**, not NED.
- Every parameter lives in `Simulation/scripts/params.m` (via the
  `scripts/init/init_*.m` builders). A number typed into a block is a bug.
- The MATLAB Function blocks are thin `_sl` wrappers; the real algorithms are
  `.m` files in `scripts/functions/` and `scripts/flatness/`. Edit those, never
  the blocks — `.slx` is binary and inline code never shows up in a diff.

---

## The system at a glance

```
OptiTrack cameras ──► Motive (PC, streams rigid-body pose via NatNet, Up Axis = Z)
        │
        ▼
bench(_flat).slx      Simulink "ground control" on the PC, 100 Hz
        │  USB frame
        ▼
gcs_sender(_flat)     ground-station Teensy ── nRF24 radio (ch 76, 250 kbps) ──►
        ▼
drone_hal(_flat)      drone Teensy 4.1: MPU-6050, battery ADC, generated
                      controller class at 1 kHz, OneShot125 → ESCs
```

The drone (m = 0.985 kg, 4S pack) is tracked by the cameras; Motive streams its
pose to the PC, the bench model computes/relays setpoints, the radio carries
them to the drone, the drone flies, the cameras see it — closed loop. Several
airframes exist (`id=1`, `id=2`, …); the firmware selects per-drone IMU mount
calibration via ID pins.

Two SysML views of this structure live next to this file:
[`DROMA_BDD.puml`](DROMA_BDD.puml) (what contains what, down to the `.m`
functions) and [`DROMA_IBD.puml`](DROMA_IBD.puml) (signal flow, logical vs.
physical interfaces). Render with PlantUML (needs Java + Graphviz).

---

## Two controller variants

|                    | Cascade                                   | Flatness-based                              |
|--------------------|-------------------------------------------|---------------------------------------------|
| Control law        | PD position (ground, 100 Hz) + geometric attitude (drone, 1 kHz) | Flatness-based tracking control (exact linearization), entirely on the drone at 1 kHz; the ground streams mocap pose + trajectory incl. feedforward (2-frame OTA protocol) |
| Simulink models    | `quadcop.slx`, `bench.slx`, `mcu.slx`, `gcu.slx`, `link.slx` | same names with `_flat` suffix              |
| Algorithm sources  | `scripts/functions/`                      | `scripts/flatness/`                         |
| Firmware           | `drone_hal.cpp`, `gcs_sender.cpp`         | `drone_hal_flat.cpp`, `gcs_sender_flat.cpp` |
| Recert pipeline    | `run_mcu_recert`, `run_mcu_arm_codegen`   | `run_mcu_flat_recert`, `run_mcu_flat_arm_codegen` |
| Git                | flight-proven state on `main`             | developed on `feature/flatness-tracking`    |

The `_flat` family is strictly additive — the cascade stays untouched and
flyable at all times. Drone **and** sender Teensy must always run the same
variant.

---

## Repository map

```
DROMA/
├── README.md                    you are here
├── DROMA_BDD.puml               SysML: composition hierarchy
├── DROMA_IBD.puml               SysML: signal flow
├── LICENSE
├── Motive/                      OptiTrack side: camera calibrations (.mcal),
│                                NatNet MATLAB plugin + DLLs, Motive quick-start guide
└── Simulation/                  the engineering content
    ├── DROMA.prj                MATLAB project — open this FIRST (paths + PreLoadFcn)
    ├── README.md                deep dive: simulation, codegen gates, firmware, safety
    ├── Testmatrix_Erstflug.md   flight-test campaign, living document (German)
    ├── Handover_Drohnenschwarm_Sim_7.md   long engineering log: locked decisions, pinouts
    ├── models/                  quadcop/bench + referenced models, plus the *_flat family
    ├── scripts/
    │   ├── params.m             single source of truth for every parameter
    │   ├── setup_buses.m        Simulink bus definitions
    │   ├── init/                init_*.m parameter builders, one per subsystem
    │   ├── functions/           cascade algorithms (the real sources)
    │   ├── flatness/            flatness controller + its init/link/eval scripts
    │   ├── motive/              NatNet path setup, mocap origin, IMU mount calibration
    │   ├── sitl/                C++ golden tests, codegen automation, SITL_Runbook.md
    │   └── test/                verify_*.m unit checks
    ├── hardware/                Teensy firmware (both variants), bench tools,
    │                            build_sketches.sh, generated ARM code (mcu_arm/, mcu_flat_arm/)
    └── data/                    flight logs and per-flight evaluation results
```

---

## New here? Read in this order

1. This file, then the two PlantUML diagrams.
2. [`Simulation/README.md`](Simulation/README.md) — how the simulation is
   built, the wrapper pattern, the safety latches, the two codegen gates, the
   firmware. Written for the cascade; the flatness variant mirrors it 1:1 with
   the `_flat` suffix.
3. [`Simulation/Testmatrix_Erstflug.md`](Simulation/Testmatrix_Erstflug.md) —
   what has been proven on hardware, what is open, current blockers (German).
4. [`Simulation/Handover_Drohnenschwarm_Sim_7.md`](Simulation/Handover_Drohnenschwarm_Sim_7.md)
   — the reasoning behind locked design decisions.
5. Before you change `mcu.slx` or `mcu_flat.slx`:
   [`Simulation/scripts/sitl/SITL_Runbook.md`](Simulation/scripts/sitl/SITL_Runbook.md).

**First hour:** open `Simulation/DROMA.prj` in MATLAB (sets up all paths), open
`models/quadcop.slx`, press Run. Opening a top model triggers `params.m`, which
fills the workspace with every parameter struct the model needs.

---

## Everyday workflows

**Run the full simulation** — `quadcop.slx` (cascade) or `quadcop_flat.slx`:
everything simulated, fixed-step ode4 at 1 ms.

**Fly on hardware** — `bench.slx` / `bench_flat.slx`: same ground station, but
mocap comes in live from Motive and commands go out over serial to the sender
Teensy. Runs at 10 ms; Simulation Pacing must be 1.0×.

**Change a parameter** — `scripts/params.m` → `scripts/init/init_*.m` (or
`scripts/flatness/init_flatness.m`). Position gains and everything ground-side
take effect on the next run, no flash. Anything inside `mcu(_flat).slx` is
firmware and needs the full cycle below.

**Change controller logic** — edit the `.m` source, then re-certify and flash:

```matlab
run_mcu_recert('<repo>\Simulation')        % cascade:  regen C++ + golden CSV
run_mcu_flat_recert('<repo>\Simulation')   % flatness: same for mcu_flat
```

then Gate B on the host (`ctest -C Release` in `scripts/sitl/build`, plus the
Debug exe if Smart App Control blocks a freshly built test binary), then ARM
codegen (`run_mcu_arm_codegen` / `run_mcu_flat_arm_codegen`), then flash.

**Flash** (from `Simulation/hardware/`, Git bash):

```bash
./build_sketches.sh --compile                    # compile-check every sketch, all modes
./build_sketches.sh --upload-drone-flight        # cascade drone firmware
./build_sketches.sh --upload-sender              # cascade ground-station Teensy
./build_sketches.sh --upload-drone-flat-flight   # flatness drone firmware
./build_sketches.sh --upload-sender-flat         # flatness ground-station Teensy
```

Firmware modes: `BENCH` (motors dead), `THRUST` (motors + telemetry report),
`FLIGHT`. There are also `--upload-drone-{bench,thrust}` /
`--upload-drone-flat-{bench,thrust}` targets and bench tools
(`--upload-scan/esccal/freq/batt/chanscan`).

**Evaluate a flight** — run the bench model during the flight, then
`scripts/functions/flight_evaluation.m` (cascade) or
`scripts/flatness/flight_evaluation_flat.m`. Both shift the logged reference
back by `T_lead` before computing errors: the ground station evaluates the
trajectory at `t + T_lead` to compensate the chain dead time, so the *logged*
reference is the time-advanced one, and errors must be measured on the
space-time schedule. Results land in `Simulation/data/`.

---

## Safety — the things that surprise everyone

Condensed; full list in [`Simulation/README.md`](Simulation/README.md).

- The drone **boots latched** (`estop = 2`, no link yet). Motors stay at zero
  until a rising `ack` edge arms it; every link loss latches again.
- The **battery latch is permanent**: filtered voltage ≤ 12.0 V forces a
  descent until power-cycle, even if the voltage recovers. Bench supplies stay
  above 12 V.
- **Battery health is a flight parameter.** A worn pack (high internal
  resistance) collapses under hover load and quietly ruins tracking long before
  the latch trips — see the pre-flight blocker note in `Testmatrix_Erstflug.md`
  and measure packs before flying.
- `bench.slx` without Simulation Pacing 1.0× trips the drone watchdog
  immediately.

---

## Requirements (short)

- **MATLAB/Simulink R2025b** with Stateflow, Aerospace Blockset, MATLAB /
  Simulink / Embedded Coder, Simulink Desktop Real-Time; Instrument Control
  Toolbox for the bench serial blocks.
- **Motive** streaming over NatNet with **Up Axis = Z**; plugin and DLLs are in
  `Motive/`, path helper in `Simulation/scripts/motive/`.
- **Host tests:** CMake ≥ 3.15, C++17 compiler (MSVC is what is used).
- **Firmware:** `arduino-cli` with Teensy core `teensy:avr@1.60.0` and the RF24
  library; board `teensy:avr:teensy41`.

Details and the version pins live in [`Simulation/README.md`](Simulation/README.md).
