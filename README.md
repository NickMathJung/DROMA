# DROMA — quadcopter testbench (Drohnenversuchsstand)

DROMA is a testbench for small quadcopters flying indoors with the use of an 
**OptiTrack infrared motion-capture system**. This repository contains everything 
that makes one drone and the swarm fly. This includes the Simulink models of 
the plant and of **two** flight controllers, the code-generation and verification 
pipeline, the Teensy firmware for drone and ground station, the motion-capture 
toolchain, and the flight-test documentation.

The core principle is model-based design taken all the way. This means the flight
controller is built and simulated in Simulink, code-generated to C++, proven
to be correct using a SiL pipeline where the response of the generated C++ 
code is compared against the response of the model to the same input  
(called golden tests), and flashed onto a **Teensy 4.1** on the drone. 

Three **conventions**:

- The model frame is **z-up**, not NED.
- Every parameter is located in `Simulation/scripts/params.m` (using the
  `scripts/init/init_*.m` functions). A number typed into a block is a bug.
  Everything should be referenced from the MATLAB Base-Workspace.
- The MATLAB Function blocks are `_sl` wrappers. The actual algorithms are
  `.m` files in `scripts/functions/` and `scripts/flatness/`. Edit those. 
  `.slx` is binary and thus cannot be properly version controlled with Git.

---

## The system

```
OptiTrack cameras --> Motive (PC, streams rigid-body pose using NatNet, Up Axis = Z)
        │
        V
bench(_flat).slx      Simulink "ground control" on the PC, 100 Hz
        |
        │  USB frame
        V
gcs_sender(_flat)     ground-station Teensy ── nRF24 radio (ch 76, 250 kbps) -->
        |
        |  OTA-link using nRF
        V
drone_hal(_flat)      drone Teensy 4.1: MPU-6050, battery ADC, generated
                      controller class at 1 kHz, OneShot125-Protokoll -> ESCs
```

The drone (m = 0.985 kg (differs between individual drones), 4S battery) is 
tracked by the cameras. Motive streams its pose to the PC, the bench model 
computes setpoints, the radio carries them to the drone, the drone flies, 
the cameras measure that. Several airframes exist (`id=1`, `id=2`, `id=3`, `id=4`). 
The firmware selects drone specific IMU mount calibration via the individual ids.

Two SysML views of this structure live next to this file:
[`DROMA_BDD.puml`](DROMA_BDD.puml) (what contains what, down to the `.m`
functions) and [`DROMA_IBD.puml`](DROMA_IBD.puml) (signal flow, logical vs.
physical interfaces). The IBD file holds two diagrams, one for the full
simulation and one for the swarm on hardware. Render with PlantUML (needs Java
+ Graphviz).

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
| Swarm              | up to four drones (`bench.slx`)           | single drone only                           |

The `_flat` family is strictly additive. The cascade stays untouched and
flyable at all times. Drone **and** sender Teensy must always run the same
variant. Swarm flights currently run on the cascade. That work happens on
`feature/swarm-ctrl`.

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
    ├── DROMA.prj                MATLAB project, open this FIRST (paths + PreLoadFcn)
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
    │   ├── swarm/               swarm reference generation, bench InitFcn, animation
    │   ├── motive/              NatNet path setup, mocap origin, IMU mount calibration
    │   ├── sitl/                C++ golden tests, codegen automation, SITL_Runbook.md
    │   └── test/                verify_*.m unit checks
    ├── hardware/                Teensy firmware (both variants), bench tools,
    │                            build_sketches.sh, generated ARM code (mcu_arm/, mcu_flat_arm/)
    └── data/                    flight logs and per-flight evaluation results
        └── videos/              swarm animations (kept local, not in Git)
```

---



**First steps:** open `Simulation/DROMA.prj` in MATLAB (sets up all paths), open
`models/quadcop.slx`, press Run. Opening a top model triggers `params.m`, which
fills the workspace with every parameter struct the model needs.

---

## Workflow

**Run the full simulation**: `quadcop.slx` (cascade) or `quadcop_flat.slx`.
Everything simulated, fixed-step ode4 at 1 ms.

**Fly on hardware**: `bench.slx` / `bench_flat.slx`. Same ground station, but
mocap comes in live from Motive and commands go out over serial to the sender
Teensy. Runs at 10 ms. Simulation Pacing must be 1.0x.

**Swarm mode vs. single-drone waypoint flight**: `bench.slx` is a four-drone
ground station. `MotiveMocapMulti` streams every rigid body listed in
`mocap.streaming_ids` (`scripts/init/init_sensors.m`), one GCS path per entry.
Each path builds its own 82 Byte radio frame, the four frames are concatenated
into a single 328 B USB frame, and the sender Teensy keeps and forwards the
freshest frame per id. Everything user-facing is addressed by **drone id**:
`init_trajectory_swarm(id)` writes `traj_id<id>`, `flight_evaluation(id)`
saves `*_id<id>.mat`, the radio frame carries the id. Which GCS path serves
which id follows from the order of `mocap.streaming_ids`. That list is the
only place where the mapping is set, and the model instance parameters follow
it, so flying other drones does not touch the model. Which mode flies is
decided purely by the workspace at Run:

- *Swarm following*: place the drones, generate the reference tables, fly.
  The full procedure:

  1. **Place the drones.** The default reference is a rotating saddle surface
     whose tracked agents sit at its four corners, all at the same height, so
     the rotation never stacks one drone above another. On the ground that
     means: a rectangle around the cage center, long side along y. Drones 1
     and 2 (order of `mocap.streaming_ids`) stand on the positive-y side,
     drones 3 and 4 on the negative-y side, and within each pair the first
     one stands at positive x. Exact spots do not matter:
     `swarm_precompute` fits the reference to the measured positions
     (translation, yaw, scale). What does matter is the handedness: a
     mirrored placement shows up as a large anchor residual in the report,
     and the tables then start away from the drones, which the InitFcn
     rejects. Yaw is free, each drone holds its measured start yaw.
     Check in Motive that every drone is tracked and all cameras are up.

  2. **Generate the tables** (fresh workspace, so no stale data survives):

     ```matlab
     clear; params
     ref = swarm_precompute(1.45, read_swarm_origins(mocap.streaming_ids));
     init_trajectory_swarm(1);      % argument = drone id, writes traj_id<id>
     init_trajectory_swarm(2);
     init_trajectory_swarm(3);
     init_trajectory_swarm(4);
     ```

     `swarm_precompute` runs the containment MAS (`main_DROMA.m` in the
     hyperbolic-2d-containment repository), stretches time by `kappa`,
     resamples to the 100 Hz grid and checks every agent against the cage
     and the flight envelope, plus every pair against a minimum distance and
     downwash. It writes `data/swarm_ref.mat`. Regenerate it before every
     flight, because test runs leave stale tables behind. Fly only when the
     report shows every agent `OK` and every pair with 0 downwash samples.

  3. **Run `bench.slx`.** The model InitFcn (`bench_init_fcn`) reads all
     origins in one Motive frame, verifies each drone against its table
     start (< 0.2 m, else the start aborts), measures the start yaw and
     holds it, prepends a 4 s arm phase, stacks the tables into the shared
     `traj` and builds `xi0_all` and `m_all`. Do not move the drones between
     `swarm_precompute` and takeoff. Arm with the ack switch, the flight is
     4 s arm plus about 29 s table. The landing spots travel with the
     rotation, so the drones do not land where they started.

  4. **Evaluate**: `flight_evaluation(id)` per drone,
     `swarm_animation([1 2 3 4])` for the video.

  Agent assignment follows the number of drones: two drones get the grid
  agents (1,1) and (20,9), three get (1,1), (20,1) and (1,9), four get all
  four corners. Override with
  `swarm_precompute(kappa, p0, struct('agents', [i1 j1; i2 j2]))`.

- *Single-drone waypoint flight (classic cascade)*: make sure no `traj_id*`
  tables are in the workspace (a fresh model open runs `params.m`, which
  clears them). The InitFcn then falls back to the waypoint trajectory
  from `scripts/init/init_trajectory.m` (`traj.P` waypoints, `traj.Tseg`
  segment durations, `traj.Tdwell` dwell times), anchored per drone at its
  measured pose. All listed ids must be *tracked*, but power up **only** the
  drone that should fly. Several powered drones in waypoint mode can collide.
  With only one physical drone in the cage, set
  `mocap.streaming_ids = [id id id id]`. All GCS paths then control the same
  drone with identical frames, and the sender forwards one frame per id.

**Swap in a different airframe** (e.g. `id=2` to `id=3`): set the BCD id pins
on the drone (the firmware binary is the same for every id), measure its IMU
mount and enter `MOUNT[id]` in `hardware/drone_hal.cpp` (procedure in the
comment above the table, identity = not yet measured), create a Motive rigid
body with that streaming id, weigh the airframe and enter the mass in
`quadcop.m_id(id)` (`scripts/init/init_quadcop.m`), and update
`mocap.streaming_ids` in `init_sensors.m`. The mass feeds the feedforward of
that GCS path, so a wrong entry leaves a constant height offset for the
integrator to remove.

**Change a parameter**: `scripts/params.m` and `scripts/init/init_*.m` (or
`scripts/flatness/init_flatness.m`). Position gains and everything ground-side
take effect on the next run, no flash. Anything inside `mcu(_flat).slx` is
firmware and needs the full cycle below.

**Change controller logic**: edit the `.m` source, then re-certify and flash.

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

**Evaluate a flight**: run the bench model during the flight, then
`flight_evaluation(id)` with the drone id (cascade, results land as
`*_id<id>.mat`) or `scripts/flatness/flight_evaluation_flat.m`. Both shift the
logged reference back by `T_lead` before computing errors. The ground station
evaluates the trajectory at `t + T_lead` to compensate the chain dead time, so
the *logged* reference is the time-advanced one, and errors must be measured on
the space-time schedule. Results land in `Simulation/data/`.

The mean commanded thrust over a steady stretch is a useful health check:
`mean(F_des)/g` should come out near the weighed mass. A large gap means the
thrust chain of that airframe is off, usually battery charge or worn
propellers, and the integrator has been hiding it.

**Animate a swarm flight**: `swarm_animation(ids)` renders the recorded mocap
poses of the given drones together with the reference surface into
`data/videos/`. `mp4_to_gif(file, t_start, t_end)` cuts a clip out of such a
video and writes a GIF of the same name next to it.

---

## Safety

Condensed. Full list in [`Simulation/README.md`](Simulation/README.md).

- The drone **boots latched** (`estop = 2`, no link yet). Motors stay at zero
  until a rising `ack` edge arms it. Every link loss latches again.
- The **battery latch is permanent**: filtered voltage <= 12.0 V forces a
  descent until power-cycle, even if the voltage recovers. Bench supplies stay
  above 12 V.
- **Battery health is a flight parameter.** A worn pack (high internal
  resistance) collapses under hover load and quietly ruins tracking long before
  the latch trips. See the pre-flight blocker note in `Testmatrix_Erstflug.md`
  and measure packs before flying.
- **The ground station trusts the mocap stream.** If Motive loses a rigid body,
  the last valid pose is held, and the position controller keeps commanding
  against a pose that no longer moves. A drone can climb away unseen, and the
  correction on re-acquire is violent. Check that all eight cameras are up and
  that every drone is tracked before arming.
- `bench.slx` without Simulation Pacing 1.0x trips the drone watchdog
  immediately.

---

## Requirements (short)

- **MATLAB/Simulink R2025b** with Stateflow, Aerospace Blockset, MATLAB /
  Simulink / Embedded Coder, Simulink Desktop Real-Time, and Instrument Control
  Toolbox for the bench serial blocks.
- **Motive** streaming over NatNet with **Up Axis = Z**. Plugin and DLLs are in
  `Motive/`, path helper in `Simulation/scripts/motive/`.
- **Host tests:** CMake >= 3.15, C++17 compiler (MSVC is what is used).
- **Firmware:** `arduino-cli` with Teensy core `teensy:avr@1.60.0` and the RF24
  library, board `teensy:avr:teensy41`.
- **Swarm references:** the hyperbolic-2d-containment repository next to
  `DROMA/`. `swarm_precompute` adds it to the path and calls `main_DROMA.m`
  there.

Details and the version pins live in [`Simulation/README.md`](Simulation/README.md).
