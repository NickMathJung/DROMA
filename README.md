# DROMA — quadcopter testbench 

DROMA is a testbench for small quadcopters flying indoors with the use of an 
**OptiTrack infrared motion-capture system**. This repository contains everything 
to make one drone and also a swarm of up to four drones fly. This includes the 
Simulink models of the plant and the flight controller, the automatic 
code-generation of the C++ code, that is flashed onto the microcontrollers of 
the quadcopters, and also a verification pipeline to verificate the generated 
C++ code before flashing it. Furthermore, the Teensy firmware for drone and 
ground station and the motion-capture toolchain.

The core principle is so-called model-based design. This means the flight
controller is built and simulated in Simulink, the code is generated to C++, 
proven to be correct using a software-in-the-loop (SiL) pipeline where the 
output of the generated C++ code is compared against the output of the SIMULINK
model to the same input (called golden tests), and flashed onto a **Teensy 4.1** 
on the drone. 

Three **conventions**:

- The body frame has its **z-axis pointing up**, this can differ from some literature.
- Every parameter is located in `Simulation/scripts/params.m` (using the
  `scripts/init/init_*.m` functions). A number typed into a SIMULINK block can 
  thus be considered a bug. Everything should be referenced from the MATLAB 
  Base-Workspace.
- The MATLAB Function blocks are `_sl` wrappers. The actual algorithms are
  `.m` files located in `scripts/functions/`. Edit those. 

---

## The system

```
OptiTrack cameras --> Motive (tracking software) streams quadcopter pose using a NatNet client
        │
        V
bench.slx = Simulink "ground control" model at 100 Hz
        |
        │  USB frame
        V
gcs_sender = ground station Teensy 4.1, which coordinates the attached nRF24 radio (ch 76, 250 kbps) 
        |
        |  
        V
drone_hal = quadcopter firmware/software with Teensy 4.1, MPU-6050, battery surveillance,
                      controller at 1 kHz, OneShot125-Protokoll (PWM) -> ESCs
```

The quadcopter (mass = 0.985 kg) is tracked by the cameras and Motive -> which streams the 
pose to the SIMULINK model -> The bench.slx model computes setpoints -> the radio transmitter 
sends them to the quadcopter -> the drone flies -> the cameras measure that. 
Several quadcopters exist (`id=1`, `id=2`, `id=3`, `id=4`). 
The firmware (on the Teensy) selects a quadcopter specific rotation matrix, which rotates the 
IMU coordinate system, due to slightly different mounting, into the body coordinate system.
In the flatness variant it also selects the quadcopter specific thrust factor.

Two SysML views of this structure are located in the workspace next to this file:
[`DROMA_BDD.puml`](DROMA_BDD.puml) and [`DROMA_IBD.puml`](DROMA_IBD.puml). 
Render with PlantUML (needs Java + Graphviz).

---

## Two controller variants exist:

**Cascade:**

| Control principle | PD position control o the ground station at 100 Hz and geometric attitude control on quadcopter at 1 kHz 
| Simulink models   | `quadcop.slx`, `bench.slx`, `mcu.slx`, `gcu.slx`, `link.slx`      
| Algorithm location| `scripts/functions/`                                              
| Firmware          | `drone_hal.cpp`, `gcs_sender.cpp`                                 
| Recert pipeline   | `run_mcu_recert`, `run_mcu_arm_codegen`                                                             
| Swarm             | up to four drones (`bench.slx`)                                   

**Flatness-based:**

| Control principle | Exact linearization, entirely on the drone at 1 kHz. The ground streams mocap pose + reference position, velocity and acceleration (one 32 B OTA frame per drone) 
| Simulink models   | same names as the cascade with `_flat` suffix                     
| Algorithm location| `scripts/flatness/`                                               
| Firmware          | `drone_hal_flat.cpp`, `gcs_sender_flat.cpp`                       
| Recert pipeline   | `run_mcu_flat_recert`, `run_mcu_flat_arm_codegen`                                         
| Swarm             | up to four drones (`bench_flat.slx`)                              

The `_flat` family is strictly additive. The cascade stays untouched and
flyable at all times. Drone **and** sender Teensy must always run the same
variant.

---

## Repository map

```
DROMA/
├── README.md                    you are here
├── DROMA_BDD.puml               hierarchy diagram
├── DROMA_IBD.puml               signal flow
├── LICENSE
├── Motive/                      OptiTrack: camera calibrations (.mcal),
│                                NatNet MATLAB plugin, Motive quick-start guide
└── Simulation/                  
    ├── DROMA.prj                MATLAB project, open this FIRST (paths + PreLoadFcn)
    ├── models/                  quadcop/bench + referenced models
    ├── scripts/
    │   ├── params.m             all parameters
    │   ├── setup_buses.m        SIMULINK bus definitions
    │   ├── init/                init_*.m called by params.m
    │   ├── functions/           flight control algorithms
    │   ├── flatness/            flatness controller + its init/link/eval scripts
    │   ├── swarm/               virtual MAS simulation, animation
    │   ├── motive/              NatNet path setup, MoCap related stuff, IMU mount calibration
    │   ├── sitl/                C++ tests, codegen automation, SITL_Runbook.md (for software-in-the-loop)
    │   └── test/                verify_*.m unit checks, generator of the quaternion test vectors
    ├── hardware/                Teensy firmware (both variants), tools like build_sketches.sh, generated ARM code (mcu_arm/, mcu_flat_arm/)
    └── data/                    flight logs 
        └── videos/              swarm animations (kept local, not in Git)
```

---

**First steps:** open `Simulation/DROMA.prj` in MATLAB (sets up all paths), open
`models/quadcop.slx`, press Run. Opening a top model triggers `params.m`, which
fills the workspace with every parameter struct the model needs.

---

## Workflow

**Run the full simulation**: `quadcop.slx` or `quadcop_flat.slx`.
Everything simulated, fixed-step ode4 at 1 ms.

**Fly the quadcopter**: `bench.slx` / `bench_flat.slx`. Same ground station, but
mocap comes in live from Motive and commands go out over serial to the sender
Teensy. Runs at 10 ms. 

**Swarm mode and waypoint flight for one quadcopter**: `bench.slx` contains four 
quadcopter controllers -> one gcs model per quadcopter. `MotiveMocapMulti` streams 
every rigid body listed in `mocap.streaming_ids` (`scripts/init/init_sensors.m`).
Each gcs model builds its own 82 Byte radio frame, the four frames are concatenated
into a single 328 B USB frame, and the sender Teensy (attached to USB) keeps and 
forwards the freshest frame per id. 
Lot of files require the **drone id** id as parameter:
`init_trajectory_swarm(id)` writes `traj_id<id>`, `flight_evaluation(id)`
saves `*_id<id>.mat`, the radio frame carries the id. Which GCS path serves
which id follows from the order of `mocap.streaming_ids`. That list is the
only place where the mapping is set, and the model instance parameters follow
it, so flying other drones does not touch the model. Which mode flies is
decided purely by the workspace at Run.

`bench_flat.slx` is the same four-drone ground station for the flatness
variant: identical Motive/selector/switch front end and the same InitFcn
(`bench_init_fcn`), but one `gcu_flat` instance per path (model argument
`drone_idx` = slice of the shared `traj`), a 106 B flat frame per drone
(424 B USB frame) and `gcs_sender_flat` forwarding per id. The flat path
feeds forward position, velocity and acceleration only, for swarm tables and
waypoint flights alike (`j_ref = s_ref = 0`, zeroed in the `traj_gen_flat`
wrapper of `gcu_flat`; the waypoint trajectory itself stays minimum-snap
planned): the drone-side controller acts as the asymptotic model-matching law
of the thesis w.r.t. the reference model (p_r, v_r, a_r). The procedure below applies verbatim, with
`flight_evaluation_flat(id)` in step 4 (logs `mocap_pos_d`, `x_ref_d`,
`v_ref_d`, `a_ref_d`, `mocap_quat_d` per path, saved as `*_id<id>.mat`).
Drone **and** sender Teensy must run the flat firmware. The flat OTA frame
(32 B: id/estop/ack, seq, mocap pose, p/v/a reference, yaw) carries no jerk,
snap or yaw rates; four drones need about 53 % airtime per 10 ms at 250 kbps.
Without jerk and snap feedforward the tracking error grows with the jerk of
the reference. Smooth swarm tables are fine, but the waypoint box of the
cascade (segments of 1.9 s) is not flyable on the flat path. In simulation it
needs segments of about 4.75 s or longer in `init_trajectory.m`.

Which mode flies is decided purely by the workspace at Run:

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
     `swarm_animation([1 2 3 4])` for the video (drone bodies over the
     containment volume and the Bezier surface, written to `data/videos/`).
     For a GIF, cut the wanted range out of the video:

     ```matlab
     mp4_to_gif('data\videos\swarm_1_2_3_4.mp4', 0, 35);
     ```

  Steps 2 and 4 need the `hyperbolic-2d-containment-control-with-bezier-`
  `surfaces` repository checked out next to `DROMA/`: step 2 runs the MAS
  (`main_DROMA.m`), step 4 renders with its `createDroneAnimation`. Both
  scripts add the path themselves.

  Agent assignment follows the number of drones: two drones get the grid
  agents (1,1) and (20,9), three get (1,1), (20,1) and (1,9), four get all
  four corners. Override with
  `swarm_precompute(kappa, p0, struct('agents', [i1 j1; i2 j2]))`.

  **Event run (robustness scenario)**: the MAS can inject two scripted events
  into the simulation, a disturbance step (the disturbance model state jumps
  from 0 to `dist_amp` at `t_dist_on`) and a containment change (all leaders
  on one box side rise by `jump_dz` at `t_leader_jump`). The observers and
  the agents re-converge after each event. Defaults live in `main_DROMA.m`
  (`t_dist_on = 10`, `dist_amp = 4.0`, `t_leader_jump = 18`,
  `jump_leaders = [1 2 5 6]`, `jump_dz = 1.0`, `Inf` = off). Flight tables
  are event-free by default: `swarm_precompute` pins both times to `Inf`.
  To fly the scenario, enable the events via `cfg_extra` and narrow the box
  in y, because the disturbance pushes mainly along y and the default sail
  already sits at the cage limit there:

  ```matlab
  clear; params
  ref = swarm_precompute(1.45, read_swarm_origins(mocap.streaming_ids), ...
      struct('t_dist_on', 10, 't_leader_jump', 18, ...
             'extent', [1.2 2.3 2.2], 'agents', [1 1; 20 1; 1 9; 20 9]));
  ```

  The release rule is unchanged: every agent `OK`, every pair 0 downwash.
  The events land at `kappa * t` of flight time and step the feedforward
  acceleration (up to about 4.6 m/s^2), the hardest transient flown so far.
  `swarm_animation` reads the event times from `swarm_ref.mat` and overlays
  `disturbance` and `change of leader positions` for 5 s each.

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
mount and enter `MOUNT[id]` in `hardware/drone_hal.cpp` and
`hardware/drone_hal_flat.cpp` (procedure in the comment above the table,
identity = not yet measured), create a Motive rigid body with that streaming
id, weigh the airframe and enter the mass in `quadcop.m_id(id)` and the thrust
factor in `quadcop.m_adap_id(id)` (`scripts/init/init_quadcop.m`), and update
`mocap.streaming_ids` in `init_sensors.m`. The thrust factor is the delivered
thrust relative to the thrust map of the firmware. The cascade feedforward of
that GCS path uses `m_id/m_adap_id`, so a wrong entry leaves a constant height
offset for the integrator to remove. The flatness firmware needs the same
factor in `K_THR[id]` in `drone_hal_flat.cpp`: it scales the commanded thrust
and is the start value of the thrust-scale estimator.

**Change a parameter**: `scripts/params.m` and `scripts/init/init_*.m` (or
`scripts/flatness/init_flatness.m`). Position gains and everything ground-side
take effect on the next run, no flash. Anything inside `mcu(_flat).slx` is
firmware and needs the full cycle below. `supervisor.z_ground` lives on both
sides: the ground station uses it for the soft landing, and it is compiled
into `mcu_flat` (battery landing, height gate of integrator and thrust-scale
estimator).

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
`flight_evaluation(id)` with the drone id (cascade) or
`flight_evaluation_flat(id)` (flatness); results land as `*_id<id>.mat`. Both shift the
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

Condensed to the rules that have bitten us.

- The drone **boots latched** (`estop = 2`, no link yet). Motors stay at zero
  until a rising `ack` edge arms it. Every link loss latches again.
- The **battery latch is permanent**: filtered voltage <= 12.0 V forces a
  descent until power-cycle, even if the voltage recovers. Bench supplies stay
  above 12 V.
- **Battery health is a flight parameter.** A worn pack (high internal
  resistance) collapses under hover load and quietly ruins tracking long before
  the latch trips. Measure packs before flying (`--upload-batt` flashes the
  load test).
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
