# Flatness-Variante — Wiring-Spec

Ziel: den flachheitsbasierten Folgeregler (exakte Zustandslinearisierung) als
**schaltbare Variante** neben die bestehende PD-Kaskade stellen — sim-first gegen
`plant.slx`/`quadcop.slx`, dann bench. Die Kaskade bleibt jederzeit lauffähig.

Alle Bausteine sind in `scripts/flatness/` **standalone validiert**
(`test_flatness_blocks.m` → 3 mm Trackingfehler, k̂=0.85 bei 15 % Schub-Defizit
und 30 ms Motor-Lag). Konvention **z-up, Schub entlang +Body-z**, identisch zu
`pos_ctrl`/`geo_attitude_ctrl`.

---

## 1. Die drei Bausteine

| Datei | Rolle | Rate | läuft auf |
|---|---|---|---|
| `flatness_ctrl.m` | vereinheitlichter Regler → `[F; tau]` | schnell (1 kHz) | MCU |
| `flatness_khat.m` | Schub-Skalenschätzer → `k_hat` | langsam (100 Hz) | GCS/Mocap |
| `init_flatness.m` | Auslegung (`coefPos`, `coefPhi`, Gains, Takte) | Init | — |

**Kernidee:** `flatness_ctrl` ersetzt in der Variante `flatness` die *gesamte*
Kaskade `pos_ctrl` (GCS) + `geo_attitude_ctrl` (MCU) — es liefert `[F; tau]` in
einem Zug. **Die Ausgabeschnittstelle `[F(N), tau(Nm)]` ist identisch zu
`geo_attitude_ctrl`** ⇒ der bestehende **Mixer** (Gamma_inv + Throttle-Map) im
`mcu` wird UNVERÄNDERT weiterverwendet.

---

## 2. Port-Belegung `flatness_ctrl`

`[F, tau] = flatness_ctrl(p, v, q, omega, p_ref, v_ref, a_ref, j_ref, s_ref, yawref, k_hat, m, g, J, coefPos, coefPhi, Ts)`

| Port | Dim | Quelle sim-first (Bus_State-Wahrheit) | Quelle HW/bench |
|---|---|---|---|
| `p` | 3 | `Bus_State.x` | `Bus_Mocap.mocap_pos` (gestreamt) |
| `v` | 3 | `Bus_State.v` | Luenberger `v_hat` |
| `q` | 4 | `Bus_State.q` | Mahony `q` |
| `omega` | 3 | `Bus_State.Omega` | `Bus_IMU.imu_gyro` (gefiltert) |
| `p_ref..s_ref` | 3 je | `traj_gen` (erweitert, s. §4) | dito, GCS→gestreamt |
| `yawref` | 3 | `traj_gen` → `[yaw; 0; 0]` | dito |
| `k_hat` | 1 | `flatness_khat` (über Unit Delay, §5) | dito, GCS→gestreamt |
| `m,g` | 1 | `quadcop.m`, `quadcop.g` (Parameter) | dito |
| `J` | 3×3 | `quadcop.J` (Parameter/Konstante) | dito |
| `coefPos` | 3×5 | `fctrl.coefPos` (Parameter) | dito |
| `coefPhi` | 1×3 | `fctrl.coefPhi` (Parameter) | dito |
| `Ts` | 1 | `fctrl.Ts_fast` (Parameter) | dito |
| **`F`** | 1 | → **Mixer** (wie `geo_attitude_ctrl.F`) | dito |
| **`tau`** | 3 | → **Mixer** (wie `geo_attitude_ctrl.tau`) | dito |

> `m,g,J,coefPos,coefPhi,Ts` als **Parameter-Scope-Daten** (Ports and Data
> Manager → *Parameter*) aus dem Base-Workspace ziehen (kein Verdrahten). `J`
> kann alternativ wie bei `geo_attitude_ctrl` als Konstanten-Block-Port kommen.

---

## 3. Port-Belegung `flatness_khat`

`k_hat = flatness_khat(v, q, F, m, g, gamma_khat, tau_lp, Ts)`

| Port | Dim | Quelle | Anmerkung |
|---|---|---|---|
| `v` | 3 | `Bus_State.v` / Luenberger `v_hat` | **Mocap-basiert, NICHT IMU** (Bias!) |
| `q` | 4 | `Bus_State.q` / Mahony | für Schubachse `zB=R(:,3)` |
| `F` | 1 | `flatness_ctrl.F` über **Unit Delay** | bricht die algebr. Schleife (§5) |
| `gamma_khat,tau_lp,Ts` | 1 je | `fctrl.*` (Parameter) | `Ts = fctrl.Ts_slow = 0.01` |
| **`k_hat`** | 1 | → `flatness_ctrl.k_hat` | geklemmt [0.5, 1.5] |

> **Warum Mocap statt IMU:** validiert — IMU-Bias verfälscht k̂ (11 cm Restfehler),
> Mocap-`v` ist bias-frei (2–3 mm). Der Schätzer ist langsam (~1–5 Hz), 100 Hz
> genügt; die 1-kHz-IMU-Rate wird nicht gebraucht.

---

## 4. Erweiterung `traj_gen` (additiv, kaskade-neutral)

`traj_gen` berechnet Ruck/Snap bereits intern (`j_ref = D*s3/T^3`, `s_ref = D*s4/T^4`),
gibt sie aber nicht aus. Zwei Ausgänge ergänzen — die Kaskade verdrahtet sie
einfach nicht, bleibt also unberührt:

```matlab
function [x_ref, v_ref, a_ref, yaw_ref, Omega_ref, tau_ref, q_ref, F_ref, ...
          j_ref, s_ref] = traj_gen(t, traj, quadcop)   % + j_ref, s_ref
```
`yawref` für den Block = `[yaw_ref; 0; 0]` (Segment-Yaw ⇒ dyaw=ddyaw=0). Bei
späteren Yaw-Trajektorien hier `dyaw_ref, ddyaw_ref` ergänzen.

---

## 5. Rate-Split & Schleifenbruch

- `flatness_ctrl` @ **1 kHz** (`Ts_fast`), `flatness_khat` @ **100 Hz**
  (`Ts_slow`). In Simulink über Rate-Transition-Blöcke koppeln (langsam→schnell
  für `k_hat`, schnell→langsam für `F`).
- `k_hat → flatness_ctrl` und `F → flatness_khat` bilden eine Schleife.
  **Unit Delay (oder Memory) auf `F`** in `flatness_khat` einfügen — da der
  Schätzer langsam ist, ist ein Takt Verzug vernachlässigbar.

---

## 6. Sim-first-Harness (empfohlener erster Schritt)

Neues Modell `quadcop_flat.slx` (nicht-destruktiv, referenziert `plant.slx`):

```
                    ┌─────────── traj_gen (erweitert) ───────────┐
 Clock ─ t ────────►│  → p_ref,v_ref,a_ref,j_ref,s_ref, yawref   │
                    └───────────────────────────────────────────┘
                                        │  (refs)
 plant ─ Bus_State ─┬─ x,v,q,Omega ─────┼──────────────► flatness_ctrl ─ F,tau ─► Mixer ─ rotor_cmd ─► plant
        (R,accel)   │                   │                    ▲
                    └─ v,q ─► flatness_khat ─ k_hat ──[UnitDelay]┘
                               ▲ F (UnitDelay)
```

- **Mixer**: das bestehende `[F;tau]→rotor_cmd`-Subsystem aus `mcu` in den
  Harness kopieren (gleiche Eingangsschnittstelle wie `geo_attitude_ctrl`).
- **Erststufe**: `flatness_ctrl`/`khat` direkt aus `Bus_State` (Wahrheit) speisen
  → isoliert Regler+Strecke+Mixer+Einheiten gegen das echte `plant.slx`.
- **Zweitstufe**: `sensors.slx` (→ `Bus_Mocap`, `Bus_IMU`) + Luenberger (`v`) +
  Mahony (`q`) dazwischenschalten → volle Sensorrealität.
- **InitFcn** des Harness:
  ```matlab
  quadcop = init_quadcop();
  fctrl   = init_flatness(quadcop);
  ```

---

## 7. Variant-Aufbau für bench (nach dem Sim-first)

In `quadcop.slx`/`bench.slx` die Regelpfade in **Variant Subsystems** mit
Schaltvariable `ARCH ('cascade' | 'flatness')` kapseln:

- `ARCH='cascade'`: `gcu` (pos_ctrl) + `mcu` (geo_attitude_ctrl) — **unverändert**.
- `ARCH='flatness'`:
  - `gcu`: `traj_gen` (erweitert) + Luenberger + `flatness_khat` → streamt
    **{mocap-Pose, Trajektorien-Refs, k_hat}** über `Bus_Cmd`.
  - `mcu`: Mahony + **`flatness_ctrl`** (statt `geo_attitude_ctrl`), Mixer
    downstream unverändert.
- **`Bus_Cmd`**: für die Flatness-Felder **additiv erweitern** (die Kaskade
  ignoriert die neuen Felder) statt die bestehenden zu ändern → Kaskade bleibt
  bitgleich. `q_des/F_des` werden in der Flatness-Variante nicht benutzt.

Geteilt/unberührt: HAL, Link-Codec, `MOUNT`, Safety, Mocap, Mixer, Busse.

---

## 8. Modellsensitivität (im Auge behalten)

Validiert: Trägheitsfehler unkritisch (+30 % → 1 mm), Motor-Lag benign
(30 ms → 0.5 mm), **Schub-Map kritisch** — daher der `flatness_khat`. Vor
Aggressivmanövern: den Referenz-Term `(tw^2 + dw)` in `flatness_ctrl`
(Kommentar dort) und die `eigenvalPos`-Auslegung prüfen.
