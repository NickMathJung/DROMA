# SITL — C++-Golden-Test (Quaternion-Helfer + Safety-Leafs)

Host-seitige Codegen-Treue-Stufe: die eingefrorenen Golden-Vektoren aus
`verify_quat_codegen.py` sowie die `verify_overspeed`/`verify_battery`-Invarianten
nach GoogleTest/CTest gezogen. Läuft **ohne MATLAB** grün gegen mitgelieferte
Referenz-Ports und zertifiziert nach einem reinen Link-Tausch den generierten Code.

```
cmake -S . -B build && cmake --build build -j && ctest --test-dir build --output-on-failure
```
Referenz-Lauf hier: **21/21 grün**, `dcm2quat` worst |dq|=1.8e-16 (Zweige 14/5/5/7),
`quat2dcm` 3.9e-16, `quatMul`/`quatConj` exakt, Round-Trips ~1e-15.

---

## Decision-Table (gelockt)

| # | Entscheidung | Wert | Grund |
|---|---|---|---|
| D1 | ABI der Leaf-Helfer | `extern "C"`, fixed-size `double`, out-Param | bit-identisch zu MATLAB Coder → Link-Tausch statt Testumbau |
| D2 | Matrix-Speicherordnung | **column-major** in der ABI; Golden ist **row-major** | Coder-Default ist column-major; Umrechnung isoliert in `test/csv.hpp` |
| D3 | Leaf-Codegen-Sprache | **C** (`gen_lib_codegen.m`) | trivialer Link; Golden-Diff zertifiziert reine Numerik. Modell `mcu.slx` bleibt **C++** |
| D4 | Golden-Toleranz | `GOLDEN_TOL=1e-9` (Default), Quat bis auf Vorzeichen | **bewusst eng** — lose Toleranz kaschiert ULP-Divergenz |
| D5 | Safety-Persistenz | `reset()`+`step()` | 1:1 auf Coders `*_initialize()`+`step()` abbildbar |
| D6 | Safety-Params | `coder.Constant` (default) | kleinere ABI; laufzeitvariabel → als echtes Arg (siehe `gen_lib_codegen.m`) |
| D7 | `mcu.slx`-Interface | ERT, C++-Klasse `MCU`, discrete `Ts_inner=1/1000`, packNGo | host-SITL-Loop instanziiert die Klasse gegen die simulierte Strecke |

Frame **z-up** und `params.m`-IST sind durchgängig respektiert; keine NED-Vorzeichen im Testpfad.

---

## Geflaggte Inkonsistenzen (Toolchain, bitte prüfen)

1. **Handover §7-Manifest stale.** Dort heißen alle vier `golden_*.csv`; der
   Generator schreibt aber `golden_quat.csv` **und** `test_data_quat*.csv`. Namen
   im Manifest an den Generator angleichen.
2. **`quat2dcm_local` ist Rekonstruktion (Handover §5, OFFEN).** Der Referenz-Port
   nutzt dieselbe Aerospace-Formel → `Quat2Dcm_MatchesGolden` ist scharf, aber
   erst gegen das **echte** `quat2dcm_local` aus `mahony_filter`/`geo_attitude_ctrl`
   aussagekräftig. Bis dahin zertifiziert der Test nur, dass Codegen == diese Formel.

---

## Auf Codegen umstellen

1. In MATLAB (`functions`-Ordner auf dem Pfad):
   ```matlab
   run matlab/gen_lib_codegen.m        % -> codegen/lib/<fn>/*.c(+.h)
   ```
2. Neu konfigurieren + bauen + testen (kein CODEGEN_ROOT-Flag noetig — Default
   zeigt auf `sitl/codegen`, wohin gen_lib_codegen.m schreibt):
   ```
   cmake -S . -B build -DQUAT_IMPL=codegen -DSAFETY_IMPL=codegen -DMATLAB_ROOT="C:/Program Files/MATLAB/R2025b"
   cmake --build build --config Release
   ctest --test-dir build -C Release --output-on-failure
   ```
   Erwartung: **20/20** (S9 bei einkompilierten Params ausgeblendet, s.u.).
   Nur die Helfer ohne Safety: zusaetzlich `-R QuatGolden` an ctest.

Drei Dinge, die sonst brechen:
- `project(... LANGUAGES C CXX)` ist gesetzt → Coders `.c` werden als **C**
  kompiliert (unmangled) → kein LNK2019.
- **`MATLAB_ROOT`** ist noetig: Coders `rtwtypes.h` zieht `tmwtypes.h` aus
  `<MATLAB_ROOT>/extern/include`. Pfad in MATLAB per `matlabroot`. Fehlt er,
  bricht CMake mit klarer Meldung ab.
- CODEGEN_ROOT wird backslash-tolerant normalisiert; am besten Flag weglassen.

### Codegen-Shims (Safety) — bereits enthalten
`src/codegen_shim_overspeed.cpp` und `src/codegen_shim_battery.cpp` adaptieren die
generierte Coder-ABI auf die Test-ABI (`overspeed_reset/step`, `battery_reset/step`).
Sie passen zu Headern mit **coder.Constant**-Params (kein Struct-Argument):
`safety_overspeed_init()` + `safety_overspeed(gyro,estop,ack,&kill,&src,dbg)`.
Params einkompiliert → **S9 (norm-Modus) nicht schaltbar**; bei
`SAFETY_IMPL=codegen` via `#ifndef SAFETY_CODEGEN_CONST_PARAMS` uebersprungen
(20 statt 21). Fuer volle S9-Abdeckung Safety-Leafs mit **Laufzeit-Params**
generieren (coder.Constant entfernen) → Shim bekommt `struct0_T` aus
`safety_overspeed_types.h`; dann liefere ich die Laufzeit-Shims.

---

## Dateien

```
include/quat_helpers.h        ABI (extern "C", column-major R)
include/safety_helpers.h      ABI reset()+step()
src/quat_helpers_ref.cpp      bit-treuer Port (Standin/CI-Referenz)
src/safety_helpers_ref.cpp    Port safety_overspeed + safety_battery
test/csv.hpp                  CSV-Reader + row->col-major-Adapter
test/test_quat_golden.cpp     5 Helfer vs Golden + Property-Round-Trips
test/test_safety.cpp          Overspeed S1–S9, Battery B1–B6
matlab/gen_lib_codegen.m      Leaf-Codegen (C-Lib) für den Golden-Diff
matlab/configure_mcu_codegen.m ERT-C++-Config für mcu.slx (+ Entry-Kontrakt)
data/test_data_quat*.csv      Golden (aus verify_quat_codegen.py)
```

## Offen (nächste Schritte)
- **[du]** `quat2dcm_local` standalone aus dem mcu-Code extrahieren → Inkonsistenz #4 schließen.
- **[du]** `mcu.slx`: `configure_mcu_codegen.m` laufen, `slbuild`, packNGo ins Host-Repo;
  dann echten Host-SITL-Loop (MCU-Klasse gegen simulierte Strecke) — separat vom Leaf-Diff.
- **[opt]** `gcs_supervisor` ist GCS-seitig, nicht in `mcu.slx`; falls onboard gespiegelt,
  analoges Sequenz-Fixture wie test_safety ergänzen.
```
