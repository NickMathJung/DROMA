# Testmatrix Erstflug (lebendes Dokument)

Ziel: **kein Crash beim Erstflug.** Jede Zeile hat ein messbares Pass-Kriterium.
Status: ⬜ offen · 🟡 läuft · ✅ pass · ❌ fail · ➖ n/a

> ## 🛑 PRE-FLIGHT-BLOCKER
>
> **B1 — Akku defekt (der bei den Frequenzmessungen verwendete Pack).** Beim
> Low-Batt-Sweep brach die Spannung unter Einzelmotor-Last (nur ~2–3 A) von
> **14.41 V auf 12.47 V** ein — 0.48 V/Zelle. Ein gesunder 4S-Pack sackt bei der
> Last 0.1–0.2 V. Impliziter Innenwiderstand ~800 mΩ, also Faktor 30–50 über
> gesund (Richtwert 10–25 mΩ für den ganzen Pack). Zusätzlich startete er schon
> bei 3.60 V/Zelle (~35 %).
> **Folge im Flug:** Hover zieht ~13 A, Manöver mehr → der Pack kollabiert, fällt
> unter `V_floor = 12.0 V` → `batt_land` latcht mitten im Flug. Die Throttle-
> Kompensation `22.2/V_filt` fährt gegen den Abfall hoch → mehr Strom → Mitkopplung.
> **→ Diesen Pack nicht fliegen.** Erstflug nur mit einem Pack, dessen Einbruch
> unter Last gemessen wurde (LiPo-Checker mit IR-Funktion, Ziel < 25 mΩ Pack;
> Ruhespannung voll ~16.8 V, unter Hover-Last > 15 V).
>
> Weitere Blocker sammeln sich hier, sobald sie auftreten.

**Drohnen:** `id=1` (leicht modifiziert), `id=2` (neu gebaut → Lötfehler möglich).
Beide: `quadcop.m = 0.985 kg`.

> ⚠️ **Firmware-Semantik NEU (Session 10):** `btn_ack` ist jetzt ein **lokaler Kill**
> (steigende Flanke → throttle 0, gehalten → Re-Arm gesperrt). **Re-Armen nur über
> den `ack`-Toggle** (`Bus_Cmd.ack`-Flanke). Zusätzlich **Tilt-Cutoff** (>80°/80 ms)
> und **`estop`-Rotary** (0/1/2). Die alten HW-Prozeduren aus §3h (btn=Re-Arm)
> sind damit teils ungültig — siehe Anmerkungen.
>
> ✅ **Codegen-Stand:** Host/Leaf/ARM-Codegen + Golden **auf m=0.985 neu erzeugt**,
> Gate B grün (ref 33+7, codegen 32+7; Golden==Modell `worst 2.27e-13`).
> `hardware/mcu_arm` ist damit **flash-bereit für m=0.985**. GCS-Gains ziehen über
> `params.m` automatisch nach.
> (Hinweis: `test_mcu_model.exe` aus dem `build/`-Ordner wurde von **Smart App
> Control** geblockt — Fehlalarm auf dem eigenen Build; das identische Modell lief
> über `build_cg/` 7/7 grün.)

---

## Stufe 0 — SITL (Bench)

| ID | Test | Kriterium | Status |
|----|------|-----------|--------|
| S0 | Gate B `ref`+`codegen` (m=0.985) | 40/40 bzw. 39/39 grün, Golden==Modell 2.27e-13 | ✅ |

---

## Stufe 1 — Props **ab**, `HAL_MODE_BENCH` (Sensorik/Link/Batt/Timing)

Motoren bleiben zwangsweise auf ESC_MIN; Beobachtung über den Serial-Report.
m-unabhängig. **Für `id=2` komplett neu (Lötkontrolle).**

| ID | Test | Pass-Kriterium | Deckt NICHT | id=1 | id=2 |
|----|------|----------------|-------------|------|------|
| T1 | Gyro-Bias/Throttle-Symmetrie | `thr` bei festem `F_des` symmetrisch (Spreizung ≈1). **Armen via `ack`-Toggle, `btn` LOS** | Mixer-Vorzeichen unter echtem τ | ✅(alt) | ✅ Re-Arm + alle `F_des`-Steps treffen Modell (s.u.) |
| T2 | Failsafe (Link-Loss) | GCS/Sender aus → nach 100 ms `estop=2`, `thr[0 0 0 0]` | Re-Arm-Pfad | ✅(alt) | ✅ **bestanden** (kein Auto-Re-Arm bei Link-Rückkehr) |
| T3 | Batt-`k` an Pin 41 | **unabhängige** Messung (Quelle/Multimeter) vs. Report-`V` | Offset `b` (1-Punkt-Messung!), Schwellen | ✅ | ✅ Quelle 15.07 V = Report 15.07 V (`count=904`) |
| T4 | Batt-Report-Plausibilität | `batt≈900–1000`, `V≈15 V` für 4S, nicht 0/4095 | Floor-Latch | ✅(alt) | ✅ `904(15.07V)`/`901(15.02V)` |
| T5 | **(NEU) Taster = Kill** (ersetzt alten Re-Arm-Test) | `btn`-Flanke → `thr[0 0 0 0]`; gehalten → bleibt 0; `ack`-Toggle OFF→ON → `thr` wieder da | — | ➖ | ✅ **bestanden** (inkl. Re-Arm-Sperre bei gehaltenem Taster) |
| T6 | Gyro/Bias-Ruhe + Timing | still: `gyro≈0`, `bias` klein & stabil; `tickmax<1000 µs`, `overruns=0/1000` | Last unter Motorlauf | ✅(alt) | ✅ `gyro≤0.007`, `tickmax=460 µs`, `overruns=0/1000` |

**Befund id=2 — nRF (BEHOBEN):** Erster Flash meldete `[boot] 4 nRF ok=0 chip=0`; Versorgung am
Modul ok (VCC=3.3 V), **CSN (Teensy Pin 0) lag auf 0 V statt 3.3 V** → Ursache war eine
**Lötbrücke an CSN**. Nach Entfernen: **`nRF ok=1 chip=1`**, `link=1…15 ms`.
⚠️ **Bei id=1 dieselbe Stelle prüfen** (gleiche Bauserie).

**Link steht (id=2):** `link=1…15 ms`, `estop=0` kommt durch. `thr[0 0 0 0]` ist dabei **korrekt**:
Die HAL bootet mit `g_cmd.estop=2` → Kill-Latch gesetzt; er löst sich nur über eine steigende
**`ack`-Flanke**. Erwartung nach `ack` OFF→ON bei `F_des=0`: **`thr[8 8 8 8]`** (Polynom-Konstante 8.404 %).

**Offen bei T3:** nur **ein** Stützpunkt (15 V) → ein möglicher **Offset `b`** ist damit nicht
ausgeschlossen. Zweiter Punkt nahe den Schwellen (13.5 / 12.5 V am Netzteil) empfohlen — erledigt
zugleich **K7** (WARN → CRIT → Floor-Latch, sticky bis Reboot).

**Nebenbefund:** `|acc|` = 9.55–9.58 statt 9.81 m/s² (−2.4 %, innerhalb ±3 % Spec); x/y ≈ 2–2.5°
(Tischneigung oder Zero-g-Bias, Spec ±50 mg). Ohne Auswirkung auf die Lageschätzung (Mahony normiert).

---

## Stufe 2 — Props **ab**, `HAL_MODE_THRUST`, Drohne **fixiert** (ESC/Motor/Mixer)

Motoren laufen (ohne Props). Beobachtung: Serial-`thr[]` = Modell-Sollwert;
Motor-Hochlauf visuell/akustisch. **Voraussetzung: m=0.985-Firmware geflasht, Kill jederzeit erreichbar.**

| ID | Test | Pass-Kriterium | id=1 | id=2 |
|----|------|----------------|------|------|
| M1 | ESC-Arming | alle 4 armen (Piep), laufen bei `thr>~8 %` an, sonst still | ⬜ | ⬜ |
| M2 | Motor-Drehrichtung | M1 CCW, M2 CW, M3 CCW, M4 CW (physisch geprüft) | ⬜ | ✅ **bestanden** |
| M3 | Throttle↔Serial | gemessener Motor-Hochlauf konsistent mit `thr[]` im Report | ⬜ | ⬜ |
| M4 | **Mixer-Muster** (dein Test) | pro `[F,τ]` das erwartete Motor-Muster (Tabelle unten) | ⬜ | ⬜ |

**Ground-Truth Throttle-Sollwerte (Modell, m=0.985, `F_hover=9.663 N`):**

| Fall | M1 (FR/CCW) | M2 (FL/CW) | M3 (RL/CCW) | M4 (RR/CW) | erwartetes Muster |
|------|:---:|:---:|:---:|:---:|---|
| F=0.1·mg, τ=0 | 9.98 | 9.98 | 9.98 | 9.98 | alle gleich |
| F=0.2·mg, τ=0 | 11.55 | 11.55 | 11.55 | 11.55 | alle gleich |
| F=0.3·mg, τ=0 (Basis) | 13.10 | 13.10 | 13.10 | 13.10 | alle gleich |
| F=0.3·mg, **+τ_x=0.10** | 11.44 | **14.75** | **14.75** | 11.44 | **links (M2,M3) schneller** |
| F=0.3·mg, **+τ_y=0.10** | 11.01 | 11.01 | **15.18** | **15.18** | **hinten (M3,M4) schneller** |
| F=0.3·mg, **+τ_z=0.02** | 10.68 | **15.50** | 10.68 | **15.50** | **CW-Paar (M2,M4) schneller** |
| F=0.5·mg, τ=0 | 16.19 | 16.19 | 16.19 | 16.19 | alle gleich |

Werte für andere `[F,τ]` rechne ich auf Zuruf exakt nach.

---

## Stufe 3 — Props **ab**, fixiert (alle Kill-Pfade auf HW)

Am sichersten in **`HAL_MODE_BENCH`** (Motoren bleiben min): der Report zeigt
`thr[]` = Modellausgang, d.h. der Kill ist an `thr→0` sichtbar, **ohne** dass sich
je etwas dreht. Vorher etwas `F_des` kommandieren, damit `thr>0` als Ausgangswert.

| ID | Kill-Quelle | Auslösen | Pass-Kriterium | id=1 | id=2 |
|----|-------------|----------|----------------|------|------|
| K1 | Overspeed | Drohne von Hand > 8.5 rad/s drehen (≈487 °/s) | `thr→0`, latcht; Report-`gyro` zeigt >8.5 | ⬜ | ✅ **bestanden** (‖gyro‖=8.90 → `[0 0 0 0]`; 3.14 löst korrekt nicht aus) |
| K2 | **Tilt (neu)** | > 80° kippen, ~0.1 s halten | `thr→0` (`fault_src=3`), latcht | ⬜ | ✅ **bestanden** (81.7° → `[0 0 0 0]`, s.u.) |
| K3 | **Taster (neu)** | `btn` drücken | `thr→0` (`fault_src=4`); gehalten → bleibt 0 | ⬜ | ✅ **über T5 belegt** |
| K4 | Hard-Kill | `estop`-Rotary → 2 | `thr→0` sofort (`fault_src=2`) | ⬜ | ✅ **bestanden** |
| K5 | Link-Loss | Sender/GCS aus | nach 100 ms `estop=2`, `thr→0` (= T2) | ⬜ | ✅ **über T2 belegt** |
| K6 | Re-Arm | nach Kill: Fehler weg + `ack`-Toggle OFF→ON | `thr` kommt zurück; **während Kippung/Overspeed bleibt gesperrt** | ⬜ | ✅ **bestanden** |
| K7 | Battery-Floor | Batt-Sense unter 12 V ziehen (Labornetzteil) | `led=CRIT`, dann Floor-Latch (`batt_land`) | ⬜ | ✅ **unbeabsichtigt belegt** (s.u.) |

### K2 — HW-Beleg + ⚠️ der Beinahe-Fehler dahinter

| | `acc` | Kippung | `thr` |
|---|---|---|---|
| kurz vor Auslösung | `[0.48 9.41 1.37]` | 81.7° | `[38 0 0 19]` (Roll-Summe −57) |
| ausgelöst | `[-0.22 9.70 2.11]` | 77.7° | **`[0 0 0 0]`** |

Dass die zweite Zeile schon wieder 77.7° zeigt, ist korrekt — der Latch hält.

> **⚠️ Der erste Versuch schlug fehl, und die Ursache ist flugkritisch.** `q_ext` war in
> bench.slx falsch verdrahtet und trug **Identität statt Nullen**. Mit `kE = 25` gegen `ka = 1`
> überstimmt eine gültige externe Lagereferenz den Accelerometer um Faktor 25:
>
> ```
> ka·sin(86.1° − θ) = kE·θ   →   θ = 2.29°   →   Roll-Summe 3.4
> ```
> Gemessen wurde 3–4 bei **86° realer Kippung**. Die Drohne stand fast auf der Seite und
> `q_hat` meldete 2°.
>
> **Konsequenz:** Der Tilt-Cutoff hängt an `q_hat`, und `q_hat` hängt fast vollständig am Mocap.
> Eine **falsche, aber gültige** externe Lage (falscher Rigid Body, eingefrorener Stream,
> gedrehtes Frame) setzt den Tilt-Schutz **still ausser Kraft** — die Drohne kann auf dem Kopf
> stehen, ohne dass er feuert. Der Accelerometer würde es sehen, wird aber 25:1 überstimmt.
> Damit ist das Mocap Single Point of Failure für Lagereferenz **und** Kippschutz zugleich.
>
> **Pre-Flight-Pflicht:** `q_ext` explizit prüfen — bei `mocap_valid = 0` muss dort `[0;0;0;0]`
> stehen, nicht `[1;0;0;0]`. Schnelltest: auf 45° kippen. Roll-Summe ≈ **−47** = `q_ext` aus,
> ≈ **−2.4** = `q_ext` steht auf waagerecht. Dazwischen gibt es nichts.
>
> **Designvorschlag (offen):** Kippwinkel aus `q_ext` gegen Kippwinkel aus `acc` vergleichen
> (bei ruhiger Drohne, `|acc| ≈ g`). Abweichung > ~15° → `kE = 0` und mit Accel weiterfahren.
> Fängt falschen Body, Freeze und gedrehtes Frame in einem Test.

### K7 — HW-Beleg (id=2, Netzteil versehentlich nicht angeschlossen)

`batt=0(0.00V)` → Tiefpass (τ=0.7 s) unter `V_floor=12.0` → `batt_land` gelatcht →
`safety_landcmd` überschreibt das GCS-Kommando mit `F=0.99·m·g`, `q_des=q_ref=I`, `Omega_ref=tau_ref=0`.

Sichtbar am Sprung des mittleren Throttles bei sonst identischem Setup:

| | `batt ≈ 14.9 V` | `batt = 0 V` |
|---|---|---|
| `t0` = mean(thr) | 15.25 / 14.75 | **23.25 / 23.5** |

Nachrechnung: `F = 0.99·0.985·9.81 = 9.57 N` → 2.39 N/Motor → `wi² = 1.271e6` →
`thr = 8.404 + 1.2316e-5·wi²` = **24.1**. Gemessen 23.3. ✅

> **⚠ Pre-Flight-Pflicht:** Der Latch ist sticky **bis Reset** und überlebt einen Akkuwechsel.
> Zustand danach: `batt` zeigt 15 V, alles wirkt normal — die Drohne kommandiert aber stur
> `0.99·m·g` waagerecht und **ignoriert jedes GCS-Kommando**. Beim Armen = unkommandierter Steigflug.
> Der LED-Zustand hilft **nicht**: `led=CRIT` ab 13.4 V, Latch erst ab 12.0 V — die LED kann
> „Warnung" und „gelatchte Blindlandung" nicht unterscheiden.
> → Nach jedem Anstecken: **Reset drücken**, dann `batt > 14 V` prüfen, erst dann armen.

---

## Stufe 1b — **Vorzeichentest auf dem Tisch** (props ab, Selftest) — *vorgezogen aus Stufe 4*

Erkenntnis aus T1: **Regel-Vorzeichen sind am Throttle-Muster ablesbar**, ohne dass sich etwas
dreht. Damit lässt sich C1 (Crash-Ursache Nr. 1) risikofrei vorziehen.
Setup: `F_des ≈ 0.3·mg` (~13 %), armiert, **max. 15–30° kippen** (>80°/80 ms feuert den Tilt-Cutoff).

**Roll/Pitch: `q_ext = [0 0 0 0]` senden** → Mahony fällt auf Accel-only zurück, `q_hat` folgt der
echten Kippung, `q_des` bleibt Identität ⇒ echter Lagefehler. Erwartung bei ~20° und `F_des=0.3·mg`
(Basis 13 %): **14 Punkte Spreizung**, nicht 1–2.

| Auslenkung (~20°) | Regler muss | erwartetes `thr[M1 M2 M3 M4]` | id=1 | id=2 |
|---|---|---|---|---|
| Nase runter | vorne hoch | **`[20 20 6 6]`** | ⬜ | ✅ **bestanden** (43°: `[25 29 0 0]`) |
| Nase hoch | hinten hoch | `[6 6 20 20]` | ⬜ | ✅ **bestanden** (1.8°: Pitch-Summe +2, erwartet +2.7) |
| links runter | links hoch | `[6 20 20 6]` | ⬜ | ✅ **bestanden** (40.8°: `[6 20 33 0]`) |
| rechts runter | rechts hoch | `[20 6 6 20]` | ⬜ | ✅ **bestanden** (16.5°: `[15 10 3 24]`) |
| Yaw drehen (**Mocap aktiv**, Marker frei!) | Muster kippt M2,M4 ↔ M1,M3 | `[8 9 8 9]` ↔ `[9 8 9 8]` | ⬜ | ✅ **bestanden** (−1.27°: `[7 19 7 19]`, Yaw-Summe +24 vs +23.1 erwartet) |

### Yaw — HW-Beleg (id=2, Mocap aktiv)

Belegt wurde die Kette `q_ext → Mahony → eR_z → Regler → Mixer → thr`.

`yaw_sum = 482·kR_z·sin(ψ) = 1043·sin(ψ)` → Sättigung ab ψ ≈ 2–3°. Der Winkel muss deshalb aus
`mocap_quat` **abgelesen** werden, nicht geschätzt; dann ist auch die Amplitude auswertbar.

| ψ (mocap) | −1.27° (CW von oben) |
|---|---|
| erwartet `yaw_sum = 1043·sin(ψ)` | **+23.1** |
| gemessen (`thr[7 19 7 19]`) | **+24** ✅ |
| Roll-Summe / Pitch-Summe | **0 / 0** — Achsen vollständig entkoppelt |

Vorzeichen: CW → `yaw_sum > 0` → M2, M4 hoch. Korrekt (kein Runaway). `kR_z` ebenfalls belegt.

**Nebenbefund — `kE`-Dominanz quantitativ bestätigt:** `acc = [−0.07, −0.30, 9.47]` entspricht 1.86°
(Zero-g-Bias aus T1). Ohne Mocap ergäbe das Roll-Summe ≈ 4.5. Gemessen 0, weil
`θ_rest = 1.86°/25 = 0.074°` → `66·1.290·sin(0.074°) = 0.11` → rundet auf 0. Damit sind beide
Test-Regime konsistent: **ohne** Mocap regiert Accel (Kipptests), **mit** Mocap regiert `q_ext`.

> **⚠ D2-Befund aus genau diesem Versuch:** id=2 hat auf der Lage von **id=1** geregelt, `mocap_valid`
> war dabei 1 — die Daten waren gültig, nur vom falschen Körper. Onboard ist das nicht erkennbar und
> im Schwarm der Fehler, bei dem eine Drohne einer anderen hinterherfliegt.
> **Pre-Flight:** `streaming_id` prüfen, indem man **die zu fliegende Drohne bewegt** und sieht,
> dass sich `q_ext` ändert — nie umgekehrt.

**Muster direkt ablesen — drei orthogonale Achsen-Summen** (erspart das Rückrechnen):

| Achse | Rechnung | Umrechnung |
|---|---|---|
| **Pitch** τ_y | `(M3+M4) − (M1+M2)` (hinten − vorne) | ÷ 83 = N·m |
| **Roll** τ_x | `(M2+M3) − (M1+M4)` (links − rechts) | ÷ 66 = N·m |
| **Yaw** τ_z | `(M2+M4) − (M1+M3)` (CW − CCW) | ÷ 482 = N·m |

**Pass-Kriterium Nase-runter:** Pitch-Summe **stark negativ, ≈ −28** (= `[20 20 6 6]`).
**Vorher `acc` prüfen:** 20° Nase-runter ⇒ `acc ≈ [−3.3, ~0, 9.0]`, `|acc| ≈ 9.6`.
Ist `|acc| > 10`, wurde beschleunigt statt gehalten → Messung verwerfen.

🔑 **Kippung ≥ 3 s ruhig halten.** Der Mahony korrigiert die Lage mit `ka = 1.0`, also ~**1 s
Zeitkonstante** (20° Fehler ⇒ nur ~20 °/s Korrekturrate), ~3 s für 95 %. Wer während der Bewegung
abliest, misst **nur den Dämpfungsterm** `−k_Ω·Ω`, während `q_hat` noch hinterherhinkt — das war die
Ursache beider Fehlversuche. Im ruhigen Halt geht `gyro → 0`, der Dämpfungsterm verschwindet, und
im `thr` bleibt **ausschließlich der Lageanteil** übrig.
⚠️ Mit `q_ext = 0` ist **Yaw nicht beobachtbar** → `q_hat`-Yaw driftet beim Hantieren und erzeugt
ein Hintergrund-τ_z. Deshalb Pitch/Roll **über die Achsen-Summen isolieren**, nicht am Rohmuster raten.

**Fehlversuch 1 id=2 (dokumentiert, damit er sich nicht wiederholt):** Handgehaltene Kippung 24.5°
(`acc[-3.58 -0.96 8.55]`) ergab `thr[16 7 21 9]`. Rückrechnung durch den Mixer: `F=2.96 N`,
`τ=[+0.045, +0.082, −0.043] N·m` — das ist **exakt `−k_Ω·Ω`** (Vorhersage `[+0.029, +0.076, −0.031]`),
also **reine Drehratendämpfung, Lageanteil ≈ 0**. Bei echtem 24.5°-Fehler wäre `τ_y ≈ −0.42 N·m`
⇒ `thr ≈ [20 20 6 6]`. **Ursache: `q_ext` eingefroren** (Marker beim Anfassen verdeckt) — bei
`ka=1` vs. `kE=25` dominiert das Mocap den Accel **25:1** und hält `q_hat` auf level.
**Tatsächliche Ursache (Nutzer):** Motive streamte den *stillstehenden* **id=1**, während id=2 neben
dem Rechner gekippt wurde → konstantes `[1 0 0 0]` als `q_ext`. Siehe daraus abgeleitet **D2**.
✅ Immerhin belegt: **Drehratendämpfung hat auf allen 3 Achsen das richtige Vorzeichen.**

**Fehlversuch 2 id=2 (mit `q_ext=0`):** `acc_x` blieb bei −0.13…−0.22 ⇒ nur **~1.3° Kippung**, und
`|acc| = 10.84` in der stärksten Zeile ⇒ vertikaler Ruck statt statischer Neigung. Rückrechnung
`thr[20 6 21 5]` → `τ = [+0.030, **0.000**, −0.061]`: **τ_y exakt null**, das große τ_z ist
Gierdämpfung + driftender Yaw-Fehler. **Lehre: erst `acc` prüfen, dann `thr` lesen.**

**Invertiertes Muster = falsches Vorzeichen dieser Achse = garantierter Flip beim Abheben.**
Der Yaw-Teil belegt zugleich, dass `q_ext` (Motive) bis in den Regler durchkommt (Teil von C2).

> **Beobachtung id=2:** im Ruhezustand `thr[8 9 8 9]` = kleines **+τ_z** (Yaw). Ursache: `eR_z ≠ 0`,
> weil `q_hat` seinen Yaw allein aus `q_ext` (Motive) bezieht — Accel liefert keine Yaw-Information —
> und die Drohne physisch verdreht zum kommandierten `q_des` (Yaw=0) liegt. Amplitude konstant
> **≈ ±0.3 % Throttle**. Korrektes Reglerverhalten; zugleich **Beleg, dass echte Motive-Lagedaten
> bis in den Regler durchkommen**, und dass der Yaw-Zweig des Mixers korrekt verdrahtet ist.
>
> 💡 **Vor dem Roll/Pitch-Vorzeichentest die Drohne im Yaw drehen, bis alle vier `thr` gleich sind**
> (`eR ≈ 0`) — sonst überlagert der Yaw-Offset die Muster und man liest Mischmuster wie `[9 10 8 9]`.

**T1-Beleg id=2 (gemessen vs. Modell, m=0.985):** 0.1·mg → Soll 9.98 % / ist `[10 10 10 10]` ·
0.2·mg → 11.55 % / `[11 12 11 12]` · 0.3·mg → 13.10 % / `[13 13 13 13]` · ~0.4·mg → ~14.6 % /
`[14 15 14 15]` · 0.5·mg → 16.19 % / `[16 16 16 16]`. **Alle innerhalb der Rundungsauflösung → Sim == HW
für die Kette `F_des → Mixer → Throttle`.**

## ⚠️ Offener Designpunkt: Mocap-Dropout-Politik (Flugrisiko)

`mahony_filter` gewichtet `omega_mes = ka·e_acc + kE·e_ext` mit **`ka=1`, `kE=25`** → das Mocap
dominiert die Lageschätzung **25:1**. Setzt Motive im Flug kurz aus und die GCS sendet weiter den
**gehaltenen** `q_ext`, kapert dieser eingefrorene Wert `q_hat`: die Drohne „glaubt" level zu sein,
während sie wegkippt. Auf dem Tisch bereits reproduziert (siehe Fehlversuch oben).

**Fix:** Die GCS muss bei **`mocap_valid = 0`** ein **Null-Quaternion `q_ext = [0 0 0 0]`** senden.
`mahony_filter` prüft `norm(q_ext) > 0.5` und fällt sonst sauber auf **Accel-only** zurück
(`e_ext = 0`). In `bench.slx` **und** `quadcop.slx` per Switch auf `mocap_valid` verdrahten.

| ID | Test | Kriterium | Status |
|----|------|-----------|--------|
| D1 | `q_ext`-Gating bei Mocap-Dropout | bei `mocap_valid=0` geht `q_ext=[0 0 0 0]` raus | ✅ Switch in `bench.slx` (allein wirkungslos, erst mit D3 vollständig) |
| D3 | **Mocap-Validity über die Funkstrecke signalisieren** | Null-Quaternion überlebt den Link; Drohne setzt `e_ext=0` | ✅ **umgesetzt + auf HW bestätigt** (s.u.) |

### ❌ D1 ist wirkungslos: `q_ext = zeros` überlebt den Link nicht

`pack_quat_sm3.m` ersetzt ein **degeneriertes (Null-)Quaternion durch die Identität**:
```matlab
if n < 1e-12,  q = [1;0;0;0];   % Fallback fuer degenerierte Eingabe
```
Der Decoder liefert ohnehin stets ein Einheitsquaternion. ⇒ Die Drohne empfängt bei „ungültig"
ein **gültig aussehendes `[1 0 0 0]`**, der Mahony zieht `q_hat` mit `kE=25` auf **level**.
Die Abfrage `if norm(q_ext) > 0.5 … else e_ext = 0` in `mahony_filter.m` ist **über Funk
unerreichbar** — der Accel-Only-Rückfall existiert nur in der Simulation.

**HW-Beleg (id=2):** Drohne statisch bei **73.7°** (`acc[-9.41 -0.10 2.75]`, `|acc|=9.80`),
`q_ext=zeros` gesendet ⇒ gemessen `thr[14 14 12 12]`, Pitch-Summe **−4** ⇒ `eR_y ≈ 0.048` ⇒
`q_hat` sieht nur **~2.7°**. Gleichgewicht `ka·sin(73.7°−θ) = kE·θ` ⇒ **θ = 2.2°** — deckt sich.

**Flugrisiko:** Bei Motive-Ausfall glaubt die Drohne dauerhaft, sie liege level, während sie
wegkippt. **Vor dem Erstflug beheben.**

### ✅ D3 umgesetzt: `code == 0` als reserviertes „ungültig"

Gewählt wurde **nicht** das flags-Bit (hätte `Bus_Cmd`, das 82-B-USB-Frame, beide Modelle und alle
Goldens angefasst), sondern das **reservierte Codewort**: Ein regulärer Encode kann `0` nie
erzeugen, weil jedes 10-Bit-Feld `u = qi + 512` mit `qi ∈ [−511,511]` stets **≥ 1** ist.

Geändert: `pack_quat_sm3.m` (Null-Quat → `code 0` statt Identität), `unpack_quat_sm3.m`
(`code 0` → Null-Quat), `mcu_packet.hpp` (beide Richtungen, C++). **`mahony_filter.m` unverändert** —
sein `norm(q_ext) > 0.5`-Guard greift dadurch endlich. `Bus_Cmd`/Frame/Modelle unberührt.

Zusätzlich **`drone_hal.cpp` Boot-Init**: `g_cmd.q_ext` bleibt jetzt das **Null-Quaternion**
(vorher Identität). Vor dem ersten Paket gibt es keine Mocap-Referenz — mit Identität hätte sich
der Mahony mit `kE=25` sofort auf „level" eingerastet. `q_des`/`q_ref` bleiben Identität
(ein Null-Quat dort ergäbe in `geo_attitude_ctrl` eine Null-DCM). Kein Gate-B-Einfluss
(Firmware-Mantel, nicht in der ctest-Suite).

**✅ HW-Nachweis D3 + Amplitude (id=2, nach Reflash beider Sketches):**
`mocap_valid = 0`, statisch bei **43.1°** Nase-runter (`acc[-6.49 -0.21 6.93]`, `|acc|=9.50`)
⇒ gemessen **`thr[25 29 0 0]`**, Pitch-Summe **−54**.
Theorie: `eR_y = sin(43.1°) = 0.684`, `τ_y = −k_R·eR = −0.687 N·m` ⇒ erwartet **`[27 27 0 0]`**;
gemessenes Mittel vorne = **27**. **Punktlandung.** (Roll/Yaw je +4 = Rest vom Handkippen;
hinten 0 = Anschlag, bei 0.3·mg Grundschub physikalisch korrekt.)
Vorher (ohne D3): `eR ≈ 2.7°` bei 74° Kippung — der Schätzer war auf level festgenagelt.
⇒ **Accel-Only-Rückfall greift, Pitch-Vorzeichen *und* -Verstärkung stimmen, Kette
Accel → Mahony → `eR` → `geo_attitude_ctrl` → Mixer → Throttle end-to-end validiert.**

⚠️ **`hardware/build/**` sind nur Kopien** und veralten still. `build_sketches.sh` zieht sie bei
jedem Lauf frisch aus `hardware/` + `scripts/sitl/include/` + `hardware/mcu_arm/` nach — vor jedem
Flash also einmal laufen lassen (die Upload-Targets tun das automatisch).

Belegt: Golden `mocap_invalid` → `tx_q(q_ext) = 0`, `rx_qe = [0 0 0 0]`; regulärer Fall (`hover`)
unverändert `537395712 → [1 0 0 0]`. **Gate B: ref 33/33, codegen 39/39.**

⚠️ **Reflash-Reihenfolge:** `mcu_packet.hpp` steckt in **beiden** Sketches.
**Erst die Drohne** (`drone_hal`, entpackt), **dann den Sende-Teensy** (`gcs_sender`, packt) —
sonst gäbe es ein Fenster, in dem ein neuer Sender `code 0` an eine alte Drohne schickt, die es als
gültige (falsche) Lage interpretiert. Der ARM-Codegen (`mcu_ert_rtw`) ist **nicht** betroffen.
| D2 | **Rigid-Body-Zuordnung** (Schwarm!) | `mocap.streaming_id` gehört zur geflogenen Drohne. ⚠️ `mocap_valid` prüft nur, ob *irgendein* Body da ist, **nicht ob es der richtige ist** — ein vertauschter Body liefert plausible, aber **fremde** Lage (bei 3 Drohnen absturzkritisch). Nachweis: Drohne bewegen → `mocap_quat` muss folgen | ⬜ |

## ⚠️ Offener Punkt: vorne/hinten getauscht — Frame-Konsistenz prüfen

Der Nutzer hat **vorne und hinten der Drohne getauscht**. Das verschiebt **zwei** Zuordnungen
gleichzeitig, und **nur ihre Konsistenz zueinander** entscheidet über Stabilität:
1. **IMU → Body** (`R_bs` in `drone_hal.cpp`, aktuell `body_x = imu_y`, `body_y = −imu_x`)
2. **Motorindex → physische Ecke** (Mixer nimmt an: M1=vorne-rechts, M2=vorne-links,
   M3=hinten-links, M4=hinten-rechts)

Wird nur **eine** davon nachgezogen, invertieren **Pitch und Roll** ⇒ sicherer Flip beim Abheben.
Hinweis: Die vom Nutzer notierte Beziehung (`imu_x=−quadcop_y`, `imu_y=quadcop_x`) ist algebraisch
**identisch zum aktuellen Code** — ein echter 180°-Tausch wäre `gyro[0]=−gs[1]; gyro[1]=+gs[0]`.
**Nicht ändern, bevor A–C gemessen sind.**

| ID | Test | Vorgehen | Pass | id=1 | id=2 |
|----|------|----------|------|------|------|
| F-A | Motor ↔ physische Ecke | `esc_calibrate`, Tasten `1..4`, je Motor einzeln | Zuordnung dokumentiert, deckt sich mit M1=VR/M2=VL/M3=HL/M4=HR | ⬜ | ✅ **bestanden** (wie erwartet) |
| F-B | IMU-x-Richtung | „neue" Nase ~30° runter, halten | `acc_x` **negativ** ⇒ IMU-x zeigt nach vorne | ⬜ | ⬜ |
| F-C | **Kreis schließt negativ** | Nase runter, statisch halten | die **physisch vorderen** Motoren gehen hoch | ⬜ | ✅ **bestanden** |

**F-C-Beleg id=2:** statisch bei 73.7° Nase-runter ⇒ `thr[14 14 12 12]`, **Pitch-Summe −4** =
Nase-hoch-Moment, **vorne höher** ⇒ **Rückkopplung negativ, Pitch-Vorzeichen korrekt.**
Zusätzlich **Yaw-Vorzeichen korrekt** (Test mit `q_ext`=Mocap: `q_hat`-Yaw −37.5° ⇒ `thr[0 100 0 100]`
= +τ_z, also Korrektur Richtung 0). ⇒ **Motorzuordnung und `R_bs` sind konsistent, kein Flip-Risiko
aus dieser Ursache.** (Amplitude war klein, weil `q_hat` durch den Codec-Bug auf level gezogen
wurde — siehe D1/D3; das Vorzeichen ist davon unberührt.)

**F-C ist der eigentliche Beweis** — F-A/F-B lokalisieren nur, *wo* der Fehler sitzt, falls F-C
durchfällt. Konventionen sind egal, solange die Rückkopplung negativ ist.

## Stufe S-1 — **Schubtest auf der Waage** (Props **dran**, angebunden)

Erster Test überhaupt mit drehenden Propellern. Belegt in einem Durchgang **C3** (Prop-Montage),
**c_T** (bisher reiner Datenblattwert) und **C4** (Hover-Throttle) — und liefert die Kalibrierung,
auf der Stufe 4 aufsetzt.

**Firmware:** `bash build_sketches.sh --upload-drone-thrust`
(= `HAL_MODE_THRUST`: Motoren scharf **und** Telemetrie. Beim Boot muss
`[boot] 5 gyro bias ...` erscheinen — fehlt es, läuft `HAL_MODE_FLIGHT` und es wird nicht armiert.)

**Stromversorgung: LiPo, voll geladen.** Ein Labornetzteil mit 1.7 A reicht nicht — siehe
Stromspalte unten; schon `0.3·mg` liegt darüber. Geht die Quelle in die Begrenzung, sackt die
Spannung, ESCs verlieren die Kommutierung (Desync → ruckartige Drehzahlsprünge) und der Teensy
kann per Brownout **mit drehenden Propellern resetten**.

**Vorbereitung:** Drohne angebunden (Gurte über die Arme, **nicht** über die Props), Schutzbrille,
niemand in der Rotorebene, Taster in Reichweite, Waage tariert bei laufendem Leerlauf.

> **⚠️ Die Sollwerte unten stammen aus der ALTEN 6S-Kennlinie und sind überholt.**
> Siehe „Kennlinien-Korrektur 4S" weiter unten — Hover liegt nicht bei 24 %, sondern
> spannungsabhängig bei 33–41 %. Die Waagen-Methode selbst hat sich als untauglich
> erwiesen (Vibration; bei 35 %/4 Motoren zeigte sie 154 g statt ~800 g).

### Sollwerte (überholt, nur als Historie)

| `F_des` | erwartet `thr` | Waage **abnehmen** um | ω [rad/s] | ≈ Strom @15 V |
|---|---|---|---|---|
| 0.2·mg | 11.6 | 197 g | 507 | 1.1 A |
| 0.3·mg | 13.2 | 296 g | 621 | 2.1 A |
| 0.4·mg | 14.7 | 394 g | 717 | 3.2 A |
| 0.5·mg | 16.3 | 493 g | 801 | 4.4 A |
| 0.6·mg | 17.9 | 591 g | 878 | 5.8 A |
| 0.7·mg | 19.5 | 690 g | 948 | 7.3 A |

Bei 0.7·mg zeigt die Waage noch ~295 g — ausreichend Abstand zum Abheben.
Strom mit η ≈ 0.78 (Motor+ESC) geschätzt; `P = 4·c_tau·ω³`.

**Betriebsregel:** nur in diesen sechs Stufen hochgehen, nie dazwischen springen, und an jedem
Punkt **beide** Werte ablesen — Waage *und* Report-Zeile. Die Waage allein sagt nicht, ob der
Regler das kommandiert hat, was man glaubt.

Nach dem `ack` laufen alle vier sofort auf ~8.4 % (Polynom-Konstante). Das ist normal, kein Messpunkt.

### Abbruchkriterien — vorher festgelegt, im Zweifel killen

| Beobachtung | bedeutet | Aktion |
|---|---|---|
| **Waage steigt statt zu fallen** | Propeller falsch herum / verkehrt montiert | **Kill**, C3 durchgefallen |
| `t0` springt ohne Zutun auf ~24 | `batt_land` gelatcht, `F_des` überschrieben | Kill, **Reset**, Akku prüfen |
| `batt` < 13.5 V | Akku unter Last am Ende | Kill, laden |
| Schub weicht > 20 % vom Sollwert ab | `c_T` stimmt nicht | **nicht weiter hochfahren** |
| `thr`-Spreizung > 5 counts bei waagerechter Drohne | Mixer-/Motorasymmetrie | Kill, untersuchen |
| `tickmax` > 900 µs | Rechenbudget reißt | Kill |
| `link` wiederholt > 20 ms | Funk instabil | Kill |
| seitlicher Zug an den Gurten, Vibration, Geräusch | irgendetwas stimmt nicht | Kill |

Der erste Eintrag ist der wichtigste: es ist der **einzige** Test der Kampagne, der die
Propellermontage prüft, und er entscheidet sich in der ersten Sekunde bei `0.2·mg`. Deshalb dort
anfangen und nicht höher.

| ID | Test | Pass-Kriterium | id=1 | id=2 |
|----|------|----------------|------|------|
| S1a | Schubrichtung (C3) | Waage nimmt bei allen Stufen **ab** | ⬜ | ⬜ |
| S1b | `c_T`-Kalibrierung | gemessener Schub vs. Sollwert, Abweichung < 20 % | ⬜ | ⬜ |
| S1c | Hover-Throttle (C4) | Extrapolation auf `m·g` → `thr ≈ 24 %`, deutlich < 100 % | ⬜ | ⬜ |
| S1d | Symmetrie | `thr`-Spreizung ≤ 5 counts bei waagerechter Drohne | ⬜ | ⬜ |

---

## Kennlinien-Korrektur 4S — ✅ umgesetzt, Gate B grün

**Befund:** Das Modell war auf das Datenblatt bei **22.2 V (6S)** kalibriert, geflogen wird
**4S (~14.8 V)**. Bei gleichem Throttle liefert das rund 45 % des Datenblattschubs.

**Physik (Nicks Herleitung, korrigiert meinen ersten Ansatz):** Für einen BLDC am PWM-ESC gilt
stationär `kt/R·(duty·U) − kt/(R·kv)·ω = c_Q·ω²`. ω hängt damit **nur vom Produkt `duty·U`** ab.
Also skaliert das **Argument**, nicht das Ergebnis:

```
throttle(U) = throttle(22.2 V) · 22.2/U
```

Mein erster Ansatz („ω ∝ U bei festem Throttle, also Schub ∝ U²") war falsch — er gilt nur, wenn
ω linear in `duty·U` wäre. Das Datenblatt sättigt aber. Belegt an Nicks Einzelmotormessung:

| thr | gemessen | Argument-Skalierung | altes Ergebnis-Modell |
|---|---|---|---|
| 5 % | 5 g | 5.5 g | 10.8 g (**+117 %**) |
| 15 % | 47 g | 49.3 g | 48.6 g |
| 20 % | 78 g | 82.1 g | 75.6 g |
| 25 % | 117 g | 123.1 g | 108.5 g (**−7 %**) |

Konstanter Versatz +5.2 % statt Drift über 124 Prozentpunkte — und bei U = 14.6 V (Ruhespannung
15.0 V sackt unter Last) trifft es auf ±0.5 %.

### Was geändert wurde

1. **Fit über ω statt ω², durch den Ursprung** ([init_quadcop.m](Simulation/scripts/init/init_quadcop.m)).
   `throttle` über `ω²` ist ab 20 % fast linear, nur der erste Abschnitt steil — ein Polynom, das
   zusätzlich den Ursprung treffen soll, verbiegt dafür den linearen Teil (Residuum 5.6 %). Über ω
   passt dieselbe Ordnung auf **0.8 %**. Nebeneffekt: der alte 8.404-%-Leerlauf-Offset (Artefakt des
   fehlenden 0-Punkts) ist weg.
   `p_from_omega = [5.5397521107262334e-06, 0.015435978627782006, 0]`
   *(Umbenannt von `p_from_omega_sq`, damit jeder Konsument laut scheitert statt still falsch zu rechnen.)*

2. **Spannungskorrektur in mcu.slx:**
   `throttle = clamp( polyval(p_from_omega, ω) · U_ds / clamp(V_filt, 11.0, 17.5), 0, 100 )`
   Neue Blöcke: `U_ds_scale`, `V_divide`, `V_clamp`, `RT_Vfilt` (V_filt lag an
   `MATLAB Function1` Port 3 bereits an und war ungenutzt). Polyval-Eingang von `Gain` auf `Sqrt`
   umgehängt.

3. **⚠️ Klemmung ist Pflicht, nicht Kosmetik.** `V_filt` steht im Nenner. Ein ausgefallener
   Batteriesensor (`V_filt = 0` — auf dem Prüfstand bereits einmal aufgetreten, siehe K7) würde
   sonst alle vier Motoren auf 100 % treiben.

4. **`RT_Vfilt` Startwert = 16.8 V.** Der Batterietask läuft mit `Ts_batt = 1 s`; mit der Default-0
   hing die **erste volle Sekunde** nach dem Boot an der unteren Klemme → `22.2/11.0` statt
   `22.2/15.7`, also **43 % zu viel Throttle**. 16.8 V ist die höchste reale 4S-Spannung; weil V im
   Nenner steht, kann der Startwert den Throttle nur zu klein machen, nie zu groß.

5. **`drone_hal.cpp`:** `MCU::initialize()` → `g_mcu.initialize()`. Die Init ist nicht mehr statisch,
   seit sie den `RT_Vfilt`-Startwert im Objekt setzt.

### Neue Hover-Werte (statt bisher 24 %)

| `V_filt` | Hover-Throttle |
|---|---|
| 16.8 V (voll) | 33.0 % |
| 14.8 V (nominal) | 37.4 % |
| 13.5 V (leer) | 41.0 % |

8 Prozentpunkte über eine Akkuladung — ohne die Kompensation änderte sich die Aktorverstärkung
im Flug um ~25 %, und mit ihr Momentenautorität und effektive Dämpfung.

### Verifikation

- **Gate B: 39/39** grün.
- Host-Invariante verschärft: statt `V_filt` zu raten, wird geprüft, dass
  `throttle_i / polyval(P, ω_i)` für **alle vier Motoren gleich** ist und die daraus implizierte
  Spannung in den Klemmgrenzen liegt. Fängt falsches Polynom **und** fehlende Klemmung.
- Unabhängig gegen das Golden nachgerechnet (5001 Ticks): Streuung über die vier Motoren
  **6.7e-16**, implizierte Spannung 15.74–16.8 V, **0 Ticks** an der unteren Klemme.
- ARM-Codegen: `na = 22.2 * poly / rtb_V_clamp`, `Polyval incorporates Sqrt`, alte 8.404 nirgends
  mehr im generierten Code. Alle drei Betriebsarten kompilieren.

### ✅ HW-Beleg der Implementierung (id=2, BENCH, `q_ext=[1 0 0 0]`, F_des-Sweep)

`q_ext` auf Identität pinnt `q_hat` waagerecht (`kE=25`), damit ist `eR≈0`, `tau=0` und der Mixer
verteilt exakt gleich. Jede Zeile mit der Spannung **aus derselben Zeile** nachgerechnet:

| `F_des` | `batt` | Modell | gemessen |
|---|---|---|---|
| 0 | 15.29 V | **0.00** | **0** ✅ |
| 0.1·mg | 15.29 V | 9.06 | 9 ✅ |
| 0.2·mg | 15.27 V | 13.44 | 14 (Rundungsgrenze, Schieberegler) |
| 0.3·mg | 15.31 V | 16.99 | 17 ✅ |
| 0.4·mg | 15.31 V | 20.17 | 20 ✅ |
| 0.5·mg | 15.34 V | 23.05 | 23 ✅ |
| 0.6·mg | 15.26 V | 25.92 | 26 ✅ |
| 0.7·mg | 15.29 V | 28.48 | 28 ✅ |

Belegt: `F_des=0 → thr[0 0 0 0]` (Ursprungs-Offset weg, vorher `[8 9 8 9]`), alle vier Motoren
identisch, Spannungskorrektur rechnet mit der tatsächlichen Spannung.

> ⚠️ **Der Test muss in `HAL_MODE_BENCH` laufen.** In THRUST kontaminiert er sich selbst:
> drehende Props → Vibration → `gyro`≠0 → Dämpfungsterm erzeugt `tau` → Mixer → Throttle auf allen
> vier → Props drehen weiter. Beobachtet: `thr[2 2 3 3]` statt `[0 0 0 0]`, `|acc|` schwankte
> zwischen 4.1 und 12.1 m/s².

---

## `abs(omega_sq)` → `max(omega_sq, 0)` — ✅ umgesetzt, Gate B grün

Aufgefallen bei der Analyse des obigen Fehlversuchs. Der Mixer rechnet
`omega_sq = Γ⁻¹·[F; tau]`; wegen `Σ omega_sq = F/c_T` sind bei kleinem `F` und nennenswertem `tau`
**zwangsläufig einzelne Kanäle negativ**. Ein negatives `omega_sq` heisst „negative Quadratdrehzahl
gewünscht" — unerfüllbar.

`abs()` machte daraus eine **positive Drehzahl gleicher Größe**, also Schub *entgegen* dem Wunsch.
Richtig ist Abschneiden bei 0: Wunsch nicht erfüllbar → Motor aus.

Relevanter Bereich: kleines `F_des` mit kräftiger Lagekorrektur — **Start und Sinkflug**, nicht ein
exotischer Randfall.

Umgesetzt als `Saturation [0, inf]` (Block `omega_sq_floor`) anstelle des `Abs`-Blocks; wirkt auf
`rotor_cmd` und `throttle` gleichermaßen, da beide hinter demselben `Sqrt` hängen.
Generierter Code: `std::sqrt(std::fmax(x, 0.0))` statt `std::sqrt(std::abs(x))`. Gate B 39/39.

---

---

## Kennlinie aus eigener Messung — ✅ umgesetzt, Gate B grün

Der Datenblatt-Fit oben ist damit **überholt**. Er taugte für `c_T` (reine Aerodynamik), nicht für
Throttle → Drehzahl.

### Methode

Einzelmotor, Propeller montiert, **Blattdurchgangsfrequenz aus dem Audiospektrum** (Handyaufnahme →
Welch-PSD → Peak über dem lokalen Median). Dreiblattschraube, also `rpm = f_Blatt/3·60`.
Rohdaten in `hardware/motor_frequenzspektrum/`.

| thr | f_Blatt | Prominenz | rpm | Zuwachs |
|---|---|---|---|---|
| 10 % | 147.4 Hz | 17.2 dB | 2 947 | |
| 15 % | 213.3 Hz | 15.5 | 4 266 | +1319 |
| 20 % | 276.6 Hz | 18.9 | 5 531 | +1265 |
| 25 % | 339.1 Hz | 18.5 | 6 783 | +1252 |
| 30 % | 399.0 Hz | 14.1 | 7 981 | +1198 |
| 35 % | 457.6 Hz | 6.8 | 9 152 | +1171 |
| 40 % | 504.0 Hz | 14.0 | 10 080 | +929 |
| 45 % | 559.9 Hz | 10.3 | 11 197 | +1117 |

Reihe 1 (10–30 %) bei 15.90 V, Reihe 2 (35–45 %) bei 15.66 V — Spannung je Punkt mitgeführt.

### Warum das Datenblatt für diesen Zweck nicht taugt

Es nennt bei 22.2 V/10 % bereits **4957 rpm**, während `kv·U·duty = 1900·22.2·0.1 = 4218 rpm` die
**Leerlaufdrehzahl** wäre. Ein belasteter Motor kann die nie erreichen. Entsprechend lag das Modell
durchweg über der Leerlaufgrenze:

| thr | Leerlaufgrenze | Datenblatt-Modell | gemessen |
|---|---|---|---|
| 10 % | 3 021 | **3 550** ❌ | 2 947 (98 %) |
| 20 % | 6 042 | **6 816** ❌ | 5 531 (92 %) |
| 30 % | 9 063 | **9 726** ❌ | 7 981 (88 %) |

Der Fehler war ein **konstanter Faktor 0.8165** in der Drehzahl (Streuung 0.008) — die *Form* der
Datenblattkurve stimmt, nur die Skala nicht. Schub geht mit ω², also **0.67 im Schub**: beim
rechnerischen Hover-Throttle wären real nur **69 % des Gewichts** herausgekommen. Die Drohne wäre
am Boden geblieben.

### Ergebnis

```
p_from_omega = [4.1705914834120731e-06, 0.022154923169740597, 0]
```

Max. Residuum **0.37 Prozentpunkte** über alle acht Punkte, die meisten unter 0.1.
Der Hover liegt bei 1133 rad/s und damit **innerhalb** des Messbereichs (45 % → 1173 rad/s) —
es wird interpoliert, nicht extrapoliert.

| `V_filt` | Hover alt (Datenblatt) | **Hover neu (Messung)** |
|---|---|---|
| 16.8 V | 32.5 % | **40.2 %** |
| 15.9 V | 34.3 % | **42.5 %** |
| 14.8 V | 36.9 % | **45.7 %** |
| 13.5 V | 40.5 % | **50.1 %** |

Unverändert bleibt die gesamte Mechanik: Spannungskorrektur, Klemmung [11.0, 17.5], `RT_Vfilt`-
Startwert 16.8 V, `max(ω², 0)`, verschärfte Host-Invariante. Nur die zwei Koeffizienten sind neu.
Gate B 39/39.

> **Methodischer Hinweis für Wiederholungen:** Live-Peaks von Hand abzulesen war unbrauchbar —
> die erste Reihe (193/236/301/322/344 Hz) hatte Zuwächse von +43/+65/+21/+22 Hz, physikalisch
> unmöglich für einen Drehzahlverlauf. Bei 35–45 % übertönt tieffrequenter Handhabungslärm
> (57–116 Hz) die Blattlinie um bis zu 13 dB. Erkennung deshalb über **Prominenz gegen den lokalen
> Median plus Harmonischen-Nachweis (2f, 3f)**, nicht über den Absolutpegel.

### ⬜ Was weiterhin offen ist

**1. Das Spannungsgesetz ist ✅ bestätigt** (im flugrelevanten Bereich). Drei Spannungsebenen,
Einzelmotor: 15.8 V (Kalibrierung), 14.9 V, 14.1 V (Netzteil, stabil, Prominenz 27–31 dB).

Test über das Frequenzverhältnis bei festem Throttle (14.1 V gegen 15.8 V):

| thr | f@14.1V / f@15.8V (gemessen) | `1/V`-Vorhersage |
|---|---|---|
| 15 % | 0.912 | 0.899 |
| 20 % | 0.912 | 0.901 |
| 25 % | 0.917 | 0.903 |
| 30 % | 0.912 | 0.904 |

Das gemessene Verhältnis ist **über alle Throttle-Werte konstant 0.912** — genau das sagt ein reines
Spannungsgesetz voraus (Verschiebung hängt nur von U ab, nicht vom Gas). Rest gegen `1/V` ~1.2 %
(real minimal schwächer als `1/V`), innerhalb der Methodengenauigkeit. Modell trifft die 14.1-V-
Frequenzen direkt auf +1…+2.5 %. **Keine Modelländerung nötig.**

Der frühere scheinbare Hochgas-Drift (14.9-V-Reihe, −6…−9 % bei 30–35 %) war **Messrauschen**:
schwache Prominenz (auf 9 dB gefallen) plus Vier-Motoren-Verschmierung, kein realer Effekt.

Hebel bleibt 14.1–15.8 V (12 %); die Ränder 12.5 V / 16.8 V sind nicht vermessen, liegen aber
außerhalb des Hover-Bereichs. Der erste gefesselte Schwebeversuch deckt sie mit ab.

**2. `c_T` bleibt Datenblattwert.** Die Kette Drehzahl → Schub ist nicht unabhängig belegt. Das
Datenblatt ist hier allerdings gut gestützt: `Schub/rpm²` ist über den ganzen Bereich auf ±4 %
konstant. Der Beleg käme beim ersten gefesselten Schwebeversuch — Hover-Throttle gegen die Tabelle
oben prüfen, bei bekanntem `V_filt`.

**3. ⚠️ IMU-Aussetzer.** Siehe unten; einmalig aufgetreten, danach per `i2c_scan` bestätigt in
Ordnung (0x68). Falls er wiederkommt: Verkabelung.

**2. ⚠️ IMU lieferte im letzten Test exakt Null.** `gyro[0 0 0]`, `acc[0.00 -0.00 0.00]`,
`bias[0 0 0]` — der Nullbias heisst, die MPU lieferte schon beim Boot nichts. Vormittags war sie
in Ordnung (`acc[0.61 9.72 -0.13]`); vermutlich hat sich beim Auf-/Abbau auf der Montageplatte eine
I2C-Leitung gelöst. Für den F_des-Sweep folgenlos (`q_ext` hält die Lage, `gyro=0` erzeugt kein
Störmoment), aber **mit totem IMU löst der Overspeed-Kill K1 nie aus**. Vor allem Weiteren klären.

---

## Stufe 4 — Regelkreis, gestaffelt (statt direkt Steigflug)

Kritik am Vorschlag „direkt 0.8 m Steigflug": zu viel Energie für den ersten Flug unter Last.
Zwei Dinge laufen dort zum ersten Mal live und sind noch **nie unter echtem Schub getestet** —
`c_T`/Hover-Punkt und der **Positionsregler** (Mocap-Frame → Solllage, C2). Beide können abstürzen.
Deshalb gestaffelt, jede Stufe de-riskt genau eine Sache, jeder Fehler bleibt harmlos:

| Stufe | Aufbau | de-riskt | Motoren |
|---|---|---|---|
| **S-2** | Tisch, Positions-Vorzeichentest (Setpoint fest, Drohne von Hand versetzen) | **C2**, ohne Flug | **aus** (BENCH) |
| **S-3** | angebunden am Boden, `F_des` langsam rampen bis leicht/anhebt | `c_T`, Hover-Punkt | an |
| **S-4** | Hover **wenige cm**, Positionshaltung, Leine schlaff | Regelkreis geschlossen, min. Fallhöhe | an |
| **S-5** | 0.8-m-Steigflug + kommandierte Sink-Landung | volle Trajektorie | an |

### S-2 — Positions-Vorzeichentest (Tisch, BENCH) — **nächster Schritt**

Mechanismus: `pos_ctrl` erzeugt `F_des_vec = m·(a_ref + g − Kp·(x−x_ref) − Kd·(v−v_ref))`; die
Solllage `q_des` kippt entlang dieser Kraft. `Kp = m·ω_n_pos²` mit `ω_n_pos = 4.2 rad/s` ist steif:
**~5 cm Versatz → ~5° Kippung → gut am `thr` ablesbar, ohne Sättigung.** Also **kleine** Versätze,
nicht 0.5 m (das gäbe 50°, gesättigt).

Setup: `init_trajectory.m` → `TEST_S2 = true`, `x0`/`yaw0` = gemessene Ruheposition (Signal `x` an
`pos_ctrl` bei laufendem Modell ablesen). Setpoint = Ruheposition → im Ruhezustand `thr` symmetrisch.

Test (selbst-interpretierend, keine Achsen-Kenntnis nötig): Drohne von Hand ~5–10 cm versetzen,
**waagerecht halten** (Marker frei). Der Regler muss eine Kippung **zurück** zum Sollpunkt
kommandieren, also **gegen** die Verschiebung. Über die Achsen-Summen ablesen. Alle vier
Horizontalrichtungen + hoch/runter.

**Pass:** kommandierte Kippung wirkt der Verschiebung in **allen** Richtungen entgegen (negative
Rückkopplung). Läuft sie mit → Positions-/Frame-Vorzeichen invertiert → **C2 ❌, nicht fliegen.**

Belegt via **Setpoint-Versatz** (Drohne steht still, `s2_offset` in init_trajectory, Baseline
`[41 42 42 42]` symmetrisch bei 16.2 V). Motorlage M1=VR/M2=VL/M3=HL/M4=HR (F-A).

| `s2_offset` | `thr` | Achsen-Summe | erwartet | id=2 |
|---|---|---|---|---|
| +x (+0.08) | `[38 39 46 46]` | Pitch +15 | > 0 (hinten hoch) | ✅ |
| −x (−0.08) | `[45 46 38 39]` | Pitch −14 | < 0 (vorne hoch) | ✅ |
| +y (+0.08) | `[45 39 38 46]` | Roll −14 | < 0 (rechts hoch) | ✅ |
| −y (−0.08) | `[38 46 46 39]` | Roll +15 | > 0 (links hoch) | ✅ |
| +z (+0.2) | `[48 49 48 49]` | kollektiv +7 | rauf | ✅ |
| −z (−0.2) | `[34 35 35 35]` | kollektiv −7 | runter | ✅ |

Beträge stimmen quantitativ (0.08 m → 8.2° → Pitch-Summe ≈ 12 vs. 14–15 gemessen; z: 0.2 m →
+36 % F → +16 % ω → +7 counts). **C2 für id=2 am Tisch geschlossen**, id=1 offen.

### S-3 — Hover-Onset / `c_T` (angebunden, am Boden)

`F_des` langsam hochrampen. Erwartung: leichtes Abheben bei `F_des ≈ m·g` (~40 % `thr` bei 15 V).
Hebt sie deutlich später ab (z. B. 1.3·m·g), ist `c_T` daneben — **am Boden gemerkt, nicht in 0.8 m.**

### S-4 / S-5 — siehe Tabelle. Landung: **kommandierte Sinktrajektorie** (traj_gen bringt `z` auf
Grund, Positionshaltung aktiv), NICHT `safety_land`. Letzteres ist der blinde Hard-Floor-Backstop
(kein Pos/Vel), driftet seitlich — Notnetz, kein reguläres Landeverfahren.

Leine: **schlaff, Fangleine**, mittig über dem Schwerpunkt. Straff + außermittig erzeugt ein Moment,
gegen das der Regler ankämpft → verfälscht S-4, kann selbst destabilisieren.

---

Top-4-Crash-Ursachen, die bis Stufe 5 zwingend grün sein müssen:

| ID | Test | Warum kritisch |
|----|------|----------------|
| C1 | **Regel-/Mixer-Vorzeichen** | ✅ **für id=2 am Tisch geschlossen** (Stufe 1b: alle 3 Achsen, Vorzeichen **und** Amplitude). Hier nur noch unter Last bestätigen. Für id=1 offen. |
| C2 | **Mocap-Frame** | ✅ **für id=2 am Tisch geschlossen** (S-2: alle 3 Achsen × 2 Richtungen, negative Rückkopplung, Vorzeichen + Betrag). Unter Bewegung im Flug noch zu bestätigen. Für id=1 offen. |
| C3 | **Prop-Montage + Drehrichtung** | 🟡 **Drehrichtung ✅ (id=2)**, Prop-Paarung/-Orientierung **offen**. Ein CW-Prop auf einem CCW-Motor erzeugt Schub *nach unten* — die Drehrichtungsprüfung fängt das nicht. Beweis nur über Waagentest (S-1). |
| C4 | **Hover-Throttle** | `m·g` → Throttle deutlich < 100 %, stabil |

## Stufe 5 — Erstflug  *(nach Stufe 4)*
Kurzer Low-Hover, Spotter, Kill scharf (Rotary + Taster in Reichweite).
