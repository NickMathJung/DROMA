#!/usr/bin/env bash
# build_sketches.sh — flashbare Arduino-Sketch-Ordner aus den kanonischen Quellen
# assemblieren (SSOT bleibt in hardware/ + scripts/sitl/include/ + hardware/mcu_arm/).
# Ergebnis: hardware/build/{gcs_sender,drone_hal}/  -> in Arduino IDE oeffnen & flashen,
# oder mit --compile / --upload direkt via arduino-cli.
#
#   ./build_sketches.sh                            # nur assemblieren
#   ./build_sketches.sh --compile                  # + fuer Teensy 4.1 kompilieren (alle 3 Modi)
#   ./build_sketches.sh --upload-sender       COM7 # + Sende-Teensy flashen
#   ./build_sketches.sh --upload-drone-bench  COM8 # Motoren tot, volle Telemetrie (Props ab)
#   ./build_sketches.sh --upload-drone-thrust COM8 # Motoren scharf + Telemetrie (Waagentest S-1)
#   ./build_sketches.sh --upload-drone-flight COM8 # Motoren scharf, kein Report
#
# FLATNESS-Variante (Flachheitsregelung, eigener Satz Sketches — die Kaskade
# oben bleibt unangetastet und jederzeit flashbar):
#   ./build_sketches.sh --upload-sender-flat       # Sende-Teensy (2-Frame-OTA)
#   ./build_sketches.sh --upload-drone-flat-bench  # / -thrust / -flight
# ACHTUNG: Sender und Drohne muessen ZUSAMMEN passen — beide Kaskade oder beide
# Flatness. Die OTA-Formate sind inkompatibel (29 B vs 2x27 B).
#
# Die Betriebsart wird bewusst nie implizit gewaehlt: --upload-drone gibt es nicht
# mehr, weil aus dem Namen nicht hervorging, ob die ESCs scharf werden.
set -euo pipefail

HW="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"      # .../hardware
INC="$HW/../scripts/sitl/include"
ARM="$HW/mcu_arm/mcu_ert_rtw"
ARMF="$HW/mcu_flat_arm/mcu_flat_ert_rtw"                # Flatness-Variante
OUT="$HW/build"
FQBN="teensy:avr:teensy41"
CLI="${ARDUINO_CLI:-/c/Program Files/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe}"
CFG="${ARDUINO_CFG:-$HOME/AppData/Local/Arduino15/arduino-cli.yaml}"

echo "== assembliere Sketches nach $OUT =="
# in-place ueberschreiben (KEIN rm -rf) -> kollidiert nicht mit offener Arduino-IDE.
mkdir -p "$OUT/gcs_sender" "$OUT/drone_hal"

# --- Sende-Teensy: Frame-Contract + Codec ---
cp "$HW/gcs_sender.cpp" "$OUT/gcs_sender/gcs_sender.ino"
cp "$INC/gcs_frame.hpp" "$INC/mcu_packet.hpp" "$OUT/gcs_sender/"

# --- Drohne: HAL + Codec + ARM-generierte MCU-Klasse (OHNE ert_main.cpp) ---
cp "$HW/drone_hal.cpp" "$OUT/drone_hal/drone_hal.ino"
cp "$INC/mcu_packet.hpp" "$OUT/drone_hal/"
if [ ! -d "$ARM" ]; then
  echo "FEHLER: $ARM fehlt. Erst run_mcu_arm_codegen.m laufen lassen (§3f)." >&2; exit 1
fi
for f in mcu.cpp mcu_data.cpp mcu.h mcu_types.h mcu_private.h rtwtypes.h; do
  cp "$ARM/$f" "$OUT/drone_hal/"
done

# --- FLATNESS-Variante (eigene Sketch-Ordner; Kaskade bleibt unberuehrt) -----
# Nur assemblieren, wenn der ARM-Codegen da ist — sonst still ueberspringen,
# damit ein reiner Kaskaden-Arbeitsplatz ohne run_mcu_flat_arm_codegen laeuft.
if [ -d "$ARMF" ]; then
  mkdir -p "$OUT/gcs_sender_flat" "$OUT/drone_hal_flat"
  cp "$HW/gcs_sender_flat.cpp" "$OUT/gcs_sender_flat/gcs_sender_flat.ino"
  cp "$INC/gcs_frame.hpp" "$INC/gcs_frame_flat.hpp" \
     "$INC/mcu_packet.hpp" "$INC/mcu_flat_packet.hpp" "$OUT/gcs_sender_flat/"
  cp "$HW/drone_hal_flat.cpp" "$OUT/drone_hal_flat/drone_hal_flat.ino"
  cp "$INC/mcu_packet.hpp" "$INC/mcu_flat_packet.hpp" "$OUT/drone_hal_flat/"
  for f in mcu_flat.cpp mcu_flat_data.cpp mcu_flat.h mcu_flat_types.h mcu_flat_private.h rtwtypes.h; do
    cp "$ARMF/$f" "$OUT/drone_hal_flat/"
  done
  FLAT_OK=1
else
  echo "   (Flatness uebersprungen: $ARMF fehlt -> run_mcu_flat_arm_codegen.m)"
  FLAT_OK=0
fi

# --- Bench-Sketches (standalone, keine geteilten Header) ---
mkdir -p "$OUT/i2c_scan" "$OUT/esc_calibrate" "$OUT/analyze_frequency" "$OUT/battery_health" "$OUT/channel_scan"
cp "$HW/i2c_scan.cpp"          "$OUT/i2c_scan/i2c_scan.ino"
cp "$HW/esc_calibrate.cpp"     "$OUT/esc_calibrate/esc_calibrate.ino"
cp "$HW/analyze_frequency.cpp" "$OUT/analyze_frequency/analyze_frequency.ino"
cp "$HW/battery_health.cpp"    "$OUT/battery_health/battery_health.ino"
cp "$HW/channel_scan.cpp"      "$OUT/channel_scan/channel_scan.ino"

# set_mode [BENCH|THRUST|FLIGHT] — Betriebsart in den Sketch schreiben.
# Ueber Header statt -D, weil die Teensy-Recipe compiler.cpp.extra_flags ignoriert
# (dort verpuffte das Define stillschweigend und es lief immer der Datei-Default).
# Schreibt in BEIDE Drohnen-Sketches (Kaskade + Flatness), damit keiner
# stillschweigend die Betriebsart eines frueheren Laufs erbt.
set_mode() {
    for d in "$OUT/drone_hal" "$OUT/drone_hal_flat"; do
        [ -d "$d" ] || continue
        cat > "$d/hal_mode.h" <<EOF
// Automatisch erzeugt von build_sketches.sh — nicht von Hand aendern.
#define HAL_MODE_$1
EOF
    done
    echo "   Betriebsart: HAL_MODE_$1"
}

# Assemblieren setzt immer auf BENCH zurueck. Sonst erbt ein spaeterer Flash aus
# der IDE stillschweigend die Betriebsart des letzten Laufs — und das ist der
# Unterschied zwischen "Motoren tot" und "Propeller drehen an".
set_mode BENCH

SKETCHES="gcs_sender drone_hal i2c_scan esc_calibrate analyze_frequency battery_health channel_scan"
[ "$FLAT_OK" = "1" ] && SKETCHES="$SKETCHES gcs_sender_flat drone_hal_flat"
for s in $SKETCHES; do echo "  $s/: $(ls "$OUT/$s" | tr '\n' ' ')"; done

# compile [sketch] — grep darf leer ausgehen, sonst killt pipefail den Lauf.
compile() { echo "== compile $1 =="; "$CLI" compile -b "$FQBN" "$OUT/$1" --config-file "$CFG" \
    2>&1 | { grep -iE "error|Memory Usage|FLASH|RAM|Build.*status" || true; } \
         | { grep -viE "Fehler beim Initialisieren|Download failed" || true; }; }
# Teensy-Uploads brauchen den "teensy port" (usb:...), NICHT COMx. Auto-erkennen.
teensy_port() { "$CLI" board list --config-file "$CFG" 2>/dev/null \
    | grep "$FQBN" | awk '{print $1}' | head -1; }
# upload [sketch] [port_hint] [extra_flags] — compile+upload (Define wirkt sicher).
# Nutzt den erkannten teensy-Port; port_hint nur Fallback, falls keiner gefunden wird.
upload()  {
    local tp; tp="$(teensy_port)"; tp="${tp:-$2}"
    if [ -z "$tp" ]; then echo "FEHLER: kein Teensy gefunden (USB angesteckt? Nur 1 Teensy?)." >&2; return 1; fi
    echo "== compile+upload $1 -> $tp =="
    echo "   (Serial-Monitor auf dem Ziel-Teensy vorher SCHLIESSEN, sonst haengt der Loader.)"
    "$CLI" compile --upload -b "$FQBN" -p "$tp" "$OUT/$1" --config-file "$CFG" \
    2>&1 | { grep -iE "error|Memory Usage|upload|verif|bytes|programming|Build.*status" || true; } \
         | { grep -viE "Fehler beim Initialisieren|Download failed" || true; }; }

while [ $# -gt 0 ]; do
  case "$1" in
    --compile)                compile gcs_sender;
                              set_mode BENCH;  compile drone_hal;
                              set_mode THRUST; compile drone_hal;
                              set_mode FLIGHT; compile drone_hal;
                              set_mode BENCH;                       # sicherer Endzustand
                              if [ "$FLAT_OK" = "1" ]; then
                                compile gcs_sender_flat;
                                set_mode BENCH;  compile drone_hal_flat;
                                set_mode THRUST; compile drone_hal_flat;
                                set_mode FLIGHT; compile drone_hal_flat;
                                set_mode BENCH;
                              fi
                              compile i2c_scan;   compile esc_calibrate;
                              compile analyze_frequency; compile battery_health;
                              compile channel_scan; shift;;
    --upload-sender)          upload gcs_sender    "$2"; shift 2;;
    # Betriebsart immer explizit: bench = Motoren tot, thrust = Motoren + Telemetrie
    # (Waagentest S-1), flight = Motoren, kein Report.
    # Nur die Betriebsart setzen (zum Flashen aus der Arduino-IDE).
    --mode)                   case "$2" in BENCH|THRUST|FLIGHT) set_mode "$2";;
                                *) echo "FEHLER: --mode braucht BENCH|THRUST|FLIGHT" >&2; exit 2;; esac; shift 2;;
    --upload-drone-bench)     set_mode BENCH;  upload drone_hal "$2"; shift 2;;
    --upload-drone-thrust)    set_mode THRUST; upload drone_hal "$2"; shift 2;;
    --upload-drone-flight)    set_mode FLIGHT; upload drone_hal "$2"; shift 2;;
    # --- FLATNESS-Variante: eigene Targets, damit nie versehentlich die
    #     Kaskade durch die Flachheitsregelung ersetzt wird (und umgekehrt).
    --upload-sender-flat)      upload gcs_sender_flat "$2"; shift 2;;
    --upload-drone-flat-bench) set_mode BENCH;  upload drone_hal_flat "$2"; shift 2;;
    --upload-drone-flat-thrust)set_mode THRUST; upload drone_hal_flat "$2"; shift 2;;
    --upload-drone-flat-flight)set_mode FLIGHT; upload drone_hal_flat "$2"; shift 2;;
    --upload-drone|--upload-drone-selftest)
                              echo "FEHLER: '$1' ist mehrdeutig geworden. Betriebsart explizit waehlen:" >&2
                              echo "  --upload-drone-bench   Motoren tot, volle Telemetrie (Props ab)" >&2
                              echo "  --upload-drone-thrust  Motoren scharf + Telemetrie (Waagentest S-1)" >&2
                              echo "  --upload-drone-flight  Motoren scharf, kein Report" >&2
                              exit 2;;
    --upload-scan)            upload i2c_scan          "$2"; shift 2;;
    --upload-esccal)          upload esc_calibrate     "$2"; shift 2;;
    --upload-freq)            upload analyze_frequency "$2"; shift 2;;
    --upload-batt)            upload battery_health    "$2"; shift 2;;
    --upload-chanscan)        upload channel_scan      "$2"; shift 2;;
    *) echo "unbekannt: $1" >&2; exit 2;;
  esac
done
echo "== fertig. IDE: $OUT/<name>/<name>.ino oeffnen & flashen. =="
