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
# Die Betriebsart wird bewusst nie implizit gewaehlt: --upload-drone gibt es nicht
# mehr, weil aus dem Namen nicht hervorging, ob die ESCs scharf werden.
set -euo pipefail

HW="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"      # .../hardware
INC="$HW/../scripts/sitl/include"
ARM="$HW/mcu_arm/mcu_ert_rtw"
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

# --- Bench-Sketches (standalone, keine geteilten Header) ---
mkdir -p "$OUT/i2c_scan" "$OUT/esc_calibrate" "$OUT/analyze_frequency" "$OUT/battery_health"
cp "$HW/i2c_scan.cpp"          "$OUT/i2c_scan/i2c_scan.ino"
cp "$HW/esc_calibrate.cpp"     "$OUT/esc_calibrate/esc_calibrate.ino"
cp "$HW/analyze_frequency.cpp" "$OUT/analyze_frequency/analyze_frequency.ino"
cp "$HW/battery_health.cpp"    "$OUT/battery_health/battery_health.ino"

# set_mode [BENCH|THRUST|FLIGHT] — Betriebsart in den Sketch schreiben.
# Ueber Header statt -D, weil die Teensy-Recipe compiler.cpp.extra_flags ignoriert
# (dort verpuffte das Define stillschweigend und es lief immer der Datei-Default).
set_mode() {
    cat > "$OUT/drone_hal/hal_mode.h" <<EOF
// Automatisch erzeugt von build_sketches.sh — nicht von Hand aendern.
#define HAL_MODE_$1
EOF
    echo "   Betriebsart: HAL_MODE_$1"
}

# Assemblieren setzt immer auf BENCH zurueck. Sonst erbt ein spaeterer Flash aus
# der IDE stillschweigend die Betriebsart des letzten Laufs — und das ist der
# Unterschied zwischen "Motoren tot" und "Propeller drehen an".
set_mode BENCH

for s in gcs_sender drone_hal i2c_scan esc_calibrate analyze_frequency battery_health; do echo "  $s/: $(ls "$OUT/$s" | tr '\n' ' ')"; done

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
                              compile i2c_scan;   compile esc_calibrate;
                              compile analyze_frequency; compile battery_health; shift;;
    --upload-sender)          upload gcs_sender    "$2"; shift 2;;
    # Betriebsart immer explizit: bench = Motoren tot, thrust = Motoren + Telemetrie
    # (Waagentest S-1), flight = Motoren, kein Report.
    # Nur die Betriebsart setzen (zum Flashen aus der Arduino-IDE).
    --mode)                   case "$2" in BENCH|THRUST|FLIGHT) set_mode "$2";;
                                *) echo "FEHLER: --mode braucht BENCH|THRUST|FLIGHT" >&2; exit 2;; esac; shift 2;;
    --upload-drone-bench)     set_mode BENCH;  upload drone_hal "$2"; shift 2;;
    --upload-drone-thrust)    set_mode THRUST; upload drone_hal "$2"; shift 2;;
    --upload-drone-flight)    set_mode FLIGHT; upload drone_hal "$2"; shift 2;;
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
    *) echo "unbekannt: $1" >&2; exit 2;;
  esac
done
echo "== fertig. IDE: $OUT/<name>/<name>.ino oeffnen & flashen. =="
