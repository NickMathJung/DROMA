// i2c_scan.cpp — I2C-Scanner (Bench). Bestaetigt die MPU-6050-Adresse:
//   0x68 = ADO->GND (Bodge R8 bestueckt, so soll es sein).  0x69 = ADO floatet
//   (Bodge fehlt). Wire(0) = SDA18/SCL19, 400 kHz — identisch zur HAL.
#include <Arduino.h>
#include <Wire.h>

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) {}
    Wire.begin();
    Wire.setClock(400000);
    Serial.println("I2C-Scan (Wire0, SDA18/SCL19, 400 kHz)...");
}

void loop() {
    int found = 0;
    for (uint8_t a = 1; a < 127; ++a) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            const char* note = (a == 0x68) ? "  <- MPU-6050 (ADO=GND, OK)"
                             : (a == 0x69) ? "  <- MPU-6050 (ADO floatet -> Bodge R8 fehlt!)"
                                           : "";
            Serial.printf("  gefunden: 0x%02X%s\n", a, note);
            ++found;
        }
    }
    if (!found) Serial.println("  nichts gefunden - Verdrahtung/Pull-ups/Strom pruefen.");
    Serial.println("---");
    delay(2000);
}
