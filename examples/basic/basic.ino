// Use a board with a spare hardware UART and accurate 460800-baud support.
// Module TX -> Serial1 RX; module RX <- Serial1 TX through suitable level shifting.
// Connect ground and power as required by the breakout. Bare LC29H uses 2.8 V I/O.
// SoftwareSerial and a 16 MHz AVR UART are unsuitable for the EA default baud.
// HILBERT: replace Serial1.begin(...) with Serial1.begin(460800, SERIAL_8N1, 8, 9).
#include <Adafruit_LC29H.h>
char firstBuffer[256], secondBuffer[256];
Adafruit_LC29H gps(firstBuffer, secondBuffer, sizeof(firstBuffer));

bool ready = false;
uint32_t lastAttempt = 0;
void received(const nmea_sentence_t& sentence, void* context);
void setup() {
  Serial.begin(115200);
  // Wait for Serial Monitor; remove this wait for unattended operation.
  while (!Serial) delay(10);
  delay(250);
  Serial.println(F("Adafruit LC29H exact position example"));
  Serial1.begin(460800);
  gps.onSentence(received);
}
void loop() {
  if (!ready) {
    if (millis() - lastAttempt < 3000) return;
    lastAttempt = millis();
    if (!gps.begin(Serial1)) {
      Serial.println(F("Waiting for firmware reply; check baud and UART wiring."));
      return;
    }
    // Eight fractional minute digits retain the EA's high-precision NMEA output.
    // This is a volatile configuration change: nothing is saved to flash.
    lc29h_precision_t precision = {3, 8, 3, 3, 3, 3};
    if (!gps.setPrecision(precision)) {
      Serial.println(F("Receiver did not accept high-precision output; retrying."));
      return;
    }
    ready = true;
    Serial.println(F("Receiver ready. Place the antenna under a clear sky."));
  }
  gps.poll();
}
void received(const nmea_sentence_t& sentence, void* context) {
  (void)context;
  lc29h_fix_t fix = Adafruit_LC29H::parseFix(sentence);
  if (fix.position.validation.status != GNSS_SENTENCE_VALID) return;
  if (!fix.position.fix) {
    Serial.println(F("Waiting for a position fix."));
    return;
  }
  char latitude[GNSS_COORDINATE_TEXT_SIZE], longitude[GNSS_COORDINATE_TEXT_SIZE];
  if (!gps.formatCoordinate(latitude, sizeof(latitude), fix.position.latitude) ||
      !gps.formatCoordinate(longitude, sizeof(longitude), fix.position.longitude)) return;
  Serial.print(F("Latitude: ")); Serial.print(latitude);
  Serial.print(F("\tLongitude: ")); Serial.print(longitude);
  Serial.print(F("\tSatellites: ")); Serial.print(fix.satellites);
  char altitude[24];
  if (gps.formatDecimal(altitude, sizeof(altitude), fix.altitude)) {
    Serial.print(F("\tAltitude (m): ")); Serial.print(altitude);
  }
  Serial.print(F("\tFix: "));
  switch (fix.position.fixQuality) {
    case 0: Serial.println(F("none")); break;
    case 1: Serial.println(F("autonomous")); break;
    case 2: Serial.println(F("differential")); break;
    case 3: Serial.println(F("precise service")); break;
    case 4: Serial.println(F("RTK fixed")); break;
    case 5: Serial.println(F("RTK float")); break;
    case 6: Serial.println(F("estimated")); break;
    default: Serial.println(F("unknown")); break;
  }
}
