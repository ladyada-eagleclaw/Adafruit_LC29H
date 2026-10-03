// Metro Mini/Uno: A4 -> SDA, A5 -> SCL, GND -> GND, 5V -> breakout VIN.
// USB alone does not power the breakout's host-side level shifters.
// A bare LC29H uses 2.8 V logic: use a level-shifted breakout.
// The breakout USB/HOST UART switch does not select or disable I2C.
#include <Adafruit_LC29H.h>
#include <Adafruit_LC29H_I2C.h>

char firstBuffer[160], secondBuffer[160];
Adafruit_LC29H gps(firstBuffer, secondBuffer, sizeof(firstBuffer));
Adafruit_LC29H_I2C gpsPort;
bool ready = false;
uint32_t lastAttempt = 0;
void received(const nmea_sentence_t& sentence, void* context);

void setup() {
  Serial.begin(115200);
  // Wait for Serial Monitor on native USB boards; remove for unattended use.
  while (!Serial) delay(10);
  delay(250);

  Serial.println(F("Adafruit LC29H I2C navigation example"));

  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000, true);
#endif
  if (!gpsPort.begin()) {
    // A host reset may interrupt the receiver's multi-step I2C protocol.
    // Recovery does not reset the GPS; the identity query below verifies it.
    if (!gpsPort.recover()) Serial.println(F("Check I2C wiring and breakout VIN power."));
  }
  gps.onSentence(received);
}

void loop() {
  if (!ready) {
    // Polling also drains a FIFO left full before the host started.
    gps.poll();
    if (millis() - lastAttempt < 3000) return;
    lastAttempt = millis();
    if (!gps.begin(gpsPort, 30000)) {
      Serial.print(F("Waiting for I2C firmware reply; bus status: "));
      Serial.println(gpsPort.lastError());
      return;
    }
    // Slow the multi-sentence satellite inventory so the AVR's 32-byte Wire
    // transfers can keep up. Position output continues at its normal rate.
    // This changes only the current port and is not saved to flash.
    if (!gps.setMessageRate("GSV", 5)) {
      Serial.println(F("Receiver did not accept a slower satellite inventory."));
    }
    lc29h_precision_t precision = {3, 8, 3, 3, 3, 3};
    if (!gps.setPrecision(precision)) {
      Serial.println(F("Receiver did not accept high-precision output."));
    }
    ready = true;
    Serial.println(F("I2C receiver ready. Place the antenna under a clear sky."));
  }
  gps.poll();
}

void received(const nmea_sentence_t& sentence, void* context) {
  (void)context;
  lc29h_fix_t fix = gps.parseFix(sentence);
  if (fix.position.validation.status != GNSS_SENTENCE_VALID) return;
  Serial.print(F("Satellites: ")); Serial.print(fix.satellites);
  if (!fix.position.fix) {
    Serial.println(F("\tWaiting for a fix."));
    return;
  }
  char coordinate[GNSS_COORDINATE_TEXT_SIZE];
  if (gps.formatCoordinate(coordinate, sizeof(coordinate), fix.position.latitude)) {
    Serial.print(F("\tLatitude: ")); Serial.print(coordinate);
  }
  if (gps.formatCoordinate(coordinate, sizeof(coordinate), fix.position.longitude)) {
    Serial.print(F("\tLongitude: ")); Serial.print(coordinate);
  }
  if (gps.formatDecimal(coordinate, sizeof(coordinate), fix.altitude)) {
    Serial.print(F("\tAltitude (m): ")); Serial.print(coordinate);
  }
  Serial.println();
}
