// Use a board with a spare hardware UART and accurate 460800-baud support.
// Module TX -> Serial1 RX; module RX <- Serial1 TX through suitable level shifting.
// Connect ground and power as required by the breakout. Bare LC29H uses 2.8 V I/O.
// SoftwareSerial and a 16 MHz AVR UART are unsuitable for the EA default baud.
// HILBERT: replace Serial1.begin(...) with Serial1.begin(460800, SERIAL_8N1, 8, 9).
#include <Adafruit_LC29H.h>
char firstBuffer[256], secondBuffer[256];
Adafruit_LC29H gps(firstBuffer, secondBuffer, sizeof(firstBuffer));

// Numeric-only columns: satellites, HDOP, altitude in meters, GGA fix quality.
// Setup/retry diagnostics are deliberately omitted from this plotter stream.
bool ready = false;
uint32_t lastAttempt = 0;
void received(const nmea_sentence_t& sentence, void* context);
void setup() {
  Serial.begin(115200);
  // Remove this wait for unattended use.
  while (!Serial) delay(10);
  delay(250);
  Serial1.begin(460800);
  gps.onSentence(received);
}
void loop() {
  if (!ready) {
    if (millis() - lastAttempt < 3000) return;
    lastAttempt = millis();
    ready = gps.begin(Serial1);
    if (!ready) return;
  }
  gps.poll();
}
void received(const nmea_sentence_t& sentence, void* context) {
  (void)context;
  lc29h_fix_t fix = Adafruit_LC29H::parseFix(sentence);
  if (fix.position.validation.status != GNSS_SENTENCE_VALID || !fix.position.fix) return;
  char hdop[24], altitude[24];
  if (!gps.formatDecimal(hdop, sizeof(hdop), fix.hdop) ||
      !gps.formatDecimal(altitude, sizeof(altitude), fix.altitude)) return;
  Serial.print(fix.satellites); Serial.print('\t');
  Serial.print(hdop); Serial.print('\t');
  Serial.print(altitude); Serial.print('\t');
  Serial.println(fix.position.fixQuality);
}
