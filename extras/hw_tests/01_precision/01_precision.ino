// HILBERT Rev A LC29H(EA), GPIO8 RX / GPIO9 TX through its fitted divider.
// Temporarily changes NMEA precision, proves the emitted text changes, restores.
// No NVM writes. UART pins only; the other HILBERT receiver is not touched.
#include <Adafruit_LC29H.h>
HardwareSerial receiverPort(1);
char first[256], second[256];
Adafruit_LC29H gps(first, second, sizeof(first));
lc29h_precision_t original;
bool haveOriginal = false;
uint32_t ggaCount = 0, matchingCount = 0;
uint8_t expectedPosition = 0, expectedAltitude = 0;
void receiveLine(const nmea_sentence_t& sentence, void* context);
void sample(uint32_t duration);
void stopTest(const __FlashStringHelper* message);
uint8_t decimals(nmea_span_t field);
void waitForRun();
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) delay(10);
  delay(250);
  Serial.println(F("Adafruit LC29H precision hardware test"));
  // Upload/reset activity must not automatically repeat a configuration test.
  waitForRun();
  receiverPort.setRxBufferSize(4096);
  receiverPort.begin(460800, SERIAL_8N1, 8, 9);
  gps.onSentence(receiveLine);
  if (!gps.begin(receiverPort, 2000)) stopTest(F("FAIL: begin"));
  Serial.println(F("PASS: firmware identification"));
  if (!gps.getPrecision(original)) stopTest(F("FAIL: original precision readback"));
  haveOriginal = true;
  Serial.println(F("PASS: original precision saved in RAM"));
  lc29h_precision_t high = original;
  high.position = 8;
  high.altitude = 3;
  if (!gps.setPrecision(high)) stopTest(F("FAIL: set high precision"));
  Serial.println(F("PASS: high precision command accepted"));
  lc29h_precision_t readback;
  if (!gps.getPrecision(readback) || readback.position != 8 || readback.altitude != 3)
    stopTest(F("FAIL: high precision readback"));
  Serial.println(F("PASS: receiver reports eight position / three altitude digits"));
  expectedPosition = 8;
  expectedAltitude = 3;
  sample(4000);
  Serial.print(F("GGA received: ")); Serial.print(ggaCount);
  Serial.print(F(" | correct precision: ")); Serial.println(matchingCount);
  if (matchingCount < 3) stopTest(F("FAIL: high precision not observed in navigation"));
  Serial.println(F("PASS: high precision observed in actual navigation sentences"));
  if (!gps.setPrecision(original)) stopTest(F("FAIL: restore original precision"));
  if (!gps.getPrecision(readback) || readback.position != original.position ||
      readback.altitude != original.altitude) stopTest(F("FAIL: restored readback"));
  expectedPosition = original.position;
  expectedAltitude = original.altitude;
  sample(4000);
  if (matchingCount < 3) stopTest(F("FAIL: restored precision not observed"));
  Serial.println(F("PASS: original precision restored and observed in navigation"));
  haveOriginal = false;
  Serial.println(F("Precision hardware test complete."));
}
void loop() { gps.poll(512); }
void sample(uint32_t duration) {
  // Let any already transmitted fix finish before assessing the new setting.
  uint32_t start = millis();
  while (millis() - start < 300) gps.poll(512);
  ggaCount = matchingCount = 0;
  start = millis();
  while (millis() - start < duration) gps.poll(512);
}
uint8_t decimals(nmea_span_t field) {
  for (size_t i = 0; i < field.length; i++)
    if (field.data[i] == '.') return field.length - i - 1;
  return 0;
}
void receiveLine(const nmea_sentence_t& sentence, void* context) {
  (void)context;
  lc29h_fix_t fix = Adafruit_LC29H::parseFix(sentence);
  if (fix.position.validation.status != GNSS_SENTENCE_VALID || !fix.position.fix) return;
  ggaCount++;
  nmea_span_t remaining = sentence.fields;
  nmea_span_t values[11];
  for (uint8_t i = 0; i < 11; i++) values[i] = Adafruit_NMEA::nextField(remaining);
  if (decimals(values[1]) == expectedPosition && decimals(values[3]) == expectedPosition &&
      decimals(values[8]) == expectedAltitude && decimals(values[10]) == expectedAltitude)
    matchingCount++;
}
void stopTest(const __FlashStringHelper* message) {
  Serial.println(message);
  if (haveOriginal) {
    Serial.print(F("Restore original setting: "));
    Serial.println(gps.setPrecision(original) ? F("accepted") : F("FAILED"));
  }
  while (true) { gps.poll(512); delay(1); }
}

void waitForRun() {
  Serial.println(F("Send RUN followed by a newline to start this test."));
  const char command[] = "RUN\n";
  uint8_t matched = 0;
  while (matched < 4) {
    if (Serial.available()) {
      char byte = Serial.read();
      if (byte == '\r') continue;
      if (byte == command[matched]) matched++;
      else matched = 0;
    }
    delay(1);
  }
}
