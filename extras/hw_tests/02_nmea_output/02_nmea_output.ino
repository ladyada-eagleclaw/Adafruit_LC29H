// HILBERT Rev A LC29H(EA), GPIO8 RX / GPIO9 TX through its fitted divider.
// Prove GGA enable/disable changes live UART output, then restore the setting.
// No NVM writes or restarts. Counts valid GGA even without a position fix.
#include <Adafruit_LC29H.h>
HardwareSerial receiverPort(1);
char first[256], second[256];
Adafruit_LC29H gps(first, second, sizeof(first));
int8_t original = -1;
uint32_t gga = 0, other = 0;
void received(const nmea_sentence_t& sentence, void* context);
void sample();
void stopTest(const __FlashStringHelper* message);
void waitForRun();
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) delay(10);
  delay(250);
  Serial.println(F("Adafruit LC29H NMEA output hardware test"));
  // Upload/reset activity must not automatically repeat a configuration test.
  waitForRun();
  receiverPort.setRxBufferSize(4096);
  receiverPort.begin(460800, SERIAL_8N1, 8, 9);
  gps.onSentence(received);
  if (!gps.begin(receiverPort, 2000)) stopTest(F("FAIL: begin"));
  Serial.println(F("PASS: firmware identification"));
  original = gps.isNMEAEnabled(LC29H_NMEA_GGA);
  if (original < 0) stopTest(F("FAIL: original GGA state"));
  Serial.println(F("PASS: original GGA state saved"));
  if (!gps.enableNMEA(LC29H_NMEA_GGA, true) || gps.isNMEAEnabled(LC29H_NMEA_GGA) != 1)
    stopTest(F("FAIL: enabling GGA"));
  sample();
  if (gga < 3) stopTest(F("FAIL: enabled GGA not observed"));
  Serial.println(F("PASS: enabled GGA is present on UART"));
  if (!gps.enableNMEA(LC29H_NMEA_GGA, false) || gps.isNMEAEnabled(LC29H_NMEA_GGA) != 0)
    stopTest(F("FAIL: disabling GGA"));
  sample();
  if (gga || other < 3) stopTest(F("FAIL: disabling GGA did not selectively stop its output"));
  Serial.println(F("PASS: GGA stopped while other navigation continued"));
  if (!gps.enableNMEA(LC29H_NMEA_GGA, original != 0) ||
      gps.isNMEAEnabled(LC29H_NMEA_GGA) != original) stopTest(F("FAIL: original GGA state restore"));
  sample();
  if ((original && gga < 3) || (!original && gga)) stopTest(F("FAIL: restored output not observed"));
  original = -1;
  Serial.println(F("PASS: original GGA output restored and observed"));
  Serial.println(F("NMEA output hardware test complete."));
}
void loop() { gps.poll(512); }
void sample() {
  uint32_t start = millis();
  while (millis() - start < 300) gps.poll(512);
  gga = other = 0;
  start = millis();
  while (millis() - start < 3000) gps.poll(512);
  Serial.print(F("GGA sentences: ")); Serial.print(gga);
  Serial.print(F(" | other navigation: ")); Serial.println(other);
}
void received(const nmea_sentence_t& sentence, void* context) {
  (void)context;
  if (sentence.status != NMEA_FRAME_VALID || sentence.address.length != 5 ||
      sentence.address.data[0] == 'P') return;
  if (!memcmp(sentence.address.data + 2, "GGA", 3)) gga++;
  else other++;
}
void stopTest(const __FlashStringHelper* message) {
  Serial.println(message);
  if (original >= 0) {
    Serial.print(F("Restore original GGA setting: "));
    Serial.println(gps.enableNMEA(LC29H_NMEA_GGA, original != 0) ? F("accepted") : F("FAILED"));
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
