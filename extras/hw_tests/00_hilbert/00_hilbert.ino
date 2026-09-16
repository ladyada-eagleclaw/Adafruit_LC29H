// HILBERT Rev A: LC29H(EA) TX -> GPIO8, GPIO9 -> 1k/5.1k divider -> RX.
// The divider reduces 3.3 V TX to about 2.76 V (module I/O domain is 2.8 V).
// Only UART1 pins are configured. No NVM writes, restart or power changes.
#include <Adafruit_LC29H.h>
HardwareSerial receiverPort(1);
char first[256], second[256];
Adafruit_LC29H gps(first, second, sizeof(first));
uint32_t validLines = 0, fixes = 0, errors = 0, lastReport = 0;
void receiveLine(const nmea_sentence_t& sentence, void* context);
void setup() {
  Serial.begin(115200);
  // Run unattended after five seconds; no permanent USB wait.
  while (!Serial && millis() < 5000) delay(10);
  delay(250);
  Serial.println(F("Adafruit LC29H HILBERT identification and readback"));
  receiverPort.setRxBufferSize(4096);
  receiverPort.begin(460800, SERIAL_8N1, 8, 9);
  gps.onSentence(receiveLine);
  bool ready = gps.begin(receiverPort, 2000);
  Serial.print(F("Begin: ")); Serial.println(ready ? F("PASS") : F("FAIL"));
  Serial.print(F("Command status: ")); Serial.println(gps.commandStatus());
  char response[160];
  if (gps.getVersion(response, sizeof(response))) Serial.print(response);
  Serial.print(F("Baud: ")); Serial.println(gps.getBaudrate());
  Serial.print(F("Fix interval: ")); Serial.println(gps.getFixInterval());
  Serial.print(F("GGA enabled: ")); Serial.println(gps.isNMEAEnabled(LC29H_NMEA_GGA));
  Serial.print(F("Navigation mode: ")); Serial.println(gps.getNavigationMode());
  Serial.print(F("AIC: ")); Serial.println(gps.isInterferenceCancellationEnabled());
  Serial.print(F("Receiver mode: ")); Serial.println(gps.getReceiverMode());
  Serial.print(F("RTCM mode: ")); Serial.println(gps.getRTCMMode());
  Serial.print(F("Reference output: ")); Serial.println(gps.isReferenceStationOutputEnabled());
  Serial.print(F("Ephemeris output: ")); Serial.println(gps.isEphemerisOutputEnabled());
  lc29h_precision_t precision;
  if (gps.getPrecision(precision)) {
    Serial.print(F("NMEA decimal places: "));
    Serial.print(precision.time); Serial.print(','); Serial.print(precision.position);
    Serial.print(','); Serial.print(precision.altitude); Serial.print(','); Serial.print(precision.dop);
    Serial.print(','); Serial.print(precision.speed); Serial.print(','); Serial.println(precision.course);
  } else { Serial.print(F("Precision query status: ")); Serial.println(gps.commandStatus()); }
  lc29h_survey_config_t survey;
  if (gps.getSurvey(survey)) { Serial.print(F("Survey mode: ")); Serial.println(survey.mode); }
  Serial.println(F("Readback finished; receiving without changing settings."));
}
void loop() {
  gps.poll(512);
  if (millis() - lastReport >= 2000) {
    lastReport = millis();
    Serial.print(F("Valid lines: ")); Serial.print(validLines);
    Serial.print(F(" | GGA fixes: ")); Serial.print(fixes);
    Serial.print(F(" | invalid lines: ")); Serial.println(errors);
  }
}
void receiveLine(const nmea_sentence_t& sentence, void* context) {
  (void)context;
  if (sentence.status != NMEA_FRAME_VALID) { errors++; return; }
  validLines++;
  lc29h_fix_t fix = Adafruit_LC29H::parseFix(sentence);
  if (fix.position.validation.status == GNSS_SENTENCE_VALID && fix.position.fix) fixes++;
  if (sentence.address.length >= 4 && !strncmp(sentence.address.data, "PAIR", 4)) {
    Serial.write((const uint8_t*)sentence.text.data, sentence.text.length);
  }
}
