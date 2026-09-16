// Use a board with a spare hardware UART and accurate 460800-baud support.
// Module TX -> Serial1 RX; module RX <- Serial1 TX through suitable level shifting.
// Connect ground and power as required by the breakout. Bare LC29H uses 2.8 V I/O.
// SoftwareSerial and a 16 MHz AVR UART are unsuitable for the EA default baud.
// HILBERT: replace Serial1.begin(...) with Serial1.begin(460800, SERIAL_8N1, 8, 9).
#include <Adafruit_LC29H.h>
char firstBuffer[256], secondBuffer[256];
Adafruit_LC29H gps(firstBuffer, secondBuffer, sizeof(firstBuffer));

// Set APPLY_SURVEY_CONFIGURATION true for a deliberate NVM configuration step.
// Keep the antenna fixed. Survey-in averages a standalone solution; it does not
// establish centimeter absolute accuracy. Use a surveyed ECEF reference for that.
// This example does not transmit RTCM over USB; use rtcm_bridge after configuring.
const bool APPLY_SURVEY_CONFIGURATION = false;
bool ready = false;
uint32_t lastAttempt = 0;
void received(const nmea_sentence_t& sentence, void* context);
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Remove for unattended use.
  delay(250);
  Serial.println(F("Adafruit LC29H survey-in example"));
  Serial1.begin(460800);
  gps.onSentence(received);
}
void loop() {
  if (!ready) {
    if (millis() - lastAttempt < 3000) return;
    lastAttempt = millis();
    if (!gps.begin(Serial1)) { Serial.println(F("Waiting for receiver.")); return; }
    ready = true;
    if (APPLY_SURVEY_CONFIGURATION) {
      // mode=survey-in, 60 observations, 1.0 m estimated 3D accuracy limit.
      // Exact values are coefficient / 10^decimalPlaces: {VALID, 10, 1} = 1.0 m.
      lc29h_survey_config_t config = {1, 60,
        {NMEA_NUMBER_VALID, 10, 1}, {NMEA_NUMBER_VALID, 0, 0},
        {NMEA_NUMBER_VALID, 0, 0}, {NMEA_NUMBER_VALID, 0, 0}};
      if (!gps.setSurvey(config) || !gps.setReceiverMode(LC29H_MODE_BASE) ||
          !gps.saveParameters()) {
        Serial.println(F("Configuration failed; do not assume the base is ready."));
      } else {
        Serial.println(F("Saved. Power-cycle the module to apply, then set the flag false."));
      }
    }
    // Availability varies by firmware; failure does not mean survey completed.
    if (!gps.setMessageRate("PQTMSVINSTATUS", 1, 1))
      Serial.println(F("Survey status output was not accepted on this firmware."));
    lc29h_survey_config_t config;
    if (gps.getSurvey(config)) {
      Serial.print(F("Configured minimum observations: ")); Serial.println(config.minimumDuration);
    }
  }
  gps.poll();
}
void received(const nmea_sentence_t& sentence, void* context) {
  (void)context;
  lc29h_survey_status_t status = gps.parseSurveyStatus(sentence);
  if (status.status != LC29H_REPLY_VALID) return;
  Serial.print(F("Survey: "));
  switch (status.validity) {
    case 0: Serial.print(F("invalid")); break;
    case 1: Serial.print(F("in progress")); break;
    case 2: Serial.print(F("valid")); break;
    default: Serial.print(F("unknown")); break;
  }
  Serial.print(F("\tObservations: ")); Serial.print(status.observations);
  char value[24];
  if (gps.formatDecimal(value, sizeof(value), status.accuracy)) {
    Serial.print(F("\tEstimated accuracy (m): ")); Serial.print(value);
  }
  Serial.println();
}
