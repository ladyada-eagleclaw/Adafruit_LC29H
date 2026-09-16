// Use a board with a spare hardware UART and accurate 460800-baud support.
// Module TX -> Serial1 RX; module RX <- Serial1 TX through suitable level shifting.
// Connect ground and power as required by the breakout. Bare LC29H uses 2.8 V I/O.
// SoftwareSerial and a 16 MHz AVR UART are unsuitable for the EA default baud.
// HILBERT: replace Serial1.begin(...) with Serial1.begin(460800, SERIAL_8N1, 8, 9).
#include <Adafruit_LC29H.h>
char firstBuffer[256], secondBuffer[256];
Adafruit_LC29H gps(firstBuffer, secondBuffer, sizeof(firstBuffer));

// Settings walkthrough: read back actual receiver values, then demonstrate
// selected volatile setters with their existing values. No NVM writes/restarts.
bool ready = false;
uint32_t lastAttempt = 0;
void showSettings();
void received(const nmea_sentence_t& sentence, void* context);
void setup() {
  Serial.begin(115200);
  // Remove the wait for unattended use.
  while (!Serial) delay(10);
  delay(250);
  Serial.println(F("Adafruit LC29H settings walkthrough"));
  Serial1.begin(460800);
  gps.onSentence(received);
}
void loop() {
  if (!ready) {
    if (millis() - lastAttempt < 3000) return;
    lastAttempt = millis();
    ready = gps.begin(Serial1);
    if (!ready) { Serial.println(F("Waiting for receiver.")); return; }
    showSettings();
  }
  gps.poll();
}
void showSettings() {
  char response[192];
  if (gps.getVersion(response, sizeof(response))) {
    lc29h_version_t version = gps.parseVersion(gps.validate(response, strlen(response)));
    Serial.print(F("Firmware: "));
    Serial.write((const uint8_t*)version.version.data, version.version.length);
    Serial.println();
  }
  Serial.print(F("UART baud: ")); Serial.println(gps.getBaudrate());
  Serial.print(F("Position interval (ms): ")); Serial.println(gps.getFixInterval());
  // setBaudrate() and setFixInterval() require save/restart planning. Do not
  // change these here: a baud change also needs a matching host UART change.
  Serial.print(F("Navigation dynamics: "));
  switch (gps.getNavigationMode()) {
    case LC29H_NAV_NORMAL: Serial.println(F("normal")); break;
    case LC29H_NAV_FITNESS: Serial.println(F("fitness")); break;
    case LC29H_NAV_STATIONARY: Serial.println(F("stationary")); break;
    case LC29H_NAV_DRONE: Serial.println(F("drone")); break;
    case LC29H_NAV_SWIMMING: Serial.println(F("swimming")); break;
    case LC29H_NAV_BIKE: Serial.println(F("bicycle")); break;
    case LC29H_NAV_UNKNOWN: Serial.println(F("query unavailable")); break;
    default: Serial.println(F("unknown")); break;
  }
  Serial.print(F("Role: "));
  switch (gps.getReceiverMode()) {
    case LC29H_MODE_ROVER: Serial.println(F("rover")); break;
    case LC29H_MODE_BASE: Serial.println(F("base")); break;
    case LC29H_MODE_UNKNOWN: Serial.println(F("unknown")); break;
    default: Serial.println(F("unknown")); break;
  }
  int8_t aic = gps.isInterferenceCancellationEnabled();
  Serial.print(F("Interference cancellation: "));
  if (aic < 0) Serial.println(F("query unavailable"));
  else {
    Serial.println(aic ? F("enabled") : F("disabled"));
    if (!gps.enableInterferenceCancellation(aic != 0)) Serial.println(F("Setting was rejected."));
  }
  int8_t gga = gps.isNMEAEnabled(LC29H_NMEA_GGA);
  Serial.print(F("GGA output: "));
  if (gga < 0) Serial.println(F("query unavailable"));
  else {
    Serial.println(gga ? F("enabled") : F("disabled"));
    if (!gps.enableNMEA(LC29H_NMEA_GGA, gga != 0)) Serial.println(F("Setting was rejected."));
  }
  lc29h_precision_t precision;
  if (gps.getPrecision(precision)) {
    Serial.print(F("NMEA position fractional digits: ")); Serial.println(precision.position);
    Serial.print(F("NMEA altitude fractional digits: ")); Serial.println(precision.altitude);
    if (!gps.setPrecision(precision)) Serial.println(F("Precision setting was rejected."));
  }
  Serial.print(F("RTCM observations: "));
  switch (gps.getRTCMMode()) {
    case LC29H_RTCM_DISABLED: Serial.println(F("disabled")); break;
    case LC29H_RTCM_MSM4: Serial.println(F("MSM4")); break;
    case LC29H_RTCM_MSM7: Serial.println(F("MSM7")); break;
    case LC29H_RTCM_UNKNOWN: Serial.println(F("query unavailable on this firmware")); break;
    default: Serial.println(F("unknown")); break;
  }
  Serial.println(F("The survey_base example covers role and survey configuration."));
  Serial.println(F("The rtcm_bridge example covers correction input and binary output."));
  Serial.println(F("Settings were not saved; receiver was not restarted."));
}
void received(const nmea_sentence_t& sentence, void* context) {
  (void)context;
  lc29h_motion_t motion = gps.parseMotion(sentence);
  if (motion.position.validation.status != GNSS_SENTENCE_VALID || !motion.position.fix) return;
  char speed[24], course[24];
  if (!gps.formatDecimal(speed, sizeof(speed), motion.speedKnots) ||
      !gps.formatDecimal(course, sizeof(course), motion.course)) return;
  Serial.print(F("Speed (knots): ")); Serial.print(speed);
  Serial.print(F("\tCourse (degrees): ")); Serial.println(course);
}
