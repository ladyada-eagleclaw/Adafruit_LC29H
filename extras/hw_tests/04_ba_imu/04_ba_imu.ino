// Metro Mini/Uno and LC29H BA breakout: A4 SDA, A5 SCL, 5V VIN, common GND.
// Temporarily selects sensor-only I2C output, checks real IMU/calibration
// reports, then restores every changed rate. No NVM writes or pad driving.
#include <Adafruit_LC29H_BA.h>
#include <Adafruit_LC29H_I2C.h>
#include <math.h>

char firstBuffer[160], secondBuffer[160];
Adafruit_LC29H_BA gps(firstBuffer, secondBuffer, sizeof(firstBuffer));
Adafruit_LC29H_I2C gpsPort;
// Remove the large satellite inventory first; restore it last.
const char* messages[] = {"GSV", "GSA", "GLL", "VTG", "RMC", "GGA"};
uint8_t originalRates[6];
int8_t originalIMU = -1, originalCalibration = -1;
uint8_t savedRates = 0;
uint16_t samples = 0, badSamples = 0, calibrationReports = 0;
uint32_t minimumInterval = UINT32_MAX, maximumInterval = 0;
// This firmware's timestamps vary by a few ms (92..101 observed at 10 Hz).
// Permit 10 ms of scheduling jitter, while rejecting duplicates/missed epochs.
const float expectedInterval = 100, maximumJitter = 10;
uint32_t previousTime = 0;
bool haveTime = false;
void received(const nmea_sentence_t& sentence, void* context);
void sample(uint32_t duration);
bool restore();
void stopTest(const __FlashStringHelper* message);
void waitForRun();

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("Adafruit LC29H BA I2C IMU hardware test"));
  waitForRun();
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000, true);
#endif
  if (!gpsPort.begin()) {
    Serial.println(F("Trying endpoint recovery after interrupted I2C operation."));
    if (!gpsPort.recover()) stopTest(F("FAIL: endpoint recovery"));
    Serial.println(F("PASS: endpoint recovery ACK"));
  }
  if (!gps.begin(gpsPort, 30000)) stopTest(F("FAIL: I2C identification"));
  Serial.println(F("PASS: I2C identification"));
  for (uint8_t i = 0; i < 6; i++) {
    int8_t rate = gps.getMessageRate(messages[i]);
    if (rate < 0) stopTest(F("FAIL: original NMEA rate readback"));
    originalRates[i] = rate;
    savedRates++;
    Serial.print(messages[i]); Serial.print(F(" original rate: ")); Serial.println(rate);
    if (!gps.setMessageRate(messages[i], 0)) stopTest(F("FAIL: select sensor-only output"));
  }
  originalIMU = gps.getIMURate();
  Serial.print(F("Original IMU rate: ")); Serial.println(originalIMU);
  originalCalibration = gps.getMessageRate("PQTMDRCAL", 1);
  Serial.print(F("Original calibration rate: ")); Serial.println(originalCalibration);
  if (originalIMU < 0 || originalCalibration < 0) stopTest(F("FAIL: original sensor rate readback"));
  Serial.println(F("PASS: original rates saved in RAM"));
  if (!gps.setIMURate(0)) stopTest(F("FAIL: disable initial IMU output"));
  int8_t dr = gps.isDeadReckoningEnabled();
  if (dr < 0) stopTest(F("FAIL: DR configuration readback"));
  Serial.print(F("Dead reckoning enabled: ")); Serial.println(dr);
  gps.onSentence(received);
  if (!gps.setMessageRate("PQTMDRCAL", 1, 1)) stopTest(F("FAIL: enable calibration reports"));
  sample(3500);
  Serial.print(F("Calibration reports: ")); Serial.println(calibrationReports);
  if (calibrationReports < 2) stopTest(F("FAIL: calibration reports not observed"));
  Serial.println(F("PASS: calibration reports decoded; driving calibration is not tested"));
  if (!gps.setMessageRate("PQTMDRCAL", 0, 1)) stopTest(F("FAIL: disable calibration reports"));
  if (!gps.setIMURate(10) || gps.getIMURate() != 10) stopTest(F("FAIL: 10 Hz IMU readback"));
  Serial.println(F("PASS: 10 Hz module-frame IMU accepted and read back"));
  sample(500);
  samples = badSamples = 0;
  minimumInterval = UINT32_MAX;
  maximumInterval = 0;
  haveTime = false;
  sample(20000);
  Serial.print(F("IMU samples in 20 seconds: ")); Serial.println(samples);
  Serial.print(F("Invalid or implausible samples: ")); Serial.println(badSamples);
  Serial.print(F("Timestamp interval min/max (ms): "));
  Serial.print(minimumInterval); Serial.print('/'); Serial.println(maximumInterval);
  if (samples < 150 || samples > 230 || badSamples || gpsPort.lastError())
    stopTest(F("FAIL: sustained IMU measurements"));
  Serial.println(F("PASS: fresh six-axis measurements and plausible gravity"));
  if (!gps.setIMURate(0) || gps.getIMURate() != 0) stopTest(F("FAIL: disable IMU readback"));
  sample(500);
  samples = 0;
  sample(2000);
  if (samples) stopTest(F("FAIL: IMU output continued after disabling"));
  Serial.println(F("PASS: disabling IMU stops actual measurements"));
  if (!restore()) stopTest(F("FAIL: original rates could not be restored"));
  Serial.println(F("PASS: all original rates restored and read back"));
  Serial.println(F("BA IMU hardware test complete. Continuing reception."));
}

void loop() { gps.poll(); }

void sample(uint32_t duration) {
  uint32_t started = millis();
  while (millis() - started < duration) gps.poll();
}

void received(const nmea_sentence_t& sentence, void* context) {
  (void)context;
  lc29h_calibration_t calibration = gps.parseCalibration(sentence);
  if (calibration.status == LC29H_REPLY_VALID) {
    calibrationReports++;
    Serial.print(F("Calibration state: ")); Serial.print(calibration.calibration);
    Serial.print(F(" | Navigation source: ")); Serial.println(calibration.source);
  }
  lc29h_imu_t imu = gps.parseIMU(sentence);
  if (imu.status == LC29H_REPLY_UNSUPPORTED) return;
  if (imu.status != LC29H_REPLY_VALID) { badSamples++; return; }
  samples++;
  float gravitySquared = 0;
  for (uint8_t i = 0; i < 3; i++) {
    gravitySquared += imu.acceleration[i] * imu.acceleration[i];
    if (!isfinite(imu.angularVelocity[i])) badSamples++;
  }
  if (!isfinite(gravitySquared) || gravitySquared < 25 || gravitySquared > 225 ||
      !isfinite(imu.temperature) || imu.temperature < -40 || imu.temperature > 100)
    badSamples++;
  if (haveTime) {
    uint32_t interval = imu.timestamp - previousTime;
    if (interval < minimumInterval) minimumInterval = interval;
    if (interval > maximumInterval) maximumInterval = interval;
    if (fabs((float)interval - expectedInterval) > maximumJitter) {
      badSamples++;
      if (badSamples <= 10) {
        Serial.print(F("Unexpected timestamp interval (ms): ")); Serial.println(interval);
      }
    }
  }
  previousTime = imu.timestamp;
  haveTime = true;
  if (samples % 20) return;
  Serial.print(F("Temperature C: ")); Serial.print(imu.temperature, 2);
  Serial.print(F(" | Acceleration m/s^2:"));
  for (uint8_t i = 0; i < 3; i++) { Serial.print(' '); Serial.print(imu.acceleration[i], 3); }
  Serial.print(F(" | Gyro rad/s:"));
  for (uint8_t i = 0; i < 3; i++) { Serial.print(' '); Serial.print(imu.angularVelocity[i], 4); }
  Serial.println();
}

bool restore() {
  bool ok = true;
  if (originalIMU >= 0) {
    if (!gps.setIMURate(originalIMU) || gps.getIMURate() != originalIMU) ok = false;
    originalIMU = -1;
  }
  if (originalCalibration >= 0) {
    if (!gps.setMessageRate("PQTMDRCAL", originalCalibration, 1) ||
        gps.getMessageRate("PQTMDRCAL", 1) != originalCalibration) ok = false;
    originalCalibration = -1;
  }
  while (savedRates) {
    savedRates--;
    if (!gps.setMessageRate(messages[savedRates], originalRates[savedRates]) ||
        gps.getMessageRate(messages[savedRates]) != originalRates[savedRates]) ok = false;
  }
  return ok;
}

void stopTest(const __FlashStringHelper* message) {
  Serial.println(message);
  Serial.print(F("Command status: ")); Serial.println(gps.commandStatus());
  Serial.print(F("I2C error: ")); Serial.println(gpsPort.lastError());
  if (savedRates || originalIMU >= 0 || originalCalibration >= 0) {
    Serial.println(restore() ? F("Original rates restored.") : F("RESTORE FAILED: power-cycle to reload saved rates."));
  }
  while (true) { gps.poll(); delay(1); }
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
