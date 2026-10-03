// LC29H BA with four-wheel firmware, using its internal accelerometer/gyro.
// Metro Mini/Uno: A4 -> SDA, A5 -> SCL, GND -> GND, 5V -> breakout VIN.
// This example disables standard NMEA on the current I2C port to make room
// for 10 Hz IMU data. No settings are saved to flash; power-cycle to restore.
// Module-frame readings work on the bench without a GNSS fix. Vehicle-frame
// readings require driving calibration and a 10 Hz navigation configuration.
#include <Adafruit_LC29H_BA.h>
#include <Adafruit_LC29H_I2C.h>

char firstBuffer[160], secondBuffer[160];
Adafruit_LC29H_BA gps(firstBuffer, secondBuffer, sizeof(firstBuffer));
Adafruit_LC29H_I2C gpsPort;
uint32_t lastPrint = 0;
void received(const nmea_sentence_t& sentence, void* context);
void halt(const __FlashStringHelper* message);

void setup() {
  Serial.begin(115200);
  // Wait for Serial Monitor on native USB boards; remove for unattended use.
  while (!Serial) delay(10);
  delay(250);

  Serial.println(F("Adafruit LC29H BA six-axis IMU example"));

  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000, true);
#endif
  if (!gpsPort.begin() && !gpsPort.recover()) halt(F("Check I2C wiring and breakout VIN power."));
  if (!gps.begin(gpsPort, 30000)) halt(F("No firmware reply. Reset the breakout and retry."));

  // Standard sentences can exceed the AVR's available I2C bandwidth alongside
  // IMU data. This is a sensor-only example, so disable them on this port.
  // Disable the large satellite inventory first.
  const char* messages[] = {"GSV", "GSA", "GLL", "VTG", "RMC", "GGA"};
  for (uint8_t i = 0; i < 6; i++) {
    if (!gps.setMessageRate(messages[i], 0)) halt(F("Could not select sensor-only output."));
  }
  // Raw module axes: X/Y/Z gyro and accelerometer. Rate is Hz, not a divisor.
  if (!gps.setIMURate(10)) halt(F("IMU output was not accepted by this firmware."));
  gps.onSentence(received);
  Serial.println(F("Acceleration: m/s^2; angular velocity: rad/s; temperature: C."));
}

void loop() {
  gps.poll();
}

void received(const nmea_sentence_t& sentence, void* context) {
  (void)context;
  lc29h_imu_t imu = gps.parseIMU(sentence);
  if (imu.status != LC29H_REPLY_VALID || millis() - lastPrint < 500) return;
  lastPrint = millis();
  Serial.print(F("Acceleration:"));
  for (uint8_t i = 0; i < 3; i++) {
    Serial.print('\t'); Serial.print(imu.acceleration[i], 3);
  }
  Serial.print(F("\tGyro:"));
  for (uint8_t i = 0; i < 3; i++) {
    Serial.print('\t'); Serial.print(imu.angularVelocity[i], 4);
  }
  Serial.print(F("\tTemperature: ")); Serial.println(imu.temperature, 2);
}

void halt(const __FlashStringHelper* message) {
  Serial.println(message);
  while (true) delay(100);
}
