// Metro Mini/Uno: A4 -> breakout SDA, A5 -> SCL, common GND.
// Breakout VIN must power the host side of its I2C level shifter.
// Use the assembled, level-shifted breakout, not a bare 2.8 V module.
// The USB/HOST UART switch may remain on USB; I2C is independent.
#include <Adafruit_LC29H.h>
#include <Adafruit_LC29H_I2C.h>

char firstBuffer[160], secondBuffer[160];
Adafruit_LC29H gps(firstBuffer, secondBuffer, sizeof(firstBuffer));
Adafruit_LC29H_I2C gpsPort;
uint16_t validLines = 0, badLines = 0, positions = 0;
uint32_t started;
void received(const nmea_sentence_t& sentence, void* context);
void halt(const __FlashStringHelper* message);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("Adafruit LC29H BA Metro Mini I2C test"));

  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000, true);
#endif
  if (!gpsPort.begin() && !gpsPort.recover()) halt(F("FAIL: I2C endpoints did not ACK"));
  Serial.println(F("PASS: I2C configuration address ACK"));
  gps.onSentence(received);
  // A board left running before its host may have a large sleep-state FIFO.
  if (!gps.begin(gpsPort, 30000)) {
    Serial.print(F("Command status: ")); Serial.println(gps.commandStatus());
    Serial.print(F("I2C error: ")); Serial.println(gpsPort.lastError());
    Serial.print(F("Received during identification: ")); Serial.println(validLines);
    halt(F("FAIL: I2C firmware identification"));
  }
  Serial.println(F("PASS: I2C firmware identification"));
  char version[100];
  if (!gps.getVersion(version, sizeof(version))) halt(F("FAIL: second identity query"));
  Serial.print(version);
  Serial.println(F("PASS: second identity query"));
  validLines = badLines = positions = 0;
  started = millis();
}

void loop() {
  gps.poll();
  if (millis() - started < 20000) return;
  Serial.print(F("Valid sentences: ")); Serial.println(validLines);
  Serial.print(F("Invalid sentences: ")); Serial.println(badLines);
  Serial.print(F("GGA records: ")); Serial.println(positions);
  if (gpsPort.lastError()) halt(F("FAIL: I2C transfer error"));
  if (badLines || validLines < 50 || positions < 10) halt(F("FAIL: sustained NMEA reception"));
  Serial.println(F("PASS: sustained NMEA reception"));
  Serial.println(F("All I2C tests passed. Continuing reception."));
  while (true) gps.poll();
}

void received(const nmea_sentence_t& sentence, void* context) {
  (void)context;
  if (sentence.status != NMEA_FRAME_VALID) {
    badLines++;
    return;
  }
  validLines++;
  if (sentence.address.length == 5 && !memcmp(sentence.address.data + 2, "GGA", 3)) {
    positions++;
    for (size_t i = 0; i < sentence.text.length; i++) Serial.write(sentence.text.data[i]);
  }
}

void halt(const __FlashStringHelper* message) {
  Serial.println(message);
  while (true) delay(100);
}
