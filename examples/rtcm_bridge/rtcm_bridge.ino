// Use a board with a spare hardware UART and accurate 460800-baud support.
// Module TX -> Serial1 RX; module RX <- Serial1 TX through suitable level shifting.
// Connect ground and power as required by the breakout. Bare LC29H uses 2.8 V I/O.
// SoftwareSerial and a 16 MHz AVR UART are unsuitable for the EA default baud.
// HILBERT: replace Serial1.begin(...) with Serial1.begin(460800, SERIAL_8N1, 8, 9).
#include <Adafruit_LC29H.h>
char firstBuffer[256], secondBuffer[256];
Adafruit_LC29H gps(firstBuffer, secondBuffer, sizeof(firstBuffer));

// Serial is a binary host connection: no status/debug text is printed here.
// Host -> module: raw correction stream. Module -> host: valid NMEA plus RTCM3.
// Feed this from your own base or NTRIP client. The library is not an NTRIP client.
// Configure role/output separately; forwarding alone cannot create an RTK fix.
// A 1029-byte buffer fits any RTCM3 packet; use a board with enough RAM.
uint8_t rtcmBuffer[1029];
uint8_t corrections[64];
size_t pending = 0, offset = 0;
bool ready = false;
uint32_t lastAttempt = 0;
void received(const nmea_sentence_t& sentence, void* context);
void receivedRTCM(const uint8_t* packet, size_t length, void* context);
void setup() {
  Serial.begin(460800);
  Serial1.begin(460800);
  gps.onSentence(received);
  gps.onRTCM(rtcmBuffer, sizeof(rtcmBuffer), receivedRTCM);
}
void loop() {
  if (!ready) {
    if (millis() - lastAttempt < 3000) return;
    lastAttempt = millis();
    ready = gps.begin(Serial1);
    if (!ready) return;
  }
  gps.poll(256);
  // Finish any short write before consuming another chunk or issuing commands.
  if (offset < pending) {
    offset += gps.writeCorrections(corrections + offset, pending - offset);
    return;
  }
  offset = pending = 0;
  while (pending < sizeof(corrections) && Serial.available()) {
    int byte = Serial.read();
    if (byte >= 0) corrections[pending++] = (uint8_t)byte;
  }
}
void received(const nmea_sentence_t& sentence, void* context) {
  (void)context;
  if (sentence.status == NMEA_FRAME_VALID)
    Serial.write((const uint8_t*)sentence.text.data, sentence.text.length);
}
void receivedRTCM(const uint8_t* packet, size_t length, void* context) {
  (void)context;
  // The host must continuously drain output. Slow/blocked USB loses UART data.
  Serial.write(packet, length);
}
