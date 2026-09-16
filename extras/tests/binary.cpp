#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <vector>

#include "Adafruit_LC29H.h"
static uint32_t crc(const std::vector<uint8_t>& data) {
  uint32_t value = 0;
  for (uint8_t byte : data) {
    value ^= (uint32_t)byte << 16;
    for (int i = 0; i < 8; i++)
      value = (value << 1) ^ ((value & 0x800000) ? 0x1864CFB : 0);
  }
  return value & 0xFFFFFF;
}
static std::vector<uint8_t> packet(size_t length) {
  std::vector<uint8_t> data = {0xD3, (uint8_t)(length >> 8), (uint8_t)length};
  for (size_t i = 0; i < length; i++)
    data.push_back((uint8_t)(i * 37));
  uint32_t checksum = crc(data);
  data.push_back(checksum >> 16);
  data.push_back(checksum >> 8);
  data.push_back(checksum);
  return data;
}
static std::string line() {
  char text[80];
  const char* body = "PAIR001,050,0";
  Adafruit_NMEA::buildCommand(text, sizeof(text), body, strlen(body));
  return text;
}
struct Counts {
  int packets = 0;
  int lines = 0;
  size_t length = 0;
};
static void binary(const uint8_t* data, size_t length, void* context) {
  Counts& count = *(Counts*)context;
  count.packets++;
  count.length = length;
  assert(data[0] == 0xD3 &&
         crc(std::vector<uint8_t>(data, data + length)) == 0);
}
static void text(const nmea_sentence_t& sentence, void* context) {
  Counts& count = *(Counts*)context;
  count.lines++;
  assert(sentence.status == NMEA_FRAME_VALID);
}
int main() {
  // CRC24Q published check value for ASCII 123456789.
  assert(crc({'1', '2', '3', '4', '5', '6', '7', '8', '9'}) == 0xCDE703);
  char a[128], b[128];
  uint8_t buffer[1029];
  Adafruit_LC29H gps(a, b, sizeof(a));
  Counts counts;
  gps.onRTCM(buffer, sizeof(buffer), binary, &counts);
  gps.onSentence(text, &counts);
  uint32_t time = UINT32_MAX - 100;
  for (size_t length : {0, 1, 64, 255, 256, 1023}) {
    auto data = packet(length);
    for (uint8_t byte : data)
      assert(gps.feed(byte, time++) == NMEA_FRAME_INCOMPLETE);
    assert(counts.length == length + 6);
    for (char byte : line())
      gps.feed(byte, time++);
  }
  assert(counts.packets == 6 && counts.lines == 6);
  // A complete, valid NMEA command inside an RTCM payload must stay binary.
  std::string embedded = line();
  std::vector<uint8_t> data = {0xD3, 0, (uint8_t)embedded.size()};
  for (char byte : embedded)
    data.push_back(byte);
  uint32_t checksum = crc(data);
  data.push_back(checksum >> 16);
  data.push_back(checksum >> 8);
  data.push_back(checksum);
  for (uint8_t byte : data)
    gps.feed(byte, time++);
  assert(counts.packets == 7 && counts.lines == 6);
  data[5] ^= 1;
  for (uint8_t byte : data)
    gps.feed(byte, time++);
  assert(counts.packets == 7 && counts.lines == 6);
  for (char byte : line())
    gps.feed(byte, time++);
  assert(counts.lines == 7);
  gps.onRTCM(buffer, 10, binary, &counts);
  for (uint8_t byte : packet(128))
    gps.feed(byte, time++);
  assert(counts.packets == 7);
  for (char byte : line())
    gps.feed(byte, time++);
  assert(counts.lines == 8);
  puts(
      "PASS: RTCM CRC24Q, all payload lengths, wraparound, embedded NMEA, bad "
      "CRC, small buffers");
  gps.feed(0xD3, time++);
  gps.feed(0, time++);
  gps.feed(50, time++);
  gps.feed(1, time++);
  time += 251;
  for (char byte : line())
    gps.feed(byte, time++);
  assert(counts.lines == 9);
  gps.feed(0xD3, time++);
  for (char byte : line())
    gps.feed(byte, time++); // '$' violates reserved header bits and resyncs.
  assert(counts.lines == 10);
  gps.feed(0xD3, time++);
  gps.feed(0, time++);
  gps.reset();
  for (char byte : line())
    gps.feed(byte, time++);
  assert(counts.lines == 11);
  for (char byte : std::string("$PAIR001,"))
    gps.feed(byte, time++);
  for (uint8_t byte : packet(12))
    gps.feed(byte, time++);
  for (char byte : std::string("050,0*3E\r\n"))
    gps.feed(byte, time++);
  assert(counts.lines == 11);
  for (char byte : line())
    gps.feed(byte, time++);
  assert(counts.lines == 12);
  gps.onRTCM(NULL, 0, NULL);
  for (uint8_t byte : packet(300))
    gps.feed(byte, time++);
  for (char byte : line())
    gps.feed(byte, time++);
  assert(counts.lines == 13);
  puts(
      "PASS: truncated RTCM gap recovery, invalid headers, parser reset, "
      "text/binary interruption, discard mode");

  gps.onRTCM(buffer, sizeof(buffer), binary, &counts);
  auto complete = packet(32);
  gps.feed(0xD3, time++);
  for (uint8_t byte : complete)
    gps.feed(byte, time++);
  int before = counts.packets;
  assert(before == 8);
  for (size_t i = 0; i < 5; i++)
    gps.feed(complete[i], time++);
  uint8_t replacement[1029];
  memset(replacement, 0xAA, sizeof(replacement));
  gps.onRTCM(replacement, sizeof(replacement), binary, &counts);
  for (size_t i = 5; i < complete.size(); i++)
    gps.feed(complete[i], time++);
  assert(counts.packets == before && replacement[0] == 0xAA);
  for (uint8_t byte : complete)
    gps.feed(byte, time++);
  assert(counts.packets == before + 1 && replacement[0] == 0xD3);
  puts(
      "PASS: overlapping RTCM preambles and replacing storage during a packet");
}
