#include <Arduino.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <deque>
#include <functional>
#include <string>
#include <vector>

#include "Adafruit_LC29H.h"

static std::string frame(const std::string& body) {
  char text[600];
  size_t length = Adafruit_NMEA::buildCommand(text, sizeof(text), body.c_str(),
                                              body.size());
  assert(length);
  return std::string(text, length);
}
static std::string nav() {
  return frame(
      "GNGGA,123456.000,4807.12345678,N,01131.98765432,E,4,19,0.7,12.345,M,4."
      "567,M,0.5,42");
}
struct FakePort : Stream {
  std::deque<uint8_t> input;
  std::vector<std::string> writes;
  std::function<std::string(const std::string&)> reply;
  size_t writeLimit = SIZE_MAX;
  bool endless = false;
  size_t endlessIndex = 0;
  void queue(const std::string& text) {
    for (unsigned char c : text)
      input.push_back(c);
  }
  int available() override {
    return input.empty() ? (endless ? 1 : 0) : (int)input.size();
  }
  int read() override {
    if (!input.empty()) {
      int byte = input.front();
      input.pop_front();
      return byte;
    }
    if (endless) {
      std::string text = nav();
      return text[endlessIndex++ % text.size()];
    }
    return -1;
  }
  size_t write(const uint8_t* data, size_t length) override {
    size_t count = length < writeLimit ? length : writeLimit;
    std::string text((const char*)data, count);
    writes.push_back(text);
    if (reply && count == length)
      queue(reply(text));
    return count;
  }
};
struct Consumer {
  Adafruit_LC29H* gps;
  size_t lines = 0;
  size_t fixes = 0;
  bool reenter = false;
};
static void consume(const nmea_sentence_t& sentence, void* context) {
  Consumer& state = *(Consumer*)context;
  state.lines++;
  lc29h_fix_t fix = Adafruit_LC29H::parseFix(sentence);
  if (fix.position.validation.status == GNSS_SENTENCE_VALID &&
      fix.position.fix) {
    assert(fix.position.latitude.fractionalMinutes == 123456780);
    assert(fix.altitude.coefficient == 12345 &&
           fix.altitude.decimalPlaces == 3);
    state.fixes++;
  }
  if (state.reenter) {
    assert(!state.gps->sendPairCommand(4));
    assert(state.gps->poll() == 0);
    uint8_t byte = 1;
    assert(state.gps->writeCorrections(&byte, 1) == 0);
    state.gps->reset(); // Must not invalidate this callback's sentence.
    assert(sentence.status == NMEA_FRAME_VALID);
  }
}
int main() {
  char a[256], b[256], response[256];
  Adafruit_LC29H gps(a, b, sizeof(a));
  FakePort port;
  assert(!gps.sendPairCommand(4) &&
         gps.commandStatus() == LC29H_COMMAND_NO_PORT);
  port.reply = [](const std::string& query) {
    assert(query == frame("PQTMVERNO"));
    return frame("PQTMVERNO") + nav() +
           frame("PQTMVERNO,LC29HEA_TEST,2026/09/16,12:34:56");
  };
  Consumer consumer;
  consumer.gps = &gps;
  gps.onSentence(consume, &consumer);
  assert(gps.begin(port));
  assert(consumer.fixes == 1);
  puts(
      "PASS: initialized Stream, version validation, exact navigation callback "
      "during begin");
  for (bool reverse : {false, true}) {
    port.reply = [reverse](const std::string& query) {
      assert(query == frame("PAIR051"));
      std::string ack = frame("PAIR001,051,0"), data = frame("PAIR051,1000");
      return frame("PAIR001,052,0") + frame("PAIR001,051,1") + nav() +
             (reverse ? data + ack : ack + data);
    };
    consumer.reenter = true;
    size_t writes = port.writes.size();
    assert(gps.getFixInterval() == 1000);
    assert(port.writes.size() == writes + 1);
    consumer.reenter = false;
  }
  puts(
      "PASS: matching IDs, PROCESSING, both reply orders, callback reentry "
      "blocked");
  port.reply = [](const std::string&) { return frame("PAIR001,051,0"); };
  gps.setCommandTimeout(200);
  assert(gps.getFixInterval() == -1 &&
         gps.commandStatus() == LC29H_COMMAND_TIMEOUT);
  port.reply = [](const std::string&) { return frame("PAIR051,1000"); };
  assert(gps.getFixInterval() == -1 &&
         gps.commandStatus() == LC29H_COMMAND_TIMEOUT);
  for (int code = 2; code <= 5; code++) {
    port.reply = [code](const std::string&) {
      return frame("PAIR001,004," + std::to_string(code));
    };
    assert(!gps.restart());
    assert(gps.commandStatus() == LC29H_COMMAND_REJECTED);
    assert(gps.commandPairResult() == code);
  }
  port.reply = [](const std::string&) { return frame("PAIR001,004,1"); };
  testClock() = UINT32_MAX - 50;
  uint32_t start = testClock();
  assert(!gps.restart());
  assert(gps.commandStatus() == LC29H_COMMAND_TIMEOUT &&
         gps.commandPairResult() == LC29H_PAIR_PROCESSING);
  assert((uint32_t)(testClock() - start) < 250);
  puts(
      "PASS: data and ACK both required, every rejection, fixed deadline and "
      "millisecond wrap");
  // Already queued replies must not satisfy a new command.
  port.queue(frame("PAIR001,004,0"));
  port.reply = nullptr;
  assert(!gps.restart() && gps.commandStatus() == LC29H_COMMAND_TIMEOUT);
  port.queue("$PAIR001,004,");
  assert(gps.poll() != 0);
  port.reply = [](const std::string&) { return std::string("0*39\r\n"); };
  assert(!gps.restart());
  // An endless receive stream cannot hold either the pre-drain or a wait
  // forever.
  port.reply = nullptr;
  port.endless = true;
  size_t writes = port.writes.size();
  start = testClock();
  assert(!gps.restart());
  assert(port.writes.size() == writes && (uint32_t)(testClock() - start) < 80);
  port.endless = false;
  port.reply = [&port](const std::string&) {
    port.endless = true;
    return std::string();
  };
  start = testClock();
  assert(!gps.restart());
  assert((uint32_t)(testClock() - start) < 250);
  port.endless = false;
  puts(
      "PASS: stale and partial pre-command replies excluded; endless traffic "
      "remains bounded");
  gps.setCommandTimeout(1500);
  port.reply = [](const std::string&) {
    return frame("PQTMCFGNMEADP,ERROR,3");
  };
  lc29h_precision_t precision = {3, 8, 3, 2, 3, 2};
  assert(!gps.setPrecision(precision) && gps.commandError() == 3);
  assert(gps.commandStatus() == LC29H_COMMAND_REJECTED);
  port.reply = [](const std::string&) {
    return frame("PQTMCFGNMEADP,ERROR,0");
  };
  assert(!gps.setPrecision(precision) &&
         gps.commandStatus() == LC29H_COMMAND_BAD_REPLY);
  port.reply = [](const std::string&) {
    return frame("PQTMCFGNMEADP,OK,unexpected");
  };
  assert(!gps.setPrecision(precision));
  port.reply = [](const std::string&) {
    return frame("PQTMCFGNMEADP,OK,3,8,3,2,3,2");
  };
  assert(gps.getPrecision(precision) && precision.position == 8);
  port.reply = [](const std::string&) {
    return frame("PQTMCFGNMEADP,OK,3,8,3,2,3,9");
  };
  assert(!gps.getPrecision(precision) && precision.course == 2);
  assert(gps.commandStatus() == LC29H_COMMAND_BAD_REPLY);
  port.reply = [](const std::string&) {
    return frame("PQTMVERNO,EA,2026/09/16,00:00:00");
  };
  memset(response, 'x', sizeof(response));
  assert(!gps.getVersion(response, 10));
  assert(!response[0]);
  port.writeLimit = 3;
  assert(!gps.restart() && gps.commandStatus() == LC29H_COMMAND_WRITE_FAILED);
  port.writeLimit = SIZE_MAX;
  writes = port.writes.size();
  assert(!gps.sendPairCommand(1000));
  assert(!gps.sendPairCommand(50, "1000\r\n$PAIR007"));
  assert(!gps.setFixInterval(99));
  precision.position = 9;
  assert(!gps.setPrecision(precision));
  assert(!gps.setMessageRate("GGA,1", 1));
  assert(gps.getMessageRate("GGA*00") == -1);
  assert(port.writes.size() == writes);
  puts(
      "PASS: PQTM errors, exact field shape, atomic readback, overflow and "
      "rejected command injection");
  port.reply = nullptr;
  const uint8_t correction[] = {0xD3, 0, 3, '$', '\n', 0, 0, 0, 0};
  port.writeLimit = 4;
  assert(gps.writeCorrections(correction, sizeof(correction)) == 4);
  assert(port.writes.back() == std::string((const char*)correction, 4));
  port.writeLimit = SIZE_MAX;
  port.queue(nav() + nav());
  size_t before = consumer.fixes;
  assert(gps.poll(1) == 1 && consumer.fixes == before);
  assert(gps.poll(1000) > 0);
  assert(consumer.fixes == before + 2);

  // A forged ACK inside a valid RTCM packet must not satisfy a command wait.
  std::string payload = frame("PAIR001,004,0");
  std::string rtcm;
  rtcm.push_back((char)0xD3);
  rtcm.push_back(0);
  rtcm.push_back((char)payload.size());
  rtcm += payload;
  uint32_t crc = 0;
  for (uint8_t byte : rtcm) {
    crc ^= (uint32_t)byte << 16;
    for (uint8_t bit = 0; bit < 8; bit++)
      crc = (crc << 1) ^ ((crc & 0x800000) ? 0x1864CFB : 0);
  }
  rtcm.push_back(crc >> 16);
  rtcm.push_back(crc >> 8);
  rtcm.push_back(crc);
  port.reply = [rtcm](const std::string&) { return rtcm; };
  assert(!gps.restart() && gps.commandStatus() == LC29H_COMMAND_TIMEOUT);
  port.reply = [rtcm](const std::string&) {
    return rtcm + frame("PAIR001,004,0");
  };
  assert(gps.restart());
  puts(
      "PASS: RTCM-embedded acknowledgment cannot complete a command "
      "transaction");
  gps.end();
  assert(gps.poll() == 0 &&
         gps.writeCorrections(correction, sizeof(correction)) == 0);
  puts(
      "PASS: raw binary correction bytes, partial-write accounting, bounded "
      "polling and detach");
}
