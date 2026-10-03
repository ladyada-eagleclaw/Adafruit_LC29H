#include <Arduino.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <deque>
#include <functional>
#include <string>

#include "Adafruit_LC29H_BA.h"

static std::string frame(const std::string& body) {
  char line[256];
  size_t size =
      Adafruit_NMEA::buildCommand(line, sizeof(line), body.data(), body.size());
  assert(size);
  return std::string(line, size);
}
static lc29h_imu_t imu(const char* body) {
  std::string text = frame(body);
  return Adafruit_LC29H_BA::parseIMU(
      Adafruit_NMEA::validate(text.data(), text.size()));
}
static lc29h_calibration_t calibration(const char* body) {
  std::string text = frame(body);
  return Adafruit_LC29H_BA::parseCalibration(
      Adafruit_NMEA::validate(text.data(), text.size()));
}
static lc29h_wheel_ticks_t ticks(const char* body) {
  std::string text = frame(body);
  return Adafruit_LC29H_BA::parseWheelTicks(
      Adafruit_NMEA::validate(text.data(), text.size()));
}
struct Port : Stream {
  std::deque<uint8_t> input;
  std::function<std::string(const std::string&)> reply;
  unsigned writes = 0;
  int available() override {
    return (int)input.size();
  }
  int read() override {
    if (input.empty())
      return -1;
    int byte = input.front();
    input.pop_front();
    return byte;
  }
  size_t write(const uint8_t* data, size_t length) override {
    writes++;
    std::string response = reply(std::string((const char*)data, length));
    input.insert(input.end(), response.begin(), response.end());
    return length;
  }
};

int main() {
  static_assert(sizeof(Adafruit_LC29H_BA) == sizeof(Adafruit_LC29H),
                "BA adds no hidden navigation or IMU cache");
  lc29h_imu_t sample = imu("PQTMSENMSG,2,4294967295,25.5,180,-90,0,1,-1,0.5");
  assert(sample.status == LC29H_REPLY_VALID && sample.timestamp == UINT32_MAX);
  assert(sample.frame == LC29H_IMU_MODULE && sample.temperature == 25.5f);
  assert(fabs(sample.angularVelocity[0] - 3.14159265f) < 0.000001f);
  assert(fabs(sample.angularVelocity[1] + 1.57079633f) < 0.000001f);
  assert(fabs(sample.acceleration[0] - 9.80665f) < 0.00001f);
  assert(fabs(sample.acceleration[1] + 9.80665f) < 0.00001f);
  assert(fabs(sample.acceleration[2] - 4.903325f) < 0.00001f);
  sample = imu("PQTMSENMSG,4,0,-10.25,0,0,0,0,0,1");
  assert(sample.status == LC29H_REPLY_VALID &&
         sample.frame == LC29H_IMU_VEHICLE);
  puts(
      "PASS: BA six-axis SI conversion, distinct coordinate frames, full "
      "32-bit time");

  for (const char* body :
       {"PQTMSENMSG,2,1,25,1,2,3,4,5", "PQTMSENMSG,2,1,25,1,2,3,4,5,6,7",
        "PQTMSENMSG,2,4294967296,25,1,2,3,4,5,6",
        "PQTMSENMSG,2,-1,25,1,2,3,4,5,6", "PQTMSENMSG,2,+1,25,1,2,3,4,5,6",
        "PQTMSENMSG,2,1.0,25,1,2,3,4,5,6", "PQTMSENMSG,3,1,25,1,2,3,4,5,6",
        "PQTMSENMSG,2,1,25,1,2,3,4,5,NaN", "PQTMSENMSG,2,1,25,1,2,3,4,5,1e3",
        "PQTMSENMSG,2,1,25,1,2,3,4,5,",
        "PQTMSENMSG,2,1,25,1,2,3,4,5,9223372036854775808"}) {
    sample = imu(body);
    assert(sample.status == LC29H_REPLY_INVALID_FIELDS);
    assert(sample.timestamp == 0 && sample.temperature == 0);
    for (int i = 0; i < 3; i++)
      assert(sample.acceleration[i] == 0 && sample.angularVelocity[i] == 0);
  }
  assert(imu("PQTMSENMSGX,2,1,25,1,2,3,4,5,6").status ==
         LC29H_REPLY_UNSUPPORTED);
  std::string corrupt = frame("PQTMSENMSG,2,1,25,1,2,3,4,5,6");
  corrupt[15] = '!';
  assert(Adafruit_LC29H_BA::parseIMU(
             Adafruit_NMEA::validate(corrupt.data(), corrupt.size()))
             .status == LC29H_REPLY_INVALID_FRAME);
  puts(
      "PASS: BA truncated/extra fields, invalid numbers, timestamp overflow "
      "and atomic failure");

  for (unsigned state = 0; state < 4; state++) {
    for (unsigned source = 0; source < 4; source++) {
      std::string body =
          "PQTMDRCAL,1," + std::to_string(state) + "," + std::to_string(source);
      auto value = calibration(body.c_str());
      assert(value.status == LC29H_REPLY_VALID && value.calibration == state &&
             value.source == source);
    }
  }
  assert(calibration("PQTMDRCAL,1,4,1").status == LC29H_REPLY_INVALID_FIELDS);
  assert(calibration("PQTMDRCAL,1,2,4").status == LC29H_REPLY_INVALID_FIELDS);
  assert(calibration("PQTMDRCAL,2,2,1").status == LC29H_REPLY_INVALID_FIELDS);
  assert(calibration("PQTMDRCAL,1,2,1,").status == LC29H_REPLY_INVALID_FIELDS);
  auto wheel = ticks("PQTMVEHMSG,2,4294967295,4294967295,2");
  assert(wheel.status == LC29H_REPLY_VALID && wheel.timestamp == UINT32_MAX);
  assert(wheel.ticks == UINT32_MAX && wheel.direction == LC29H_WHEEL_BACKWARD);
  wheel = ticks("PQTMVEHMSG,2,123,100,1");
  assert(wheel.status == LC29H_REPLY_VALID &&
         wheel.direction == LC29H_WHEEL_FORWARD);
  assert(ticks("PQTMVEHMSG,2,0,0,0").status == LC29H_REPLY_VALID);
  for (const char* body :
       {"PQTMVEHMSG,2,0,-1,1", "PQTMVEHMSG,2,0,4294967296,1",
        "PQTMVEHMSG,2,0,1,3", "PQTMVEHMSG,2,0,1,1,0", "PQTMVEHMSG,1,0,1,1"}) {
    wheel = ticks(body);
    assert(wheel.status == LC29H_REPLY_INVALID_FIELDS && wheel.ticks == 0);
  }
  puts(
      "PASS: BA calibration/navigation sources and wheel count/direction "
      "reports");

  char a[160], b[160];
  Adafruit_LC29H_BA gps(a, b, sizeof(a));
  Port port;
  port.reply = [](const std::string& command) {
    assert(command == frame("PQTMVERNO"));
    return frame("PQTMVERNO,LC29HBANR11A06S_CSA4,2025/05/28,09:36:24");
  };
  assert(gps.begin(port));
  for (int rate : {0, 10, 20, 50, 100}) {
    port.reply = [rate](const std::string& command) {
      assert(command == frame("PQTMCFGMSGRATE,W,PQTMSENMSG," +
                              std::to_string(rate) + ",2"));
      return frame("PQTMCFGMSGRATE,OK");
    };
    assert(gps.setIMURate(rate));
    port.reply = [rate](const std::string& command) {
      assert(command == frame("PQTMCFGMSGRATE,R,PQTMSENMSG,2"));
      return frame("PQTMCFGMSGRATE,OK,PQTMSENMSG," + std::to_string(rate) +
                   ",2");
    };
    assert(gps.getIMURate() == rate);
  }
  unsigned writes = port.writes;
  assert(!gps.setIMURate(1) && !gps.setIMURate(99));
  assert(!gps.setMessageRate("PQTMDRCAL", 50, 1));
  assert(port.writes == writes);
  for (const char* body : {"PQTMCFGMSGRATE,OK,PQTMSENMSG,11,2",
                           "PQTMCFGMSGRATE,OK,PQTMSENMSG,100,4",
                           "PQTMCFGMSGRATE,OK,PQTMDRCAL,10,2"}) {
    port.reply = [body](const std::string&) { return frame(body); };
    assert(gps.getIMURate() == -1 &&
           gps.commandStatus() == LC29H_COMMAND_BAD_REPLY);
  }
  port.reply = [](const std::string& command) {
    assert(command == frame("PQTMCFGDR,R"));
    return frame("PQTMCFGDR,OK,1");
  };
  assert(gps.isDeadReckoningEnabled() == 1);
  port.reply = [](const std::string&) { return frame("PQTMCFGDR,OK,2"); };
  assert(gps.isDeadReckoningEnabled() == -1);
  port.reply = [](const std::string&) { return frame("PQTMCFGDR,ERROR,3"); };
  assert(gps.isDeadReckoningEnabled() == -1 && gps.commandError() == 3);
  puts(
      "PASS: BA IMU Hz configuration/readback, version matching and DR query "
      "failures");
}
