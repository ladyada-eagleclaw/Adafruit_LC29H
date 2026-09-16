#include <Arduino.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <string>

#include "Adafruit_LC29H.h"
static std::string frame(const std::string& body) {
  char text[512];
  size_t length = Adafruit_NMEA::buildCommand(text, sizeof(text), body.c_str(),
                                              body.size());
  assert(length);
  return std::string(text, length);
}
class Port : public Stream {
 public:
  std::string expected, answer, received;
  size_t position = 0;
  int calls = 0;
  int available() override {
    return received.size() - position;
  }
  int read() override {
    return position < received.size() ? (uint8_t)received[position++] : -1;
  }
  size_t write(const uint8_t* data, size_t size) override {
    assert(std::string((const char*)data, size) == frame(expected));
    calls++;
    received = answer;
    position = 0;
    return size;
  }
  void expect(const std::string& body, const std::string& reply) {
    assert(position == received.size());
    expected = body;
    answer = reply;
  }
  void pair(const std::string& body, const char* id,
            const std::string& reply = "") {
    expect(body, frame(std::string("PAIR001,") + id + ",0") +
                     (reply.empty() ? "" : frame(reply)));
  }
  void pqtm(const std::string& body, const std::string& reply) {
    expect(body, frame(reply));
  }
};
int main() {
  char a[256], b[256];
  Adafruit_LC29H gps(a, b, sizeof(a));
  Port port;
  port.pqtm("PQTMVERNO", "PQTMVERNO,EA,2026/09/16,12:34:56");
  assert(gps.begin(port));
  port.pair("PAIR050,100", "050");
  assert(gps.setFixInterval(100));
  port.pair("PAIR051", "051", "PAIR051,100");
  assert(gps.getFixInterval() == 100);
  port.pair("PAIR051", "051", "PAIR051,99");
  assert(gps.getFixInterval() == -1);
  port.pair("PAIR062,4,0", "062");
  assert(gps.enableNMEA(LC29H_NMEA_RMC, false));
  port.pair("PAIR063,4", "063", "PAIR063,4,0");
  assert(gps.isNMEAEnabled(LC29H_NMEA_RMC) == 0);
  port.pair("PAIR063,4", "063", "PAIR063,0,1");
  assert(gps.isNMEAEnabled(LC29H_NMEA_RMC) == -1);
  for (auto mode : {LC29H_NAV_NORMAL, LC29H_NAV_FITNESS, LC29H_NAV_STATIONARY,
                    LC29H_NAV_DRONE, LC29H_NAV_SWIMMING, LC29H_NAV_BIKE}) {
    port.pair("PAIR080," + std::to_string(mode), "080");
    assert(gps.setNavigationMode(mode));
    port.pair("PAIR081", "081", "PAIR081," + std::to_string(mode));
    assert(gps.getNavigationMode() == mode);
  }
  port.pair("PAIR081", "081", "PAIR081,2");
  assert(gps.getNavigationMode() == LC29H_NAV_UNKNOWN);
  for (bool enabled : {false, true}) {
    std::string value = enabled ? "1" : "0";
    port.pair("PAIR074," + value, "074");
    assert(gps.enableInterferenceCancellation(enabled));
    port.pair("PAIR075", "075", "PAIR075," + value);
    assert(gps.isInterferenceCancellationEnabled() == enabled);
    port.pair("PAIR434," + value, "434");
    assert(gps.enableReferenceStationOutput(enabled));
    port.pair("PAIR435", "435", "PAIR435," + value);
    assert(gps.isReferenceStationOutputEnabled() == enabled);
    port.pair("PAIR436," + value, "436");
    assert(gps.enableEphemerisOutput(enabled));
    port.pair("PAIR437", "437", "PAIR437," + value);
    assert(gps.isEphemerisOutputEnabled() == enabled);
  }
  for (auto mode : {LC29H_RTCM_DISABLED, LC29H_RTCM_MSM4, LC29H_RTCM_MSM7}) {
    port.pair("PAIR432," + std::to_string(mode), "432");
    assert(gps.setRTCMMode(mode));
    port.pair("PAIR433", "433", "PAIR433," + std::to_string(mode));
    assert(gps.getRTCMMode() == mode);
  }
  for (uint32_t baud : {4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800,
                        921600, 3000000}) {
    port.pair("PAIR864,0,0," + std::to_string(baud), "864");
    assert(gps.setBaudrate(baud));
    port.pair("PAIR865,0,0", "865", "PAIR865," + std::to_string(baud));
    assert(gps.getBaudrate() == (int32_t)baud);
  }
  int before = port.calls;
  assert(!gps.setBaudrate(12345) && port.calls == before);
  puts(
      "PASS: PAIR configuration packets, real query decoders, every "
      "navigation/RTCM mode and UART baud");
  lc29h_precision_t precision = {3, 8, 3, 2, 3, 2};
  port.pqtm("PQTMCFGNMEADP,W,3,8,3,2,3,2", "PQTMCFGNMEADP,OK");
  assert(gps.setPrecision(precision));
  port.pqtm("PQTMCFGNMEADP,R", "PQTMCFGNMEADP,OK,3,8,3,2,3,2");
  assert(gps.getPrecision(precision));
  for (auto mode : {LC29H_MODE_ROVER, LC29H_MODE_BASE}) {
    port.pqtm("PQTMCFGRCVRMODE,W," + std::to_string(mode),
              "PQTMCFGRCVRMODE,OK");
    assert(gps.setReceiverMode(mode));
    port.pqtm("PQTMCFGRCVRMODE,R",
              "PQTMCFGRCVRMODE,OK," + std::to_string(mode));
    assert(gps.getReceiverMode() == mode);
  }
  lc29h_survey_config_t config = {2,
                                  0,
                                  {NMEA_NUMBER_VALID, 0, 0},
                                  {NMEA_NUMBER_VALID, -24724464619LL, 4},
                                  {NMEA_NUMBER_VALID, 48283041363LL, 4},
                                  {NMEA_NUMBER_VALID, 33437302653LL, 4}};
  port.pqtm("PQTMCFGSVIN,W,2,0,0,-2472446.4619,4828304.1363,3343730.2653",
            "PQTMCFGSVIN,OK");
  assert(gps.setSurvey(config));
  port.pqtm("PQTMCFGSVIN,R",
            "PQTMCFGSVIN,OK,2,0,0.0,-2472446.4619,4828304.1363,3343730.2653");
  assert(gps.getSurvey(config) && config.x.coefficient == -24724464619LL &&
         config.x.decimalPlaces == 4);
  port.pqtm("PQTMCFGSVIN,R",
            "PQTMCFGSVIN,OK,2,0,0.0,-2472446.4619,4828304.1363,broken");
  assert(!gps.getSurvey(config) && config.z.coefficient == 33437302653LL);
  before = port.calls;
  config.accuracy.coefficient = -1;
  assert(!gps.setSurvey(config));
  assert(port.calls == before);
  port.pqtm("PQTMCFGMSGRATE,W,PQTMPVT,1,1", "PQTMCFGMSGRATE,OK");
  assert(gps.setMessageRate("PQTMPVT", 1, 1));
  port.pqtm("PQTMCFGMSGRATE,R,PQTMPVT,1", "PQTMCFGMSGRATE,OK,PQTMPVT,1,1");
  assert(gps.getMessageRate("PQTMPVT", 1) == 1);
  port.pqtm("PQTMCFGMSGRATE,R,PQTMPVT,1", "PQTMCFGMSGRATE,OK,PQTMPVT,1,2");
  assert(gps.getMessageRate("PQTMPVT", 1) == -1);
  for (const char* reply :
       {"PQTMCFGMSGRATE,OK,GGA,1", "PQTMCFGMSGRATE,OK,GGA,1,"}) {
    port.pqtm("PQTMCFGMSGRATE,R,GGA", reply);
    assert(gps.getMessageRate("GGA") == 1);
  }
  port.pqtm("PQTMCFGMSGRATE,R,GGA", "PQTMCFGMSGRATE,OK,RMC,1,");
  assert(gps.getMessageRate("GGA") == -1);
  port.pqtm("PQTMCFGMSGRATE,R,GGA", "PQTMCFGMSGRATE,OK,GGA,1,extra");
  assert(gps.getMessageRate("GGA") == -1);
  puts(
      "PASS: exact ECEF set/readback, atomic failure, PQTM precision, roles "
      "and message identity/version");
  port.pqtm("PQTMSAVEPAR", "PQTMSAVEPAR,OK");
  assert(gps.saveParameters());
  port.pqtm("PQTMRESTOREPAR", "PQTMRESTOREPAR,OK");
  assert(gps.restoreParameters());
  port.pair("PAIR513", "513");
  assert(gps.savePairSettings());
  for (auto mode : {LC29H_START_HOT, LC29H_START_WARM, LC29H_START_COLD,
                    LC29H_START_FULL_COLD}) {
    char id[4];
    snprintf(id, sizeof(id), "%03d", mode);
    port.pair(std::string("PAIR") + id, id);
    assert(gps.restart(mode));
  }
  puts(
      "PASS: explicit persistence and restart commands (simulated transport; "
      "no hardware reset)");

  char shared[100] = "PQTMCFGRCVRMODE,R";
  port.pqtm(shared, "PQTMCFGRCVRMODE,OK,1");
  assert(gps.sendPQTMCommand(shared, shared, sizeof(shared)));
  assert(std::string(shared) == frame("PQTMCFGRCVRMODE,OK,1"));
  port.pair("PAIR865,0,0", "865", "PAIR865,12345");
  assert(gps.getBaudrate() == -1 &&
         gps.commandStatus() == LC29H_COMMAND_BAD_REPLY);
  puts(
      "PASS: reusable command/response buffer and unsupported wire baud "
      "rejected");
}
