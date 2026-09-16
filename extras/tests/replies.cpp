#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <initializer_list>

#include "Adafruit_LC29H.h"

static char line[256];
static nmea_sentence_t sentence(const char* body) {
  size_t size =
      Adafruit_NMEA::buildCommand(line, sizeof(line), body, strlen(body));
  assert(size);
  return Adafruit_NMEA::validate(line, size);
}

static void noAck(const char* body, lc29h_reply_status_t status) {
  lc29h_pair_ack_t ack = Adafruit_LC29H::parsePairAck(sentence(body));
  assert(ack.status == status && ack.commandID == 0 &&
         ack.result == LC29H_PAIR_UNKNOWN);
}

static void noVersion(const char* body, lc29h_reply_status_t status) {
  lc29h_version_t version = Adafruit_LC29H::parseVersion(sentence(body));
  assert(version.status == status && !version.errorCode);
  assert(!version.version.data && !version.version.length);
  assert(!version.buildDate.data && !version.buildDate.length);
  assert(!version.buildTime.data && !version.buildTime.length);
}

int main() {
  const char* results[] = {"PAIR001,062,0", "PAIR001,062,1", "PAIR001,062,2",
                           "PAIR001,062,3", "PAIR001,062,4", "PAIR001,062,5"};
  for (unsigned i = 0; i < 6; i++) {
    lc29h_pair_ack_t ack = Adafruit_LC29H::parsePairAck(sentence(results[i]));
    assert(ack.status == LC29H_REPLY_VALID && ack.commandID == 62 &&
           ack.result == i);
  }
  lc29h_pair_ack_t ack =
      Adafruit_LC29H::parsePairAck(sentence("PAIR001,999,000"));
  assert(ack.status == LC29H_REPLY_VALID && ack.commandID == 999 &&
         ack.result == LC29H_PAIR_ACCEPTED);
  ack = Adafruit_LC29H::parsePairAck(sentence("PAIR001,000,5"));
  assert(ack.status == LC29H_REPLY_VALID && !ack.commandID &&
         ack.result == LC29H_PAIR_BUSY);
  const char* badAck[] = {"PAIR001",
                          "PAIR001,",
                          "PAIR001,62",
                          "PAIR001,,0",
                          "PAIR001,62,",
                          "PAIR001,62,0,",
                          "PAIR001,62,0,1",
                          "PAIR001,-1,0",
                          "PAIR001,+62,0",
                          "PAIR001,6.2,0",
                          "PAIR001, 62,0",
                          "PAIR001,62,0 ",
                          "PAIR001,62,+0",
                          "PAIR001,62,-0",
                          "PAIR001,62,0.0",
                          "PAIR001,1000,0",
                          "PAIR001,62,6",
                          "PAIR001,62,255",
                          "PAIR001,4294967358,0",
                          "PAIR001,62,999999999999999999"};
  for (const char* body : badAck) {
    noAck(body, LC29H_REPLY_INVALID_FIELDS);
  }
  for (const char* body : {"PAIR001X,62,0", "PAIR01,62,0", "pair001,62,0",
                           "GPGLL,,,,,,V", "PQTMVERNO"}) {
    noAck(body, LC29H_REPLY_UNSUPPORTED);
  }
  puts(
      "PASS: all PAIR results, exact addresses, field counts, syntax, "
      "overflow");

  nmea_sentence_t frame = sentence("PQTMVERNO,LC29HEA,2026/09/16,12:00:00");
  lc29h_version_t version = Adafruit_LC29H::parseVersion(frame);
  assert(version.status == LC29H_REPLY_VALID && !version.errorCode);
  assert(version.version.length == 7 &&
         !memcmp(version.version.data, "LC29HEA", 7));
  assert(version.buildDate.length == 10 && version.buildTime.length == 8);
  assert(version.version.data >= frame.text.data &&
         version.version.data < frame.text.data + frame.text.length);
  for (const char* code : {"1", "2", "3", "004", "255"}) {
    char body[64];
    snprintf(body, sizeof(body), "PQTMVERNO,ERROR,%s", code);
    version = Adafruit_LC29H::parseVersion(sentence(body));
    assert(version.status == LC29H_REPLY_RECEIVER_ERROR && version.errorCode);
    assert(!version.version.data && !version.buildDate.data &&
           !version.buildTime.data);
  }
  version = Adafruit_LC29H::parseVersion(sentence("PQTMVERNO,ERROR,255"));
  assert(version.errorCode == 255);
  const char* badVersion[] = {
      "PQTMVERNO",
      "PQTMVERNO,",
      "PQTMVERNO,V",
      "PQTMVERNO,V,D",
      "PQTMVERNO,,D,T",
      "PQTMVERNO,V,,T",
      "PQTMVERNO,V,D,",
      "PQTMVERNO,V,D,T,",
      "PQTMVERNO,V,D,T,X",
      "PQTMVERNO,ERROR",
      "PQTMVERNO,ERROR,",
      "PQTMVERNO,ERROR,0",
      "PQTMVERNO,ERROR,256",
      "PQTMVERNO,ERROR,+1",
      "PQTMVERNO,ERROR,-1",
      "PQTMVERNO,ERROR,1.0",
      "PQTMVERNO,ERROR, 1",
      "PQTMVERNO,ERROR,1,",
      "PQTMVERNO,ERROR,D,T",
      "PQTMVERNO,ERROR,9999999999999999999999999999999999"};
  for (const char* body : badVersion) {
    noVersion(body, LC29H_REPLY_INVALID_FIELDS);
  }
  noVersion("PQTMVERNOX,V,D,T", LC29H_REPLY_UNSUPPORTED);
  noVersion("PAIR001,62,0", LC29H_REPLY_UNSUPPORTED);
  version = Adafruit_LC29H::parseVersion(
      sentence("PQTMVERNO,V,opaque-date,opaque-time"));
  assert(version.status == LC29H_REPLY_VALID);
  puts(
      "PASS: firmware spans, receiver errors, query echoes, exact payload "
      "shape");

  for (const char* body : {"PAIR001,62,0", "PQTMVERNO,V,D,T"}) {
    frame = sentence(body);
    line[0] = '!'; // Checksum covers the body, not the start marker.
    frame = Adafruit_NMEA::validate(line, frame.text.length);
    assert(frame.status == NMEA_FRAME_VALID);
    assert(Adafruit_LC29H::parsePairAck(frame).status ==
           LC29H_REPLY_UNSUPPORTED);
    assert(Adafruit_LC29H::parseVersion(frame).status ==
           LC29H_REPLY_UNSUPPORTED);
    line[0] = '$';
    line[frame.text.length - 4] = 'Z';
    frame = Adafruit_NMEA::validate(line, frame.text.length);
    assert(Adafruit_LC29H::parsePairAck(frame).status ==
           LC29H_REPLY_INVALID_FRAME);
    assert(Adafruit_LC29H::parseVersion(frame).status ==
           LC29H_REPLY_INVALID_FRAME);
  }
  frame = sentence("PAIR001,62,0");
  line[1] = 'Q';
  frame = Adafruit_NMEA::validate(line, frame.text.length);
  assert(frame.status == NMEA_FRAME_BAD_CHECKSUM);
  assert(Adafruit_LC29H::parsePairAck(frame).result == LC29H_PAIR_UNKNOWN);
  frame = Adafruit_NMEA::validate(NULL, 0);
  assert(Adafruit_LC29H::parseVersion(frame).status ==
         LC29H_REPLY_INVALID_FRAME);
  puts(
      "PASS: frame failures and alternate markers cannot become successful "
      "replies");
}
