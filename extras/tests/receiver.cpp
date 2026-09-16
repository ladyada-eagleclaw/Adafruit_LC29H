#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <type_traits>

#include "Adafruit_LC29H.h"

static size_t build(char* line, const char* body) {
  size_t length = Adafruit_NMEA::buildCommand(line, 160, body, strlen(body));
  assert(length);
  return length;
}
static nmea_frame_status_t feed(Adafruit_LC29H& gps, const char* line,
                                size_t length, uint32_t start = 100) {
  nmea_frame_status_t status = NMEA_FRAME_INCOMPLETE;
  for (size_t i = 0; i < length; i++) {
    status = gps.feed(line[i], start + (uint32_t)i);
  }
  return status;
}
int main() {
  static_assert(sizeof(Adafruit_LC29H) == sizeof(Adafruit_GNSS),
                "No extra storage");
  static_assert(!std::is_copy_constructible<Adafruit_LC29H>::value &&
                    !std::is_copy_assignable<Adafruit_LC29H>::value,
                "Cannot copy writable receive storage");
  char a[160], b[160], c[160], d[160], nav[160], reply[160];
  Adafruit_LC29H one(a, b, sizeof(a)), two(c, d, sizeof(c));
  Adafruit_LC29H disabled(NULL, NULL, 0);
  assert(disabled.feed('$', 0) == NMEA_FRAME_BAD_FORMAT);
  assert(one.lastPairAck().status == LC29H_REPLY_INVALID_FRAME);
  assert(one.lastVersion().status == LC29H_REPLY_INVALID_FRAME);
  size_t navLength = build(
      nav,
      "GNGGA,010203.004,8959.123456789,S,17959.123456789,W,4,12,0.8,1,M,2");
  size_t replyLength = build(reply, "PAIR001,062,1");
  uint32_t start = UINT32_MAX - 10;
  for (size_t i = 0; i < navLength || i < replyLength; i++) {
    if (i < navLength)
      one.feed(nav[i], start + (uint32_t)i);
    if (i < replyLength)
      two.feed(reply[i], 500 + (uint32_t)i);
  }
  gnss_position_t position = one.lastPosition();
  assert(position.validation.status == GNSS_SENTENCE_VALID);
  assert(position.fixQuality == 4 &&
         position.latitude.fractionalMinutes == 123456789);
  char coordinate[GNSS_COORDINATE_TEXT_SIZE];
  assert(
      one.formatCoordinate(coordinate, sizeof(coordinate), position.longitude));
  assert(!strcmp(coordinate, "-179.98539094648"));
  assert(one.sentenceStartedAt() == start &&
         one.sentenceReceivedAt() == start + (uint32_t)navLength - 1);
  assert(two.lastPairAck().commandID == 62 &&
         two.lastPairAck().result == LC29H_PAIR_PROCESSING);
  assert(two.sentenceStartedAt() == 500);
  assert(one.lastPairAck().status == LC29H_REPLY_UNSUPPORTED);
  puts(
      "PASS: two independent receivers, wraparound timestamps, RTK quality and "
      "exact text");

  navLength = build(
      nav,
      "GNGGA,010203.004,8959.123456790,S,17959.123456790,W,4,12,0.8,1,M,2");
  assert(feed(one, nav, navLength) == NMEA_FRAME_VALID);
  gnss_position_t changed = one.lastPosition();
  assert(changed.longitude.degreesE7 == position.longitude.degreesE7);
  assert(changed.longitude.fractionalMinutes ==
         position.longitude.fractionalMinutes + 1);
  assert(
      one.formatCoordinate(coordinate, sizeof(coordinate), changed.longitude));
  assert(!strcmp(coordinate, "-179.98539094650"));
  replyLength = build(reply, "PAIR001,063,0");
  assert(feed(one, reply, replyLength) == NMEA_FRAME_VALID);
  assert(one.lastPairAck().commandID == 63);
  assert(one.lastPosition().validation.status == GNSS_SENTENCE_UNSUPPORTED);
  assert(position.longitude.fractionalMinutes == 123456789);
  replyLength = build(reply, "PAIR001,062,0");
  assert(feed(two, reply, replyLength) == NMEA_FRAME_VALID);
  lc29h_pair_ack_t saved = two.lastPairAck();
  assert(saved.result == LC29H_PAIR_ACCEPTED && saved.commandID == 62);
  puts(
      "PASS: sub-E7 changes retained; reply IDs and positions are never "
      "merged");

  replyLength = build(reply, "PQTMVERNO,LC29HEA,2026/09/16,12:00:00");
  assert(feed(one, reply, replyLength) == NMEA_FRAME_VALID);
  lc29h_version_t version = one.lastVersion();
  assert(version.status == LC29H_REPLY_VALID && version.version.length == 7);
  assert(!memcmp(version.version.data, "LC29HEA", 7));
  assert(one.feed('$', 1000) == NMEA_FRAME_INCOMPLETE);
  unsigned overflows = 0;
  for (unsigned i = 0; i < sizeof(a) + 5; i++) {
    overflows += one.feed('A', 1001 + i) == NMEA_FRAME_OVERFLOW;
  }
  assert(overflows == 1 && one.lastVersion().status == LC29H_REPLY_VALID);
  assert(!memcmp(version.version.data, "LC29HEA", 7));
  replyLength = build(reply, "PQTMVERNO,ERROR,3");
  assert(feed(one, reply, replyLength) == NMEA_FRAME_VALID);
  assert(one.lastVersion().status == LC29H_REPLY_RECEIVER_ERROR &&
         one.lastVersion().errorCode == 3);
  // The old borrowed spans have expired; do not use them after replacement.
  reply[1] = 'R';
  assert(feed(one, reply, replyLength) == NMEA_FRAME_BAD_CHECKSUM);
  assert(one.lastVersion().status == LC29H_REPLY_INVALID_FRAME);
  assert(!one.lastVersion().version.data);
  assert(two.lastPairAck().commandID == 62);
  one.reset();
  assert(one.lastPairAck().status == LC29H_REPLY_INVALID_FRAME);
  assert(one.lastPosition().validation.status == GNSS_SENTENCE_INVALID_FRAME);
  assert(saved.commandID == 62 && saved.result == LC29H_PAIR_ACCEPTED);
  assert(feed(one, nav, navLength) == NMEA_FRAME_VALID);
  assert(one.lastPosition().validation.status == GNSS_SENTENCE_VALID);
  puts(
      "PASS: reply lifetime, overflow recovery, invalid-line replacement and "
      "reset");
}
