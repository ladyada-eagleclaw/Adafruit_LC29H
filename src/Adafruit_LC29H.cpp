/**************************************************************************/
/*!
  @file Adafruit_LC29H.cpp
  @brief LC29H reply decoding on the shared NMEA/GNSS receiver.

  Written for Adafruit Industries. MIT license; see license.txt.
*/
/**************************************************************************/

#include "Adafruit_LC29H.h"

#include <string.h>

/**
 * @brief Create a receiver with caller-owned storage and no transport I/O.
 * @param firstBuffer First writable buffer, kept alive with this receiver.
 * @param secondBuffer Second, non-overlapping writable buffer.
 * @param capacity Bytes per buffer including NUL. Invalid storage disables
 * feed.
 */
Adafruit_LC29H::Adafruit_LC29H(volatile char* firstBuffer,
                               volatile char* secondBuffer, size_t capacity)
    : Adafruit_GNSS(firstBuffer, secondBuffer, capacity),
      _port(NULL),
      _timeout(1500),
      _busy(false),
      _dispatching(false),
      _commandStatus(LC29H_COMMAND_NO_PORT),
      _pairResult(LC29H_PAIR_UNKNOWN),
      _error(0),
      _sentenceCallback(NULL),
      _sentenceContext(NULL),
      _rtcmBuffer(NULL),
      _rtcmCapacity(0),
      _rtcmCallback(NULL),
      _rtcmContext(NULL),
      _rtcmLength(0),
      _rtcmTotal(0),
      _rtcmCRC(0),
      _byteTime(0),
      _rtcmHigh(0),
      _rtcmCanDeliver(false),
      _nmeaActive(false) {}

/**
 * @brief Decode the latest complete line as a PAIR acknowledgment.
 * @return Owned result; VALID may indicate rejection or continued processing.
 *
 * No earlier reply is cached. Invalid completed lines replace earlier replies;
 * other addresses return UNSUPPORTED. Before a line or after reset(), returns
 * INVALID_FRAME. Partial/overflowing input preserves the previous complete
 * line. Synchronize concurrent access with feed(). No line is consumed by this
 * call.
 */
lc29h_pair_ack_t Adafruit_LC29H::lastPairAck() const {
  return parsePairAck(lastSentence());
}

/**
 * @brief Decode the latest complete line as a firmware reply.
 * @return Result with borrowed spans, a receiver error, or decoding failure.
 *
 * Text expires on the next complete line, reset(), or destruction. Copy needed
 * text before feeding more lines, and synchronize concurrent access with
 * feed(). No earlier firmware reply is cached or consumed.
 */
lc29h_version_t Adafruit_LC29H::lastVersion() const {
  return parseVersion(lastSentence());
}

/**
 * @brief Decode an exact PAIR001 acknowledgment from a validated sentence.
 * @param sentence Unchanged view from validate() or lastSentence().
 * @return Owned ID/result on VALID; otherwise ID zero and result UNKNOWN.
 *
 * Trust the supplied frame status. Require '$', exactly two nonempty unsigned
 * integer fields, command ID 0..999, and result 0..5. Leading zeros are
 * allowed. Signs, decimals, extra fields, and unknown results are
 * INVALID_FIELDS. VALID describes decoding only: callers must match commandID
 * and interpret result. ACCEPTED does not imply a fix or query readback;
 * PROCESSING is not completion.
 */
lc29h_pair_ack_t Adafruit_LC29H::parsePairAck(const nmea_sentence_t& sentence) {
  lc29h_pair_ack_t result = {replyStatus(sentence, "PAIR001"), 0,
                             LC29H_PAIR_UNKNOWN};
  if (result.status != LC29H_REPLY_VALID) {
    return result;
  }
  result.status = LC29H_REPLY_INVALID_FIELDS;
  nmea_span_t remaining = sentence.fields;
  int32_t command = parseUnsigned(nextField(remaining), 999);
  int32_t code = parseUnsigned(nextField(remaining), 5);
  if (command < 0 || code < 0 || nextField(remaining).data) {
    return result;
  }
  result.commandID = (uint16_t)command;
  result.result = (lc29h_pair_result_t)code;
  result.status = LC29H_REPLY_VALID;
  return result;
}

/**
 * @brief Decode an exact PQTMVERNO success or error response.
 * @param sentence Unchanged view from validate() or lastSentence().
 * @return Borrowed version/date/time on VALID, code on RECEIVER_ERROR, or
 * failure.
 *
 * Trust the supplied frame status. Success requires three nonempty text fields;
 * date/time remain opaque text. ERROR is reserved as the first field and
 * requires exactly one unsigned error code, 1..255, including unknown codes. A
 * zero-field query/echo is INVALID_FIELDS. Failure clears all payload values.
 * Spans are not NUL-terminated and remain valid only while the source storage
 * is unchanged.
 */
lc29h_version_t Adafruit_LC29H::parseVersion(const nmea_sentence_t& sentence) {
  lc29h_version_t result = {
      replyStatus(sentence, "PQTMVERNO"), {NULL, 0}, {NULL, 0}, {NULL, 0}, 0};
  if (result.status != LC29H_REPLY_VALID) {
    return result;
  }
  result.status = LC29H_REPLY_INVALID_FIELDS;
  nmea_span_t remaining = sentence.fields;
  nmea_span_t version = nextField(remaining);
  nmea_span_t date = nextField(remaining);
  nmea_span_t time = nextField(remaining);
  if (matches(version, "ERROR")) {
    int32_t code = parseUnsigned(date, 255);
    if (code > 0 && !time.data) {
      result.errorCode = (uint8_t)code;
      result.status = LC29H_REPLY_RECEIVER_ERROR;
    }
    return result;
  }
  if (!version.length || !date.length || !time.length ||
      nextField(remaining).data) {
    return result;
  }
  result.version = version;
  result.buildDate = date;
  result.buildTime = time;
  result.status = LC29H_REPLY_VALID;
  return result;
}

/**
 * @brief Compare a bounded field with an entire constant string.
 * @param field Borrowed field, possibly absent.
 * @param text NUL-terminated expected text.
 * @return True only for exact equality.
 */
bool Adafruit_LC29H::matches(nmea_span_t field, const char* text) {
  return field.data && field.length == strlen(text) &&
         memcmp(field.data, text, field.length) == 0;
}

/**
 * @brief Parse unsigned decimal digits with an explicit upper bound.
 * @param field Complete field with no signs, spaces, or decimal point.
 * @param maximum Largest allowed value.
 * @return Integer value, or -1 for absent/empty, malformed, or oversized input.
 */
int32_t Adafruit_LC29H::parseUnsigned(nmea_span_t field, uint16_t maximum) {
  if (!field.data || !field.length) {
    return -1;
  }
  int32_t value = 0;
  for (size_t i = 0; i < field.length; i++) {
    char digit = field.data[i];
    if (digit < '0' || digit > '9') {
      return -1;
    }
    value = value * 10 + digit - '0';
    if (value > maximum) {
      return -1;
    }
  }
  return value;
}

/**
 * @brief Check frame validity, marker, and exact receiver-specific address.
 * @param sentence Unchanged shared-validator result.
 * @param address Required NUL-terminated address.
 * @return VALID, INVALID_FRAME, or UNSUPPORTED before field decoding.
 */
lc29h_reply_status_t Adafruit_LC29H::replyStatus(
    const nmea_sentence_t& sentence, const char* address) {
  if (sentence.status != NMEA_FRAME_VALID || !sentence.text.data ||
      !sentence.text.length) {
    return LC29H_REPLY_INVALID_FRAME;
  }
  if (sentence.text.data[0] != '$' || !matches(sentence.address, address)) {
    return LC29H_REPLY_UNSUPPORTED;
  }
  return LC29H_REPLY_VALID;
}
