/** @file LC29H_transport.cpp
 *  @brief Single-reader UART transactions and mixed NMEA/RTCM reception.
 *  Written for Adafruit Industries. MIT license; see license.txt.
 */
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "Adafruit_LC29H.h"

/** @brief Attach an already initialized duplex Stream and query identity.
 * @param port UART configured by the sketch, typically 460800 baud for EA.
 * @param timeout Total command timeout in ms, 1..2147483647.
 * @return True if a well-formed firmware response is received. Does not
 * identify variant capabilities, change baud, reset hardware, or save settings.
 * On failure the port remains attached for diagnosis with poll(). */
bool Adafruit_LC29H::begin(Stream& port, uint32_t timeout) {
  if (_busy || _dispatching)
    return false;
  _port = &port;
  setCommandTimeout(timeout);
  if (!timeout || timeout > 0x7FFFFFFFUL)
    return badArgument();
  reset();
  char version[160];
  return getVersion(version, sizeof(version));
}
/** @brief Detach the port without closing it or changing receiver power. */
void Adafruit_LC29H::end() {
  if (_busy || _dispatching)
    return;
  _port = NULL;
  reset();
  _commandStatus = LC29H_COMMAND_NO_PORT;
}
/** @brief Set the total command deadline; invalid values leave it unchanged.
 * @param timeout Milliseconds, 1..2147483647; PROCESSING never extends it. */
void Adafruit_LC29H::setCommandTimeout(uint32_t timeout) {
  if (timeout && timeout <= 0x7FFFFFFFUL && !_busy && !_dispatching)
    _timeout = timeout;
}
/** @brief Clear NMEA and binary framing without restarting the hardware.
 * Callback registrations and attached port are retained. */
void Adafruit_LC29H::reset() {
  if (_dispatching)
    return;
  Adafruit_GNSS::reset();
  _rtcmLength = 0;
  _rtcmTotal = 0;
  _rtcmCRC = 0;
  _rtcmCanDeliver = false;
  _nmeaActive = false;
}
/** @brief Install the complete-line consumer, called also during commands.
 * @param callback Function or NULL to disable. Do not reenter this receiver.
 * @param context Borrowed application pointer passed unchanged. */
void Adafruit_LC29H::onSentence(lc29h_sentence_callback_t callback,
                                void* context) {
  if (_busy || _dispatching)
    return;
  _sentenceCallback = callback;
  _sentenceContext = context;
}
/** @brief Install an optional CRC-checked RTCM3 packet consumer.
 * @param buffer Borrowed writable packet buffer; 1029 bytes fits any RTCM3
 * frame.
 * @param capacity Buffer size; oversized packets are safely skipped.
 * @param callback Function or NULL to discard binary packets.
 * @param context Borrowed application pointer. Callback must not reenter.
 * Replacing storage during a packet skips delivery of that packet.
 * Binary packets are excluded from NMEA parsing even without a consumer. */
void Adafruit_LC29H::onRTCM(uint8_t* buffer, size_t capacity,
                            lc29h_rtcm_callback_t callback, void* context) {
  if (_busy || _dispatching)
    return;
  // A replacement buffer has not seen the beginning of an in-flight packet.
  _rtcmCanDeliver = false;
  _rtcmBuffer = buffer;
  _rtcmCapacity = capacity;
  _rtcmCallback = callback;
  _rtcmContext = context;
}
/** @brief Update the CRC24Q remainder of a byte stream.
 * @param crc Previous 24-bit remainder, initially zero.
 * @param byte Next header, payload, or CRC byte.
 * @return Updated remainder; zero after a complete valid packet. */
uint32_t Adafruit_LC29H::crc24(uint32_t crc, uint8_t byte) {
  crc ^= (uint32_t)byte << 16;
  for (uint8_t i = 0; i < 8; i++) {
    crc <<= 1;
    if (crc & 0x1000000UL)
      crc ^= 0x1864CFBUL;
  }
  return crc & 0xFFFFFFUL;
}
/** @brief Feed one received byte, separating RTCM3 from NMEA.
 * @param byte Received byte.
 * @param receivedAtMs Monotonic millisecond timestamp (wrap is supported).
 * @return NMEA completion status; binary traffic returns INCOMPLETE.
 * A binary inter-byte gap over 250 ms abandons an interrupted frame. Header
 * length and CRC are checked. Bad/oversized RTCM never invokes the callback.
 * Use this method, not a base-class feed(), for mixed binary/text input. */
nmea_frame_status_t Adafruit_LC29H::feed(uint8_t byte, uint32_t receivedAtMs) {
  if (_dispatching)
    return NMEA_FRAME_INCOMPLETE;
  if (_rtcmLength && (uint32_t)(receivedAtMs - _byteTime) > 250) {
    _rtcmLength = 0;
    _rtcmTotal = 0;
  }
  if (!_rtcmLength && byte == 0xD3) {
    // Binary cannot belong to a valid ASCII sentence. Abandon partial text.
    _nmeaActive = false;
    _rtcmTotal = 0;
    _rtcmCRC = 0;
    _rtcmLength = 1;
    _rtcmCanDeliver = _rtcmBuffer && _rtcmCapacity >= 6;
  } else if (_rtcmLength) {
    _rtcmLength++;
  }
  if (_rtcmLength) {
    _byteTime = receivedAtMs;
    if (_rtcmLength == 2) {
      if (byte & 0xFC) {
        if (byte == 0xD3) {
          // Overlapping preambles: this byte can start a fresh packet.
          _rtcmLength = 1;
          _rtcmCRC = crc24(0, byte);
          _rtcmCanDeliver = _rtcmBuffer && _rtcmCapacity >= 6;
          if (_rtcmCanDeliver)
            _rtcmBuffer[0] = byte;
          return NMEA_FRAME_INCOMPLETE;
        }
        _rtcmLength = 0;
        // Reconsider an ASCII start marker following a corrupt header.
        if (byte != '$' && byte != '!')
          return NMEA_FRAME_INCOMPLETE;
      } else {
        _rtcmHigh = byte;
      }
    }
    if (_rtcmLength) {
      if (_rtcmLength == 3)
        _rtcmTotal = ((uint16_t)_rtcmHigh << 8) + byte + 6;
      _rtcmCRC = crc24(_rtcmCRC, byte);
      if (_rtcmCanDeliver && _rtcmLength <= _rtcmCapacity)
        _rtcmBuffer[_rtcmLength - 1] = byte;
      if (_rtcmTotal && _rtcmLength == _rtcmTotal) {
        if (!_rtcmCRC && _rtcmCanDeliver && _rtcmTotal <= _rtcmCapacity &&
            _rtcmCallback) {
          _dispatching = true;
          _rtcmCallback(_rtcmBuffer, _rtcmTotal, _rtcmContext);
          _dispatching = false;
        }
        _rtcmLength = 0;
      }
      return NMEA_FRAME_INCOMPLETE;
    }
  }
  if (byte == '$' || byte == '!')
    _nmeaActive = true;
  // Do not finish a partial text frame through an intervening binary packet.
  if (!_nmeaActive && byte != '$' && byte != '!')
    return NMEA_FRAME_INCOMPLETE;
  nmea_frame_status_t result = Adafruit_GNSS::feed(byte, receivedAtMs);
  if (byte == '\n') {
    _nmeaActive = false;
    if (result != NMEA_FRAME_INCOMPLETE && result != NMEA_FRAME_OVERFLOW &&
        _sentenceCallback && lastText().length) {
      _dispatching = true;
      _sentenceCallback(lastSentence(), _sentenceContext);
      _dispatching = false;
    }
  }
  return result;
}
/** @brief Read a bounded number of bytes and dispatch complete messages.
 * @param maximumBytes Work budget; zero performs no reads.
 * @return Number of bytes read. Only this library should read the Stream. */
size_t Adafruit_LC29H::poll(size_t maximumBytes) {
  if (!_port || _busy || _dispatching)
    return 0;
  size_t count = 0;
  while (count < maximumBytes && _port->available() > 0) {
    int byte = _port->read();
    if (byte < 0)
      break;
    feed((uint8_t)byte, millis());
    count++;
  }
  return count;
}
/** @brief Forward raw RTCM correction bytes without NMEA wrapping.
 * @param data Correction stream chunk, or NULL only when length is zero.
 * @param length Number of bytes offered.
 * @return Bytes accepted by Stream::write; retry any unsent suffix before
 * sending commands. The caller owns packet framing/CRC and serialization of
 * chunks; this function does not claim receiver acceptance or an RTK fix.
 * A Stream write can block according to the underlying transport's policy. */
size_t Adafruit_LC29H::writeCorrections(const uint8_t* data, size_t length) {
  if (_busy || _dispatching)
    return 0;
  if (!_port || (!data && length))
    return 0;
  return _port->write(data, length);
}
/** @brief Get the last completed transaction status.
 * @return Local status; receiver-specific details have separate accessors. */
lc29h_command_status_t Adafruit_LC29H::commandStatus() const {
  return _commandStatus;
}
/** @brief Get the matching PAIR result from the last command.
 * @return UNKNOWN if none; PROCESSING may accompany a timeout. */
lc29h_pair_result_t Adafruit_LC29H::commandPairResult() const {
  return _pairResult;
}
/** @brief Get the last PQTM receiver error.
 * @return Nonzero wire error code, or zero when none was received. */
uint8_t Adafruit_LC29H::commandError() const {
  return _error;
}
/** @brief Record argument failure.
 * @return Always false. */
bool Adafruit_LC29H::badArgument() {
  if (!_busy && !_dispatching) {
    _commandStatus = LC29H_COMMAND_INVALID_ARGUMENT;
    _pairResult = LC29H_PAIR_UNKNOWN;
    _error = 0;
  }
  return false;
}
/** @brief Record invalid matching payload.
 * @return Always false. */
bool Adafruit_LC29H::badReply() {
  _commandStatus = LC29H_COMMAND_BAD_REPLY;
  return false;
}
/** @brief Execute one transaction with a fixed elapsed-time deadline.
 * @param body NMEA command body, excluding delimiters/checksum.
 * @param pairID PAIR command ID, or -1 for PQTM.
 * @param address Expected data address, or NULL for PAIR acknowledgment only.
 * @param response Caller-owned full response storage, or NULL for setters.
 * @param capacity Response capacity including NUL.
 * @param version Expect PQTMVERNO's special payload instead of OK.
 * @return True on acceptance plus requested readback; never for PROCESSING.
 * Bytes already queued are drained with a 20 ms/4096-byte bound before sending.
 * There is no wire transaction token: a late reply to an identical earlier
 * request cannot be distinguished. After timeout, let old traffic settle and
 * read back configuration before retrying mutations. Stream I/O and callbacks
 * must themselves return promptly; the deadline cannot interrupt user code. */
bool Adafruit_LC29H::transact(const char* body, int16_t pairID,
                              const char* address, char* response,
                              size_t capacity, bool version) {
  if (_busy || _dispatching)
    return false; // Preserve the outer transaction.
  _pairResult = LC29H_PAIR_UNKNOWN;
  _error = 0;
  if (!_port) {
    if (response && capacity)
      response[0] = 0;
    _commandStatus = LC29H_COMMAND_NO_PORT;
    return false;
  }
  char command[192];
  if (!body || (response && !capacity) || (!response && capacity))
    return badArgument();
  size_t length = buildCommand(command, sizeof(command), body, strlen(body));
  // Build first so a PQTM query body may share the caller's response buffer.
  if (response && capacity)
    response[0] = 0;
  if (!length)
    return badArgument();
  _busy = true;
  _commandStatus = LC29H_COMMAND_TIMEOUT;
  uint32_t started = millis();
  size_t drained = 0;
  while (_port->available() > 0) {
    if (drained >= 4096 || (uint32_t)(millis() - started) >= 20) {
      _busy = false;
      return false;
    }
    int byte = _port->read();
    if (byte < 0)
      break;
    feed((uint8_t)byte, millis());
    drained++;
  }
  // Drop a stale, incomplete text reply but retain binary packet alignment.
  Adafruit_GNSS::reset();
  _nmeaActive = false;
  started = millis();
  if (_port->write((const uint8_t*)command, length) != length) {
    _commandStatus = LC29H_COMMAND_WRITE_FAILED;
    _busy = false;
    return false;
  }
  bool accepted = pairID < 0;
  bool haveData = address == NULL;
  while ((uint32_t)(millis() - started) < _timeout) {
    if (_port->available() <= 0) {
      delay(1);
      continue;
    }
    int byte = _port->read();
    if (byte < 0) {
      delay(1);
      continue;
    }
    if (feed((uint8_t)byte, millis()) != NMEA_FRAME_VALID)
      continue;
    nmea_sentence_t sentence = lastSentence();
    if (pairID >= 0) {
      lc29h_pair_ack_t ack = parsePairAck(sentence);
      if (ack.status == LC29H_REPLY_VALID && ack.commandID == pairID) {
        _pairResult = ack.result;
        if (ack.result == LC29H_PAIR_ACCEPTED)
          accepted = true;
        else if (ack.result != LC29H_PAIR_PROCESSING) {
          _commandStatus = LC29H_COMMAND_REJECTED;
          break;
        }
      }
    }
    if (address && replyStatus(sentence, address) == LC29H_REPLY_VALID) {
      // Ignore an exact command echo (in particular a zero-field query).
      if (sentence.text.length == length &&
          !memcmp(sentence.text.data, command, length))
        continue;
      bool valid = true;
      if (pairID < 0) {
        nmea_span_t remaining = sentence.fields;
        nmea_span_t first = nextField(remaining);
        if (matches(first, "ERROR")) {
          int32_t code = parseUnsigned(nextField(remaining), 255);
          if (code <= 0 || nextField(remaining).data) {
            _commandStatus = LC29H_COMMAND_BAD_REPLY;
          } else {
            _error = (uint8_t)code;
            _commandStatus = LC29H_COMMAND_REJECTED;
          }
          break;
        }
        if (version)
          valid = parseVersion(sentence).status == LC29H_REPLY_VALID;
        else
          valid =
              matches(first, "OK") && (response || !nextField(remaining).data);
      }
      if (!valid || (response && sentence.text.length >= capacity)) {
        _commandStatus = LC29H_COMMAND_BAD_REPLY;
        break;
      }
      if (response) {
        memcpy(response, sentence.text.data, sentence.text.length);
        response[sentence.text.length] = 0;
      }
      haveData = true;
    }
    if (accepted && haveData) {
      _commandStatus = LC29H_COMMAND_OK;
      break;
    }
  }
  _busy = false;
  if (_commandStatus != LC29H_COMMAND_OK && response && capacity)
    response[0] = 0;
  return _commandStatus == LC29H_COMMAND_OK;
}
/** @brief Query firmware into caller-owned complete NMEA sentence storage.
 * @param response Output buffer; 160 bytes suits documented version replies.
 * @param capacity Buffer capacity, including NUL; too small fails without
 * truncation.
 * @return True for a fully validated version. Decode with
 * parseVersion(validate()). */
bool Adafruit_LC29H::getVersion(char* response, size_t capacity) {
  if (!response || !capacity)
    return badArgument();
  return transact("PQTMVERNO", -1, "PQTMVERNO", response, capacity, true);
}
/** @brief Send a PAIR command and wait for its terminal acknowledgment.
 * @param command Packet ID, 0..999; command-specific prerequisites apply.
 * @param parameters Comma-separated parameters without a leading comma, or
 * NULL.
 * @return True only for matching ACCEPTED; use queryPair() for readback. */
bool Adafruit_LC29H::sendPairCommand(uint16_t command, const char* parameters) {
  if (command > 999)
    return badArgument();
  char body[160];
  int length = snprintf(body, sizeof(body), "PAIR%03u%s%s", command,
                        parameters && *parameters ? "," : "",
                        parameters ? parameters : "");
  if (length < 0 || (size_t)length >= sizeof(body))
    return badArgument();
  return transact(body, command, NULL, NULL, 0);
}
/** @brief Query PAIR data, requiring both its ACK and data in either order.
 * @param command Packet ID, 0..999.
 * @param parameters Parameter list or NULL.
 * @param response Complete NMEA response buffer; use only on success.
 * @param capacity Bytes in response, including NUL.
 * @return True when both messages arrive; caller validates query-specific
 * fields. */
bool Adafruit_LC29H::queryPair(uint16_t command, const char* parameters,
                               char* response, size_t capacity) {
  if (command > 999 || !response || !capacity)
    return badArgument();
  char address[8], body[160];
  snprintf(address, sizeof(address), "PAIR%03u", command);
  int length = snprintf(body, sizeof(body), "%s%s%s", address,
                        parameters && *parameters ? "," : "",
                        parameters ? parameters : "");
  if (length < 0 || (size_t)length >= sizeof(body))
    return badArgument();
  return transact(body, command, address, response, capacity);
}
/** @brief Send a PQTM command whose response begins with OK or ERROR.
 * @param body Full body, e.g. PQTMCFGNMEADP,R; no '$' or checksum.
 * @param response Optional complete reply buffer for queries.
 * @param capacity Response bytes including NUL, or zero with NULL.
 * @return True for an exact matching OK address; query fields need decoding.
 * Unsupported firmware commands report REJECTED or TIMEOUT. This escape hatch
 * does not imply support for all commands or LC29H variants. Use getVersion()
 * for VERNO.
 */
bool Adafruit_LC29H::sendPQTMCommand(const char* body, char* response,
                                     size_t capacity) {
  if (!body || strncmp(body, "PQTM", 4))
    return badArgument();
  const char* comma = strchr(body, ',');
  size_t length = comma ? (size_t)(comma - body) : strlen(body);
  char address[24];
  if (length <= 4 || length >= sizeof(address))
    return badArgument();
  memcpy(address, body, length);
  address[length] = 0;
  return transact(body, -1, address, response, capacity);
}
