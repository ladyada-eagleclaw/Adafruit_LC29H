/** @file Adafruit_LC29H_I2C.cpp
 * @brief Quectel I2C Application Note V1.5, chapters 3 and 4.
 * Written for Adafruit Industries. MIT license; see license.txt.
 */
#include "Adafruit_LC29H_I2C.h"

#include "Adafruit_LC29H.h"

/** @brief Create a detached adapter with one AVR-sized receive buffer. */
Adafruit_LC29H_I2C::Adafruit_LC29H_I2C()
    : _wire(NULL),
      _offset(0),
      _length(0),
      _remaining(0),
      _lastTransfer(0),
      _error(0) {}

/** @brief Start Wire and address the configuration endpoint.
 * @param wire Borrowed bus, default Wire; NULL fails.
 * @return True on an address ACK only. gps.begin(adapter) verifies identity.
 * No receiver reset, pin changes beyond Wire.begin(), or persistent writes.
 */
bool Adafruit_LC29H_I2C::begin(TwoWire* wire) {
  end();
  _wire = wire;
  if (!_wire)
    return false;
  _wire->begin();
  _lastTransfer = millis();
  return select();
}

/** @brief Try Quectel's endpoint recovery after an interrupted bus sequence.
 * @return True if one of the three endpoints acknowledges the recovery byte.
 * Call only after a bus failure. Discards this adapter's cached input, sends a
 * zero byte to each documented endpoint until one ACKs, then stops. It does not
 * reset the module, clear a full receiver FIFO, or prove firmware readiness.
 * Reset the caller's parser afterward and query identity again. A failed write
 * may have partly reached the receiver; never use this to blindly resend it.
 * See I2C Application Note V1.5, chapter 6, Recovery_I2c.
 */
bool Adafruit_LC29H_I2C::recover() {
  if (!_wire)
    return false;
  _offset = _length = 0;
  _remaining = 0;
  const uint8_t addresses[] = {LC29H_I2C_CONFIG, LC29H_I2C_READ,
                               LC29H_I2C_WRITE};
  uint8_t zero = 0;
  for (uint8_t i = 0; i < sizeof(addresses); i++) {
    waitForTransfer();
    _wire->beginTransmission(addresses[i]);
    size_t written = _wire->write(&zero, 1);
    _error = _wire->endTransmission();
    _lastTransfer = millis();
    if (!written && !_error)
      _error = 4;
    if (!_error)
      return true;
  }
  return false;
}

/** @brief Detach without closing the shared Wire bus or resetting hardware. */
void Adafruit_LC29H_I2C::end() {
  _wire = NULL;
  _offset = _length = 0;
  _remaining = 0;
  _error = 0;
}

/** @brief Observe the vendor's processing interval, including millis wrap. */
void Adafruit_LC29H_I2C::waitForTransfer() {
  uint32_t elapsed = (uint32_t)(millis() - _lastTransfer);
  if (elapsed < LC29H_I2C_GAP_MS)
    delay(LC29H_I2C_GAP_MS - elapsed);
}

/** @brief Select 0x50 before a sequence, as required on a shared bus.
 * @return True when the address is acknowledged. */
bool Adafruit_LC29H_I2C::select() {
  if (!_wire)
    return false;
  // Quectel allows a temporarily unresponsive endpoint. Retry only an address
  // NACK: no command payload has reached the receiver at that point.
  for (uint8_t attempt = 0; attempt < LC29H_I2C_ADDRESS_ATTEMPTS; attempt++) {
    waitForTransfer();
    _wire->beginTransmission(LC29H_I2C_CONFIG);
    _error = _wire->endTransmission();
    _lastTransfer = millis();
    if (_error != 2)
      break;
  }
  return _error == 0;
}

/** @brief Send the two little-endian words of a FIFO command.
 * @param code Named register/FIFO operation.
 * @param length Requested byte count.
 * @return True if all eight bytes were acknowledged. */
bool Adafruit_LC29H_I2C::command(uint32_t code, uint32_t length) {
  uint8_t bytes[8];
  // This is a wire-format command, not an MCU register bit field.
  for (uint8_t i = 0; i < 4; i++) {
    bytes[i] = (uint8_t)(code >> (8 * i));
    bytes[i + 4] = (uint8_t)(length >> (8 * i));
  }
  waitForTransfer();
  _wire->beginTransmission(LC29H_I2C_CONFIG);
  size_t written = _wire->write(bytes, sizeof(bytes));
  _error = _wire->endTransmission();
  _lastTransfer = millis();
  if (written != sizeof(bytes) && !_error)
    _error = 4;
  return _error == 0;
}

/** @brief Read one bounded block from the selected FIFO/length register.
 * @param data Destination with room for length bytes.
 * @param length At most the classic AVR Wire capacity.
 * @return True only for an exact-length transfer. */
bool Adafruit_LC29H_I2C::receive(uint8_t* data, uint8_t length) {
  waitForTransfer();
  uint8_t received = _wire->requestFrom((uint8_t)LC29H_I2C_READ, length);
  _lastTransfer = millis();
  uint8_t count = 0;
  while (_wire->available()) {
    int byte = _wire->read();
    if (count < length && byte >= 0)
      data[count++] = (uint8_t)byte;
  }
  _error = received == length && count == length ? 0 : 4;
  return _error == 0;
}

/** @brief Query a four-byte FIFO length, without assuming host byte order.
 * @param code Receive count or transmit space register.
 * @param length Updated only on success.
 * @return True when the complete count was received. */
bool Adafruit_LC29H_I2C::readLength(uint32_t code, uint32_t& length) {
  uint8_t bytes[4];
  if (!command(code, sizeof(bytes)) || !receive(bytes, sizeof(bytes)))
    return false;
  length = 0;
  for (uint8_t i = 0; i < 4; i++)
    length |= (uint32_t)bytes[i] << (8 * i);
  return true;
}

/** @brief Return buffered bytes, fetching at most one new 32-byte chunk.
 * @return Available bytes, or zero for no data, detachment, or bus failure.
 * Check lastError() to distinguish a bus failure from an empty FIFO.
 * Only consume the count reported by the current FIFO length snapshot.
 */
int Adafruit_LC29H_I2C::available() {
  if (_offset < _length)
    return _length - _offset;
  _offset = _length = 0;
  if (!select())
    return 0;
  if (!_remaining && !readLength(LC29H_I2C_RX_LENGTH, _remaining))
    return 0;
  if (!_remaining)
    return 0;
  uint8_t count = LC29H_I2C_CHUNK;
  if (_remaining < count)
    count = (uint8_t)_remaining;
  if (!command(LC29H_I2C_RX_DATA, count) || !receive(_buffer, count)) {
    // A short transaction may have consumed bytes: query a fresh count next.
    _remaining = 0;
    return 0;
  }
  _remaining -= count;
  _length = count;
  return count;
}

/** @brief Read one byte, fetching a chunk if needed.
 * @return Byte 0..255, or -1 if none. */
int Adafruit_LC29H_I2C::read() {
  if (!available())
    return -1;
  return _buffer[_offset++];
}

/** @brief Inspect the next byte without consuming it.
 * @return Byte 0..255, or -1 if none. */
int Adafruit_LC29H_I2C::peek() {
  if (!available())
    return -1;
  return _buffer[_offset];
}

/** @brief No-op: outgoing writes are synchronous; input is never discarded. */
void Adafruit_LC29H_I2C::flush() {}

/** @brief Write a single byte through the complete FIFO write sequence.
 * @param byte Byte to send.
 * @return One on success, zero on failure. Prefer a single buffered write. */
size_t Adafruit_LC29H_I2C::write(uint8_t byte) {
  return write(&byte, 1);
}

/** @brief Write commands or binary corrections in AVR-sized chunks.
 * @param data Bytes to send, or NULL only if length is zero.
 * @param length Number of bytes offered.
 * @return Number of bytes in fully acknowledged chunks. A failed I2C
 * transaction may have partially reached the receiver; do not blindly retry
 * it. Does not wait for free space or reset/save the receiver.
 * Complete writes also wake an I2C transmitter that slept after FIFO overflow.
 */
size_t Adafruit_LC29H_I2C::write(const uint8_t* data, size_t length) {
  if (!_wire || (!data && length))
    return 0;
  size_t sent = 0;
  while (sent < length) {
    uint32_t space;
    if (!select() || !readLength(LC29H_I2C_TX_SPACE, space) || !space)
      break;
    size_t count = length - sent;
    if (count > LC29H_I2C_CHUNK)
      count = LC29H_I2C_CHUNK;
    if (count > space)
      count = (size_t)space;
    if (!command(LC29H_I2C_TX_DATA, count))
      break;
    waitForTransfer();
    _wire->beginTransmission(LC29H_I2C_WRITE);
    size_t written = _wire->write(data + sent, count);
    _error = _wire->endTransmission();
    _lastTransfer = millis();
    if (written != count && !_error)
      _error = 4;
    if (_error)
      break;
    sent += count;
  }
  return sent;
}

/** @brief Read the most recent bus status.
 * @return Wire endTransmission status, or 4 for a short transfer; zero is OK.
 */
uint8_t Adafruit_LC29H_I2C::lastError() const {
  return _error;
}

/** @brief Snapshot cached and receiver-queued bytes before a command.
 * @return Exact byte count, or UINT32_MAX for a failed/invalid length read.
 * Refresh the hardware count even if an earlier snapshot was partly consumed.
 * This prevents an older queued reply from completing a new transaction.
 */
uint32_t Adafruit_LC29H_I2C::pendingInput() {
  if (!select() || !readLength(LC29H_I2C_RX_LENGTH, _remaining))
    return UINT32_MAX;
  uint32_t buffered = _length - _offset;
  if (_remaining >= UINT32_MAX - buffered) {
    _error = 4;
    _remaining = 0;
    return UINT32_MAX;
  }
  return _remaining + buffered;
}

/** @brief Attach an initialized I2C adapter and verify receiver identity.
 * @param port Adapter whose begin() succeeded; its lifetime must exceed ours.
 * @param timeout Command and stale-input drain deadlines in ms.
 * @return True for a valid firmware reply. No reset or configuration changes.
 * I2C uses a longer drain deadline than UART because each FIFO transfer needs
 * a processing gap. An overflowing receiver FIFO may need poll() followed by
 * another begin() to wake its transmitter through a complete command write.
 */
bool Adafruit_LC29H::begin(Adafruit_LC29H_I2C& port, uint32_t timeout) {
  return beginPort(port, timeout, timeout, [](Stream& stream) {
    return ((Adafruit_LC29H_I2C&)stream).pendingInput();
  });
}
