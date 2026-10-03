/** @file Adafruit_LC29H_I2C.h
 * @brief Quectel's three-address I2C interface as an Arduino Stream.
 * Written for Adafruit Industries. MIT license; see license.txt.
 */
#ifndef ADAFRUIT_LC29H_I2C_H
#define ADAFRUIT_LC29H_I2C_H

#include <Arduino.h>
#include <Wire.h>

/// Fixed seven-bit addresses from Quectel I2C Application Note V1.5.
enum : uint8_t {
  LC29H_I2C_CONFIG = 0x50,        ///< Configuration commands.
  LC29H_I2C_READ = 0x54,          ///< Length and received data.
  LC29H_I2C_WRITE = 0x58,         ///< Outgoing data.
  LC29H_I2C_CHUNK = 32,           ///< Fits classic AVR Wire buffers.
  LC29H_I2C_GAP_MS = 10,          ///< Processing interval between transfers.
  LC29H_I2C_ADDRESS_ATTEMPTS = 20 ///< Bound for a temporarily busy address.
};

/// Quectel length registers and FIFO commands, transmitted little endian.
enum : uint32_t {
  LC29H_I2C_RX_LENGTH = 0xAA510008UL, ///< Bytes waiting for the host.
  LC29H_I2C_TX_SPACE = 0xAA510004UL,  ///< Space available for host writes.
  LC29H_I2C_RX_DATA = 0xAA512000UL,   ///< Select incoming FIFO data.
  LC29H_I2C_TX_DATA = 0xAA531000UL    ///< Select outgoing FIFO data.
};

/** Bounded I2C Stream for LC29H BA/CA/DA and other documented I2C variants.
 * No heap allocation. UART users need not instantiate this adapter.
 * Calls perform synchronous Wire transfers with the vendor's 10 ms gaps;
 * available() may take about 50 ms to fetch a new 32-byte chunk, plus up to
 * 200 ms of address-only NACK retries and the Wire core's own transfer time.
 * Call frequently, keep message rates low on AVR, and enable a Wire timeout
 * where the core supports it. Do not access Wire from an interrupt/callback.
 * Other I2C devices may be used between calls, but not during a call.
 */
class Adafruit_LC29H_I2C : public Stream {
 public:
  Adafruit_LC29H_I2C();
  bool begin(TwoWire* wire = &Wire);
  bool recover();
  void end();
  int available() override;
  int read() override;
  int peek() override;
  void flush() override;
  size_t write(uint8_t byte) override;
  size_t write(const uint8_t* data, size_t length) override;
  using Print::write;
  uint8_t lastError() const;

 private:
  // Doxygen 1.8.13 mistakes this friend for a nested constructor.
  /// @cond INTERNAL
  friend class Adafruit_LC29H;
  /// @endcond
  TwoWire* _wire;                   ///< Borrowed bus; adapter never closes it.
  uint8_t _buffer[LC29H_I2C_CHUNK]; ///< Received bytes independent of Wire.
  uint8_t _offset;                  ///< Next unread byte.
  uint8_t _length;                  ///< Buffered byte count.
  uint32_t _remaining;    ///< Unread bytes from the last FIFO length snapshot.
  uint32_t _lastTransfer; ///< End of the last bus operation, for spacing.
  uint8_t _error;         ///< Wire status; 4 also denotes a short read/write.
  void waitForTransfer();
  bool select();
  bool command(uint32_t code, uint32_t length);
  bool readLength(uint32_t code, uint32_t& length);
  bool receive(uint8_t* data, uint8_t length);
  uint32_t pendingInput();
};

#endif
