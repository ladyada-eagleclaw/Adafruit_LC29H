// Minimal host transport interface, not a hardware emulation.
#ifndef LC29H_TEST_ARDUINO_H
#define LC29H_TEST_ARDUINO_H
#include <stddef.h>
#include <stdint.h>
class Stream {
 public:
  virtual ~Stream() {}
  virtual int available() = 0;
  virtual int read() = 0;
  virtual size_t write(const uint8_t* data, size_t length) = 0;
};
inline uint32_t& testClock() {
  static uint32_t value = 0;
  return value;
}
inline unsigned long millis() {
  return testClock()++;
}
inline void delay(unsigned long value) {
  testClock() += value;
}
#endif
