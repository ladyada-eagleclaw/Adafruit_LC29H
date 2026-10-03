// Minimal host transport interface, not a hardware emulation.
#ifndef LC29H_TEST_ARDUINO_H
#define LC29H_TEST_ARDUINO_H
#include <stddef.h>
#include <stdint.h>
class Print {
 public:
  virtual ~Print() {}
  virtual size_t write(uint8_t byte) {
    return write(&byte, 1);
  }
  virtual size_t write(const uint8_t* data, size_t length) = 0;
};
class Stream : public Print {
 public:
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() {
    return -1;
  }
  virtual void flush() {}
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
