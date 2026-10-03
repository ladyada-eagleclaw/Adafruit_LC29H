// Injectable Wire transactions for protocol tests; no GNSS emulation.
#ifndef LC29H_TEST_WIRE_H
#define LC29H_TEST_WIRE_H
#include <stddef.h>
#include <stdint.h>

#include <deque>
#include <functional>
#include <vector>

class TwoWire {
 public:
  std::function<uint8_t(uint8_t, const std::vector<uint8_t>&)> transmit;
  std::function<std::vector<uint8_t>(uint8_t, uint8_t)> request;
  std::vector<uint8_t> output;
  std::deque<uint8_t> input;
  uint8_t address = 0;
  void begin() {}
  void beginTransmission(uint8_t value) {
    address = value;
    output.clear();
  }
  size_t write(const uint8_t* data, size_t length) {
    output.insert(output.end(), data, data + length);
    return length;
  }
  uint8_t endTransmission() {
    return transmit ? transmit(address, output) : 2;
  }
  uint8_t requestFrom(uint8_t value, uint8_t length) {
    input.clear();
    if (request) {
      auto bytes = request(value, length);
      input.insert(input.end(), bytes.begin(), bytes.end());
    }
    return (uint8_t)input.size();
  }
  int available() {
    return (int)input.size();
  }
  int read() {
    if (input.empty())
      return -1;
    int result = input.front();
    input.pop_front();
    return result;
  }
};
extern TwoWire Wire;
#endif
