#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <deque>
#include <string>
#include <vector>

#include "Adafruit_LC29H.h"
#include "Adafruit_LC29H_I2C.h"

TwoWire Wire;

static std::string frame(const char* body) {
  char line[256];
  size_t size =
      Adafruit_NMEA::buildCommand(line, sizeof(line), body, strlen(body));
  assert(size);
  return std::string(line, size);
}

struct FIFO {
  TwoWire bus;
  std::deque<uint8_t> input;
  std::string output;
  uint32_t operation = 0;
  uint32_t selectedLength = 0;
  uint32_t snapshot = 0;
  uint32_t freeSpace = 4096;
  uint32_t previousTransfer = 0;
  bool firstTransfer = true;
  bool shortRead = false;
  bool shortData = false;
  bool nackWrite = false;
  bool nackCommand = false;
  unsigned nackSelections = 0;
  bool keepReceiving = false;
  bool recoveryMode = false;
  uint8_t recoveryACK = 0;
  std::vector<uint8_t> recoveryAddresses;
  bool reply = false;
  unsigned reads = 0;
  unsigned selections = 0;

  void queue(const std::string& bytes) {
    input.insert(input.end(), bytes.begin(), bytes.end());
  }
  void checkGap() {
    uint32_t now = testClock();
    if (!firstTransfer)
      assert((uint32_t)(now - previousTransfer) >= LC29H_I2C_GAP_MS);
    previousTransfer = now;
    firstTransfer = false;
  }
  static uint32_t word(const std::vector<uint8_t>& bytes, size_t offset) {
    uint32_t value = 0;
    for (size_t i = 0; i < 4; i++)
      value |= (uint32_t)bytes[offset + i] << (8 * i);
    return value;
  }
  FIFO() {
    bus.transmit = [this](uint8_t address, const std::vector<uint8_t>& bytes) {
      checkGap();
      assert(bytes.size() <= 32);
      if (recoveryMode) {
        assert(bytes.size() == 1 && bytes[0] == 0);
        recoveryAddresses.push_back(address);
        return (uint8_t)(address == recoveryACK ? 0 : 2);
      }
      if (address == 0x50) {
        if (bytes.empty()) {
          selections++;
          if (nackSelections) {
            nackSelections--;
            return (uint8_t)2;
          }
          return (uint8_t)0;
        }
        if (nackCommand)
          return (uint8_t)2;
        assert(bytes.size() == 8);
        operation = word(bytes, 0);
        selectedLength = word(bytes, 4);
        if (operation == 0xAA510008 || operation == 0xAA510004)
          assert(selectedLength == 4);
        else if (operation == 0xAA512000) {
          assert(selectedLength && selectedLength <= snapshot);
          assert(selectedLength <= input.size());
        } else {
          assert(operation == 0xAA531000);
          assert(selectedLength && selectedLength <= freeSpace);
        }
      } else {
        assert(address == 0x58 && operation == 0xAA531000);
        assert(bytes.size() == selectedLength);
        if (nackWrite)
          return (uint8_t)3;
        output.append(bytes.begin(), bytes.end());
        if (reply && !output.empty() && output.back() == '\n') {
          assert(output == frame("PQTMVERNO"));
          keepReceiving = false;
          queue(frame("PQTMVERNO,LC29HBANR11A06S_CSA4,2025/05/28,09:36:24"));
          output.clear();
        }
      }
      return (uint8_t)0;
    };
    bus.request = [this](uint8_t address, uint8_t length) {
      checkGap();
      assert(address == 0x54 && length == selectedLength && length <= 32);
      reads++;
      std::vector<uint8_t> bytes;
      if (operation == 0xAA510008 || operation == 0xAA510004) {
        uint32_t value = freeSpace;
        if (operation == 0xAA510008)
          value = snapshot = (uint32_t)input.size();
        for (uint8_t i = 0; i < 4; i++)
          bytes.push_back((uint8_t)(value >> (8 * i)));
      } else {
        assert(operation == 0xAA512000);
        for (uint8_t i = 0; i < length; i++) {
          bytes.push_back(input.front());
          input.pop_front();
          snapshot--;
        }
        if (keepReceiving)
          queue(std::string(length, 'x'));
      }
      if (shortRead || (shortData && operation == 0xAA512000))
        bytes.pop_back();
      return bytes;
    };
  }
};

int main() {
  FIFO fifo;
  Adafruit_LC29H_I2C port;
  assert(!port.begin(NULL));
  assert(port.available() == 0 && port.read() == -1 && port.peek() == -1);
  assert(port.write((uint8_t)0) == 0);
  assert(port.begin(&fifo.bus));
  assert(port.available() == 0 && port.lastError() == 0);
  std::string input;
  for (int i = 0; i < 99; i++)
    input.push_back((char)i); // Includes NUL and all low-valued binary bytes.
  fifo.queue(input);
  assert(port.available() == 32);
  unsigned reads = fifo.reads;
  assert(port.peek() == 0 && port.peek() == 0 && fifo.reads == reads);
  port.flush();
  for (uint8_t byte : input)
    assert(port.read() == byte);
  assert(port.read() == -1);
  puts(
      "PASS: I2C wire words, 32-byte reads, binary data, peek and FIFO "
      "snapshots");

  assert(port.write((const uint8_t*)input.data(), input.size()) ==
         input.size());
  assert(fifo.output == input);
  fifo.output.clear();
  fifo.freeSpace = 3;
  assert(port.write((const uint8_t*)"hello", 5) == 5);
  assert(fifo.output == "hello");
  fifo.freeSpace = 0;
  assert(port.write((uint8_t)'x') == 0);
  fifo.freeSpace = 4096;
  fifo.nackWrite = true;
  assert(port.write((uint8_t)'x') == 0 && port.lastError() == 3);
  fifo.nackWrite = false;
  fifo.nackCommand = true;
  assert(port.write((uint8_t)'x') == 0 && port.lastError() == 2);
  fifo.nackCommand = false;
  puts("PASS: I2C write chunking, backpressure and NACK propagation");

  fifo.queue("an incomplete read");
  fifo.shortRead = true;
  assert(port.available() == 0 && port.lastError() == 4);
  fifo.shortRead = false;
  assert(port.available() == 18);
  fifo.shortRead = true;
  while (port.read() >= 0) {
  }
  fifo.shortRead = false;
  fifo.queue("fresh");
  assert(port.available() == 5);
  for (char byte : std::string("fresh"))
    assert(port.read() == byte);
  puts("PASS: short reads never deliver a partial chunk; later reads recover");

  // The length read succeeds, but the actual data transaction comes back short.
  // None of those bytes may escape as an apparently complete receive chunk.
  fifo.queue(std::string(40, 'z'));
  fifo.shortData = true;
  assert(port.available() == 0 && port.lastError() == 4);
  fifo.shortData = false;
  assert(port.available() == 8);
  for (int i = 0; i < 8; i++)
    assert(port.read() == 'z');
  assert(port.read() == -1);
  puts(
      "PASS: short FIFO data reads discard the incomplete chunk and "
      "resnapshot");

  char a[160], b[160];
  Adafruit_LC29H gps(a, b, sizeof(a));
  fifo.output.clear();
  fifo.reply = true;
  assert(gps.begin(port));
  assert(gps.commandStatus() == LC29H_COMMAND_OK);
  // Fresh identity query is also exercised after the previous response.
  char version[100];
  assert(gps.getVersion(version, sizeof(version)));
  assert(strstr(version, "LC29HBANR11A06S_CSA4"));
  puts("PASS: complete receiver identification through real I2C adapter");

  // A cached chunk and old snapshot must not conceal newer stale replies.
  fifo.queue(frame("PQTMVERNO,OLD,2020/01/01,00:00:00"));
  assert(port.available() == 32);
  fifo.queue(frame("PQTMVERNO,ALSO_OLD,2020/01/01,00:00:00"));
  // New input arriving during the drain means the FIFO never goes quiet.
  // A finite snapshot still permits the next command to be transmitted.
  fifo.keepReceiving = true;
  assert(gps.getVersion(version, sizeof(version)));
  assert(strstr(version, "LC29HBANR11A06S_CSA4"));
  assert(!fifo.keepReceiving);
  puts(
      "PASS: stale cached/FIFO replies drained while new input keeps arriving");

  unsigned selections = fifo.selections;
  fifo.nackSelections = 2;
  assert(port.begin(&fifo.bus));
  assert(fifo.selections == selections + 3);
  selections = fifo.selections;
  fifo.nackSelections = 100;
  assert(!port.begin(&fifo.bus) && port.lastError() == 2);
  assert(fifo.selections == selections + LC29H_I2C_ADDRESS_ATTEMPTS);
  fifo.nackSelections = 0;
  assert(port.begin(&fifo.bus));
  puts("PASS: address-only NACK retries recover and remain bounded");

  fifo.recoveryMode = true;
  fifo.recoveryACK = 0x54;
  assert(port.recover());
  assert((fifo.recoveryAddresses == std::vector<uint8_t>{0x50, 0x54}));
  fifo.recoveryAddresses.clear();
  fifo.recoveryACK = 0;
  assert(!port.recover() && port.lastError() == 2);
  assert((fifo.recoveryAddresses == std::vector<uint8_t>{0x50, 0x54, 0x58}));
  fifo.recoveryMode = false;
  assert(gps.begin(port));
  puts(
      "PASS: explicit endpoint recovery stops at ACK and requires fresh "
      "identity");

  testClock() = UINT32_MAX - 3;
  fifo.firstTransfer = true;
  assert(port.begin(&fifo.bus));
  assert(port.write((const uint8_t*)frame("PQTMVERNO").data(), 15) == 15);
  port.end();
  assert(!port.recover());
  assert(port.available() == 0 && port.read() == -1);
  puts("PASS: transfer spacing, clock wrap, detach and reinitialization");
}
