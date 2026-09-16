# Adafruit LC29H [![Arduino Library CI](https://github.com/adafruit/Adafruit_LC29H/actions/workflows/githubci.yml/badge.svg)](https://github.com/adafruit/Adafruit_LC29H/actions) [![Documentation](https://img.shields.io/badge/documentation-doxygen-blue.svg)](https://adafruit.github.io/Adafruit_LC29H/html/index.html)

Arduino UART driver for **Quectel LC29H(EA)** GNSS receivers, including exact
position decoding, configuration, survey/base settings, and RTCM transport.
Other LC29H variants need separate protocol and hardware verification.

The driver uses the NMEA/GNSS core in **Adafruit GPS Library 1.9.0 or later**.
It builds on `Adafruit_GNSS` and keeps MTK-specific `Adafruit_GPS` commands separate.

## Installation and wiring

Install Adafruit GPS Library 1.9.0 or later, then install this repository's ZIP
through Arduino's **Sketch > Include Library > Add .ZIP Library** menu.
If Library Manager has not indexed the dependency yet, use the
[GPS 1.9.0 ZIP](https://github.com/adafruit/Adafruit_GPS/archive/refs/tags/1.9.0.zip).

Use a board with a spare **hardware UART** capable of the receiver's baud rate.
The tested EA firmware uses **460800 baud, 8N1**. SoftwareSerial and a 16 MHz
AVR UART are unsuitable for that default rate; AVR compile coverage verifies the
API, not a reliable 460800-baud physical connection. ESP32-S3 is the tested host.

Connect receiver TX to host RX, receiver RX to host TX through suitable level
shifting, and connect ground. Follow your breakout's power requirements.
A bare LC29H has a 2.8 V I/O domain; do not connect a 5 V TX directly to it.
Initialize your UART, including board-specific pins, before calling `begin()`.
HILBERT's tested connection is RX GPIO8/TX GPIO9 through its fitted divider.

## Start with the basic example

Open [basic](examples/basic/basic.ino). It identifies firmware, selects eight
fractional position digits, then displays exact latitude/longitude, altitude,
satellite count, and fix quality. `begin()` itself only queries identity; it does
not change baud, reset the module, or write nonvolatile memory.

```cpp
#include <Adafruit_LC29H.h>

char first[256], second[256];
Adafruit_LC29H gps(first, second, sizeof(first));

void received(const nmea_sentence_t& sentence, void*) {
  lc29h_fix_t fix = gps.parseFix(sentence);
  if (fix.position.validation.status != GNSS_SENTENCE_VALID ||
      !fix.position.fix) return;
  char latitude[GNSS_COORDINATE_TEXT_SIZE];
  if (gps.formatCoordinate(latitude, sizeof(latitude), fix.position.latitude))
    Serial.println(latitude);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(460800); // Set UART pins as required by your board.
  // HILBERT: Serial1.begin(460800, SERIAL_8N1, 8, 9);
  gps.onSentence(received);
  if (!gps.begin(Serial1)) return;
  lc29h_precision_t precision = {3, 8, 3, 3, 3, 3};
  if (!gps.setPrecision(precision))
    Serial.println(F("High-precision output was not accepted."));
}

void loop() {
  gps.poll();
}
```

The two non-overlapping buffers must outlive the receiver. There is no library
heap allocation or hidden fix history. Use 256-byte buffers for these examples;
shorter buffers reject longer messages cleanly. RTCM reception has separate,
optional caller-owned storage (1029 bytes fits a maximum-length RTCM3 packet).

## Precision and valid data

- `parseFix()` returns coherent GGA position/time/quality, satellites, HDOP,
  altitude, geoid separation, and correction age. Quality 4 is RTK fixed;
  quality 5 is RTK float. A valid sentence alone is not a position fix.
- `parseMotion()` returns RMC position/time/date plus exact speed in knots and
  true course. `parsePVT()` returns PQTMPVT position, velocity and other fields
  as exact decimals; PQTMPVT's fix mode does not distinguish RTK float/fixed.
- `parseSurveyStatus()` retains exact ECEF coordinates and accuracy.
- `formatCoordinate()` emits decimal-degree text without an intermediate
  floating-point conversion. `formatDecimal()` retains a decimal coefficient
  and its scale, including trailing zeros. Neither silently truncates output.

Keep exact components, decimal values, formatted text, or the raw sentences
when logging RTK data. E7 coordinates and `float` conversions discard detail.
Set NMEA position precision to eight digits when that output is needed; retaining
precision in the parser cannot recover digits the receiver did not transmit.
This describes numerical precision, not a claim of positioning accuracy.

Check each result's validation status and each optional number's status.
Empty, missing, invalid, and valid numeric fields are distinct. Decoders return
owned measurement values; firmware text and sentence views borrow storage.

## Commands and configuration

Settings are read from the receiver, never from a cached setter value. Typed
configuration outputs remain unchanged if a query fails. Scalar getters use
documented failure sentinels such as `-1` or an `UNKNOWN` enum.

| Area | API |
| --- | --- |
| Identity and diagnostics | `getVersion()`, `commandStatus()`, `commandPairResult()`, `commandError()` |
| Fix interval and NMEA | `set/getFixInterval()`, `enableNMEA()`, `isNMEAEnabled()` |
| Navigation | `set/getNavigationMode()`, `enableInterferenceCancellation()`, `isInterferenceCancellationEnabled()` |
| Decimal output | `set/getPrecision()` |
| Base and survey | `set/getReceiverMode()`, `set/getSurvey()`, `parseSurveyStatus()` |
| Proprietary output | `set/getMessageRate()` with exact message/version readback |
| RTCM output | `set/getRTCMMode()`, reference-station and ephemeris enable/readback methods |
| UART | `set/getBaudrate()` |
| Explicit persistence | `saveParameters()`, `savePairSettings()`, `restoreParameters()` |
| Restart | `restart()`; `reset()` only resets the parser |
| Additional commands | `sendPairCommand()`, `queryPair()`, `sendPQTMCommand()` |

Commands have a total elapsed-time deadline, including repeated PROCESSING
replies. PAIR queries require both the matching acknowledgment and query data,
in either order. Negative acknowledgments, malformed data, partial writes, and
timeouts are reported separately. There are no automatic command retries.

`poll()` and command waits use the same reader and dispatch navigation to
`onSentence()`. Callbacks must return promptly and must not reenter the receiver,
issue commands, or read the UART. The driver is single-threaded. A Stream's own
blocking writes and user callbacks cannot be interrupted by the command deadline.

Queued input is drained with a bound before sending. PAIR/PQTM have no unique
transaction token: a delayed reply to an earlier identical request remains
ambiguous. After a timeout, let pending traffic settle and read back the setting
before deciding whether to repeat a mutation.

Changing base role/survey settings needs `saveParameters()` and a receiver
restart. Baud changes take effect after reboot and need a matching host UART
change. Fix interval and PAIR persistence have additional sequencing requirements
in the API documentation. No setter automatically saves, powers off, or restarts
hardware. Avoid repeated NVM writes in `loop()`.

## RTCM corrections and base operation

`writeCorrections()` forwards raw binary chunks and returns the number of bytes
the Stream accepted. Finish any unsent suffix before sending another command.
It does not validate correction content or claim that the receiver used it.
Supply corrections from your own base or an external NTRIP client; this library
does not provide an NTRIP client or correction service.

Incoming RTCM3 is separated from NMEA, checked with CRC24Q, and delivered to
`onRTCM()` only as a complete valid packet that fits the supplied buffer. Without
a callback, binary packets are skipped. A gap over 250 ms abandons an incomplete
packet. Arbitrary binary debug protocols are not supported; keep debug output off.

A fixed base needs an accurate antenna reference. `setSurvey()` uses exact decimal
meters for ECEF, avoiding AVR's 32-bit `double` limitation. Survey-in averaging
does not by itself establish centimeter absolute accuracy. See
[survey_base](examples/survey_base/survey_base.ino) and
[rtcm_bridge](examples/rtcm_bridge/rtcm_bridge.ino).

## Examples and testing

| Example | Purpose |
| --- | --- |
| [basic](examples/basic/basic.ino) | High-precision position and human-readable fix quality |
| [plotter](examples/plotter/plotter.ino) | Numeric-only satellites, HDOP, altitude and quality |
| [fulltest](examples/fulltest/fulltest.ino) | Firmware and settings walkthrough without NVM writes |
| [survey_base](examples/survey_base/survey_base.ino) | Explicit opt-in base/survey configuration |
| [rtcm_bridge](examples/rtcm_bridge/rtcm_bridge.ino) | Binary corrections from a host and checked output packets |
| [serial_decode](examples/serial_decode/serial_decode.ino) | Paste sentences into Serial Monitor without a GPS |

Run every host regression on Linux:

```sh
python3 extras/tests/run_tests.py --gps-dir /path/to/Adafruit_GPS
```

CI automatically discovers all C++ tests, runs address/undefined-behavior
sanitizers, builds examples, checks formatting, and generates documentation.
Hardware-UART examples use explicit board selectors, including ESP32-S3; the
hardware-independent decoder also builds on Uno. The GPS 1.9.0 tag is pinned in
CI because the current Adafruit dependency installer accepts names, not version
constraints in `library.properties`.

[Hardware test notes](extras/hw_tests/README.md) distinguish observed behavior
from host simulation. Tested firmware: `LC29HEANR11A03S_RSA` (2023-10-31).
On this firmware, PAIR433/435/437 RTCM readback requests time out. Those APIs
report failure rather than inventing a setting; RTCM/base configuration support
must be checked on the actual firmware. No corrected RTK fix is claimed without
a live correction source.

Protocol and ownership details are recorded in the
[receiver contract](extras/design/LC29H.md).

## License

MIT license; see [license.txt](license.txt). Written for Adafruit Industries.
The separately installed Adafruit GPS dependency retains its BSD license.
