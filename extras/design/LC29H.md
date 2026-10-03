# LC29H receiver contract

The driver targets LC29H EA and BA, using Adafruit GPS 1.9.0's shared NMEA/GNSS core.
It inherits `Adafruit_GNSS`, independently of MTK-specific `Adafruit_GPS`.
Other LC29H variants remain unverified. BA-specific reports live in
`Adafruit_LC29H_BA`; UART and I²C use the same framing and command path.

## Sources and fixture

- Quectel [LC29H Series & LC79H(AL) GNSS Protocol Specification V1.4](https://www.quectel.com/content/uploads/2022/02/Quectel_LC29H_SeriesLC79HAL_GNSS_Protocol_Specification_V1.4.pdf):
  PAIR/PQTM framing and command tables, navigation, survey and RTCM formats.
- Quectel [LC29H Series Hardware Design V1.2](https://www.quectel.com/content/uploads/2022/06/Quectel_LC29H_Series_Hardware_Design_V1.2.pdf),
  table 12: the 2.8 V domain accepts input-high levels from 1.75 to 3.08 V.
- Quectel [I²C Application Note V1.5](https://device.report/m/a1aacd50b78285954555ab06bb552709fa404ca720f5faa45ec6f5b2e6bd4341.pdf),
  chapters 3–6: three-address FIFO, little-endian command words, processing
  intervals, shared-bus selection and endpoint recovery.
- Quectel [DR & RTK Application Note V1.5](https://forums.quectel.com/uploads/short-url/jY1yx4elee34iZdJmNsKJZ3dLu2.pdf),
  chapter 4: BA sensor, calibration, vehicle and message-rate formats.
- HILBERT Rev A schematic: U2 LC29H(EA), TXD to ESP32-S3 GPIO8;
  GPIO9 through R8 1 kOhm to RXD, R14 5.1 kOhm to ground. The divider gives
  approximately 2.76 V from a 3.3 V TX. Reset and unrelated pins are untouched.
- RTCM CRC24Q polynomial cross-check: [RTKLIB rtkcmn.c](https://github.com/tomojitakasu/RTKLIB/blob/master/src/rtkcmn.c). The bitwise implementation here was written independently.
- [Bench evidence and firmware limits](../hw_tests/README.md).

## Ownership and precision

Two non-overlapping NMEA buffers belong to the caller. The optional RTCM buffer
also belongs to the caller. The library allocates no heap memory and stores no
position history. All mutable framing/transaction state belongs to one instance.

`lastPosition()`, `lastPairAck()` and `lastVersion()` describe the latest complete
text line. Invalid completed lines replace older lines; partial/overflowing
input preserves the preceding complete line. Binary packets do not replace it.
Views expire on the next completed text line, reset, a new command's input drain,
or destruction. Results containing numbers own their values. Firmware fields
remain bounded borrowed spans, not NUL-terminated strings.

Exact coordinate components and decimal coefficient/scale values retain RTK
numerical detail. Decimal formatting uses integer arithmetic, including INT64_MIN
and very small fractions, and refuses insufficient storage rather than rounding.
A decoder validates all supported populated fields before publishing values.
Optional missing/empty states remain distinct. No navigation records are merged
across epochs or sentence types.

## Transport and transactions

The sketch configures its UART and attaches it with `begin(Stream&)`. A valid
firmware reply proves bidirectional communication, not every firmware capability.
`poll()` limits bytes per call. Commands synchronously use the same `feed()` path
and callbacks, so navigation continues during waits. Reentry is refused; callers
must serialize threads, callbacks and correction chunks themselves.

Commands use checked bounded construction. UART drains queued input with a
4096-byte/20-ms limit before transmitting. I²C refreshes and drains an exact
queued-byte snapshot, including locally buffered bytes, within the timeout
passed to its `begin()` overload. Newly arriving navigation cannot keep this
drain running forever; failed reads abort before any command is transmitted.
The response has its own command deadline. Both paths drop stale partial text and
uses a wrapping-safe fixed deadline. Slow Stream methods or callbacks are beyond
the driver's ability to preempt. No state-changing command is automatically
retried. After timeout, a delayed response to an identical command is inherently
ambiguous because the protocol supplies no unique request token.

PAIR setters require a matching ID and terminal result. PROCESSING never means
success or extends the deadline. Queries require acknowledgment plus data, in
either order. PQTM uses an exact address and OK/ERROR payload; VERNO is decoded
separately. Generic queries validate framing/correlation; typed getters validate
their complete payload and keep output structs unchanged on failure.

RTCM3 header length separates binary packets from text, including payload bytes
that resemble complete NMEA commands. Optional packet delivery requires a valid
CRC24Q and sufficient buffer space. Maximum payload is 1023 bytes. An inter-byte
gap over 250 ms abandons partial binary framing. Binary debug output formats are
not understood and must remain disabled. Raw correction writes report partial
acceptance without altering bytes or wrapping them in NMEA.

## Configuration policy

Readbacks always query the receiver. Setters do not cache settings, save NVM,
change the host UART, stop tracking, or reset hardware implicitly. The public
comments identify settings requiring save/reboot or stopped GNSS operation.
`reset()` only clears parser state; `restart()` sends an explicit PAIR start
command. Restart acknowledgment is not readiness or a reacquired fix.

EA-specific omissions are intentional: APIs are not advertised for AA-only
constellation, elevation-mask, dual-band toggle, DGPS-source, or NMEA-version
commands. Additional firmware-specific commands can use the explicit PAIR/PQTM
transaction APIs. Unsupported firmware may reject commands or remain silent;
check status rather than assuming every command in a family-wide PDF exists.

## BA sensor and I²C contract

`Adafruit_LC29H_I2C` borrows Wire and owns one 32-byte buffer. It uses 7-bit
addresses 0x50/0x54/0x58, little-endian two-word commands and length snapshots.
Transfers are synchronous with 10 ms gaps; failed data chunks are not exposed as
complete input. Only address-only NACKs are retried, at most 20 attempts. Payload
writes are never automatically repeated. Explicit `recover()` follows the vendor
endpoint recovery sequence and discards local buffered input; it does not reboot
the receiver or retry a command. Use a Wire timeout on cores that support it.
Other I²C devices can share the bus between adapter calls, not within a sequence.
UART-only sketches do not allocate an I²C adapter or its receive buffer.

On the BA breakout, VIN feeds the host-side pullups and level-shifter supply.
USB supplies module power but does not backfeed VIN. Metro Mini uses 5 V VIN,
A4 SDA, A5 SCL and common ground through the breakout's fitted level shifting.
The USB/HOST switch selects UART routing only. No extra pads are driven here.

The BA subclass adds no persistent measurement storage. Each PQTMSENMSG result
contains all six validated axes, temperature, frame and full uint32 timestamp.
Gyro degrees/s become rad/s, acceleration g becomes m/s²; GNSS decimals are
unchanged. Version 2 is raw module axes on four-wheel firmware. Version 4 is
vehicle axes and requires driving calibration plus 10 Hz navigation. PQTMDRCAL
reports calibration/source separately from fix quality. PQTMVEHMSG version 2
preserves the full unsigned cumulative tick count and direction. Physical wheel
input, movement calibration and corrected RTK are separate unperformed tests.

## Verification

Every C++ regression under `extras/tests` is discovered automatically. Tests
cover framing, all defined acknowledgment codes, exact command and reply fields,
interleaved navigation, ordering, stale input, timeout/wraparound, short writes,
reentry, two receivers, exact coordinates/PVT/ECEF, and binary CRC/recovery.
The minimal host Stream fixture tests driver behavior; it does not emulate
physical UART timing or establish positioning accuracy. Arduino builds exercise
the real core interfaces. Focused HILBERT tests separately prove identification,
precision changes and NMEA output effects with original settings restored.
