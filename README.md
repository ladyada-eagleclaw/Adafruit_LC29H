# Adafruit LC29H [![Arduino Library CI](https://github.com/adafruit/Adafruit_LC29H/actions/workflows/githubci.yml/badge.svg)](https://github.com/adafruit/Adafruit_LC29H/actions) [![Documentation](https://img.shields.io/badge/documentation-doxygen-blue.svg)](https://adafruit.github.io/Adafruit_LC29H/html/index.html)

Arduino library for Quectel LC29H GNSS modules, starting with the LC29H(EA).

This first implementation receives caller-supplied NMEA bytes, preserves exact
GGA/RMC/GLL positions, and decodes PAIR acknowledgments and firmware-version
replies. UART startup, command transactions, configuration, and RTCM input are
still to come. Additional LC29H variants require protocol and hardware testing.

## Installation

Install **Adafruit GPS Library 1.9.0 or later**, then install this library from
its source ZIP. The shared NMEA/GNSS implementation is a dependency and is not
copied here. If Library Manager has not indexed GPS 1.9.0 yet, install the
[released source ZIP](https://github.com/adafruit/Adafruit_GPS/archive/refs/tags/1.9.0.zip).

The current Adafruit CI helper installs dependencies by name, so
`library.properties` lists the established GPS library name. Version 1.9.0 is
the minimum: older releases do not provide `Adafruit_GNSS.h`. CI checks out the
1.9.0 release explicitly for both host tests and Arduino builds.

## Receiving sentences

Provide two non-overlapping receive buffers that outlive the `Adafruit_LC29H`
object. Pass each byte and its receive time to `feed()`. When it returns
`NMEA_FRAME_VALID`, inspect `lastPosition()`, `lastPairAck()`, or `lastVersion()`.
These accessors describe the same latest completed line; they do not retain
older positions or replies. Process the line before feeding another one.

Position and acknowledgment results own their values. Firmware text spans
borrow the receive buffer and expire on the next complete line or parser reset.
Check result and field statuses. A decoded acknowledgment can report rejection
or continued processing; a positive acknowledgment does not prove a position fix
or query readback. Match its command ID in the application.

Use exact coordinate components or `formatCoordinate()` to retain finer detail
than E7/float convenience values. The
[serial_decode example](examples/serial_decode/serial_decode.ino) accepts pasted
sentences through Serial Monitor and displays results without sending receiver
commands. The [receiver contract](extras/design/LC29H.md) records protocol,
lifetime, and future transport requirements.

## Tests

With a GPS source checkout available, run on Linux:

```sh
python3 extras/tests/run_tests.py --gps-dir /path/to/Adafruit_GPS
```

The runner discovers every C++ test under `extras/tests`, using address and
undefined-behavior sanitizers. CI runs the same command, compiles the Arduino
example, checks formatting, and generates documentation. These parser tests do
not establish physical UART-command or RTK correction performance.

## License

MIT license; see [license.txt](license.txt). Written for Adafruit Industries.
The separately installed Adafruit GPS dependency retains its BSD license.
The interface originated in [Adafruit GPS PR #201](https://github.com/adafruit/Adafruit_GPS/pull/201)
and continues here so receiver-specific code has its own repository.
