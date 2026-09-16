# Adafruit LC29H [![Arduino Library CI](https://github.com/adafruit/Adafruit_LC29H/actions/workflows/githubci.yml/badge.svg)](https://github.com/adafruit/Adafruit_LC29H/actions) [![Documentation](https://img.shields.io/badge/documentation-doxygen-blue.svg)](https://adafruit.github.io/Adafruit_LC29H/html/index.html)

Arduino library for Quectel LC29H GNSS modules, starting with the LC29H(EA).

**Under development: this repository currently contains an API proposal, not a
working Arduino library.** Review the [proposed header](extras/design/Adafruit_LC29H.h)
and [receiver contract](extras/design/LC29H.md). They cover shared exact-position
decoding, PAIR command acknowledgments, and firmware-version replies.

The driver will support additional LC29H variants as their protocols and
hardware are tested. General LC29-family compatibility is not promised.

## Development dependency

The shared `Adafruit_NMEA` and `Adafruit_GNSS` classes live in
[Adafruit GPS](https://github.com/adafruit/Adafruit_GPS). They are reused through
`#include <Adafruit_GNSS.h>`; their source is not copied into this repository.

For proposal compile checks, use GPS revision
[`4213cb547883b55e70e76cba07cfff1bae7a680b`](https://github.com/adafruit/Adafruit_GPS/tree/4213cb547883b55e70e76cba07cfff1bae7a680b)
and add its `src` directory to the compiler include path. The released GPS
1.8.0 does not contain this new core. Arduino Library Manager dependency metadata
will be added when a released GPS version supplies it and this driver is ready.

The proposal deliberately remains under `extras/design` until its interface is
reviewed. Its methods have no implementations and cannot yet be linked into a
sketch. Library examples, Arduino Library CI, and generated documentation will
be enabled with implementation; the badges above reserve their normal links.

## License

BSD license; see [license.txt](license.txt). Written for Adafruit Industries.
The proposal originated in [Adafruit GPS PR #201](https://github.com/adafruit/Adafruit_GPS/pull/201)
and continues here so receiver-specific code has its own repository.
