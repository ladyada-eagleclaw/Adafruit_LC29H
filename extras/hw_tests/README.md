# HILBERT LC29H hardware tests

Fixture: HILBERT Rev A, ESP32-S3, LC29H(EA), L1+L5 antenna. LC29H TX is GPIO8;
GPIO9 drives RX through the board's 1 kOhm/5.1 kOhm divider. Only those UART pins
are configured. The other receiver, reset, PPS and power-control pins are untouched.

Arduino target used:

```
esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,FlashSize=4M,FlashMode=qio,PSRAM=enabled
```

The engineering fixtures are outside the normal public-example CI matrix.
They must be compiled and run deliberately on the named hardware. Send `RUN`
with a newline to start either configuration test after uploading. This prevents
USB reset/upload activity from repeating a setter test mid-upload. Each setter
test restores the original volatile setting and performs no NVM writes.

## Observed on 2026-09-16

Firmware: `LC29HEANR11A03S_RSA`, build `2023/10/31,16:52:14`.

| Test | Observed result |
| --- | --- |
| `00_hilbert` | Firmware identification and PAIR/PQTM readback succeed. UART 460800, fix interval 100 ms, GGA enabled, normal navigation, AIC enabled, rover role, survey disabled. |
| `01_precision` | Eight fractional minute digits and three altitude/geoid digits requested and read back. 40/40 sampled GGA lines contained those digits. Original precision restored and observed in subsequent navigation. |
| `02_nmea_output` | GGA enabled: 30 sentences/3 s; disabled: 0/3 s while 150 other navigation lines continued; restored: 30/3 s. Readback matched each state. |

Original precision readback was `3,6,1,2,3,2` (time, position, altitude, DOP,
speed, course). Tests restore that original state; the basic example deliberately
selects high precision for its own operation.

PAIR433, PAIR435 and PAIR437 queries did not answer on this firmware. Their
readback APIs return failure after the deadline. Their setters were not exercised
because a successful restore could not be established. The same API paths,
including signed RTCM mode and exact packets, are covered by host tests.

Survey/base writes, NVM persistence, baud changes, restart behavior and corrected
RTK positioning are not established by these tests. They have protocol-derived
APIs and host regressions. A live correction feed and suitable base reference
are required to validate an RTK solution. Do not infer corrected accuracy from
the antenna, numeric precision, a command acknowledgment or an autonomous fix.

Raw serial logs stay outside Git because they may contain location data.

The final driver was also compiled across 17 public-example builds: all six
examples on ESP32-S3 and Mega, the decoder on Uno, and basic/fulltest on Adafruit
SAMD M4 and nRF52840. The Uno decoder uses 9,424 bytes of flash and 727 bytes of
static RAM; this is a parser example, not a claim that Uno can sustain the EA's
default UART baud. All six host regression sources pass with ASan/UBSan.
