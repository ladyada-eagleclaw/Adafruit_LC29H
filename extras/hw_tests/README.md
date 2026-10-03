# LC29H hardware tests

## BA breakout over I²C, observed on 2026-10-02

Fixture: LC29H BA breakout Rev A and a USB-connected Metro Mini (ATmega328P).
Metro 5 V powers breakout VIN/host-side pullups; A4 is SDA, A5 is SCL, and GND
is shared. The breakout's CH9102 USB port is also connected, with its UART
switch on USB. Neither that switch nor USB power alone replaces VIN for I²C.
No wheel-tick, FWD, PPS, wake or reset GPIO connection is configured by the tests.

Firmware: `LC29HBANR11A06S_CSA4`, build `2025/05/28,09:36:24`.
Arduino target: `arduino:avr:uno`. Enable the AVR Wire timeout as in the sketches.

| Test | Observed result |
| --- | --- |
| `03_ba_i2c` | Two firmware queries completed over I²C. The measurement window received 434 checksum-valid sentences, zero invalid sentences and 23 GGA records. Counters exclude startup draining. Autonomous fixes were also observed; no RTK correction source was connected. |
| `04_ba_imu` | Saved the six standard NMEA rates (all 1), module-frame IMU rate (0) and calibration rate (0). Temporarily selected sensor-only I²C output. DR configuration reported enabled; calibration reports decoded as uncalibrated. Received 200 six-axis reports in 20 seconds at a requested 10 Hz, with zero invalid/implausible samples. Disabling IMU output produced zero reports in the subsequent two-second window. All eight original rates were restored and read back. |

The IMU was stationary at about 27 °C. Acceleration was approximately
`(-2.18, -2.54, 9.15) m/s²` in module axes, consistent with a tilted board under
gravity. Gyro output was near zero. Report timestamps had small scheduling
variation: 92–101 ms was observed during test development; the passing run was
95–101 ms. The test permits ±10 ms around 100 ms and rejects duplicate/missed
epochs. This is a data-path/plausibility check, not an IMU accuracy calibration.

Send `RUN` followed by a newline to arm `04_ba_imu`. It disables GSV first to
reduce bandwidth before the remaining configuration queries, and restores GSV
last. It restores settings after a failure when their originals are known.
Neither test writes NVM. Raw logs stay outside Git because navigation includes
location data.

The public `i2c_basic` sketch also ran on the Metro: startup accepted the GSV
divisor of 5 and high-precision settings, followed by 67 exact position records
in a 90-second capture. A simultaneous passive USB capture confirmed five
checksum-valid GGA records with eight fractional-minute digits for both
coordinates and an autonomous 16-satellite fix. USB stayed available while I²C
was active. The sketch remains loaded for further testing.
A subsequent Metro-only reset, with the GPS still running and fixed, completed
startup and delivered 37 position records with one sketch startup. No GPS reset
was needed once the slower satellite output had been selected.

All eight host regression sources pass with ASan/UBSan and warnings as errors.
All eight public examples compile on Mega and Feather ESP32-S3. The two I²C
examples and the decoder also compile on Uno (19 builds total). Uno memory:
`i2c_basic` uses 19,440 bytes flash / 984 bytes static RAM; `ba_imu` uses 16,846 /
1,025. The decoder uses 9,456 / 733; the new transport bookkeeping adds six bytes
of static RAM to the prior decoder build. Doxygen 1.8.13 reports no diagnostics.

The default full satellite inventory can outpace 32-byte AVR I²C reads. A long
pause or host reset may leave a large backlog or interrupt the receiver's
multi-step I²C protocol. Vendor endpoint recovery restored an ACK, but that
alone did not establish communication with a full/backlogged FIFO. A full
module reboot through the independent USB UART cleared the backlog and allowed
the tests to start. PAIR004 hot navigation restart did not clear that FIFO.
The successful USB recovery used `PAIR023`; it is not added as an automatic
library reset. Keep this startup limitation visible when evaluating I²C use.
The public navigation example slows GSV and the IMU example disables standard
NMEA on its port. Command acceptance alone is not proof of available bandwidth.

Wheel-count/direction decoding has host coverage, but physical wheel input and
the other breakout pads remain untested. Vehicle-frame IMU, driving calibration,
dead-reckoning accuracy, high IMU rates and corrected RTK fixes remain unverified.

## EA on HILBERT

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
