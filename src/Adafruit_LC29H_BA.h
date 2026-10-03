/** @file Adafruit_LC29H_BA.h
 * @brief LC29H BA inertial sensing and dead-reckoning reports.
 * Written for Adafruit Industries. MIT license; see license.txt.
 */
#ifndef ADAFRUIT_LC29H_BA_H
#define ADAFRUIT_LC29H_BA_H

#include "Adafruit_LC29H.h"

/// Standard gravity in meters per second squared, for the vendor's g values.
#define LC29H_STANDARD_GRAVITY 9.80665f
/// Radians per degree, for the vendor's degrees-per-second gyro values.
#define LC29H_DEGREES_TO_RADIANS 0.017453292519943295f

/// Coordinate frame of PQTMSENMSG measurements; not an IMU axis remapping.
typedef enum : uint8_t {
  LC29H_IMU_MODULE = 2, ///< Unfiltered module axes; four-wheel firmware only.
  LC29H_IMU_VEHICLE =
      4 ///< Vehicle axes; requires DR calibration and 10 Hz fixes.
} lc29h_imu_frame_t;

/// One six-axis IMU report, converted to SI units. Check status first.
typedef struct {
  lc29h_reply_status_t status; ///< VALID only after every field is decoded.
  lc29h_imu_frame_t frame; ///< Module or vehicle axes; no implicit remapping.
  uint32_t
      timestamp; ///< Milliseconds since receiver power-on, wraps at 32 bits.
  float temperature;        ///< IMU temperature, degrees Celsius.
  float acceleration[3];    ///< X/Y/Z acceleration, meters per second squared.
  float angularVelocity[3]; ///< X/Y/Z angular rate, radians per second.
} lc29h_imu_t;

/// Calibration states reported by PQTMDRCAL, not inferred from GPS fix quality.
typedef enum : uint8_t {
  LC29H_DR_UNCALIBRATED = 0,       ///< Calibration has not completed.
  LC29H_DR_LIGHTLY_CALIBRATED = 1, ///< Partial calibration.
  LC29H_DR_CALIBRATED = 2,         ///< Fully calibrated.
  LC29H_DR_PRECISE_HEADING =
      3 ///< Fully calibrated with high-precision heading.
} lc29h_calibration_state_t;

/// Source of the position reported by the BA dead-reckoning engine.
typedef enum : uint8_t {
  LC29H_DR_NO_POSITION = 0, ///< No position solution.
  LC29H_DR_GNSS = 1,        ///< GNSS only.
  LC29H_DR_INERTIAL = 2,    ///< Dead reckoning only.
  LC29H_DR_COMBINED = 3     ///< GNSS and dead reckoning.
} lc29h_navigation_source_t;

/// One complete PQTMDRCAL record. No earlier calibration state is cached.
typedef struct {
  lc29h_reply_status_t status; ///< Check before using calibration/source.
  lc29h_calibration_state_t calibration; ///< Receiver calibration state.
  lc29h_navigation_source_t
      source; ///< Source of the current navigation result.
} lc29h_calibration_t;

/// Direction in a PQTMVEHMSG wheel-tick report; distinct from FWD pin voltage.
typedef enum : uint8_t {
  LC29H_WHEEL_UNKNOWN = 0, ///< Invalid/unknown movement direction.
  LC29H_WHEEL_FORWARD = 1, ///< Forward movement.
  LC29H_WHEEL_BACKWARD = 2 ///< Backward movement.
} lc29h_wheel_direction_t;

/// One version-2 cumulative wheel-tick report, ready for future pad tests.
typedef struct {
  lc29h_reply_status_t status; ///< VALID only on a complete version-2 report.
  uint32_t timestamp;          ///< Receiver milliseconds since power-on.
  uint32_t ticks; ///< Unsigned cumulative wheel ticks, not a distance.
  lc29h_wheel_direction_t direction; ///< Receiver-reported movement direction.
} lc29h_wheel_ticks_t;

/** BA extras over the shared LC29H navigation and UART/I2C command path.
 * The module runs sensor fusion itself. This class decodes its reports; it
 * neither reads the IMU chip directly nor implements a second fusion engine.
 * Firmware variants differ: a BA identity alone does not prove a command is
 * supported. Check each command's result. Navigation remains precision-safe;
 * only IMU convenience values are converted to float SI units.
 * Raw module-frame IMU reports do not require a GNSS fix. Vehicle-frame reports
 * require calibration while driving; stationary bench tests cannot prove DR.
 */
class Adafruit_LC29H_BA : public Adafruit_LC29H {
 public:
  Adafruit_LC29H_BA(volatile char* firstBuffer, volatile char* secondBuffer,
                    size_t capacity);
  bool setIMURate(uint8_t rate, lc29h_imu_frame_t frame = LC29H_IMU_MODULE);
  int8_t getIMURate(lc29h_imu_frame_t frame = LC29H_IMU_MODULE);
  int8_t isDeadReckoningEnabled();
  static lc29h_imu_t parseIMU(const nmea_sentence_t& sentence);
  static lc29h_calibration_t parseCalibration(const nmea_sentence_t& sentence);
  static lc29h_wheel_ticks_t parseWheelTicks(const nmea_sentence_t& sentence);

 private:
  static bool unsigned32(nmea_span_t field, uint32_t& value);
  static bool real(nmea_span_t field, float& value);
};

#endif
