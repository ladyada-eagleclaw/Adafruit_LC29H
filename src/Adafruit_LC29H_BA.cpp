/** @file Adafruit_LC29H_BA.cpp
 * @brief BA reports from Quectel DR & RTK Application Note V1.5, chapter 4.
 * Written for Adafruit Industries. MIT license; see license.txt.
 */
#include "Adafruit_LC29H_BA.h"

#include <string.h>

/** @brief Construct a BA receiver without attaching or configuring hardware.
 * @param firstBuffer First receive buffer, kept alive with this receiver.
 * @param secondBuffer Second non-overlapping buffer.
 * @param capacity Bytes per buffer, including NUL. 160 suits these reports.
 */
Adafruit_LC29H_BA::Adafruit_LC29H_BA(volatile char* firstBuffer,
                                     volatile char* secondBuffer,
                                     size_t capacity)
    : Adafruit_LC29H(firstBuffer, secondBuffer, capacity) {}

/** @brief Select IMU output on the current port without saving to flash.
 * @param rate Samples per second: 0 disables, or 10, 20, 50, 100.
 * @param frame Module axes (four-wheel firmware) or calibrated vehicle axes.
 * @return True only when the receiver accepts the command. Acceptance does
 * not prove calibration or available I2C/host bandwidth. Vehicle-frame output
 * needs 10 Hz position fixes and completed DR calibration. Start at 10 Hz,
 * especially on an AVR with 32-byte I2C transfers.
 */
bool Adafruit_LC29H_BA::setIMURate(uint8_t rate, lc29h_imu_frame_t frame) {
  return setMessageRate("PQTMSENMSG", rate, frame);
}

/** @brief Read the IMU rate from the current receiver port.
 * @param frame Module or vehicle reference frame.
 * @return 0, 10, 20, 50, or 100 Hz, or -1 on rejected/invalid/timed-out reply.
 */
int8_t Adafruit_LC29H_BA::getIMURate(lc29h_imu_frame_t frame) {
  return getMessageRate("PQTMSENMSG", frame);
}

/** @brief Query whether the BA's dead-reckoning engine is enabled.
 * @return 0 disabled, 1 enabled, or -1 on failure. Four-wheel firmware only.
 * This reports configuration, not calibration or a valid navigation solution.
 */
int8_t Adafruit_LC29H_BA::isDeadReckoningEnabled() {
  char response[64];
  if (!sendPQTMCommand("PQTMCFGDR,R", response, sizeof(response)))
    return -1;
  nmea_span_t values[2];
  int32_t value;
  if (!fields(validate(response, strlen(response)).fields, values, 2) ||
      !integer(values[1], 0, 1, value)) {
    badReply();
    return -1;
  }
  return (int8_t)value;
}

/** @brief Decode a complete unsigned 32-bit decimal field.
 * @param field Entire field; signs and fractions are forbidden.
 * @param value Updated only after syntax and range checks pass.
 * @return True for 0..4294967295, including timestamps beyond signed range.
 */
bool Adafruit_LC29H_BA::unsigned32(nmea_span_t field, uint32_t& value) {
  if (!field.data || !field.length)
    return false;
  for (size_t i = 0; i < field.length; i++)
    if (field.data[i] < '0' || field.data[i] > '9')
      return false;
  nmea_decimal_t parsed = parseDecimal(field);
  if (parsed.status != NMEA_NUMBER_VALID || parsed.coefficient > UINT32_MAX)
    return false;
  value = (uint32_t)parsed.coefficient;
  return true;
}

/** @brief Convert a validated IMU decimal to a convenience float.
 * @param field Complete numeric field; empty/NaN/Inf/exponent syntax fails.
 * @param value Updated only on success.
 * @return True for a valid bounded decimal. Never used for GNSS coordinates.
 */
bool Adafruit_LC29H_BA::real(nmea_span_t field, float& value) {
  nmea_decimal_t parsed = parseDecimal(field);
  if (parsed.status != NMEA_NUMBER_VALID)
    return false;
  value = (float)parsed.coefficient;
  for (uint8_t i = 0; i < parsed.decimalPlaces; i++)
    value /= 10;
  return true;
}

/** @brief Decode PQTMSENMSG version 2 or 4 into one coherent SI sample.
 * @param sentence Unchanged checked view from validate()/lastSentence().
 * @return VALID with all six axes and temperature, otherwise an empty result.
 * Frame 2 uses module axes; frame 4 uses vehicle axes. Preserve that
 * distinction. Datasheet fields are gyro degrees/s and acceleration g; they are
 * converted to radians/s and meters/s squared here. No GNSS or DR calibration
 * is inferred.
 */
lc29h_imu_t Adafruit_LC29H_BA::parseIMU(const nmea_sentence_t& sentence) {
  lc29h_imu_t result = {};
  result.status = replyStatus(sentence, "PQTMSENMSG");
  if (result.status != LC29H_REPLY_VALID)
    return result;
  result.status = LC29H_REPLY_INVALID_FIELDS;
  nmea_span_t values[9];
  int32_t version;
  lc29h_imu_t parsed = {};
  if (!fields(sentence.fields, values, 9) ||
      !integer(values[0], 2, 4, version) || (version != 2 && version != 4) ||
      !unsigned32(values[1], parsed.timestamp) ||
      !real(values[2], parsed.temperature))
    return result;
  for (uint8_t i = 0; i < 3; i++) {
    if (!real(values[3 + i], parsed.angularVelocity[i]) ||
        !real(values[6 + i], parsed.acceleration[i]))
      return result;
    parsed.angularVelocity[i] *= LC29H_DEGREES_TO_RADIANS;
    parsed.acceleration[i] *= LC29H_STANDARD_GRAVITY;
  }
  parsed.frame = (lc29h_imu_frame_t)version;
  parsed.status = LC29H_REPLY_VALID;
  return parsed;
}

/** @brief Decode one version-1 PQTMDRCAL report.
 * @param sentence Validated NMEA view.
 * @return Owned calibration/source, or failure with cleared payload.
 * Enable reports with setMessageRate("PQTMDRCAL", 1, 1). This is independent
 * of the standard NMEA fix-quality and RTK status.
 */
lc29h_calibration_t Adafruit_LC29H_BA::parseCalibration(
    const nmea_sentence_t& sentence) {
  lc29h_calibration_t result = {};
  result.status = replyStatus(sentence, "PQTMDRCAL");
  if (result.status != LC29H_REPLY_VALID)
    return result;
  result.status = LC29H_REPLY_INVALID_FIELDS;
  nmea_span_t values[3];
  int32_t version, state, source;
  if (!fields(sentence.fields, values, 3) ||
      !integer(values[0], 1, 1, version) || !integer(values[1], 0, 3, state) ||
      !integer(values[2], 0, 3, source))
    return result;
  result.calibration = (lc29h_calibration_state_t)state;
  result.source = (lc29h_navigation_source_t)source;
  result.status = LC29H_REPLY_VALID;
  return result;
}

/** @brief Decode the cumulative wheel count and direction in PQTMVEHMSG v2.
 * @param sentence Validated NMEA view.
 * @return Complete unsigned 32-bit count/timestamp, or cleared failure.
 * Enable reports with setMessageRate("PQTMVEHMSG", 1, 2). A count alone does
 * not prove wheel-pin wiring, scaling, or DR calibration. This method never
 * drives the WHEELTICK/FWD pads or injects a simulated speed into navigation.
 */
lc29h_wheel_ticks_t Adafruit_LC29H_BA::parseWheelTicks(
    const nmea_sentence_t& sentence) {
  lc29h_wheel_ticks_t result = {};
  result.status = replyStatus(sentence, "PQTMVEHMSG");
  if (result.status != LC29H_REPLY_VALID)
    return result;
  result.status = LC29H_REPLY_INVALID_FIELDS;
  lc29h_wheel_ticks_t parsed = {};
  nmea_span_t values[4];
  int32_t version, direction;
  if (!fields(sentence.fields, values, 4) ||
      !integer(values[0], 2, 2, version) ||
      !unsigned32(values[1], parsed.timestamp) ||
      !unsigned32(values[2], parsed.ticks) ||
      !integer(values[3], 0, 2, direction))
    return result;
  parsed.direction = (lc29h_wheel_direction_t)direction;
  parsed.status = LC29H_REPLY_VALID;
  return parsed;
}
