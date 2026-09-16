/** @file LC29H_navigation.cpp
 * @brief Exact GGA and survey status decoding.
 * Written for Adafruit Industries. MIT license; see license.txt.
 */
#include <string.h>

#include "Adafruit_LC29H.h"

/** @brief Decode GGA position, altitude and differential correction
 * information.
 * @param sentence Validated, unchanged sentence view.
 * @return Owned values; check position.validation and each number status. Empty
 * fields remain EMPTY. Any malformed populated field clears all payload values.
 * Altitude and separation require meter units when populated. No float is used.
 */
lc29h_fix_t Adafruit_LC29H::parseFix(const nmea_sentence_t& sentence) {
  lc29h_fix_t result = {};
  result.position = parsePosition(sentence);
  result.satellitesStatus = NMEA_NUMBER_MISSING;
  result.hdop.status = result.altitude.status = result.separation.status =
      result.correctionAge.status = NMEA_NUMBER_MISSING;
  if (result.position.validation.status != GNSS_SENTENCE_VALID)
    return result;
  if (sentence.address.length != 5 ||
      memcmp(sentence.address.data + 2, "GGA", 3)) {
    result.position = parsePosition(validate(NULL, 0));
    result.position.validation.status = GNSS_SENTENCE_UNSUPPORTED;
    return result;
  }
  lc29h_fix_t parsed = result;
  nmea_span_t remaining = sentence.fields, values[14];
  for (uint8_t i = 0; i < 14; i++)
    values[i] = nextField(remaining);
  // GGA has fourteen field positions; correction age and station ID can be
  // empty.
  bool valid = values[13].data && !nextField(remaining).data;
  int32_t satellites = 0;
  if (values[6].length) {
    valid = valid && integer(values[6], 0, 999, satellites);
    parsed.satellitesStatus = NMEA_NUMBER_VALID;
  } else
    parsed.satellitesStatus = NMEA_NUMBER_EMPTY;
  parsed.satellites = satellites;
  parsed.hdop = parseDecimal(values[7]);
  parsed.altitude = parseDecimal(values[8]);
  parsed.separation = parseDecimal(values[10]);
  parsed.correctionAge = parseDecimal(values[12]);
  const nmea_decimal_t numbers[] = {parsed.hdop, parsed.altitude,
                                    parsed.separation, parsed.correctionAge};
  for (uint8_t i = 0; i < 4; i++) {
    if (numbers[i].status != NMEA_NUMBER_VALID &&
        numbers[i].status != NMEA_NUMBER_EMPTY)
      valid = false;
  }
  if (parsed.hdop.coefficient < 0 || parsed.correctionAge.coefficient < 0)
    valid = false;
  if (values[8].length && !matches(values[9], "M"))
    valid = false;
  if (values[10].length && !matches(values[11], "M"))
    valid = false;
  if (values[13].length) {
    int32_t station;
    valid = valid && integer(values[13], 0, 4095, station);
  }
  if (valid)
    return parsed;
  result.position = parsePosition(validate(NULL, 0));
  result.position.validation.status = GNSS_SENTENCE_INVALID_FIELD;
  return result;
}
/** @brief Decode one survey-in status with exact ECEF values.
 * @param sentence Unchanged view from validate()/lastSentence().
 * @return Owned data with VALID status only after the entire payload validates.
 * Reserved fields are retained only structurally; validity 0/1/2 does not imply
 * an independently surveyed base location or centimeter absolute accuracy. */
lc29h_survey_status_t Adafruit_LC29H::parseSurveyStatus(
    const nmea_sentence_t& sentence) {
  lc29h_survey_status_t result = {};
  result.status = replyStatus(sentence, "PQTMSVINSTATUS");
  result.x.status = result.y.status = result.z.status = result.accuracy.status =
      NMEA_NUMBER_MISSING;
  if (result.status != LC29H_REPLY_VALID)
    return result;
  result.status = LC29H_REPLY_INVALID_FIELDS;
  nmea_span_t value[11];
  int32_t version, tow, validity, observations, duration;
  if (!fields(sentence.fields, value, 11) ||
      !integer(value[0], 1, 1, version) ||
      !integer(value[1], 0, 604799999, tow) ||
      !integer(value[2], 0, 2, validity) ||
      !integer(value[5], 0, INT32_MAX, observations) ||
      !integer(value[6], 0, 86400, duration))
    return result;
  nmea_decimal_t x = parseDecimal(value[7]), y = parseDecimal(value[8]),
                 z = parseDecimal(value[9]), accuracy = parseDecimal(value[10]);
  if (x.status != NMEA_NUMBER_VALID || y.status != NMEA_NUMBER_VALID ||
      z.status != NMEA_NUMBER_VALID || accuracy.status != NMEA_NUMBER_VALID ||
      accuracy.coefficient < 0)
    return result;
  result.timeOfWeek = tow;
  result.validity = validity;
  result.observations = observations;
  result.configuredDuration = duration;
  result.x = x;
  result.y = y;
  result.z = z;
  result.accuracy = accuracy;
  result.status = LC29H_REPLY_VALID;
  return result;
}

/** @brief Check an exact decimal against integral bounds without rounding.
 * @param value Parsed decimal.
 * @param minimum Inclusive lower bound.
 * @param maximum Inclusive upper bound.
 * @return True for a VALID value inside the range. */
bool Adafruit_LC29H::decimalRange(nmea_decimal_t value, int32_t minimum,
                                  int32_t maximum) {
  if (value.status != NMEA_NUMBER_VALID)
    return false;
  int64_t whole = value.coefficient;
  bool remainder = false;
  for (uint16_t i = 0; i < value.decimalPlaces; i++) {
    remainder = remainder || whole % 10 != 0;
    whole /= 10;
  }
  if (whole < minimum || whole > maximum)
    return false;
  if (remainder && ((whole == minimum && value.coefficient < 0) ||
                    (whole == maximum && value.coefficient > 0)))
    return false;
  return true;
}
/** @brief Decode exact speed/course from RMC alongside its position and UTC.
 * @param sentence Validated unchanged NMEA view.
 * @return Owned values. Check position.validation and position.fix, then each
 * numeric status. Knots remain exact; converting to SI is application policy.
 * Invalid populated fields clear the entire result, including position. */
lc29h_motion_t Adafruit_LC29H::parseMotion(const nmea_sentence_t& sentence) {
  lc29h_motion_t result = {};
  result.speedKnots.status = result.course.status = NMEA_NUMBER_MISSING;
  result.position = parsePosition(sentence);
  if (result.position.validation.status != GNSS_SENTENCE_VALID)
    return result;
  if (sentence.address.length != 5 ||
      memcmp(sentence.address.data + 2, "RMC", 3)) {
    result.position = parsePosition(validate(NULL, 0));
    result.position.validation.status = GNSS_SENTENCE_UNSUPPORTED;
    return result;
  }
  nmea_span_t remaining = sentence.fields;
  for (uint8_t i = 0; i < 6; i++)
    nextField(remaining);
  nmea_decimal_t speed = parseDecimal(nextField(remaining));
  nmea_decimal_t course = parseDecimal(nextField(remaining));
  bool validSpeed =
      speed.status == NMEA_NUMBER_EMPTY ||
      (speed.status == NMEA_NUMBER_VALID && speed.coefficient >= 0);
  bool validCourse =
      course.status == NMEA_NUMBER_EMPTY || decimalRange(course, 0, 360);
  if (!validSpeed || !validCourse) {
    result.position = parsePosition(validate(NULL, 0));
    result.position.validation.status = GNSS_SENTENCE_INVALID_FIELD;
    return result;
  }
  result.speedKnots = speed;
  result.course = course;
  return result;
}
/** @brief Decode PQTMPVT version 1 without reducing coordinate precision.
 * @param sentence Unchanged validated sentence view.
 * @return Owned exact measurements with EMPTY statuses for unavailable values.
 * A valid record with fixMode zero is not a fix. PQTMPVT does not distinguish
 * RTK float/fixed; use contemporaneous GGA quality for that information.
 * Any invalid populated measurement clears all measurements atomically. */
lc29h_pvt_t Adafruit_LC29H::parsePVT(const nmea_sentence_t& sentence) {
  lc29h_pvt_t result = {};
  result.status = replyStatus(sentence, "PQTMPVT");
  result.time.status = NMEA_NUMBER_MISSING;
  result.leapSeconds.status = result.latitude.status = result.longitude.status =
      result.altitude.status = result.separation.status =
          result.northVelocity.status = result.eastVelocity.status =
              result.downVelocity.status = result.speed.status =
                  result.heading.status = result.hdop.status =
                      result.pdop.status = NMEA_NUMBER_MISSING;
  if (result.status != LC29H_REPLY_VALID)
    return result;
  result.status = LC29H_REPLY_INVALID_FIELDS;
  nmea_span_t field[19];
  int32_t version, tow, date, mode, satellites;
  if (!fields(sentence.fields, field, 19) ||
      !integer(field[0], 1, 1, version) ||
      !integer(field[1], 0, 604799999, tow) ||
      !integer(field[2], 10000101, 99991231, date) || field[2].length != 8 ||
      !integer(field[5], 0, 3, mode) || mode == 1 ||
      !integer(field[6], 0, 999, satellites))
    return result;
  uint16_t year = date / 10000;
  uint8_t month = date / 100 % 100, day = date % 100;
  const uint8_t days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (!month || month > 12)
    return result;
  uint8_t maximumDay = days[month - 1];
  if (month == 2 && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)))
    maximumDay++;
  if (!day || day > maximumDay)
    return result;
  gnss_time_t time = parseTime(field[3]);
  if (time.status != NMEA_NUMBER_VALID)
    return result;
  nmea_decimal_t values[12];
  for (uint8_t i = 0; i < 12; i++) {
    values[i] = parseDecimal(field[i + 7]);
    if (values[i].status != NMEA_NUMBER_VALID &&
        values[i].status != NMEA_NUMBER_EMPTY)
      return result;
  }
  // Leap seconds is an optional unsigned integral count.
  if (values[0].status == NMEA_NUMBER_VALID &&
      (values[0].decimalPlaces || !decimalRange(values[0], 0, 255)))
    return result;
  if ((values[1].status == NMEA_NUMBER_VALID &&
       !decimalRange(values[1], -90, 90)) ||
      (values[2].status == NMEA_NUMBER_VALID &&
       !decimalRange(values[2], -180, 180)) ||
      (values[9].status == NMEA_NUMBER_VALID &&
       !decimalRange(values[9], 0, 360)))
    return result;
  if ((values[1].status == NMEA_NUMBER_EMPTY) !=
      (values[2].status == NMEA_NUMBER_EMPTY))
    return result;
  if (values[8].coefficient < 0 || values[10].coefficient < 0 ||
      values[11].coefficient < 0)
    return result;
  result.status = LC29H_REPLY_VALID;
  result.timeOfWeek = tow;
  result.date = date;
  result.time = time;
  result.fixMode = mode;
  result.satellites = satellites;
  result.leapSeconds = values[0];
  result.latitude = values[1];
  result.longitude = values[2];
  result.altitude = values[3];
  result.separation = values[4];
  result.northVelocity = values[5];
  result.eastVelocity = values[6];
  result.downVelocity = values[7];
  result.speed = values[8];
  result.heading = values[9];
  result.hdop = values[10];
  result.pdop = values[11];
  return result;
}
