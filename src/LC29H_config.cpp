/** @file LC29H_config.cpp
 * @brief LC29H(EA) settings with receiver readback, without cached
 * configuration. Written for Adafruit Industries. MIT license; see license.txt.
 */
#include <stdio.h>
#include <string.h>

#include "Adafruit_LC29H.h"

/** @brief Split exactly the requested number of fields.
 * @param remaining Input span.
 * @param values Output span array.
 * @param count Required field count.
 * @return True if no fields are missing or extra; empty fields are retained. */
bool Adafruit_LC29H::fields(nmea_span_t remaining, nmea_span_t* values,
                            size_t count) {
  for (size_t i = 0; i < count; i++) {
    values[i] = nextField(remaining);
    if (!values[i].data)
      return false;
  }
  return !nextField(remaining).data;
}
/** @brief Parse a bounded integer, rejecting spaces, plus signs and decimals.
 * @param field Entire field.
 * @param minimum Inclusive lower bound.
 * @param maximum Inclusive upper bound.
 * @param result Written only on success.
 * @return True when syntax and range are valid. */
bool Adafruit_LC29H::integer(nmea_span_t field, int32_t minimum,
                             int32_t maximum, int32_t& result) {
  if (!field.data || !field.length)
    return false;
  size_t start = 0;
  if (minimum < 0 && field.data[0] == '-')
    start = 1;
  if (start == field.length)
    return false;
  for (size_t i = start; i < field.length; i++)
    if (field.data[i] < '0' || field.data[i] > '9')
      return false;
  nmea_decimal_t value = parseDecimal(field);
  if (value.status != NMEA_NUMBER_VALID || value.coefficient < minimum ||
      value.coefficient > maximum)
    return false;
  result = (int32_t)value.coefficient;
  return true;
}
/** @brief Send one signed integer parameter in a PAIR command.
 * @param command Command ID.
 * @param value Parameter.
 * @return Matching positive acknowledgment. */
bool Adafruit_LC29H::pairValue(uint16_t command, int32_t value) {
  char text[12];
  snprintf(text, sizeof(text), "%ld", (long)value);
  return sendPairCommand(command, text);
}
/** @brief Query and validate a single signed integer PAIR result.
 * @param command Query ID.
 * @param minimum Minimum valid result.
 * @param maximum Maximum valid result.
 * @param failure Failure sentinel outside the allowed range.
 * @param parameters Optional query parameters.
 * @return Receiver value or failure sentinel. */
int32_t Adafruit_LC29H::readPairValue(uint16_t command, int32_t minimum,
                                      int32_t maximum, int32_t failure,
                                      const char* parameters) {
  char response[64];
  if (!queryPair(command, parameters, response, sizeof(response)))
    return failure;
  nmea_span_t value[1];
  int32_t result;
  if (!fields(validate(response, strlen(response)).fields, value, 1) ||
      !integer(value[0], minimum, maximum, result)) {
    badReply();
    return failure;
  }
  return result;
}
/** @brief Request a receiver restart; parser reset() is a separate operation.
 * @param mode Hot, warm, cold or full cold. Full cold discards configurations.
 * @return True if acknowledged, not a promise that reboot or reacquisition
 * ended. Readiness must be established again using getVersion()/received
 * navigation. */
bool Adafruit_LC29H::restart(lc29h_start_t mode) {
  return sendPairCommand(mode);
}
/** @brief Save PQTM configuration to nonvolatile memory.
 * @return True if accepted. Writes NVM; call deliberately, never every loop.
 * Survey/receiver role changes require this followed by a physical restart. */
bool Adafruit_LC29H::saveParameters() {
  return sendPQTMCommand("PQTMSAVEPAR");
}
/** @brief Restore PQTM plus PAIR050/062 defaults on EA after the next reboot.
 * @return True if accepted. This is not a reset of every PAIR setting. */
bool Adafruit_LC29H::restoreParameters() {
  return sendPQTMCommand("PQTMRESTOREPAR");
}
/** @brief Save PAIR settings from RTC RAM to nonvolatile memory.
 * @return True if accepted. At fix rates above 1 Hz the vendor requires
 * PAIR382,1 then PAIR003 before saving, then PAIR002 to resume. Keep power
 * on/off transitions more than two seconds apart. This method does not perform
 * that sequence implicitly. See protocol section 2.4.46. */
bool Adafruit_LC29H::savePairSettings() {
  return sendPairCommand(513);
}
/** @brief Set position fix interval using PAIR050.
 * @param milliseconds 100..1000 ms. Above 1 Hz only RMC/GGA follow this rate;
 * GSA/GSV remain 1 Hz and other NMEA output is suppressed. The vendor requires
 * saving and restarting after changing this setting (protocol 2.4.9).
 * @return Matching acceptance, not confirmation of the effective output rate.
 */
bool Adafruit_LC29H::setFixInterval(uint16_t milliseconds) {
  if (milliseconds < 100 || milliseconds > 1000)
    return badArgument();
  return pairValue(50, milliseconds);
}
/** @brief Read configured position interval from the receiver.
 * @return 100..1000 milliseconds, or -1 on failure. */
int16_t Adafruit_LC29H::getFixInterval() {
  return readPairValue(51, 100, 1000);
}
/** @brief Enable or disable one standard NMEA message (EA permits 0/1 only).
 * @param message Standard message type supported on EA.
 * @param enabled Output once per applicable fix when true.
 * @return True on acceptance. Fix rates above 1 Hz have additional
 * restrictions. */
bool Adafruit_LC29H::enableNMEA(lc29h_nmea_t message, bool enabled) {
  char parameters[12];
  snprintf(parameters, sizeof(parameters), "%u,%u", message, enabled ? 1 : 0);
  return sendPairCommand(62, parameters);
}
/** @brief Read one standard NMEA message's enable state.
 * @param message Standard message type.
 * @return 0 disabled, 1 enabled, or -1 on failure; verifies the returned type.
 */
int8_t Adafruit_LC29H::isNMEAEnabled(lc29h_nmea_t message) {
  char parameters[4], response[64];
  snprintf(parameters, sizeof(parameters), "%u", message);
  if (!queryPair(63, parameters, response, sizeof(response)))
    return -1;
  nmea_span_t value[2];
  int32_t type, enabled;
  if (!fields(validate(response, strlen(response)).fields, value, 2) ||
      !integer(value[0], 0, 5, type) || type != message ||
      !integer(value[1], 0, 1, enabled)) {
    badReply();
    return -1;
  }
  return enabled;
}
/** @brief Select navigation dynamics.
 * @param mode Supported PAIR080 navigation mode.
 * @return True on acceptance; firmware may reject unsupported settings. */
bool Adafruit_LC29H::setNavigationMode(lc29h_navigation_mode_t mode) {
  return pairValue(80, mode);
}
/** @brief Read navigation dynamics from the receiver.
 * @return Decoded mode, or UNKNOWN on failure/reserved wire values. */
lc29h_navigation_mode_t Adafruit_LC29H::getNavigationMode() {
  int32_t mode = readPairValue(81, 0, 9);
  switch (mode) {
    case 0:
    case 1:
    case 4:
    case 5:
    case 7:
    case 9:
      return (lc29h_navigation_mode_t)mode;
    default:
      if (mode >= 0)
        badReply();
      return LC29H_NAV_UNKNOWN;
  }
}
/** @brief Enable or disable active interference cancellation.
 * @param enabled Desired PAIR074 setting.
 * @return True on acceptance. */
bool Adafruit_LC29H::enableInterferenceCancellation(bool enabled) {
  return pairValue(74, enabled);
}
/** @brief Read interference cancellation state.
 * @return 0 disabled, 1 enabled, -1 on failure. */
int8_t Adafruit_LC29H::isInterferenceCancellationEnabled() {
  return readPairValue(75, 0, 1);
}
/** @brief Set fractional digits for standard NMEA output.
 * @param precision Time/altitude/DOP/speed/course 0..3, position 0..8.
 * @return True on acceptance. Position 8 retains the EA's high precision;
 * reducing digits permanently loses information in the transmitted text. */
bool Adafruit_LC29H::setPrecision(const lc29h_precision_t& precision) {
  if (precision.time > 3 || precision.position > 8 || precision.altitude > 3 ||
      precision.dop > 3 || precision.speed > 3 || precision.course > 3)
    return badArgument();
  char body[64];
  snprintf(body, sizeof(body), "PQTMCFGNMEADP,W,%u,%u,%u,%u,%u,%u",
           precision.time, precision.position, precision.altitude,
           precision.dop, precision.speed, precision.course);
  return sendPQTMCommand(body);
}
/** @brief Read all six NMEA precision settings atomically.
 * @param precision Updated only after every reply field validates.
 * @return True on successful receiver readback; unchanged output on failure. */
bool Adafruit_LC29H::getPrecision(lc29h_precision_t& precision) {
  char response[80];
  nmea_span_t values[7];
  int32_t parsed[6];
  if (!sendPQTMCommand("PQTMCFGNMEADP,R", response, sizeof(response)))
    return false;
  if (!fields(validate(response, strlen(response)).fields, values, 7))
    return badReply();
  for (uint8_t i = 0; i < 6; i++)
    if (!integer(values[i + 1], 0, i == 1 ? 8 : 3, parsed[i]))
      return badReply();
  lc29h_precision_t result = {(uint8_t)parsed[0], (uint8_t)parsed[1],
                              (uint8_t)parsed[2], (uint8_t)parsed[3],
                              (uint8_t)parsed[4], (uint8_t)parsed[5]};
  precision = result;
  return true;
}
/** @brief Configure rover or base role; does not save or reboot automatically.
 * @param mode Desired role. Base enables RTCM MSM4/1005 and disables NMEA;
 * rover restores default NMEA output. Requires saveParameters() then restart.
 * @return True if accepted. Ensure a correct surveyed antenna position first.
 */
bool Adafruit_LC29H::setReceiverMode(lc29h_receiver_mode_t mode) {
  char body[32];
  snprintf(body, sizeof(body), "PQTMCFGRCVRMODE,W,%u", mode);
  return sendPQTMCommand(body);
}
/** @brief Read configured receiver role.
 * @return Rover/base/unknown; inspect commandStatus() to distinguish failure.
 */
lc29h_receiver_mode_t Adafruit_LC29H::getReceiverMode() {
  char response[64];
  nmea_span_t values[2];
  int32_t mode;
  if (!sendPQTMCommand("PQTMCFGRCVRMODE,R", response, sizeof(response)))
    return LC29H_MODE_UNKNOWN;
  if (!fields(validate(response, strlen(response)).fields, values, 2) ||
      !integer(values[1], 0, 2, mode)) {
    badReply();
    return LC29H_MODE_UNKNOWN;
  }
  return (lc29h_receiver_mode_t)mode;
}
/** @brief Format an exact decimal without floating point or scientific
 * notation.
 * @param output Destination, cleared on failure if it has space.
 * @param capacity Bytes including NUL.
 * @param value VALID coefficient and decimal-place count.
 * @return Characters excluding NUL, or zero on failure. All digits are
 * retained, including trailing zeros. Negative zero is not represented by the
 * core. */
size_t Adafruit_LC29H::formatDecimal(char* output, size_t capacity,
                                     const nmea_decimal_t& value) {
  if (!output || !capacity)
    return 0;
  output[0] = 0;
  if (value.status != NMEA_NUMBER_VALID)
    return 0;
  bool negative = value.coefficient < 0;
  uint64_t magnitude = negative ? (uint64_t)(-(value.coefficient + 1)) + 1
                                : (uint64_t)value.coefficient;
  char reversed[20];
  size_t digits = 0;
  do {
    reversed[digits++] = '0' + magnitude % 10;
    magnitude /= 10;
  } while (magnitude);
  size_t whole =
      digits > value.decimalPlaces ? digits - value.decimalPlaces : 1;
  size_t length = whole + value.decimalPlaces + (value.decimalPlaces ? 1 : 0) +
                  (negative ? 1 : 0);
  if (length >= capacity)
    return 0;
  size_t offset = 0;
  if (negative)
    output[offset++] = '-';
  for (size_t i = whole + value.decimalPlaces; i > 0; i--) {
    if (value.decimalPlaces && i == value.decimalPlaces)
      output[offset++] = '.';
    output[offset++] = i <= digits ? reversed[i - 1] : '0';
  }
  output[offset] = 0;
  return offset;
}
/** @brief Configure survey-in or a fixed antenna reference in exact meters.
 * @param config Mode 0..2, duration 0..86400, nonnegative accuracy limit and
 * valid exact decimal ECEF coordinates. Each decimal must fit 23 characters.
 * @return True on acceptance. Requires saveParameters() and a restart to take
 * effect. Fixed-base position errors transfer directly to rover positions. */
bool Adafruit_LC29H::setSurvey(const lc29h_survey_config_t& config) {
  char accuracy[24], x[24], y[24], z[24], body[160];
  if (config.mode > 2 || config.minimumDuration > 86400 ||
      config.accuracy.coefficient < 0 ||
      !formatDecimal(accuracy, sizeof(accuracy), config.accuracy) ||
      !formatDecimal(x, sizeof(x), config.x) ||
      !formatDecimal(y, sizeof(y), config.y) ||
      !formatDecimal(z, sizeof(z), config.z))
    return badArgument();
  snprintf(body, sizeof(body), "PQTMCFGSVIN,W,%u,%lu,%s,%s,%s,%s", config.mode,
           (unsigned long)config.minimumDuration, accuracy, x, y, z);
  return sendPQTMCommand(body);
}
/** @brief Read survey settings, preserving all reported decimal digits.
 * @param config Updated only on complete valid readback; unchanged on failure.
 * @return True on successful receiver readback. */
bool Adafruit_LC29H::getSurvey(lc29h_survey_config_t& config) {
  char response[192];
  nmea_span_t values[7];
  int32_t mode, duration;
  if (!sendPQTMCommand("PQTMCFGSVIN,R", response, sizeof(response)))
    return false;
  if (!fields(validate(response, strlen(response)).fields, values, 7) ||
      !integer(values[1], 0, 2, mode) ||
      !integer(values[2], 0, 86400, duration))
    return badReply();
  lc29h_survey_config_t result = {
      (uint8_t)mode,           (uint32_t)duration,
      parseDecimal(values[3]), parseDecimal(values[4]),
      parseDecimal(values[5]), parseDecimal(values[6])};
  if (result.accuracy.status != NMEA_NUMBER_VALID ||
      result.accuracy.coefficient < 0 || result.x.status != NMEA_NUMBER_VALID ||
      result.y.status != NMEA_NUMBER_VALID ||
      result.z.status != NMEA_NUMBER_VALID)
    return badReply();
  config = result;
  return true;
}
/** @brief Set a PQTM/standard message output divisor.
 * @param message Name without '$', e.g. PQTMSVINSTATUS or GGA.
 * @param rate Zero disables, 1..20 emits every N applicable fixes; EA standard
 * messages should use enableNMEA() because their documented range is 0/1.
 * @param version Required message version for PQTM, or -1 to omit for standard.
 * @return True if accepted by this firmware. Does not guarantee a fix. */
bool Adafruit_LC29H::setMessageRate(const char* message, uint8_t rate,
                                    int8_t version) {
  if (!message || !*message || strlen(message) > 24 || rate > 20 ||
      version < -1)
    return badArgument();
  for (const char* p = message; *p; p++)
    if (!((*p >= 'A' && *p <= 'Z') || (*p >= '0' && *p <= '9')))
      return badArgument();
  char body[64];
  if (version < 0)
    snprintf(body, sizeof(body), "PQTMCFGMSGRATE,W,%s,%u", message, rate);
  else
    snprintf(body, sizeof(body), "PQTMCFGMSGRATE,W,%s,%u,%u", message, rate,
             version);
  return sendPQTMCommand(body);
}
/** @brief Read a message output divisor, checking returned name and version.
 * @param message Name without '$'.
 * @param version PQTM version or -1 for standard NMEA.
 * @return 0..20 divisor, or -1 on failure. A trailing empty version is allowed
 * for standard NMEA as shown in Quectel's protocol example. */
int8_t Adafruit_LC29H::getMessageRate(const char* message, int8_t version) {
  if (!message || !*message || strlen(message) > 24 || version < -1) {
    badArgument();
    return -1;
  }
  for (const char* p = message; *p; p++)
    if (!((*p >= 'A' && *p <= 'Z') || (*p >= '0' && *p <= '9'))) {
      badArgument();
      return -1;
    }
  char body[64], response[96];
  if (version < 0)
    snprintf(body, sizeof(body), "PQTMCFGMSGRATE,R,%s", message);
  else
    snprintf(body, sizeof(body), "PQTMCFGMSGRATE,R,%s,%u", message, version);
  if (!sendPQTMCommand(body, response, sizeof(response)))
    return -1;
  nmea_span_t remaining = validate(response, strlen(response)).fields;
  nextField(remaining);
  nmea_span_t name = nextField(remaining), rate = nextField(remaining),
              ver = nextField(remaining);
  int32_t parsed, parsedVersion;
  if (!matches(name, message) || !integer(rate, 0, 20, parsed) ||
      nextField(remaining).data ||
      (version < 0 ? ver.length != 0
                   : (!integer(ver, 0, 127, parsedVersion) ||
                      parsedVersion != version))) {
    badReply();
    return -1;
  }
  return parsed;
}
/** @brief Select RTCM observation output format.
 * @param mode Disabled/MSM4/MSM7. Independent of 1005 and ephemeris settings.
 * @return True on acceptance; use onRTCM() to receive complete binary packets.
 */
bool Adafruit_LC29H::setRTCMMode(lc29h_rtcm_mode_t mode) {
  return pairValue(432, mode);
}
/** @brief Read RTCM observation output format.
 * @return Disabled/MSM4/MSM7 or UNKNOWN on failure. */
lc29h_rtcm_mode_t Adafruit_LC29H::getRTCMMode() {
  return (lc29h_rtcm_mode_t)readPairValue(433, -1, 1, -2);
}
/** @brief Enable stationary antenna reference point (RTCM1005) output.
 * @param enabled Desired state.
 * @return True on acceptance. Correct reference coordinates are essential. */
bool Adafruit_LC29H::enableReferenceStationOutput(bool enabled) {
  return pairValue(434, enabled);
}
/** @brief Read RTCM1005 output state.
 * @return 0 disabled, 1 enabled, -1 on failure. */
int8_t Adafruit_LC29H::isReferenceStationOutputEnabled() {
  return readPairValue(435, 0, 1);
}
/** @brief Enable satellite ephemeris RTCM output.
 * @param enabled Desired state.
 * @return True on acceptance. */
bool Adafruit_LC29H::enableEphemerisOutput(bool enabled) {
  return pairValue(436, enabled);
}
/** @brief Read ephemeris output state.
 * @return 0 disabled, 1 enabled, -1 on failure. */
int8_t Adafruit_LC29H::isEphemerisOutputEnabled() {
  return readPairValue(437, 0, 1);
}
/** @brief Configure UART1 baud rate for the next receiver reboot.
 * @param baudrate 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800,
 * 921600 or 3000000. Below 115200 the vendor warns of lost output messages.
 * @return True on acceptance. Does not change the host UART, save or restart.
 * Prepare to reconfigure the host UART after the receiver restart. */
bool Adafruit_LC29H::setBaudrate(uint32_t baudrate) {
  switch (baudrate) {
    case 4800:
    case 9600:
    case 19200:
    case 38400:
    case 57600:
    case 115200:
    case 230400:
    case 460800:
    case 921600:
    case 3000000:
      break;
    default:
      return badArgument();
  }
  char parameters[24];
  snprintf(parameters, sizeof(parameters), "0,0,%lu", (unsigned long)baudrate);
  return sendPairCommand(864, parameters);
}
/** @brief Read UART1 configured baud rate.
 * @return Baud rate in bits/s, or -1 on failed/malformed readback. */
int32_t Adafruit_LC29H::getBaudrate() {
  int32_t baudrate = readPairValue(865, 4800, 3000000, -1, "0,0");
  switch (baudrate) {
    case 4800:
    case 9600:
    case 19200:
    case 38400:
    case 57600:
    case 115200:
    case 230400:
    case 460800:
    case 921600:
    case 3000000:
      return baudrate;
    default:
      if (baudrate >= 0)
        badReply();
      return -1;
  }
}
