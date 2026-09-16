#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <vector>

#include "Adafruit_LC29H.h"
static nmea_sentence_t sentence(const char* body) {
  static char text[400];
  size_t size =
      Adafruit_NMEA::buildCommand(text, sizeof(text), body, strlen(body));
  assert(size);
  return Adafruit_NMEA::validate(text, size);
}
int main() {
  char output[300];
  for (const char* text :
       {"0", "1", "-1", "0.000000001", "-0.000000001", "1234567890123456789",
        "-9223372036854775808", "9223372036854775807", "-2472446.4619",
        "0.00000000000000000000000001", "1.2300"}) {
    nmea_decimal_t decimal = Adafruit_NMEA::parseDecimal({text, strlen(text)});
    assert(decimal.status == NMEA_NUMBER_VALID);
    assert(Adafruit_LC29H::formatDecimal(output, sizeof(output), decimal) ==
           strlen(text));
    assert(!strcmp(output, text));
    memset(output, 'x', sizeof(output));
    assert(!Adafruit_LC29H::formatDecimal(output, strlen(text), decimal) &&
           output[0] == 0);
  }
  nmea_decimal_t tiny = {NMEA_NUMBER_VALID, 1, 255};
  assert(Adafruit_LC29H::formatDecimal(output, sizeof(output), tiny) == 257);
  assert(output[0] == '0' && output[1] == '.' && output[256] == '1');
  tiny.status = NMEA_NUMBER_EMPTY;
  assert(!Adafruit_LC29H::formatDecimal(output, sizeof(output), tiny) &&
         !output[0]);
  assert(!Adafruit_LC29H::formatDecimal(NULL, 10, tiny));
  puts(
      "PASS: exact decimal formatting, int64 endpoints, tiny fractions, "
      "trailing zeros, bounded output");
  const char* body =
      "GNGGA,123456.123,4807.12345678,N,01131.98765432,E,4,19,0.7,-12.345,M,4."
      "567,M,0.5,42";
  lc29h_fix_t fix = Adafruit_LC29H::parseFix(sentence(body));
  assert(fix.position.validation.status == GNSS_SENTENCE_VALID &&
         fix.position.fixQuality == 4);
  assert(fix.satellites == 19 && fix.hdop.coefficient == 7 &&
         fix.hdop.decimalPlaces == 1);
  assert(fix.altitude.coefficient == -12345 && fix.altitude.decimalPlaces == 3);
  assert(fix.separation.coefficient == 4567 &&
         fix.correctionAge.coefficient == 5);
  for (const char* invalid : {"GNGGA,123456.123,4807.12345678,N,01131.98765432,"
                              "E,4,19,0.7,-12.345,F,4.567,M,0.5,42",
                              "GNGGA,123456.123,4807.12345678,N,01131.98765432,"
                              "E,4,19,0.7,-12.345,M,4.567,M,0.5,99999",
                              "GNGGA,123456.123,4807.12345678,N,01131.98765432,"
                              "E,4,19,0.7,-12.345,M,4.567,M,0.5",
                              "GNGGA,123456.123,4807.12345678,N,01131.98765432,"
                              "E,4,19,0.7,-12.345,M,4.567,M,-0.5,42",
                              "GNGGA,123456.123,4807.12345678,N,01131.98765432,"
                              "E,4,19,0.7,-12.345,M,4.567,M,0.5,42,extra"}) {
    fix = Adafruit_LC29H::parseFix(sentence(invalid));
    assert(fix.position.validation.status != GNSS_SENTENCE_VALID);
    assert(fix.position.latitude.fractionalMinutes == 0 &&
           fix.altitude.coefficient == 0);
  }
  fix = Adafruit_LC29H::parseFix(
      sentence("GNGGA,123456.000,,,,,0,00,99.99,,,,,,"));
  assert(fix.position.validation.status == GNSS_SENTENCE_VALID &&
         !fix.position.fix);
  assert(fix.altitude.status == NMEA_NUMBER_EMPTY &&
         fix.correctionAge.status == NMEA_NUMBER_EMPTY);
  puts(
      "PASS: precise RTK GGA, altitude, geoid, correction age, no-fix empties, "
      "atomic invalidation");
  lc29h_survey_status_t survey = Adafruit_LC29H::parseSurveyStatus(
      sentence("PQTMSVINSTATUS,1,2241,1,,01,538,43200,-2472436.0802,4828383."
               "0026,3343698.4839,9.5"));
  assert(survey.status == LC29H_REPLY_VALID && survey.validity == 1 &&
         survey.observations == 538);
  assert(survey.x.coefficient == -24724360802LL && survey.x.decimalPlaces == 4);
  assert(survey.y.coefficient == 48283830026LL &&
         survey.z.coefficient == 33436984839LL);
  assert(survey.accuracy.coefficient == 95 &&
         survey.accuracy.decimalPlaces == 1);
  for (const char* invalid : {"PQTMSVINSTATUS,2,2241,1,,01,538,43200,-2472436."
                              "0802,4828383.0026,3343698.4839,9.5",
                              "PQTMSVINSTATUS,1,604800000,1,,01,538,43200,-"
                              "2472436.0802,4828383.0026,3343698.4839,9.5",
                              "PQTMSVINSTATUS,1,2241,1,,01,538,43200,-2472436."
                              "0802,4828383.0026,3343698.4839,-9.5",
                              "PQTMSVINSTATUS,1,2241,1,,01,538,43200,-2472436."
                              "0802,4828383.0026,bad,9.5",
                              "PQTMSVINSTATUS,1,2241,1,,01,538,43200,-2472436."
                              "0802,4828383.0026,3343698.4839,9.5,extra"}) {
    survey = Adafruit_LC29H::parseSurveyStatus(sentence(invalid));
    assert(survey.status == LC29H_REPLY_INVALID_FIELDS &&
           survey.observations == 0);
    assert(survey.x.status == NMEA_NUMBER_MISSING && survey.x.coefficient == 0);
  }
  puts("PASS: exact survey ECEF components and complete-payload validation");

  lc29h_motion_t motion = Adafruit_LC29H::parseMotion(
      sentence("GNRMC,123456.000,A,4807.12345678,N,01131.98765432,E,12.3456,"
               "359.99999,160926,,,A"));
  assert(motion.position.validation.status == GNSS_SENTENCE_VALID);
  assert(motion.speedKnots.coefficient == 123456 &&
         motion.speedKnots.decimalPlaces == 4);
  assert(motion.course.coefficient == 35999999 &&
         motion.course.decimalPlaces == 5);
  motion = Adafruit_LC29H::parseMotion(
      sentence("GNRMC,123456.000,A,4807.12345678,N,01131.98765432,E,12.3456,"
               "360.00001,160926,,,A"));
  assert(motion.position.validation.status != GNSS_SENTENCE_VALID &&
         motion.speedKnots.coefficient == 0);
  lc29h_pvt_t pvt = Adafruit_LC29H::parsePVT(sentence(
      "PQTMPVT,1,31075000,20221225,083737.000,,3,09,18,31.127382910123,117."
      "263729100987,34.212,5.267,3.212,2.928,0.238,4.346,34.12,2.16,4.38"));
  assert(pvt.status == LC29H_REPLY_VALID && pvt.fixMode == 3 &&
         pvt.satellites == 9);
  assert(pvt.latitude.coefficient == 31127382910123LL &&
         pvt.latitude.decimalPlaces == 12);
  assert(pvt.longitude.coefficient == 117263729100987LL &&
         pvt.longitude.decimalPlaces == 12);
  assert(pvt.time.hour == 8 && pvt.time.minute == 37 && pvt.time.second == 37);
  assert(pvt.speed.coefficient == 4346 && pvt.speed.decimalPlaces == 3);
  pvt = Adafruit_LC29H::parsePVT(sentence(
      "PQTMPVT,1,1000,20221225,163355.000,,0,00,,,,,,,,,,,99.99,99.99"));
  assert(pvt.status == LC29H_REPLY_VALID && pvt.fixMode == 0);
  assert(pvt.latitude.status == NMEA_NUMBER_EMPTY &&
         pvt.leapSeconds.status == NMEA_NUMBER_EMPTY);
  for (const char* bad :
       {"PQTMPVT,1,31075000,20230229,083737.000,,3,09,18,31.1,117.2,34.2,5.2,3."
        "2,2.9,0.2,4.3,34.1,2.1,4.3",
        "PQTMPVT,1,31075000,20221225,083737.000,,3,09,18,90.00000000001,117.2,"
        "34.2,5.2,3.2,2.9,0.2,4.3,34.1,2.1,4.3",
        "PQTMPVT,1,31075000,20221225,083737.000,,3,09,18,31.1,-180.00000000001,"
        "34.2,5.2,3.2,2.9,0.2,4.3,34.1,2.1,4.3",
        "PQTMPVT,1,31075000,20221225,083737.000,,3,09,18,31.1,117.2,34.2,5.2,3."
        "2,2.9,0.2,4.3,360.00000000001,2.1,4.3",
        "PQTMPVT,1,31075000,20221225,083737.000,,3,09,18,31.1,117.2,34.2,5.2,3."
        "2,2.9,0.2,-4.3,34.1,2.1,4.3",
        "PQTMPVT,1,31075000,20221225,083737.000,,3,09,18,,117.2,34.2,5.2,3.2,2."
        "9,0.2,4.3,34.1,2.1,4.3"}) {
    pvt = Adafruit_LC29H::parsePVT(sentence(bad));
    assert(pvt.status == LC29H_REPLY_INVALID_FIELDS &&
           pvt.latitude.coefficient == 0);
    assert(pvt.latitude.status == NMEA_NUMBER_MISSING && pvt.satellites == 0);
  }
  puts(
      "PASS: exact RMC motion and PQTMPVT, no-fix fields, calendars, sub-E7 "
      "range boundaries");
}
