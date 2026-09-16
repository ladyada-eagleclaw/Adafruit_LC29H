/**************************************************************************/
/*!
  @file Adafruit_LC29H.h
  @brief LC29H framing, exact positions, and command reply decoding.

  Written for Adafruit Industries. MIT license; see license.txt.
*/
/**************************************************************************/

#ifndef ADAFRUIT_LC29H_H
#define ADAFRUIT_LC29H_H

// Shared parser dependency: Adafruit GPS 1.9.0 or later.
#include <Adafruit_GNSS.h>

/// Decoding outcome, independent of whether a command was accepted.
typedef enum : uint8_t {
  LC29H_REPLY_INVALID_FRAME,  ///< Incomplete or invalid NMEA frame.
  LC29H_REPLY_UNSUPPORTED,    ///< Valid frame with a different address/marker.
  LC29H_REPLY_INVALID_FIELDS, ///< Recognized address with malformed payload.
  LC29H_REPLY_VALID,          ///< Supported response payload decoded.
  LC29H_REPLY_RECEIVER_ERROR  ///< Well-formed PQTM error response decoded.
} lc29h_reply_status_t;

/// PAIR001 result codes; UNKNOWN is a library sentinel, not a wire value.
typedef enum : uint8_t {
  LC29H_PAIR_ACCEPTED = 0,    ///< Positive acknowledgment; not a fix/readback.
  LC29H_PAIR_PROCESSING = 1,  ///< Wait for a subsequent result.
  LC29H_PAIR_FAILED = 2,      ///< Command failed.
  LC29H_PAIR_UNSUPPORTED = 3, ///< Command ID is unsupported.
  LC29H_PAIR_PARAMETER_ERROR = 4, ///< Receiver rejected command parameters.
  LC29H_PAIR_BUSY = 5,            ///< Receiver is busy.
  LC29H_PAIR_UNKNOWN = 255        ///< No recognized result was decoded.
} lc29h_pair_result_t;

/** Owned acknowledgment; no pointers into the receive buffer.
 *  Check status before using commandID or result. VALID includes rejection
 *  and PROCESSING responses; it does not mean the requested operation finished.
 *  Any decoding failure leaves commandID zero and result UNKNOWN. */
typedef struct {
  lc29h_reply_status_t status; ///< Frame/address/field decoding outcome.
  uint16_t commandID;          ///< Acknowledged PAIR ID, 0 through 999.
  lc29h_pair_result_t result;  ///< Decoded receiver result.
} lc29h_pair_ack_t;

/** Borrowed firmware text, without fixed string limits or numeric conversion.
 *  Success populates three nonempty spans and leaves errorCode zero. A decoded
 *  receiver error populates only errorCode. Other failures clear every value.
 *  Text is not NUL-terminated; copy it before its source sentence expires. */
typedef struct {
  lc29h_reply_status_t status; ///< VALID, RECEIVER_ERROR, or decoding failure.
  nmea_span_t version;   ///< Firmware identity text; not a capability map.
  nmea_span_t buildDate; ///< Opaque firmware build date text.
  nmea_span_t buildTime; ///< Opaque firmware build time text.
  uint8_t errorCode;     ///< PQTM error, 1 through 255; zero otherwise.
} lc29h_version_t;

class Stream;

/// Result of a blocking command transaction (distinct from a PAIR result).
typedef enum : uint8_t {
  LC29H_COMMAND_OK,               ///< Expected, validated reply received.
  LC29H_COMMAND_NO_PORT,          ///< begin() has not attached a port.
  LC29H_COMMAND_INVALID_ARGUMENT, ///< Invalid or oversized command/storage.
  LC29H_COMMAND_TIMEOUT, ///< Deadline expired, including PROCESSING replies.
  LC29H_COMMAND_WRITE_FAILED, ///< Transport accepted only part of the command.
  LC29H_COMMAND_REJECTED,     ///< Receiver returned a negative result.
  LC29H_COMMAND_BAD_REPLY,    ///< Matching reply has malformed fields.
} lc29h_command_status_t;

/// EA standard messages supported by PAIR062/063.
typedef enum : uint8_t {
  LC29H_NMEA_GGA = 0, ///< Position, altitude and fix quality.
  LC29H_NMEA_GLL = 1, ///< Geographic position.
  LC29H_NMEA_GSA = 2, ///< Active satellites and DOP.
  LC29H_NMEA_GSV = 3, ///< Satellites in view.
  LC29H_NMEA_RMC = 4, ///< Position, speed and course.
  LC29H_NMEA_VTG = 5  ///< Course and speed.
} lc29h_nmea_t;

/// Navigation dynamics for PAIR080.
typedef enum : uint8_t {
  LC29H_NAV_NORMAL = 0,     ///< General navigation.
  LC29H_NAV_FITNESS = 1,    ///< Walking/running.
  LC29H_NAV_STATIONARY = 4, ///< Stationary antenna.
  LC29H_NAV_DRONE = 5,      ///< Drone dynamics.
  LC29H_NAV_SWIMMING = 7,   ///< Swimming.
  LC29H_NAV_BIKE = 9,       ///< Bicycle dynamics.
  LC29H_NAV_UNKNOWN = 255   ///< Query failed.
} lc29h_navigation_mode_t;

/// Receiver operating role. Changing it requires save and restart.
typedef enum : uint8_t {
  LC29H_MODE_UNKNOWN = 0, ///< Unknown or query failure.
  LC29H_MODE_ROVER = 1,   ///< NMEA rover; accepts RTCM corrections.
  LC29H_MODE_BASE =
      2 ///< Base; enables RTCM output and disables NMEA by default.
} lc29h_receiver_mode_t;

/// RTCM observation output format; does not control 1005/ephemeris output.
typedef enum : int8_t {
  LC29H_RTCM_UNKNOWN = -2,  ///< Query failed.
  LC29H_RTCM_DISABLED = -1, ///< No observation output.
  LC29H_RTCM_MSM4 = 0,      ///< MSM4 observations.
  LC29H_RTCM_MSM7 = 1       ///< MSM7 observations.
} lc29h_rtcm_mode_t;

/// Restart choices, separate from parser reset().
typedef enum : uint8_t {
  LC29H_START_HOT = 4,      ///< Retain navigation data.
  LC29H_START_WARM = 5,     ///< Discard ephemeris.
  LC29H_START_COLD = 6,     ///< Discard time, position and navigation data.
  LC29H_START_FULL_COLD = 7 ///< Also restore configuration defaults.
} lc29h_start_t;

/// Number of fractional digits in NMEA output (not floating-point precision).
typedef struct {
  uint8_t time;     ///< UTC seconds, 0..3.
  uint8_t position; ///< Latitude/longitude minutes, 0..8.
  uint8_t altitude; ///< Altitude/separation meters, 0..3.
  uint8_t dop;      ///< Dilution of precision, 0..3.
  uint8_t speed;    ///< Speed, 0..3.
  uint8_t course;   ///< Course, 0..3.
} lc29h_precision_t;

/// Survey configuration. Decimal members are exact SI meters, never float.
typedef struct {
  uint8_t mode;             ///< 0 disabled, 1 survey-in, 2 fixed ECEF.
  uint32_t minimumDuration; ///< Minimum observations, 0..86400 (1 Hz: seconds).
  nmea_decimal_t accuracy;  ///< Nonnegative 3D limit in meters; zero: no limit.
  nmea_decimal_t x;         ///< WGS84 ECEF X in meters.
  nmea_decimal_t y;         ///< WGS84 ECEF Y in meters.
  nmea_decimal_t z;         ///< WGS84 ECEF Z in meters.
} lc29h_survey_config_t;

/// One owned survey status. Check status before using any payload member.
typedef struct {
  lc29h_reply_status_t status; ///< VALID only after all fields validate.
  uint32_t timeOfWeek;         ///< GPS milliseconds of week.
  uint8_t validity;            ///< 0 invalid, 1 in progress, 2 valid.
  uint32_t observations;       ///< Accepted position observations.
  uint32_t configuredDuration; ///< Configured minimum observations.
  nmea_decimal_t x;            ///< Mean ECEF X, meters.
  nmea_decimal_t y;            ///< Mean ECEF Y, meters.
  nmea_decimal_t z;            ///< Mean ECEF Z, meters.
  nmea_decimal_t accuracy;     ///< Mean position accuracy, meters.
} lc29h_survey_status_t;

/// Owned GGA details, preserving all transmitted decimal digits.
typedef struct {
  gnss_position_t position; ///< Shared exact position, time and RTK quality.
  uint16_t
      satellites; ///< Satellites used, zero if blank; see satellitesStatus.
  nmea_number_status_t satellitesStatus; ///< Satellite count availability.
  nmea_decimal_t hdop;          ///< Horizontal dilution; EMPTY if blank.
  nmea_decimal_t altitude;      ///< Altitude above mean sea level in meters.
  nmea_decimal_t separation;    ///< Geoid separation in meters.
  nmea_decimal_t correctionAge; ///< Age of differential corrections, seconds.
} lc29h_fix_t;

/// RMC speed and course together with its coherent position/time/date.
typedef struct {
  gnss_position_t position;  ///< Shared exact position and validity.
  nmea_decimal_t speedKnots; ///< Ground speed, knots as transmitted by RMC.
  nmea_decimal_t course;     ///< True course in degrees, 0..360.
} lc29h_motion_t;

/// One PQTMPVT record with owned exact decimal values; no float conversion.
typedef struct {
  lc29h_reply_status_t
      status;          ///< Check before any field; failures clear payload.
  uint32_t timeOfWeek; ///< GPS time of week, milliseconds.
  uint32_t date;       ///< UTC YYYYMMDD, validated calendar date.
  gnss_time_t time;    ///< UTC clock.
  uint8_t fixMode;     ///< 0 no fix, 2 two dimensional, 3 three dimensional.
  uint16_t satellites; ///< Satellites used.
  nmea_decimal_t leapSeconds; ///< UTC leap seconds; EMPTY when unknown.
  nmea_decimal_t latitude;  ///< Signed decimal degrees; EMPTY when unavailable.
  nmea_decimal_t longitude; ///< Signed decimal degrees; EMPTY when unavailable.
  nmea_decimal_t altitude;  ///< Altitude above mean sea level, meters.
  nmea_decimal_t separation;    ///< Geoid separation, meters.
  nmea_decimal_t northVelocity; ///< Northward velocity, meters/second.
  nmea_decimal_t eastVelocity;  ///< Eastward velocity, meters/second.
  nmea_decimal_t downVelocity;  ///< Downward velocity, meters/second.
  nmea_decimal_t speed;         ///< Ground speed, meters/second.
  nmea_decimal_t heading;       ///< True heading, degrees.
  nmea_decimal_t hdop; ///< Horizontal DOP; vendor 99.99 sentinel is retained.
  nmea_decimal_t pdop; ///< Position DOP; vendor 99.99 sentinel is retained.
} lc29h_pvt_t;

/// Called for every complete NMEA line, including during command waits.
/// Views expire when callback returns. Do not reenter the receiver or port.
typedef void (*lc29h_sentence_callback_t)(const nmea_sentence_t& sentence,
                                          void* context);
/// Called only for complete CRC-valid RTCM3 packets that fit supplied storage.
/// Packet storage is borrowed and expires when callback returns. No reentry.
typedef void (*lc29h_rtcm_callback_t)(const uint8_t* packet, size_t length,
                                      void* context);

/** LC29H-specific decoder above the shared precision-safe GNSS core.
 *  Inherits feed(), lastSentence(), timestamps, reset(), and lastPosition().
 *  Caller-owned receive buffers, one optional Stream reader, no heap
 * allocation. Single-threaded: do not read the attached Stream elsewhere or
 * reenter callbacks. reset() clears parser state only; it does not restart the
 * physical module.
 */
class Adafruit_LC29H : public Adafruit_GNSS {
 public:
  // Borrow two non-overlapping buffers, each capacity bytes including NUL.
  // Buffers must outlive this object. Invalid storage disables feed(), as in
  // Adafruit_NMEA. Copying remains disabled by the base class.
  Adafruit_LC29H(volatile char* firstBuffer, volatile char* secondBuffer,
                 size_t capacity);

  bool begin(Stream& port, uint32_t timeout = 1500);
  void end();
  size_t poll(size_t maximumBytes = 256);
  void reset();
  nmea_frame_status_t feed(uint8_t byte, uint32_t receivedAtMs);
  void onSentence(lc29h_sentence_callback_t callback, void* context = NULL);
  void onRTCM(uint8_t* buffer, size_t capacity, lc29h_rtcm_callback_t callback,
              void* context = NULL);
  size_t writeCorrections(const uint8_t* data, size_t length);
  lc29h_command_status_t commandStatus() const;
  lc29h_pair_result_t commandPairResult() const;
  uint8_t commandError() const;
  void setCommandTimeout(uint32_t timeout);
  bool getVersion(char* response, size_t capacity);
  bool sendPairCommand(uint16_t command, const char* parameters = NULL);
  bool queryPair(uint16_t command, const char* parameters, char* response,
                 size_t capacity);
  bool sendPQTMCommand(const char* body, char* response = NULL,
                       size_t capacity = 0);
  bool restart(lc29h_start_t mode = LC29H_START_HOT);
  bool saveParameters();
  bool restoreParameters();
  bool savePairSettings();
  bool setFixInterval(uint16_t milliseconds);
  int16_t getFixInterval();
  bool enableNMEA(lc29h_nmea_t message, bool enabled);
  int8_t isNMEAEnabled(lc29h_nmea_t message);
  bool setNavigationMode(lc29h_navigation_mode_t mode);
  lc29h_navigation_mode_t getNavigationMode();
  bool enableInterferenceCancellation(bool enabled);
  int8_t isInterferenceCancellationEnabled();
  bool setPrecision(const lc29h_precision_t& precision);
  bool getPrecision(lc29h_precision_t& precision);
  bool setReceiverMode(lc29h_receiver_mode_t mode);
  lc29h_receiver_mode_t getReceiverMode();
  bool setSurvey(const lc29h_survey_config_t& config);
  bool getSurvey(lc29h_survey_config_t& config);
  bool setMessageRate(const char* message, uint8_t rate, int8_t version = -1);
  int8_t getMessageRate(const char* message, int8_t version = -1);
  bool setRTCMMode(lc29h_rtcm_mode_t mode);
  lc29h_rtcm_mode_t getRTCMMode();
  bool enableReferenceStationOutput(bool enabled);
  int8_t isReferenceStationOutputEnabled();
  bool enableEphemerisOutput(bool enabled);
  int8_t isEphemerisOutputEnabled();
  bool setBaudrate(uint32_t baudrate);
  int32_t getBaudrate();
  static lc29h_motion_t parseMotion(const nmea_sentence_t& sentence);
  static lc29h_pvt_t parsePVT(const nmea_sentence_t& sentence);
  static lc29h_fix_t parseFix(const nmea_sentence_t& sentence);
  static lc29h_survey_status_t parseSurveyStatus(
      const nmea_sentence_t& sentence);
  static size_t formatDecimal(char* output, size_t capacity,
                              const nmea_decimal_t& value);

  // Decode only the latest complete line, without caching an earlier reply.
  // Before any complete line: INVALID_FRAME. After navigation or another reply
  // type: UNSUPPORTED. Invalid completed lines replace earlier replies too.
  lc29h_pair_ack_t lastPairAck() const;
  lc29h_version_t lastVersion() const;

  // Accept a view produced by validate()/lastSentence(), kept unchanged while
  // decoding. As in parsePosition(), trust the frame status; do not rechecksum.
  // Require '$' and the exact address PAIR001, not a prefix. Require exactly
  // two nonempty unsigned integer fields: ID 0..999 and result 0..5. Leading
  // zeros are allowed; signs, decimals, unknown results, and extra fields are
  // INVALID_FIELDS. Bad fields never yield a partial acknowledgment.
  static lc29h_pair_ack_t parsePairAck(const nmea_sentence_t& sentence);

  // Require '$' and the exact address PQTMVERNO. Accept exactly three nonempty
  // text fields for success, or exactly ERROR,<code> with unsigned code 1..255.
  // Reserve ERROR as the first field; malformed error payloads cannot become
  // version strings. Preserve unknown nonzero error codes for diagnostics.
  // Date/time remain text, without calendar or timezone interpretation.
  // The zero-field query/echo is INVALID_FIELDS, never a successful response.
  // Spans borrow sentence storage. For lastVersion(), they expire on the next
  // complete line, reset(), or destruction. Synchronize concurrent feed/read.
  static lc29h_version_t parseVersion(const nmea_sentence_t& sentence);

 private:
  Stream* _port;     ///< Borrowed initialized UART or other duplex Stream.
  uint32_t _timeout; ///< Command deadline in milliseconds.
  bool _busy;        ///< Prevent callback reentry into commands/writes.
  bool _dispatching; ///< Callback recursion guard.
  lc29h_command_status_t _commandStatus; ///< Last transaction result.
  lc29h_pair_result_t _pairResult;       ///< Last matching acknowledgment code.
  uint8_t _error; ///< Last PQTM error code, zero otherwise.
  lc29h_sentence_callback_t _sentenceCallback; ///< Optional NMEA consumer.
  void* _sentenceContext;              ///< Application's NMEA callback context.
  uint8_t* _rtcmBuffer;                ///< Optional borrowed RTCM storage.
  size_t _rtcmCapacity;                ///< Bytes available in RTCM storage.
  lc29h_rtcm_callback_t _rtcmCallback; ///< Optional RTCM consumer.
  void* _rtcmContext;                  ///< Application's RTCM callback context.
  uint16_t _rtcmLength;                ///< RTCM bytes collected so far.
  uint16_t _rtcmTotal; ///< Expected packet size including header and CRC.
  uint32_t _rtcmCRC;   ///< Running CRC24Q including received trailer.
  uint32_t _byteTime;  ///< Last binary byte time for gap recovery.
  uint8_t _rtcmHigh;   ///< High two payload-length bits.
  bool
      _rtcmCanDeliver; ///< Buffer has received this packet from its first byte.
  bool _nmeaActive;    ///< A text frame is in progress.
  bool transact(const char* body, int16_t pairID, const char* address,
                char* response, size_t capacity, bool version = false);
  bool pairValue(uint16_t command, int32_t value);
  int32_t readPairValue(uint16_t command, int32_t minimum, int32_t maximum,
                        int32_t failure = -1, const char* parameters = NULL);
  bool badArgument();
  bool badReply();
  static bool decimalRange(nmea_decimal_t value, int32_t minimum,
                           int32_t maximum);
  static bool fields(nmea_span_t remaining, nmea_span_t* values, size_t count);
  static bool integer(nmea_span_t field, int32_t minimum, int32_t maximum,
                      int32_t& result);
  static uint32_t crc24(uint32_t crc, uint8_t byte);
  static bool matches(nmea_span_t field, const char* text);
  static int32_t parseUnsigned(nmea_span_t field, uint16_t maximum);
  static lc29h_reply_status_t replyStatus(const nmea_sentence_t& sentence,
                                          const char* address);
};

#endif // ADAFRUIT_LC29H_H
