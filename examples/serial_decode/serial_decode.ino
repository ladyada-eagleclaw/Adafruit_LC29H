// Paste complete LC29H NMEA sentences into Serial Monitor at 115200 baud.
// Select a newline ending. This example sends no commands to a GPS module.
#include <Adafruit_LC29H.h>

char firstBuffer[160];
char secondBuffer[160];
Adafruit_LC29H gps(firstBuffer, secondBuffer, sizeof(firstBuffer));

void setup() {
  Serial.begin(115200);
  // Wait for Serial Monitor on native USB boards; remove for unattended use.
  while (!Serial) {
    delay(10);
  }
  delay(250);

  Serial.println(F("Adafruit LC29H sentence decoder"));

  Serial.println(F("Paste a complete NMEA sentence with a newline ending."));
}

void loop() {
  if (!Serial.available()) {
    return;
  }
  nmea_frame_status_t status = gps.feed(Serial.read(), millis());
  if (status == NMEA_FRAME_INCOMPLETE) {
    return;
  }
  if (status != NMEA_FRAME_VALID) {
    Serial.println(F("Sentence rejected: check its length, format, and checksum."));
    return;
  }

  lc29h_pair_ack_t ack = gps.lastPairAck();
  if (ack.status == LC29H_REPLY_VALID) {
    Serial.print(F("PAIR command "));
    Serial.print(ack.commandID);
    Serial.print(F(": "));
    switch (ack.result) {
      case LC29H_PAIR_ACCEPTED: Serial.println(F("accepted")); break;
      case LC29H_PAIR_PROCESSING: Serial.println(F("still processing")); break;
      case LC29H_PAIR_FAILED: Serial.println(F("failed")); break;
      case LC29H_PAIR_UNSUPPORTED: Serial.println(F("unsupported command")); break;
      case LC29H_PAIR_PARAMETER_ERROR: Serial.println(F("parameter error")); break;
      case LC29H_PAIR_BUSY: Serial.println(F("busy")); break;
      case LC29H_PAIR_UNKNOWN: Serial.println(F("unknown result")); break;
      default: Serial.println(F("unknown result")); break;
    }
    return;
  }

  lc29h_version_t version = gps.lastVersion();
  if (version.status == LC29H_REPLY_VALID) {
    Serial.print(F("Firmware: "));
    printSpan(version.version);
    Serial.print(F(" built "));
    printSpan(version.buildDate);
    Serial.print(' ');
    printSpan(version.buildTime);
    Serial.println();
    return;
  }
  if (version.status == LC29H_REPLY_RECEIVER_ERROR) {
    Serial.print(F("Firmware query error: "));
    Serial.println(version.errorCode);
    return;
  }

  gnss_position_t position = gps.lastPosition();
  if (position.validation.status == GNSS_SENTENCE_VALID &&
      position.fixStatus == NMEA_NUMBER_VALID && position.fix &&
      position.latitude.status == NMEA_NUMBER_VALID &&
      position.longitude.status == NMEA_NUMBER_VALID) {
    char coordinate[GNSS_COORDINATE_TEXT_SIZE];
    Serial.print(F("Latitude: "));
    gps.formatCoordinate(coordinate, sizeof(coordinate), position.latitude);
    Serial.print(coordinate);
    Serial.print(F("	Longitude: "));
    gps.formatCoordinate(coordinate, sizeof(coordinate), position.longitude);
    Serial.println(coordinate);
    return;
  }
  Serial.println(F("Valid frame; no supported reply or valid position to display."));
}

void printSpan(nmea_span_t text) {
  Serial.write((const uint8_t*)text.data, text.length);
}
