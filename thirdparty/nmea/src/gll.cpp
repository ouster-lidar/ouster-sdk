#include <nmea/message/gll.hpp>

#include "parse.hpp"

#include <sstream>
#include <iomanip>

using namespace nmea;

// CONSTRUCTORS
gll::gll(const nmea::sentence& sentence)
{
    // Get talker.
    gll::talker = sentence.talker();

    // Parse latitude.
    nmea::parse_latitude(gll::latitude, sentence, 0);

    // Parse longitude.
    nmea::parse_longitude(gll::longitude, sentence, 2);

    // Parse UTC time of day.
    nmea::parse_utc(gll::utc, sentence, 4);

    // Parse status.
    nmea::parse_status(gll::status, sentence, 5);

    // Parse mode.
    nmea::parse_mode(gll::mode, sentence, 6);
}