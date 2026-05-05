#include <nmea/message/zda.hpp>

#include "parse.hpp"

using namespace nmea;

// CONSTRUCTORS
zda::zda(const nmea::sentence& sentence)
{
    // Get talker.
    zda::talker = sentence.talker();

    // Parse UTC.
    nmea::parse_utc(zda::utc, sentence, 0);

    // Parse day.
    nmea::parse_uint8(zda::day, sentence, 1);

    // Parse month.
    nmea::parse_uint8(zda::month, sentence, 2);

    // Parse year.
    nmea::parse_uint16(zda::year, sentence, 3);

    // Parse GMT offset hours.
    nmea::parse_int8(zda::gmt_offset_hours, sentence, 4);

    // Parse GMT offset minutes.
    nmea::parse_uint8(zda::gmt_offset_minutes, sentence, 5);
}