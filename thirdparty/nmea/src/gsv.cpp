#include <nmea/message/gsv.hpp>

#include "parse.hpp"

using namespace nmea;

// CONSTRUCTORS
gsv::gsv(const nmea::sentence& sentence)
{
    // Get talker.
    gsv::talker = sentence.talker();

    // Parse message count.
    nmea::parse_uint8(gsv::message_count, sentence, 0);

    // Parse message number.
    nmea::parse_uint8(gsv::message_number, sentence, 1);

    // Parse satellite count.
    nmea::parse_uint8(gsv::satellite_count, sentence, 2);

    // Use n_fields to determine how many satellites are in this message.
    uint8_t n_entries = (sentence.field_count() - 3) / 4;

    // Iterate over satellites in message.
    gsv::satellites.reserve(n_entries);
    for(uint8_t i = 0; i < n_entries; ++i)
    {
        // Create new satellite struct.
        gsv::satellite satellite;

        // Parse PRN.
        nmea::parse_uint8(satellite.prn, sentence, 3 + i*4);

        // Parse elevation.
        nmea::parse_uint8(satellite.elevation, sentence, 4 + i*4);

        // Parse azimuth.
        nmea::parse_uint16(satellite.azimuth, sentence, 5 + i*4);

        // Parse SNR.
        nmea::parse_uint8(satellite.snr, sentence, 6 + i*4);

        // Add satellite to vector.
        gsv::satellites.emplace_back(satellite);
    }
}