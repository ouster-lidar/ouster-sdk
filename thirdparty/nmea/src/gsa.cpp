#include <nmea/message/gsa.hpp>

#include "parse.hpp"

using namespace nmea;

// CONSTRUCTORS
gsa::gsa(const nmea::sentence& sentence)
{
    // Get talker.
    gsa::talker = sentence.talker();

    // Parse mode.
    std::string mode_string = sentence.get_field(0);
    if(!mode_string.empty())
    {
        if(mode_string == "M")
        {
            gsa::mode.set(gsa::mode_type::MANUAL);
        }
        else if(mode_string == "A")
        {
            gsa::mode.set(gsa::mode_type::AUTOMATIC);
        }
    }

    // Parse fix type.
    nmea::parse_enum(gsa::fix, sentence, 1);

    // Parse satellite IDs.
    gsa::satellites.reserve(12);
    for(uint8_t i = 0; i < 12; ++i)
    {
        // Get current satellite field.
        std::string satellite_string = sentence.get_field(2 + i);
        if(!satellite_string.empty())
        {
            gsa::satellites.push_back(std::stoul(satellite_string));
        }
        else
        {
            break;
        }
    }

    // Parse PDOP.
    nmea::parse_float(gsa::pdop, sentence, 14);

    // Parse HDOP.
    nmea::parse_float(gsa::hdop, sentence, 15);

    // Parse VDOP.
    nmea::parse_float(gsa::vdop, sentence, 16);

    // Parse system type.
    nmea::parse_enum(gsa::system, sentence, 17);
}