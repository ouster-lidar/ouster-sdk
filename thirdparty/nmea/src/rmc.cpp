#include <nmea/message/rmc.hpp>

#include "parse.hpp"

using namespace nmea;

// CONSTRUCTORS
rmc::rmc(const nmea::sentence& sentence)
{
    // Get talker.
    rmc::talker = sentence.talker();

    // Parse UTC time of day.
    nmea::parse_utc(rmc::utc, sentence, 0);

    // Parse status.
    nmea::parse_status(rmc::status, sentence, 1);

    // Parse latitude.
    nmea::parse_latitude(rmc::latitude, sentence, 2);

    // Parse longitude.
    nmea::parse_longitude(rmc::longitude, sentence, 4);

    // Parse speed.
    nmea::parse_float(rmc::speed, sentence, 6);

    // Parse track angle.
    nmea::parse_float(rmc::track_angle, sentence, 7);

    // Parse date.
    nmea::parse_date(rmc::date, sentence, 8);

    // Parse magnetic variation.
    // Get field strings.
    std::string variation_string = sentence.get_field(9);
    std::string direction_string = sentence.get_field(10);
    // Verify that both field strings exist.
    if(!variation_string.empty() && !direction_string.empty())
    {
        // Create value.
        float magnetic_variation = std::stof(variation_string);

        // Apply direction.
        if(direction_string == "W")
        {
            magnetic_variation *= -1.0F;
        }

        // Set field.
        rmc::magnetic_variation.set(magnetic_variation);
    }

    // Parse mode.
    nmea::parse_mode(rmc::mode, sentence, 11);
}