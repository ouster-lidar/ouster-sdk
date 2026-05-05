#include <nmea/message/vtg.hpp>

#include "parse.hpp"

using namespace nmea;

// CONSTRUCTORS
vtg::vtg(const nmea::sentence& sentence)
{
    // Get talker.
    vtg::talker = sentence.talker();

    // Parse track angle true.
    nmea::parse_float(vtg::track_angle_true, sentence, 0);

    // Parse track angle magnetic.
    nmea::parse_float(vtg::track_angle_magnetic, sentence, 2);

    // Parse speed in knots.
    nmea::parse_float(vtg::speed_knots, sentence, 4);

    // Parse speed in kph.
    nmea::parse_float(vtg::speed_kph, sentence, 6);

    // Parse mode.
    nmea::parse_mode(vtg::mode, sentence, 8);
}