/// \file nmea/message/vtg.hpp
/// \brief Defines the nmea::vtg struct.
#ifndef NMEA___VTG_H
#define NMEA___VTG_H

#include <nmea/sentence.hpp>
#include <nmea/field.hpp>
#include <nmea/object/mode.hpp>

namespace nmea {

/// \brief An NMEA VTG sentence.
struct vtg
{
    // CONSTRUCTORS
    /// \brief Creates a new VTG instance from an NMEA sentence.
    /// \param sentence The NMEA sentence to parse the RMC sentence from.
    vtg(const nmea::sentence& sentence);

    // FIELDS
    /// \brief The name of the sentence talker.
    std::string talker;
    /// \brief The track angle from true north in degrees.
    nmea::field<float> track_angle_true;
    /// \brief The track angle from magnetic north in degrees.
    nmea::field<float> track_angle_magnetic;
    /// \brief The speed over ground in knots.
    nmea::field<float> speed_knots;
    /// \brief The speed over ground in kilometers/hour.
    nmea::field<float> speed_kph;
    /// \brief The mode of the receiver.
    nmea::field<nmea::mode> mode;
};

}

#endif