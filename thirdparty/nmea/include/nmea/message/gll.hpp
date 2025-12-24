/// \file nmea/message/gll.hpp
/// \brief Defines the nmea::gll struct.
#ifndef NMEA___GLL_H
#define NMEA___GLL_H

#include <nmea/sentence.hpp>
#include <nmea/field.hpp>
#include <nmea/object/status.hpp>
#include <nmea/object/mode.hpp>

namespace nmea {

/// \brief An NMEA GLL sentence.
struct gll
{
    // CONSTRUCTORS
    /// \brief Creates a new GLL instance from an NMEA sentence.
    /// \param sentence The NMEA sentence to parse the GLL sentence from.
    gll(const nmea::sentence& sentence);

    // FIELDS
    /// \brief The name of the sentence talker.
    std::string talker;
    /// \brief The latitude measurement in degrees (N = +, S = -).
    nmea::field<double> latitude;
    /// \brief The longitude measurement in degrees (E = +, W = -).
    nmea::field<double> longitude;
    /// \brief The UTC time of day, in seconds.
    nmea::field<double> utc;
    /// \brief The status of the fix data.
    nmea::field<nmea::status> status;
    /// \brief The mode of the receiver.
    nmea::field<nmea::mode> mode;
};

}

#endif