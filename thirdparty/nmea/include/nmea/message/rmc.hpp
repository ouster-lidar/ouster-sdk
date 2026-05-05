/// \file nmea/message/rmc.hpp
/// \brief Defines the nmea::rmc struct.
#ifndef NMEA___RMC_H
#define NMEA___RMC_H

#include <nmea/sentence.hpp>
#include <nmea/field.hpp>
#include <nmea/object/status.hpp>
#include <nmea/object/date.hpp>
#include <nmea/object/mode.hpp>

namespace nmea {

/// \brief An NMEA RMC sentence.
struct rmc
{
    // CONSTRUCTORS
    /// \brief Creates a new RMC instance from an NMEA sentence.
    /// \param sentence The NMEA sentence to parse the RMC sentence from.
    rmc(const nmea::sentence& sentence);

    // FIELDS
    /// \brief The name of the sentence talker.
    std::string talker;
    /// \brief The UTC time of day, in seconds.
    nmea::field<double> utc;
    /// \brief The status of the fix data.
    nmea::field<nmea::status> status;
    /// \brief The latitude measurement in degrees (N = +, S = -).
    nmea::field<double> latitude;
    /// \brief The longitude measurement in degrees (E = +, W = -).
    nmea::field<double> longitude;
    /// \brief The speed over ground in knots.
    nmea::field<float> speed;
    /// \brief The track angle in true degrees.
    nmea::field<float> track_angle;
    /// \brief The current date.
    nmea::field<nmea::date> date;
    /// \brief The magnetic variation in true degrees (E = +, W = -);
    nmea::field<float> magnetic_variation;
    /// \brief The mode of the receiver.
    nmea::field<nmea::mode> mode;
};

}

#endif