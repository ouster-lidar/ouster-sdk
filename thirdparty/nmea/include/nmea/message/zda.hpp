/// \file nmea/message/zda.hpp
/// \brief Defines the nmea::zda struct.
#ifndef NMEA___ZDA_H
#define NMEA___ZDA_H

#include <nmea/sentence.hpp>
#include <nmea/field.hpp>

namespace nmea {

/// \brief An NMEA ZDA sentence.
struct zda
{
    // CONSTRUCTORS
    /// \brief Creates a new ZDA instance from an NMEA sentence.
    /// \param sentence The NMEA sentence to parse the RMC sentence from.
    zda(const nmea::sentence& sentence);

    // FIELDS
    /// \brief The name of the sentence talker.
    std::string talker;
    /// \brief The UTC time of day, in seconds.
    nmea::field<double> utc;
    /// \brief The day component of the GMT date.
    nmea::field<uint8_t> day;
    /// \brief The month component of the GMT date.
    nmea::field<uint8_t> month;
    /// \brief The year component of the GMT date.
    nmea::field<uint16_t> year;
    /// \brief The hour component of the local time zone offset from GMT.
    nmea::field<int8_t> gmt_offset_hours;
    /// \brief The minute component of the local time zone offset from GMT.
    nmea::field<uint8_t> gmt_offset_minutes;
};

}

#endif