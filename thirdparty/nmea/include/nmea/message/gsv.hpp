/// \file nmea/message/gsv.hpp
/// \brief Defines the nmea::gsv struct.
#ifndef NMEA___GSV_H
#define NMEA___GSV_H

#include <nmea/sentence.hpp>
#include <nmea/field.hpp>

#include <vector>

namespace nmea {

/// \brief An NMEA GSV sentence.
struct gsv
{
    // OBJECTS
    /// \brief A satellite being tracked by the receiver.
    struct satellite
    {
        /// \brief The PRN number of the satellite.
        nmea::field<uint8_t> prn;
        /// \brief The elevation of the satellite in degrees.
        nmea::field<uint8_t> elevation;
        /// \brief The azimuth of the satellite in degrees from true north.
        nmea::field<uint16_t> azimuth;
        /// \brief The SNR of the satellite in dB.
        nmea::field<uint8_t> snr;
    };

    // CONSTRUCTORS
    /// \brief Creates a new GSV instance from an NMEA sentence.
    /// \param sentence The NMEA sentence to parse the GSV sentence from.
    gsv(const nmea::sentence& sentence);

    // FIELDS
    /// \brief The name of the sentence talker.
    std::string talker;
    /// \brief The total number of messages in the GSV message cycle.
    nmea::field<uint8_t> message_count;
    /// \brief The message number of this GSV message in the total GSV message cycle.
    nmea::field<uint8_t> message_number;
    /// \brief The total number of satellites visible.
    nmea::field<uint8_t> satellite_count;
    /// \brief The collection of satellites documented by this message in the GSV message cycle.
    std::vector<satellite> satellites;
};

}

#endif