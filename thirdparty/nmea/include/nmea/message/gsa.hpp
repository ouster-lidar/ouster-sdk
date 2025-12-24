/// \file nmea/message/gsa.hpp
/// \brief Defines the nmea::gsa struct.
#ifndef NMEA___GSA_H
#define NMEA___GSA_H

#include <nmea/sentence.hpp>
#include <nmea/field.hpp>

#include <vector>

namespace nmea {

/// \brief An NMEA GSA sentence.
struct gsa
{
    // OBJECTS
    /// \brief Enumerates mode status types.
    enum class mode_type
    {
        MANUAL = 0,     ///< Fix type set manually.
        AUTOMATIC = 1   ///< Fix type set automatically.
    };
    /// \brief Enumerates the fix types.
    enum class fix_type
    {
        NONE = 1,       ///< No fix.
        FIX_2D = 2,     ///< Fix is 2D.
        FIX_3D = 3      ///< Fix is 3D.
    };
    /// \brief Enumerates the GNSS system types.
    enum class system_type
    {
        QZSS = 0,
        GPS = 1,
        GLONASS = 2,
        GALILEO = 3,
        BEIDOU = 4
    };

    // CONSTRUCTORS
    /// \brief Creates a new GSA instance from an NMEA sentence.
    /// \param sentence The NMEA sentence to parse the GSA sentence from.
    gsa(const nmea::sentence& sentence);

    // FIELDS
    /// \brief The name of the sentence talker.
    std::string talker;
    /// \brief The mode in which the fix type is set.
    nmea::field<mode_type> mode;
    /// \brief The type of the current fix.
    nmea::field<fix_type> fix;
    /// \brief The collection of satellite IDs being used.
    std::vector<uint8_t> satellites;
    /// \brief The positional dilution of precision (PDOP).
    nmea::field<float> pdop;
    /// \brief The horizontal dilution of precision (HDOP).
    nmea::field<float> hdop;
    /// \brief The vertical dilution of precision (VDOP).
    nmea::field<float> vdop;
    /// \brief The GNSS system type being used.
    nmea::field<system_type> system;
};

}

#endif