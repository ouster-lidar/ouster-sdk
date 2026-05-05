/// \file nmea/message/gga.hpp
/// \brief Defines the nmea::gga struct.
#ifndef NMEA___GGA_H
#define NMEA___GGA_H

#include <nmea/sentence.hpp>
#include <nmea/field.hpp>

namespace nmea {

/// \brief An NMEA GGA sentence.
struct gga
{
    // OBJECTS
    /// \brief Enumerates the potential fix types that can be reported.
    enum class fix_type
    {
        NONE = 0,               ///< No fix.
        GPS = 1,                ///< GPS fix.
        DGPS = 2,               ///< Differential GPS fix.
        PPS = 3,                ///< Pulse Per Second (PPS) fix.
        RTK = 4,                ///< Real-Time Kinematic (RTK) fix.
        RTK_FLOAT = 5,          ///< Real-Time Kinematic (RTK) floating point fix.
        ESTIMATED = 6,          ///< Estimated dead reckoning fix.
        MANUAL = 7,             ///< Manual input mode.
        SIMULATION = 8          ///< Simulation mode.
    };

    // CONSTRUCTORS
    /// \brief Creates a new GGA instance from an NMEA sentence.
    /// \param sentence The NMEA sentence to parse the GGA sentence from.
    gga(const nmea::sentence& sentence);

    // FIELDS
    /// \brief The name of the sentence talker.
    std::string talker;
    /// \brief The UTC time of day, in seconds.
    nmea::field<double> utc;
    /// \brief The latitude measurement in degrees (N = +, S = -).
    nmea::field<double> latitude;
    /// \brief The longitude measurement in degrees (E = +, W = -).
    nmea::field<double> longitude;
    /// \brief The type of fix reported.
    nmea::field<fix_type> fix;
    /// \brief The number of satellites used in the fix.
    nmea::field<uint8_t> satellite_count;
    /// \brief The horizontal dilution of precision (HDOP), in meters.
    nmea::field<float> hdop;
    /// \brief The altitude measurement, in meters above sea level.
    nmea::field<float> altitude;
    /// \brief The height of the geoid above sea level at this location, in meters.
    nmea::field<float> geoid_separation;
    /// \brief The time since the last differential GPS update, in seconds.
    nmea::field<float> dgps_age;
    /// \brief The ID of the differential GPS station.
    nmea::field<uint16_t> dgps_station;
};

}

#endif