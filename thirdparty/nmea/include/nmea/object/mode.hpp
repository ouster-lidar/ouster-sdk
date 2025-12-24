/// \file nmea/object/mode.hpp
/// \brief Defines the nmea::mode enumeration.
#ifndef NMEA___MODE_H
#define NMEA___MODE_H

namespace nmea {

/// \brief Enumerates the FFA mode types.
enum class mode
{
    AUTONOMOUS = 0,     ///< Autonomous mode.
    DIFFERENTIAL = 1,   ///< Differential GPS (DGPS) mode.
    ESTIMATED = 2,      ///< Estimated dead-reckoning mode.
    MANUAL = 3,         ///< Manual input mode.
    SIMULATED = 4,      ///< Simulation mode.
    INVALID = 5         ///< Data is invalid.
};

}

#endif