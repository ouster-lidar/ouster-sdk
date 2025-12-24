/// \file nmea/object/status.hpp
/// \brief Defines the nmea::status enumeration.
#ifndef NMEA___STATUS_H
#define NMEA___STATUS_H

namespace nmea {

/// \brief Enumerates data status values.
enum class status
{
    ACTIVE = 0,     ///< Data is valid.
    VOID = 1        ///< Data is invalid.
};

}

#endif