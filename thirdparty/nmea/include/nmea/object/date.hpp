/// \file nmea/object/date.hpp
/// \brief Defines the nmea::date struct.
#ifndef NMEA___DATE_H
#define NMEA___DATE_H

#include <stdint.h>

namespace nmea {

/// \brief Represents a date field.
struct date
{
    /// \brief The day of the date.
    uint8_t day;
    /// \brief The month of the date.
    uint8_t month;
    /// \brief The year of the date.
    uint8_t year;
};

}

#endif