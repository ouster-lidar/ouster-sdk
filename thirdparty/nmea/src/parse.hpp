/// \file parse.hpp
/// \brief Defines common NMEA parsing functions.
#ifndef NMEA___PARSE_H
#define NMEA___PARSE_H

#include <nmea/field.hpp>
#include <nmea/sentence.hpp>
#include <nmea/object/date.hpp>
#include <nmea/object/status.hpp>
#include <nmea/object/mode.hpp>

namespace nmea {

// BASE
/// \brief Parses a INT8 field from an NMEA sentence.
/// \param field The NMEA field to parse the value into.
/// \param sentence The NMEA sentence to read the value from.
/// \param field_index The index of the field in the NMEA sentence.
void parse_int8(nmea::field<int8_t>& field, const nmea::sentence& sentence, uint8_t field_index);
/// \brief Parses a UINT8 field from an NMEA sentence.
/// \param field The NMEA field to parse the value into.
/// \param sentence The NMEA sentence to read the value from.
/// \param field_index The index of the field in the NMEA sentence.
void parse_uint8(nmea::field<uint8_t>& field, const nmea::sentence& sentence, uint8_t field_index);
/// \brief Parses a UINT16 field from an NMEA sentence.
/// \param field The NMEA field to parse the value into.
/// \param sentence The NMEA sentence to read the value from.
/// \param field_index The index of the field in the NMEA sentence.
void parse_uint16(nmea::field<uint16_t>& field, const nmea::sentence& sentence, uint8_t field_index);
/// \brief Parses a float field from an NMEA sentence.
/// \param field The NMEA field to parse the value into.
/// \param sentence The NMEA sentence to read the value from.
/// \param field_index The index of the field in the NMEA sentence.
void parse_float(nmea::field<float>& field, const nmea::sentence& sentence, uint8_t field_index);
/// \brief Parses an enumeration field from an NMEA sentence.
/// \tparam enum_type The object type of the enumeration to parse.
/// \param field The NMEA field to parse the value into.
/// \param sentence The NMEA sentence to read the value from.
/// \param field_index The index of the field in the NMEA sentence.
template <class enum_type>
void parse_enum(nmea::field<enum_type>& field, const nmea::sentence& sentence, uint8_t field_index)
{
    // Get field string.
    std::string field_string = sentence.get_field(field_index);

    // Check if field exists.
    if(!field_string.empty())
    {
        // Set field with enum value.
        field.set(static_cast<enum_type>(std::stoi(field_string)));
    }
}

// EXTENDED
/// \brief Parses UTC time of day from an NMEA sentence.
/// \param field The NMEA field to parse the value into.
/// \param sentence The NMEA sentence to read the value from.
/// \param field_index The index of the field in the NMEA sentence.
void parse_utc(nmea::field<double>& field, const nmea::sentence& sentence, uint8_t field_index);
/// \brief Parses a date from an NMEA sentence.
/// \param field The NMEA field to parse the value into.
/// \param sentence The NMEA sentence to read the value from.
/// \param field_index The index of the field in the NMEA sentence.
void parse_date(nmea::field<nmea::date>& field, const nmea::sentence& sentence, uint8_t field_index);
/// \brief Parses latitude from an NMEA sentence.
/// \param field The NMEA field to parse the value into.
/// \param sentence The NMEA sentence to read the value from.
/// \param field_index The index of the field in the NMEA sentence.
void parse_latitude(nmea::field<double>& field, const nmea::sentence& sentence, uint8_t field_index);
/// \brief Parses longitude from an NMEA sentence.
/// \param field The NMEA field to parse the value into.
/// \param sentence The NMEA sentence to read the value from.
/// \param field_index The index of the field in the NMEA sentence.
void parse_longitude(nmea::field<double>& field, const nmea::sentence& sentence, uint8_t field_index);
/// \brief Parses a status from an NMEA sentence.
/// \param field The NMEA field to parse the value into.
/// \param sentence The NMEA sentence to read the value from.
/// \param field_index The index of the field in the NMEA sentence.
void parse_status(nmea::field<nmea::status>& field, const nmea::sentence& sentence, uint8_t field_index);
/// \brief Parses a mode from an NMEA sentence.
/// \param field The NMEA field to parse the value into.
/// \param sentence The NMEA sentence to read the value from.
/// \param field_index The index of the field in the NMEA sentence.
void parse_mode(nmea::field<nmea::mode>& field, const nmea::sentence& sentence, uint8_t field_index);

}

#endif