/// \file nmea/sentence.hpp
/// \brief Defines the nmea::sentence class.
#ifndef NMEA___SENTENCE_H
#define NMEA___SENTENCE_H

#include <cstdint>
#include <string>
#include <vector>

/// \brief Contains all code related to NMEA processing.
namespace nmea {

/// \brief An NMEA 0183 sentence.
class sentence
{
public:
    // CONSTRUCTORS
    /// \brief Creates a new sentence instance from a received NMEA string.
    /// \param nmea_string The NMEA 0183 string to parse.
    sentence(const std::string& nmea_string);
    /// \brief Creates a new empty sentence instance.
    /// \param talker The name of the sentence talker.
    /// \param type The type of the sentence.
    /// \param n_fields The number of data fields in the sentence.
    sentence(const std::string& talker, const std::string& type, uint8_t n_fields = 0);

    // UTILITY
    /// \brief Validates an NMEA string.
    /// \param nmea_string The NMEA string to validate.
    /// \returns TRUE if the string is valid, otherwise FALSE.
    static bool validate(const std::string& nmea_string);

    // PROPERTIES
    /// \brief Gets the talker of the sentence.
    /// \returns The talker name.
    std::string talker() const;
    /// \brief Gets the type of the sentence.
    /// \returns The sentence type.
    std::string type() const;
    /// \brief Gets the number of data fields in the sentence.
    /// \brief The number of data fields.
    uint8_t field_count() const;
    /// \brief Gets a data field from the sentence.
    /// \param field The index of the data field to retrieve.
    /// \returns The value of the field, or empty string if the field does not exist.
    std::string get_field(uint8_t field) const;
    /// \brief Sets a data field in the sentence.
    /// \param field The index of the data field to write.
    /// \param value The value to write to the data field.
    /// \note Ignores fields that do not exist.
    void set_field(uint8_t field, const std::string& value);
    /// \brief Gets the sentence as an NMEA string, with CRLF appended.
    /// \param encapsulated Indicates if this sentence is an encapsulated type, and should start with '!' instead of '$'.
    /// \returns The NMEA string representation of the sentence.
    std::string nmea_string(bool encapsulated = false) const;    

private:
    // VARIABLES
    /// \brief The name of the sentence talker.
    std::string m_talker;
    /// \brief The sentence type.
    std::string m_type;
    /// \brief The data fields of the sentence.
    std::vector<std::string> m_fields;

    // UTILITY
    /// \brief Calculates the checksum of an NMEA string.
    /// \param nmea_string The NMEA string to calculate the checksum for.
    /// \param checksum_index The index of the checksum delimeter (*) in the string.
    /// \returns The 2-character hex checksum.
    static std::string checksum(const std::string& nmea_string, std::size_t checksum_index);
};

}

#endif
