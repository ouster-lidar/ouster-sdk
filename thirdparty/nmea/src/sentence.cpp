#include <nmea/sentence.hpp>

#include <sstream>
#include <iomanip>

using namespace nmea;

// CONSTRUCTOR
sentence::sentence(const std::string& nmea_string)
{
    // Get first field, which is the tag.
    std::size_t delimeter_index = nmea_string.find_first_of(',');
    // Talker, assuming the last 3 characters of the tag field are the message type.
    // NOTE: This allows for talkers that are greater than 2 characters.
    sentence::m_talker = nmea_string.substr(1, delimeter_index - 4);
    // Type, assuming it is always 3 characters.
    sentence::m_type = nmea_string.substr(delimeter_index - 3, 3);

    // Get remainder of fields.
    while(delimeter_index != std::string::npos)
    {
        // Delimeter is pointing to the comma at the start of this field.

        // Find the index of the next comma.
        std::size_t next_delimiter_index = nmea_string.find_first_of(',', delimeter_index + 1);

        // Calculate the read length.
        std::size_t read_length;
        if(next_delimiter_index == std::string::npos)
        {
            // No following comma found, so must be at end of the string.
            // NOTE: string is already checked to be valid, so it must have checksum.
            read_length = nmea_string.find_last_of('*') - delimeter_index - 1;
        }
        else
        {
            // Following comma found, set length using position of next comma.
            read_length = next_delimiter_index - delimeter_index - 1;
        }

        // Pull substring into field.
        sentence::m_fields.push_back(nmea_string.substr(delimeter_index + 1, read_length));

        // Update delimeter index.
        delimeter_index = next_delimiter_index;
    }
}
sentence::sentence(const std::string& talker, const std::string& type, uint8_t n_fields)
    : m_talker(talker),
      m_type(type),
      m_fields(n_fields, "")
{}

// STATIC
bool sentence::validate(const std::string& nmea_string)
{
    // Check for empty string.
    if(nmea_string.empty())
    {
        return false;
    }

    // Check for $/! as first character.
    if(!(nmea_string.front() == '$' || nmea_string.front() == '!'))
    {
        return false;
    }
    
    // Check for * character.
    std::size_t checksum_index = nmea_string.find_last_of('*');
    if(checksum_index == std::string::npos)
    {
        return false;
    }

    // Verify string includes checksum data.
    if(nmea_string.length() < checksum_index + 3)
    {
        return false;
    }

    // Calculate expected checksum.
    std::string expected_checksum = sentence::checksum(nmea_string, checksum_index);

    // Compare to actual checksum.
    return nmea_string.compare(checksum_index + 1, 2, expected_checksum, 0, 2) == 0;
}
std::string sentence::checksum(const std::string& nmea_string, std::size_t checksum_index)
{
    // Calculate checksum = XOR of everything between $/! and *
    // NOTE: checksum_index = index of * delimiter.
    uint8_t checksum = 0;
    for(uint8_t i = 1; i < checksum_index; ++i)
    {
        checksum ^= nmea_string.at(i);
    }

    // Convert checksum to hex string.
    std::stringstream stream;
    stream << std::hex << std::uppercase << std::setfill('0') << std::setw(2) << static_cast<uint32_t>(checksum);

    return stream.str();
}

// PROPERTIES
std::string sentence::talker() const
{
    return sentence::m_talker;
}
std::string sentence::type() const
{
    return sentence::m_type;
}
uint8_t sentence::field_count() const
{
    return sentence::m_fields.size();
}
std::string sentence::get_field(uint8_t field) const
{
    // Check if field exists.
    if(field < sentence::m_fields.size())
    {
        // Get field.
        return sentence::m_fields.at(field);
    }
    else
    {
        // Field doesn't exist, return empty string.
        return "";
    }
}
void sentence::set_field(uint8_t field, const std::string& value)
{
    // Check if field exists.
    if(field < sentence::m_fields.size())
    {
        // Set field.
        sentence::m_fields.at(field) = value;
    }
}
std::string sentence::nmea_string(bool encapsulated) const
{
    // Create output string.
    std::string nmea_string;

    // Write start delimiter.
    if(!encapsulated)
    {
        nmea_string.append("$");
    }
    else
    {
        nmea_string.append("!");
    }

    // Write talker and type.
    nmea_string.append(sentence::m_talker);
    nmea_string.append(sentence::m_type);
    
    // Write fields.
    for(auto field = sentence::m_fields.cbegin(); field != sentence::m_fields.cend(); ++field)
    {
        nmea_string.append(",");
        nmea_string.append(*field);
    }

    // Write checksum delimeter.
    nmea_string.append("*");

    // Calculate and write checksum.
    nmea_string.append(sentence::checksum(nmea_string, nmea_string.length() - 1));

    // Write CRLF.
    nmea_string.append("\r\n");

    return nmea_string;
}