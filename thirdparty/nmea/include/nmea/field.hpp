/// \file nmea/field.hpp
/// \brief Defines the nmea::field class.
#ifndef NMEA___FIELD_H
#define NMEA___FIELD_H

namespace nmea {

/// \brief Represents an optional field in an NMEA sentence.
/// \tparam object_type The object type of the field.
template <class object_type>
class field
{
public:
    // CONSTRUCTORS
    /// \brief Creates an empty field instance.
    field()
        : m_exists(false)
    {}
    /// \brief Creates a new field instance with a set value.
    /// \param value The value to initialize the field with.
    field(const object_type& value)
        : m_value(value),
          m_exists(true)
    {}

    // ACCESS
    /// \brief Indicates if the field has an existing value.
    /// \returns TRUE if the field exists, otherwise FALSE.
    bool exists() const
    {
        return m_exists;
    }
    /// \brief Sets the value of the field.
    /// \param value The value to set.
    void set(const object_type& value)
    {
        m_value = value;
        m_exists = true;
    }
    /// \brief Gets the value of the field.
    /// \returns A const reference to the value.
    const object_type& get() const
    {
        return m_value;
    }
    /// \brief Clears the value from the field.
    void clear()
    {
        m_exists = false;
    }

private:
    // VARIABLES
    /// \brief The stored value.
    object_type m_value;
    /// \brief Flag indicating if the value has been set.
    bool m_exists;
};

}

#endif