#pragma once
#include <string>
#include <unordered_map>

#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * A mapping from class IDs to class names.
 */
struct OUSTER_API_CLASS ClassMap {
    /// Mapping of class IDs to class names
    std::unordered_map<int, std::string> class_map;
    /**
     * Compare this ClassMap to another for equality. Two ClassMaps are equal if
     * they contain the same class IDs and the corresponding class names are
     * equal.
     * @param[in] other The ClassMap to compare to.
     * @return True if the ClassMaps are equal, false otherwise.
     */
    OUSTER_API_FUNCTION
    bool operator==(const ClassMap& other) const {
        return class_map == other.class_map;
    }

    /**
     * Compare this ClassMap to another for inequality. Two ClassMaps are not
     * equal if they do not contain the same class IDs or the corresponding
     * class names are not equal.
     * @param[in] other The ClassMap to compare to.
     * @return True if the ClassMaps are not equal, false otherwise.
     */
    OUSTER_API_FUNCTION
    bool operator!=(const ClassMap& other) const {
        return !(*this == other);
    }

    /**
     * @return iterator to the beginning of the class map
     */
    OUSTER_API_FUNCTION
    std::unordered_map<int, std::string>::const_iterator begin() const {
        return class_map.begin();
    }

    /**
     * @return iterator to the end of the class map
     */
    OUSTER_API_FUNCTION
    std::unordered_map<int, std::string>::const_iterator end() const {
        return class_map.end();
    }

    /**
     * @return iterator to the beginning of the class map
     */
    OUSTER_API_FUNCTION
    std::unordered_map<int, std::string>::iterator begin() {
        return class_map.begin();
    }

    /**
     * @return iterator to the end of the class map
     */
    OUSTER_API_FUNCTION
    std::unordered_map<int, std::string>::iterator end() {
        return class_map.end();
    }
};

/**
 * A set of ClassMaps, indexed by string keys.
 */
struct OUSTER_API_CLASS ClassMapSet {
    /// Mapping of string keys to ClassMaps
    std::unordered_map<std::string, ClassMap> class_maps;
    /**
     * Compare this ClassMapSet to another for equality. Two ClassMapSets are
     * equal if they contain the same keys and the corresponding ClassMaps are
     * equal.
     * @param[in] other The ClassMapSet to compare to.
     * @return True if the ClassMapSets are equal, false otherwise.
     */
    OUSTER_API_FUNCTION
    bool operator==(const ClassMapSet& other) const {
        return class_maps == other.class_maps;
    }

    /**
     * Compare this ClassMapSet to another for inequality. Two ClassMapSets are
     * not equal if they do not contain the same keys or the corresponding
     * ClassMaps are not equal.
     * @param[in] other The ClassMapSet to compare to.
     * @return True if the ClassMapSets are not equal, false otherwise.
     */
    OUSTER_API_FUNCTION
    bool operator!=(const ClassMapSet& other) const {
        return !(*this == other);
    }

    /**
     * Get the keys of the ClassMaps in this set.
     * @return A vector of strings representing the keys of the ClassMaps in
     * this set.
     */
    OUSTER_API_FUNCTION
    std::vector<std::string> keys() const {  // TODO[tws] de-inline
        std::vector<std::string> result;
        result.reserve(class_maps.size());
        for (const auto& item : class_maps) {
            result.push_back(item.first);
        }
        return result;
    }

    /**
     * @return iterator to the beginning of the class maps
     */
    OUSTER_API_FUNCTION
    std::unordered_map<std::string, ClassMap>::const_iterator begin() const {
        return class_maps.begin();
    }

    /**
     * @return iterator to the end of the class maps
     */
    OUSTER_API_FUNCTION
    std::unordered_map<std::string, ClassMap>::const_iterator end() const {
        return class_maps.end();
    }

    /**
     * @return iterator to the beginning of the class maps
     */
    OUSTER_API_FUNCTION
    std::unordered_map<std::string, ClassMap>::iterator begin() {
        return class_maps.begin();
    }

    /**
     * @return iterator to the end of the class maps
     */
    OUSTER_API_FUNCTION
    std::unordered_map<std::string, ClassMap>::iterator end() {
        return class_maps.end();
    }
};

}  // namespace core
}  // namespace sdk
}  // namespace ouster
