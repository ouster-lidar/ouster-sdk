/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <cstddef>
#include <initializer_list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ouster/core/deprecation.h"
#include "ouster/core/field.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/object.h"
#include "ouster/core/types.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * @brief An iterator for valid elements in a container
 * @tparam T The type of the elements in the container
 * @tparam Iter The type of the underlying iterator
 * @tparam IndexValue If true, the dereference operator returns the index
 * (size_t) of the valid element; if false, the dereference operator returns a
 * reference to the underlying object (T& or const T&)
 */
template <typename T, typename Iter, bool IndexValue = false>
class ValidElementsIterator {
    Iter position_;
    Iter end_;
    Iter begin_;

   public:
    /**
     * @brief Constructor
     * @param[in] begin The begin iterator of the underlying container
     * @param[in] end The end iterator of the underlying container
     */
    ValidElementsIterator(Iter begin, Iter end) : position_(begin), end_(end), begin_(begin) {
        advance_to_valid();
    }

    /**
     * @brief Provide range-based for loop support
     * @return begin iterator
     */
    ValidElementsIterator begin() {
        return *this;
    }

    /**
     * @brief Provide range-based for loop support
     * @return end iterator
     */
    ValidElementsIterator end() {
        auto it = *this;
        it.position_ = it.end_;
        return it;
    }

    /**
     * @brief dereference operator
     * @note enabled when IndexValue == true
     * @return index (size_t)
     */
    template <bool B = IndexValue>
    typename std::enable_if<B, size_t>::type operator*() {
        return static_cast<size_t>(std::distance(begin_, position_));
    }

    /**
     * @brief dereference operator
     * @note enabled when IndexValue == true
     * @return index (size_t)
     */
    template <bool B = IndexValue>
    typename std::enable_if<B, size_t>::type operator*() const {
        return static_cast<size_t>(std::distance(begin_, position_));
    }

    /**
     * @brief dereference operator
     * @note enabled when IndexValue == false
     * @return reference to underlying T (T&)
     */
    template <bool B = IndexValue>
    typename std::enable_if<!B, const T&>::type operator*() {
        return *position_;
    }

    /**
     * @brief dereference operator
     * @note enabled when IndexValue == false
     * @return reference to underlying T (const T&)
     */
    template <bool B = IndexValue>
    typename std::enable_if<!B, const T&>::type operator*() const {
        return *position_;
    }

    /**
     * @brief pre-increment operator
     * @return reference to this iterator
     */
    ValidElementsIterator& operator++() {
        ++position_;
        advance_to_valid();
        return *this;
    }

    /**
     * @brief post-increment operator
     * @return copy of this iterator before increment
     */
    ValidElementsIterator operator++(int) {
        ValidElementsIterator tmp = *this;
        ++position_;
        advance_to_valid();
        return tmp;
    }

    /**
     * @brief equality operator
     * @param[in] other the other iterator to compare to
     * @return true if both iterators point to the same position
     */
    bool operator==(const ValidElementsIterator& other) const {
        return position_ == other.position_;
    }

    /**
     * @brief inequality operator
     * @param[in] other the other iterator to compare to
     * @return true if both iterators point to different positions
     */
    bool operator!=(const ValidElementsIterator& other) const {
        return position_ != other.position_;
    }

   private:
    void advance_to_valid() {
        while (position_ != end_ && !(*position_)) ++position_;
    }
};

/**
 * @brief Iterator over valid (non-null) shared object entries.
 *
 * Alias for a ValidElementsIterator that traverses a
 * std::vector<std::shared_ptr<T>>, automatically skipping any entries deemed
 * invalid (e.g. null std::shared_ptr).
 *
 * @tparam T The pointee/object type stored inside std::shared_ptr<T>.
 *
 * @see ValidElementsIterator
 */
template <typename T>
using ValidObjectIterator =
    ValidElementsIterator<std::shared_ptr<T>, typename std::vector<std::shared_ptr<T>>::iterator,
                          false>;

/**
 * @brief Same as ValidObjectIterator but provides const access.
 *
 * @tparam T The pointee/object type stored inside std::shared_ptr<T>.
 *
 * @see ValidElementsIterator
 * @see ValidObjectIterator
 */
template <typename T>
using ValidObjectConstIterator =
    ValidElementsIterator<std::shared_ptr<T>,
                          typename std::vector<std::shared_ptr<T>>::const_iterator, false>;

/**
 * @brief Iterator over a vector of shared pointers that yields the indices of
 * valid (non-null) entries.
 *
 * Alias for a ValidElementsIterator that traverses a
 * std::vector<std::shared_ptr<T>>, automatically skipping any entries deemed
 * invalid (e.g. null std::shared_ptr) returning their indices.
 *
 * @tparam T The pointee/object type stored inside std::shared_ptr<T>.
 *
 * @see ValidElementsIterator
 */
template <typename T>
using ValidIndexIterator =
    ValidElementsIterator<std::shared_ptr<T>, typename std::vector<std::shared_ptr<T>>::iterator,
                          true>;
/**
 * @brief Same as ValidIndexIterator but provides const access.
 *
 * @tparam T The pointee/object type stored inside std::shared_ptr<T>.
 *
 * @see ValidElementsIterator
 * @see ValidIndexIterator
 */
template <typename T>
using ValidIndexConstIterator =
    ValidElementsIterator<std::shared_ptr<T>,
                          typename std::vector<std::shared_ptr<T>>::const_iterator, true>;

/**
 * Container of lidar frames that allows storing collection-associated
 * fields that cannot be referenced back to any individual lidar frame.
 *
 * Internal resources are kept under shared pointers so copying the set
 * returns a shallow copy. Use FrameSet::clone() if necessary.
 */
class OUSTER_API_CLASS FrameSet {
   public:
    /** The default constructor creates an empty set. */
    OUSTER_API_FUNCTION FrameSet();

    /**
     * Initialize a FrameSet with a vector of lidar frames.
     *
     * @param[in] frames vector of lidar frame shared pointers.
     */
    OUSTER_API_FUNCTION
    FrameSet(const std::vector<std::shared_ptr<LidarFrame>>& frames);

    /**
     * Initialize a FrameSet with a vector of lidar frames.
     *
     * @param[in] frames vector of lidar frame shared pointers to steal.
     */
    OUSTER_API_FUNCTION
    FrameSet(std::vector<std::shared_ptr<LidarFrame>>&& frames);

    /**
     * Initialize a FrameSet with an initializer list.
     *
     * @param[in] frames initializer list of lidar frame shared pointers.
     */
    OUSTER_API_FUNCTION
    FrameSet(std::initializer_list<std::shared_ptr<LidarFrame>> frames);

    /**
     * @return iterator to the beginning of the lidar frames
     */
    OUSTER_API_FUNCTION
    std::vector<std::shared_ptr<LidarFrame>>::iterator begin();

    /**
     * @return iterator past the end of the lidar frames
     */
    OUSTER_API_FUNCTION
    std::vector<std::shared_ptr<LidarFrame>>::iterator end();

    /**
     * @return const iterator to the beginning of the lidar frames
     */
    OUSTER_API_FUNCTION
    std::vector<std::shared_ptr<LidarFrame>>::const_iterator begin() const;

    /**
     * @return const iterator past the end of the lidar frames
     */
    OUSTER_API_FUNCTION
    std::vector<std::shared_ptr<LidarFrame>>::const_iterator end() const;

    /**
     * @return an iterator over valid (non-null) lidar frames
     */
    OUSTER_API_FUNCTION
    ValidObjectIterator<LidarFrame> valid_frames() {
        return ValidObjectIterator<LidarFrame>(frames_.begin(), frames_.end());
    }

    /**
     * @return a const iterator over valid (non-null) lidar frames
     */
    OUSTER_API_FUNCTION
    ValidObjectConstIterator<LidarFrame> valid_frames() const {
        return ValidObjectConstIterator<LidarFrame>(frames_.begin(), frames_.end());
    }

    /// @deprecated Use valid_frames() instead
    /// @copydoc valid_frames()
    OUSTER_DEPRECATED_MSG("valid_frames()", OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
    OUSTER_API_FUNCTION
    ValidObjectIterator<LidarFrame> valid_scans() {
        return valid_frames();
    }

    /// @deprecated Use valid_frames() instead
    /// @copydoc valid_frames() const
    OUSTER_DEPRECATED_MSG("valid_frames()", OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
    OUSTER_API_FUNCTION
    ValidObjectConstIterator<LidarFrame> valid_scans() const {
        return valid_frames();
    }

    /**
     * @return an iterator that yields the indices of valid (non-null) lidar
     * frames
     */
    OUSTER_API_FUNCTION
    ValidIndexIterator<LidarFrame> valid_indices() {
        return ValidIndexIterator<LidarFrame>(frames_.begin(), frames_.end());
    }

    /**
     * @return a const iterator that yields the indices of valid (non-null)
     * lidar frames
     */
    OUSTER_API_FUNCTION
    ValidIndexConstIterator<LidarFrame> valid_indices() const {
        return ValidIndexConstIterator<LidarFrame>(frames_.begin(), frames_.end());
    }

    /**
     * Returns nth frame in the FrameSet.
     *
     * @param[in] index index of the frame to return
     *
     * @return shared ptr of frame at the given index
     */
    OUSTER_API_FUNCTION
    const std::shared_ptr<LidarFrame>& operator[](size_t index) const;

    /**
     * Returns nth frame in the FrameSet.
     *
     * @param[in] index index of the frame to return
     *
     * @return shared ptr of frame at the given index
     */
    OUSTER_API_FUNCTION
    std::shared_ptr<LidarFrame>& operator[](size_t index);

    /**
     * Returns number of lidar frames stored in the FrameSet
     *
     * @return number of lidar frames
     */
    OUSTER_API_FUNCTION size_t size() const;

    /**
     * Add a new zero-filled field to FrameSet.
     *
     * @throw std::invalid_argument if key duplicates a preexisting field
     *
     * @param[in] name string key of the field to add
     * @param[in] desc descriptor of the field to add
     *
     * @return field
     */
    OUSTER_API_FUNCTION
    Field& add_field(const std::string& name, const FieldDescriptor& desc);

    /**
     * Release the field and remove it from FrameSet
     *
     * @throw std::invalid_argument if field under key does not exist
     *
     * @param[in] name string key of the field to remove
     *
     * @return field The deleted field.
     */
    OUSTER_API_FUNCTION
    Field del_field(const std::string& name);

    /**
     * Check if a field exists
     *
     * @param[in] name string key of the field to check
     *
     * @return true if the FrameSet has the field, else false
     */
    OUSTER_API_FUNCTION
    bool has_field(const std::string& name) const;

    /**
     * @param[in] name string key of the field to access
     *
     * @return Field reference of the requested field
     */
    OUSTER_API_FUNCTION
    Field& field(const std::string& name);

    /**
     * @copydoc field
     */
    OUSTER_API_FUNCTION
    const Field& field(const std::string& name) const;

    /**
     * Reference to the internal fields map
     *
     * @return The unordered map of names and fields.
     */
    OUSTER_API_FUNCTION
    std::unordered_map<std::string, Field>& fields();

    /** @copydoc fields() */
    OUSTER_API_FUNCTION
    const std::unordered_map<std::string, Field>& fields() const;

    /**
     * Reference to the internal vector of frames
     *
     * @return The vector of shared pointers to lidar frames.
     */
    OUSTER_API_FUNCTION
    std::vector<std::shared_ptr<LidarFrame>>& frames();

    /** @copydoc frames() (const overload) */
    OUSTER_API_FUNCTION
    const std::vector<std::shared_ptr<LidarFrame>>& frames() const;

    /// @deprecated Use frames() instead
    /// @copydoc frames()
    OUSTER_DEPRECATED_MSG("frames()", OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
    OUSTER_API_FUNCTION
    std::vector<std::shared_ptr<LidarFrame>>& scans();

    /// @deprecated Use frames() instead
    /// @copydoc frames() const
    OUSTER_DEPRECATED_MSG("frames()", OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
    OUSTER_API_FUNCTION
    const std::vector<std::shared_ptr<LidarFrame>>& scans() const;

    /**
     * Reference to the internal objects map
     *
     * @return The unordered map of names and object lists.
     */
    OUSTER_API_FUNCTION
    std::unordered_map<std::string, std::vector<Object>>& objects();

    /** @copydoc objects() */
    OUSTER_API_FUNCTION
    const std::unordered_map<std::string, std::vector<Object>>& objects() const;

    /**
     * Swaps two FrameSets
     *
     * @param[in] other FrameSet to swap resources with
     */
    OUSTER_API_FUNCTION
    void swap(FrameSet& other) noexcept;

    /**
     * Get a deep copy of a FrameSet.
     *
     * @return deep copy
     */
    OUSTER_API_FUNCTION
    FrameSet clone() const;

   private:
    std::vector<std::shared_ptr<LidarFrame>> frames_;
    std::shared_ptr<std::unordered_map<std::string, Field>> fields_;
    std::shared_ptr<std::unordered_map<std::string, std::vector<Object>>> objects_;
};

/**
 * Compare two FrameSets.
 *
 * @param[in] a The first FrameSet to compare.
 * @param[in] b The second FrameSet to compare.
 *
 * @return true if FrameSets are equal
 */
OUSTER_API_FUNCTION
bool operator==(const FrameSet& a, const FrameSet& b);

}  // namespace core
}  // namespace sdk
}  // namespace ouster

namespace std {

/**
 * std::swap overload, used by some std algorithms
 *
 * @param[in] a FrameSet to swap with b
 * @param[in] b FrameSet to swap with a
 */
OUSTER_API_FUNCTION
void swap(ouster::sdk::core::FrameSet& a, ouster::sdk::core::FrameSet& b);

}  // namespace std

namespace ouster {
namespace sdk {
namespace core {

OUSTER_DEPRECATED_TYPE(LidarScanSet, FrameSet, OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)

}  // namespace core
}  // namespace sdk
}  // namespace ouster
