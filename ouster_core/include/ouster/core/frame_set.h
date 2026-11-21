/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <cstddef>
#include <initializer_list>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "ouster/core/camera_frame.h"
#include "ouster/core/field.h"
#include "ouster/core/frame.h"
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
     * Initialize a FrameSet with a single vector of base frame pointers.
     *
     * Entries may be null (e.g. empty slots produced by collation) and may mix
     * concrete frame types (LidarFrame, CameraFrame, ...).
     *
     * @param[in] frames vector of base frame shared pointers.
     */
    OUSTER_API_FUNCTION
    explicit FrameSet(std::vector<std::shared_ptr<Frame>> frames);

    /**
     * Initialize a FrameSet with a vector of lidar frames.
     *
     * Convenience overload equivalent to constructing from a vector of base
     * Frame pointers. To include camera (or mixed) frames, use the base
     * std::vector<std::shared_ptr<Frame>> constructor.
     *
     * @param[in] frames vector of lidar frame shared pointers.
     */
    OUSTER_API_FUNCTION
    FrameSet(const std::vector<std::shared_ptr<LidarFrame>>& frames);

    /**
     * Initialize a FrameSet with an initializer list of lidar frames.
     *
     * @param[in] frames initializer list of lidar frame shared pointers.
     */
    OUSTER_API_FUNCTION
    FrameSet(std::initializer_list<std::shared_ptr<LidarFrame>> frames);

    /**
     * @return iterator to the beginning of the frames
     */
    OUSTER_API_FUNCTION
    std::vector<std::shared_ptr<Frame>>::iterator begin();

    /**
     * @return iterator past the end of the frames
     */
    OUSTER_API_FUNCTION
    std::vector<std::shared_ptr<Frame>>::iterator end();

    /**
     * @return const iterator to the beginning of the frames
     */
    OUSTER_API_FUNCTION
    std::vector<std::shared_ptr<Frame>>::const_iterator begin() const;

    /**
     * @return const iterator past the end of the frames
     */
    OUSTER_API_FUNCTION
    std::vector<std::shared_ptr<Frame>>::const_iterator end() const;

    /**
     * @return an iterator over valid (non-null) frames
     */
    OUSTER_API_FUNCTION
    ValidObjectIterator<Frame> valid_frames() {
        return ValidObjectIterator<Frame>(frames_.begin(), frames_.end());
    }

    /**
     * @return a const iterator over valid (non-null) frames
     */
    OUSTER_API_FUNCTION
    ValidObjectConstIterator<Frame> valid_frames() const {
        return ValidObjectConstIterator<Frame>(frames_.begin(), frames_.end());
    }

    /**
     * Get the valid (non-null) frames of a single concrete frame type.
     *
     * Filters the frames to those whose concrete type is FrameT, discarding
     * null slots and frames of any other kind.
     *
     * @note the returned positions are not frame-slot indices. Use
     * frames() / operator[] when a frame's slot position matters (for example
     * to index into FrameSetSource::sensor_info()).
     *
     * @tparam FrameT the concrete frame type to select, e.g. LidarFrame.
     *
     * @return a vector of shared pointers to the valid frames of type FrameT.
     */
    template <typename FrameT>
    std::vector<std::shared_ptr<FrameT>> valid_frames_of() const {
        // std::is_base_of_v is C++17; library code is C++14.
        // NOLINTNEXTLINE(modernize-type-traits)
        static_assert(std::is_base_of<Frame, FrameT>::value,
                      "valid_frames_of's FrameT must derive from Frame");
        std::vector<std::shared_ptr<FrameT>> out;
        for (const auto& frame : frames_) {
            if (auto typed = std::dynamic_pointer_cast<FrameT>(frame)) {
                out.push_back(std::move(typed));
            }
        }
        return out;
    }

    /**
     * Get the valid (non-null) lidar frames, downcast to LidarFrame.
     *
     * Named alias for valid_frames_of<LidarFrame>().
     *
     * @return a vector of shared pointers to the valid lidar frames.
     */
    OUSTER_API_FUNCTION
    std::vector<std::shared_ptr<LidarFrame>> valid_lidar_frames() const;

    /**
     * Get the valid (non-null) camera frames, downcast to CameraFrame.
     *
     * Named alias for valid_frames_of<CameraFrame>().
     *
     * @return a vector of shared pointers to the valid camera frames.
     */
    OUSTER_API_FUNCTION
    std::vector<std::shared_ptr<CameraFrame>> valid_camera_frames() const;

    /**
     * @return an iterator that yields the indices of valid (non-null) frames
     */
    OUSTER_API_FUNCTION
    inline ValidIndexIterator<Frame> valid_indices() {
        return ValidIndexIterator<Frame>(frames_.begin(), frames_.end());
    }

    /**
     * @return a const iterator that yields the indices of valid (non-null)
     * frames
     */
    OUSTER_API_FUNCTION
    inline ValidIndexConstIterator<Frame> valid_indices() const {
        return ValidIndexConstIterator<Frame>(frames_.begin(), frames_.end());
    }

    /**
     * Returns nth frame in the FrameSet.
     *
     * @param[in] index index of the frame to return
     *
     * @return shared ptr of frame at the given index
     */
    OUSTER_API_FUNCTION
    const std::shared_ptr<Frame>& operator[](size_t index) const;

    /**
     * Returns nth frame in the FrameSet.
     *
     * @param[in] index index of the frame to return
     *
     * @return shared ptr of frame at the given index
     */
    OUSTER_API_FUNCTION
    std::shared_ptr<Frame>& operator[](size_t index);

    /**
     * Returns number of frames stored in the FrameSet
     *
     * @return number of frames
     */
    OUSTER_API_FUNCTION size_t size() const;

    /**
     * Add a new zero-filled field to FrameSet.
     *
     * @throw std::invalid_argument if key duplicates a preexisting field
     *
     * @param[in] name string key of the field to add
     * @param[in] desc descriptor of the field to add
     * @param[in] zero If true, zero the field otherwise memory is uninitialized.
     *
     * @return field
     */
    OUSTER_API_FUNCTION
    Field& add_field(const std::string& name, const FieldDescriptor& desc, bool zero = true);

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
     * @return The vector of shared pointers to base frames. Entries may be null
     *         and may be of any concrete frame type.
     */
    OUSTER_API_FUNCTION
    std::vector<std::shared_ptr<Frame>>& frames();

    /** @copydoc frames() (const overload) */
    OUSTER_API_FUNCTION
    const std::vector<std::shared_ptr<Frame>>& frames() const;

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
    std::vector<std::shared_ptr<Frame>> frames_;
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

/**
 * Compare two FrameSets for inequality.
 *
 * @param[in] a The first FrameSet to compare.
 * @param[in] b The second FrameSet to compare.
 *
 * @return true if the FrameSets are not equal
 */
OUSTER_API_FUNCTION
bool operator!=(const FrameSet& a, const FrameSet& b);

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
