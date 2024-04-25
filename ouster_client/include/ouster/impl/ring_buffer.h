/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace ouster {
namespace sensor {
namespace impl {

/**
 * Ring buffer class for internal use.
 *
 * This is NOT thread safe, thread safety is delegated to the user.
 * Correct read/write procedure is:
 * \code
 * auto rb = RingBuffer<T>{size, T{...}};
 *
 * // write
 * if (!rb.full()) {
 *    T& element = rb.back();
 *    do_write(element);
 *    rb.push();
 * }
 *
 * // read
 * if (!rb.empty()) {
 *    T& element = rb.front();
 *    do_read(element);
 *    rb.pop();
 * }
 *
 * \endcode
 */
template <typename T>
class RingBuffer {
    static_assert(std::is_copy_constructible<T>::value,
                  "must be copy constructible");

    std::atomic<size_t> r_idx_, w_idx_;
    std::vector<T> bufs_;

    size_t _capacity() const { return bufs_.size(); }

   public:
    RingBuffer(size_t size, T value = {})
        : r_idx_(0), w_idx_(0), bufs_(size + 1, value) {}

    RingBuffer(RingBuffer&& other) {
        std::swap(bufs_, other.bufs_);
        r_idx_ = other.r_idx_.load();
        w_idx_ = other.w_idx_.load();
    }

    /**
     * Report the total capacity of allocated elements.
     */
    size_t capacity() const { return _capacity() - 1; }

    /**
     * Report the size of currently used elements.
     */
    size_t size() const {
        return (_capacity() + w_idx_ - r_idx_) % _capacity();
    }

    /**
     * Check whether ring buffer is empty.
     *
     * NOTE:
     * \code
     *    if (!rb.empty()) {
     *        // inside the block, rb.empty() is not guaranteed to stay the same
     *        // *unless* it is called by the *only* thread advancing read
     *        // indices to this buffer
     *    }
     * \endcode
     */
    bool empty() const { return w_idx_ == r_idx_; }

    /**
     * Check whether ring buffer is empty.
     *
     * NOTE:
     * \code
     *    if (!rb.full()) {
     *        // inside the block, rb.full() is not guaranteed to stay the same
     *        // *unless* it is called by the *only* thread advancing write
     *        // indices to this buffer
     *    }
     * \endcode
     */
    bool full() const { return r_idx_ == ((w_idx_ + 1) % _capacity()); }

    /**
     * Get element at the front of the ring buffer.
     */
    T& front() { return bufs_[r_idx_]; }
    const T& front() const { return bufs_[r_idx_]; }

    /**
     * Get element at the back of the ring buffer.
     */
    T& back() { return bufs_[w_idx_]; }
    const T& back() const { return bufs_[w_idx_]; }

    /**
     * Flush the ring buffer, making it empty.
     */
    void flush() { r_idx_ = w_idx_.load(); };

    /**
     * Atomically increment read index.
     *
     * Throws if ring buffer is empty.
     */
    void pop() {
        if (empty()) throw std::underflow_error("popped an empty ring buffer");
        size_t read_idx = r_idx_.load();
        while (!r_idx_.compare_exchange_strong(read_idx,
                                               (read_idx + 1) % _capacity())) {
        }
    }

    /**
     * Atomically increment write index.
     *
     * Throws if ring buffer is full.
     */
    void push() {
        if (full()) throw std::overflow_error("pushed a full ring buffer");
        size_t write_idx = r_idx_.load();
        // atomic increment modulo
        while (!w_idx_.compare_exchange_strong(write_idx,
                                               (write_idx + 1) % _capacity())) {
        }
    }
};

/**
 * Convenience class for working with multiple ring buffers.
 */
template <typename K, typename V>
class RingBufferMap {
    using MapBuffers = std::unordered_map<K, RingBuffer<V>>;
    MapBuffers rb_map_;

   public:
    using MapInputs = std::unordered_map<K, std::pair<size_t, V>>;

    RingBufferMap() {}

    RingBufferMap(const MapInputs& inputs) : rb_map_{} {
        for (const auto& pair : inputs) {
            allocate(pair.first, pair.second.first, pair.second.second);
        }
    }

    /**
     * Allocate a new ring buffer.
     *
     * @param[in] key key to allocate with
     * @param[in] size size, in elements, to allocate
     * @param[in] value default value of elements in newly allocated ring buffer
     */
    void allocate(K key, size_t size, V value) {
        if (rb_map_.find(key) != rb_map_.end()) {
            throw std::invalid_argument(
                "RingBufferMap: failed allocating a ring buffer, key already "
                "exists");
        }

        rb_map_.emplace(key, RingBuffer<V>{size, value});
    }

    /**
     * Retrieve value at the front of the ring buffer at specified key.
     */
    V& front(const K& key) { return rb_map_.at(key).front(); }
    const V& front(const K& key) const { return rb_map_.at(key).front(); }

    /**
     * Retrieve value at the back of the ring buffer at specified key.
     */
    V& back(const K& key) { return rb_map_.at(key).back(); }
    const V& back(const K& key) const { return rb_map_.at(key).back(); }

    /**
     * Advance read index of the ring buffer at specified key.
     */
    void pop(const K& key) { rb_map_.at(key).pop(); }

    /**
     * Advance write index of the ring buffer at specified key.
     */
    void push(const K& key) { rb_map_.at(key).push(); }

    /**
     * Check if the ring buffer at specified key is empty.
     */
    bool empty(const K& key) const { return rb_map_.at(key).empty(); }

    /**
     * Check if the ring buffer at specified key is full.
     */
    bool full(const K& key) const { return rb_map_.at(key).full(); }

    /**
     * Report the capacity of the ring buffer at specified key.
     */
    size_t capacity(const K& key) const { return rb_map_.at(key).capacity(); }

    /**
     * Report the current size of the ring buffer at specified key.
     */
    size_t size(const K& key) const { return rb_map_.at(key).size(); }

    /**
     * Flush an internal buffer at specified key.
     */
    void flush(const K& key) { return rb_map_.at(key).flush(); }

    /**
     * Flush all internal buffers.
     */
    void flush() {
        for (auto& kv : rb_map_) kv.second.flush();
    }

    /**
     * Check if any one of the internal buffers is full.
     */
    bool any_full() const {
        return std::any_of(rb_map_.begin(), rb_map_.end(),
                           [](const auto& kv) { return kv.second.full(); });
    }

    /**
     * Check if any one of the internal buffers is empty.
     */
    bool any_empty() const {
        return std::any_of(rb_map_.begin(), rb_map_.end(),
                           [](const auto& kv) { return kv.second.empty(); });
    }

    /**
     * Reports total amount of currently stored packets in internal buffers.
     *
     * NOTE: this is not a great metric since it does not report specific
     * buffers, but the total amount instead.
     */
    size_t size() const {
        return std::accumulate(rb_map_.begin(), rb_map_.end(), size_t{0},
                               [](size_t total, const auto& kv) {
                                   return total + kv.second.size();
                               });
    }

    /**
     * Reports total allocated capacity of packets stored in internal buffers.
     *
     * NOTE: this is not a great metric since it does not report specific
     * buffers, but the total amount instead.
     */
    size_t capacity() const {
        return std::accumulate(rb_map_.begin(), rb_map_.end(), size_t{0},
                               [](size_t total, const auto& kv) {
                                   return total + kv.second.capacity();
                               });
    }
};

}  // namespace impl
}  // namespace sensor
}  // namespace ouster
