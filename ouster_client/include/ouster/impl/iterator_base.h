/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <cstddef>
#include <memory>
#include <stdexcept>

namespace ouster {
namespace impl {

/// Abstract impl for a BaseIterator<T, Parent>
/// @tparam T yield type of iterator
template <class T>
class BaseIteratorImpl {
   public:
    virtual ~BaseIteratorImpl() {}

    virtual bool advance(size_t offset) = 0;

    virtual T& value() = 0;

    virtual int64_t length() {
        throw std::runtime_error(
            "length is not supported on non-indexed scan sources.");
    }
};

/// Pimpl'd generic iterator implementation
/// @tparam T yield type of iterator
/// @tparam Parent type of parent container
template <class T, class Parent>
class BaseIterator {
    std::unique_ptr<BaseIteratorImpl<T>> impl;
    bool first = true;
    uint64_t position_ = 0;
    const void* parent_ =
        0;  // used to check if these iterators are from the same container

    void do_advance(size_t offset) {
        first = false;
        if (impl->advance(offset)) {
            impl.reset();
        }
    }

   public:
    BaseIterator(const Parent* parent, BaseIteratorImpl<T>* i)
        : parent_(parent) {
        impl.reset(i);
    }

    BaseIterator(const Parent* parent) : parent_(parent) {}

    BaseIterator() {}

    /// Advance iterator
    void operator++() {
        if (!impl) {
            throw std::runtime_error("Cannot advance end iterator");
        }
        if (first) {
            do_advance(1);
        }
        do_advance(1);
        position_++;
    }

    /// Advance iterator
    void operator++(int) {
        if (!impl) {
            throw std::runtime_error("Cannot advance end iterator");
        }
        if (first) {
            do_advance(1);
        }
        do_advance(1);
        position_++;
    }

    /// Seek through iterator
    void operator+=(size_t offset) {
        if (offset == 0) {
            return;
        }
        if (!impl) {
            throw std::runtime_error("Cannot advance end iterator");
        }
        if (first) {
            do_advance(1);
        }
        do_advance(offset);
        position_ += offset;
    }

    /// Get difference between iterator positions
    int operator-(const BaseIterator<T, Parent>& o) {
        // both end
        if (!impl && !o.impl) {
            return 0;
        }
        // other was end
        if (!o.impl) {
            return position_ - impl->length();
        }
        // we are end
        if (!impl) {
            return o.impl->length() - o.position_;
        }
        return position_ - o.position_;
    }

    /// Compare two iterators
    bool operator!=(const BaseIterator<T, Parent>& o) {
        if (first && impl) {
            do_advance(1);
        }

        // throw if parents dont match
        if (parent_ != o.parent_) {
            throw std::runtime_error("Compared incompatible iterators.");
            return true;
        }

        // if we are both end, we match
        if (!impl && !o.impl) {
            return false;
        }

        // if only one is end, we dont match
        if (!impl) {
            return false;
        }
        if (!o.impl) {
            return true;
        }
        // finally check if they are at the same position
        return position_ != o.position_;
    }

    /// Compare two iterators
    bool operator==(const BaseIterator<T, Parent>& o) { return !(*this != o); }

    /// Dereference an iterator
    T& operator*() {
        if (!impl) {
            throw std::runtime_error("Cannot dereference end iterator");
        }
        if (first) {
            do_advance(1);
        }
        return impl->value();
    }
};
}  // namespace impl
}  // namespace ouster
