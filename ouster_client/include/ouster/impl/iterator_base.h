/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>

namespace ouster {
namespace sdk {
namespace core {
namespace impl {

/// Abstract impl for a BaseIterator<T, Parent>
/// @tparam T yield type of iterator
template <class T>
class BaseIteratorImpl {
   public:
    virtual ~BaseIteratorImpl() = default;

    virtual bool advance(size_t offset) = 0;

    virtual T value() = 0;

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
    std::unique_ptr<BaseIteratorImpl<T>> impl_;
    bool first_ = true;
    uint64_t position_ = 0;
    const void* parent_ = nullptr;  // used to check if these iterators are from
                                    // the same container

    void do_advance(size_t offset) {
        first_ = false;
        if (impl_->advance(offset)) {
            impl_.reset();
        }
    }

   public:
    BaseIterator(const Parent* parent, BaseIteratorImpl<T>* i)
        : parent_(parent) {
        impl_.reset(i);
    }

    BaseIterator(const Parent* parent) : parent_(parent) {}

    BaseIterator() = default;

    /// Advance iterator
    void operator++() {
        if (!impl_) {
            throw std::runtime_error("Cannot advance end iterator");
        }
        if (first_) {
            do_advance(1);
        }
        do_advance(1);
        position_++;
    }

    /// Advance iterator
    void operator++(int) {
        if (!impl_) {
            throw std::runtime_error("Cannot advance end iterator");
        }
        if (first_) {
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
        if (!impl_) {
            throw std::runtime_error("Cannot advance end iterator");
        }
        if (first_) {
            do_advance(1);
        }
        do_advance(offset);
        position_ += offset;
    }

    /// Get difference between iterator positions
    int operator-(const BaseIterator<T, Parent>& o) {
        // both end
        if (!impl_ && !o.impl_) {
            return 0;
        }
        // other was end
        if (!o.impl_) {
            return position_ - impl_->length();
        }
        // we are end
        if (!impl_) {
            return o.impl_->length() - o.position_;
        }
        return position_ - o.position_;
    }

    /// Compare two iterators
    bool operator!=(const BaseIterator<T, Parent>& o) {
        if (first_ && impl_) {
            do_advance(1);
        }

        // throw if parents dont match
        if (parent_ != o.parent_) {
            throw std::runtime_error("Compared incompatible iterators.");
            return true;
        }

        // if we are both end, we match
        if (!impl_ && !o.impl_) {
            return false;
        }

        // if only one is end, we dont match
        if (!impl_) {
            return false;
        }
        if (!o.impl_) {
            return true;
        }
        // finally check if they are at the same position
        return position_ != o.position_;
    }

    /// Compare two iterators
    bool operator==(const BaseIterator<T, Parent>& o) { return !(*this != o); }

    /// Dereference an iterator
    T operator*() {
        if (!impl_) {
            throw std::runtime_error("Cannot dereference end iterator");
        }
        if (first_) {
            do_advance(1);
        }
        return impl_->value();
    }
};
}  // namespace impl
}  // namespace core
}  // namespace sdk
}  // namespace ouster
