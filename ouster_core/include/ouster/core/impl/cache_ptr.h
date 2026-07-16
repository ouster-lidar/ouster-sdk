#pragma once

#include <memory>

namespace ouster {
namespace sdk {
namespace core {
namespace impl {

// movable but non-copyable private cache object
template <class T>
class CachePtr {
    std::unique_ptr<T> value_;

   public:
    CachePtr() : value_{new T} {}

    CachePtr(T* val) : value_{val} {}

    CachePtr(const CachePtr&) : value_{new T} {}

    CachePtr& operator=(const CachePtr&) {
        value_.reset(new T);
        return *this;
    }

    CachePtr(CachePtr&& other) {
        value_.reset(other.value_.release());
        other.value_.reset(new T);
    }

    CachePtr& operator=(CachePtr&& other) {
        if (this != &other) {
            value_.reset(other.value_.release());
            other.value_.reset(new T);
        }
        return *this;
    }

    ~CachePtr() {}

    T* operator->() {
        return value_.get();
    }

    const T* operator->() const {
        return value_.get();
    }
};

}  // namespace impl
}  // namespace core
}  // namespace sdk
}  // namespace ouster
