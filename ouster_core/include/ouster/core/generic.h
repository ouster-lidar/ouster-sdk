#pragma once

#include <memory>

#include "ouster/core/class_map.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * @brief A type-erasing wrapper class a la "Runtime Polymorphism" by Sean
 * Parent et al.
 */
class OUSTER_API_CLASS Generic {
   public:
    OUSTER_API_FUNCTION
    virtual ~Generic() = default;

    /**
     * Construct a Generic object from any type T.
     *
     * @param[in] value The value to store in the Generic object.
     */
    template <typename T>
    Generic(T value) : holder_(std::make_shared<Model<T>>(value)) {}

    /**
     * Check if the Generic object holds a value of type T.
     *
     * @tparam T The type to check for.
     * @return True if the Generic object holds a value of type T, false
     *         otherwise.
     */
    template <typename T>
    bool is() const {
        return dynamic_cast<const Model<T>*>(holder_.get()) != nullptr;
    }

    /**
     * Get the value stored in the Generic object as type T.
     *
     * @tparam T The type to get the value as.
     * @return The value stored in the Generic object as type T.
     * @throws std::bad_cast if the Generic object does not hold a value of
     *         type T.
     */
    template <typename T>
    const T& as() const {
        auto derived = dynamic_cast<const Model<T>*>(holder_.get());
        if (!derived) {
            throw std::bad_cast();
        }
        return derived->data;
    }

   private:
    struct Concept {
        virtual ~Concept() = default;
    };
    template <typename T>
    struct Model final : Concept {
        Model(T value) : data(std::move(value)) {}
        T data;
    };

    std::shared_ptr<const Concept> holder_;
};

}  // namespace core
}  // namespace sdk
}  // namespace ouster
