#pragma once
#include <deque>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "camera.h"
#include "ouster/point_viz.h"

namespace ouster {
namespace sdk {
namespace viz {
namespace {

/*
 * Helper for addable / removable drawable objects
 */
template <typename GL, typename T>
class Indexed {
    using GLStatePtr = std::unique_ptr<GL>;
    using FrontStatePtr = std::unique_ptr<T>;
    using BackStatePtr = std::shared_ptr<T>;
    struct Front {
        GLStatePtr gl;
        FrontStatePtr state;
    };

   protected:
    // Represents the drawables the user interacts with
    std::unordered_set<BackStatePtr> back_;
    // Represents a snapshot of drawable state used by OpenGL
    std::unordered_map<BackStatePtr, Front> front_;

    std::deque<Front> death_queue_;

    bool back_contains(const std::shared_ptr<T>& obj) {
        return back_.find(obj) != back_.end();
    }

    bool front_contains(const std::shared_ptr<T>& obj) {
        return front_.find(obj) != front_.end();
    }

   public:
    Indexed() = default;

    void add(const std::shared_ptr<T>& obj) {
        // any object being added will automatically be considered "dirty",
        // since it may not have prior GL state
        obj->dirty();
        back_.insert(obj);
    }

    bool remove(const std::shared_ptr<T>& obj) { return back_.erase(obj); }

    void draw(const WindowCtx& ctx, const impl::CameraData& camera) {
        for (auto& obj : front_) {
            if (!obj.second.gl) {
                // init GL for added
                obj.second.gl = std::make_unique<GL>(*obj.second.state);
            }
            obj.second.gl->draw(ctx, camera, *obj.second.state);
        }
    }

    void clean_up_removed_drawables() { death_queue_.clear(); }

    void update_new_and_existing_drawables() {
        for (auto it = front_.begin(); it != front_.end();) {
            if (back_.find(it->first) == back_.end()) {
                // The drawable was removed.
                // Remove its state.
                death_queue_.push_back(std::move(it->second));
                it = front_.erase(it);
            } else {
                it++;
            }
        }
        for (auto& back : back_) {
            auto front_it = front_.find(back);
            if (front_it == front_.end()) {
                // The drawable was added.
                // Create its state.
                front_[back].state = std::make_unique<T>(*back);
            }
            front_[back].state->update_from(*back);
            back->clear();
        }
    }
};

}  // namespace
}  // namespace viz
}  // namespace sdk
}  // namespace ouster
