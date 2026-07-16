#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <vector>

namespace ouster {
namespace sdk {
namespace viz {
namespace impl {

using GlResourceMap = std::map<uint32_t, std::vector<unsigned int>>;
struct GlResources {
    std::mutex freed_mutex;
    GlResourceMap freed_vbos;
    GlResourceMap freed_textures;
};

/// Manages a CPU buffer that we eventually want as a GPU vertex one
class BufferReference {
   public:
    bool dynamic_;
    unsigned int vbo_ = 0;  // zero is invalid id
    std::unique_ptr<std::vector<float>> buffer_;
    std::unique_ptr<std::vector<uint8_t>> char_buffer_;
    uint32_t viz_instance_id_ = 0;
    std::weak_ptr<GlResources> gl_resources_;

   public:
    BufferReference(size_t width, float value, bool dynamic = false);

    BufferReference(std::vector<float>&& data, bool dynamic = false);

    BufferReference(std::vector<uint8_t>&& data, bool dynamic = false);

    ~BufferReference();

    // make sure to only call on the UI thread
    unsigned int vbo(uint32_t viz_instance_id);
};

/// Manages a CPU buffer that we eventually want as a GPU texture one
class TextureReference {
   public:
    unsigned int texture_ = 0;  // zero is invalid id
    std::unique_ptr<std::vector<float>> buffer_;
    std::unique_ptr<std::vector<uint8_t>> char_buffer_;
    uint32_t viz_instance_id_ = 0;
    std::weak_ptr<GlResources> gl_resources_;

    uint32_t width_;
    uint32_t height_;

    unsigned int internal_format_;

   public:
    TextureReference(std::vector<float>&& data, int width, int height, int internal_format);

    TextureReference(std::vector<uint8_t>&& data, int width, int height, int internal_format);

    ~TextureReference();

    // make sure to only call on the UI thread
    unsigned int texture(uint32_t viz_instance_id);
};

/// Free any no-longer used GPU buffers
void free_buffers(uint32_t instance_id);

}  // namespace impl
}  // namespace viz
}  // namespace sdk
}  // namespace ouster
