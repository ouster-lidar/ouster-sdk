#include "ouster/viz/impl/buffers.h"

// NOLINTNEXTLINE(unused-includes)
#include "glfw.h"

namespace ouster {
namespace sdk {
namespace viz {
namespace impl {

namespace {

std::shared_ptr<GlResources>& get_gl_resources() {
    static std::shared_ptr<GlResources> gl_resources = std::make_shared<GlResources>();
    return gl_resources;
}

template <class F>
void load_texture(const F& texture, size_t width, size_t height, GLuint texture_id,
                  GLenum internal_format = GL_RGB, GLenum format = GL_RGB, GLenum type = GL_FLOAT) {
    glBindTexture(GL_TEXTURE_2D, texture_id);

    // we have only 1 level, so we override base/max levels
    // https://www.khronos.org/opengl/wiki/Common_Mistakes#Creating_a_complete_texture
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    if (format == GL_RGB) {
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    }

    glTexImage2D(GL_TEXTURE_2D, 0, internal_format, width, height, 0, format, type, texture);
}
}  // namespace

BufferReference::BufferReference(size_t width, float value, bool dynamic)
    : dynamic_(dynamic),
      buffer_{std::make_unique<std::vector<float>>(width, value)},
      gl_resources_(get_gl_resources()) {}

BufferReference::BufferReference(std::vector<float>&& data, bool dynamic)
    : dynamic_(dynamic),
      buffer_{std::make_unique<std::vector<float>>(std::move(data))},
      gl_resources_(get_gl_resources()) {}

BufferReference::BufferReference(std::vector<uint8_t>&& data, bool dynamic)
    : dynamic_(dynamic),
      char_buffer_{std::make_unique<std::vector<uint8_t>>(std::move(data))},
      gl_resources_(get_gl_resources()) {}

BufferReference::~BufferReference() {
    // if we have a GPU object, add it to a queue to free
    if (vbo_ > 0) {
        auto rsrcs = gl_resources_.lock();
        if (rsrcs) {
            rsrcs->freed_mutex.lock();
            rsrcs->freed_vbos[viz_instance_id_].push_back(vbo_);
            rsrcs->freed_mutex.unlock();
        }
    }
}

unsigned int BufferReference::vbo(uint32_t viz_instance_id) {
    if (vbo_ > 0) {
        return vbo_;
    }

    viz_instance_id_ = viz_instance_id;

    // generate the buffer!
    glGenBuffers(1, &vbo_);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    if (char_buffer_) {
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(sizeof(uint8_t) * char_buffer_->size()),
                     char_buffer_->data(), dynamic_ ? GL_DYNAMIC_DRAW : GL_STATIC_DRAW);
    } else {
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(sizeof(GLfloat) * buffer_->size()),
                     buffer_->data(), dynamic_ ? GL_DYNAMIC_DRAW : GL_STATIC_DRAW);
    }

    // we no longer need the buffer once it is in GPU memory
    buffer_.reset();
    char_buffer_.reset();
    return vbo_;
}

TextureReference::TextureReference(std::vector<float>&& data, int width, int height,
                                   int internal_format)
    : buffer_{std::make_unique<std::vector<float>>(std::move(data))},
      gl_resources_(get_gl_resources()),
      width_(width),
      height_(height),
      internal_format_(internal_format) {}

TextureReference::TextureReference(std::vector<uint8_t>&& data, int width, int height,
                                   int internal_format)
    : char_buffer_{std::make_unique<std::vector<uint8_t>>(std::move(data))},
      gl_resources_(get_gl_resources()),
      width_(width),
      height_(height),
      internal_format_(internal_format) {}

TextureReference::~TextureReference() {
    // if we have a GPU object, add it to a queue to free
    if (texture_ > 0) {
        auto rsrcs = gl_resources_.lock();
        if (rsrcs) {
            rsrcs->freed_mutex.lock();
            rsrcs->freed_textures[viz_instance_id_].push_back(texture_);
            rsrcs->freed_mutex.unlock();
        }
    }
}

unsigned int TextureReference::texture(uint32_t viz_instance_id) {
    if (texture_ > 0) {
        return texture_;
    }

    viz_instance_id_ = viz_instance_id;

    // generate the buffer!
    glGenTextures(1, &texture_);
    if (char_buffer_) {
        load_texture(char_buffer_->data(), width_, height_, texture_, internal_format_,
                     internal_format_, GL_UNSIGNED_BYTE);
    } else {
        load_texture(buffer_->data(), width_, height_, texture_, internal_format_);
    }

    // we no longer need the buffer once it is in GPU memory
    buffer_.reset();
    char_buffer_.reset();
    return texture_;
}

void free_buffers(uint32_t viz_instance_id) {
    std::vector<unsigned int> vbos;
    std::vector<unsigned int> textures;
    auto& gl_resources = get_gl_resources();
    gl_resources->freed_mutex.lock();
    auto vbo_iter = gl_resources->freed_vbos.find(viz_instance_id);
    if (vbo_iter != gl_resources->freed_vbos.end()) {
        std::swap(vbos, vbo_iter->second);
        gl_resources->freed_vbos.erase(vbo_iter);
    }
    auto tex_iter = gl_resources->freed_textures.find(viz_instance_id);
    if (tex_iter != gl_resources->freed_textures.end()) {
        std::swap(textures, tex_iter->second);
        gl_resources->freed_textures.erase(tex_iter);
    }
    gl_resources->freed_mutex.unlock();

    if (!vbos.empty()) {
        glDeleteBuffers(static_cast<GLsizei>(vbos.size()), vbos.data());
    }
    if (!textures.empty()) {
        glDeleteTextures(static_cast<GLsizei>(textures.size()), textures.data());
    }
}

}  // namespace impl
}  // namespace viz
}  // namespace sdk
}  // namespace ouster
