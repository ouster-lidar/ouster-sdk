#pragma once

#include <GL/glew.h>

#include <array>
#include <cstddef>
#include <vector>

#include "camera.h"
#include "common.h"

namespace ouster {
namespace viz {
namespace impl {

/**
 * Class to deal with showing an image. The image is auto-scaled to be
 * positioned either at the top (for wide images) or the left (for tall images)
 */
class Image {
    size_t width;
    size_t height;
    GLfloat aspect_ratio;  // height divided by width of true aspect ratio
    int size_fraction;
    const int size_fraction_max;
    std::vector<GLfloat> data;
    std::vector<GLfloat> mask_data;
    bool texture_changed;
    bool mask_changed;
    std::array<GLuint, 2> vertexbuffers;
    GLuint image_program_id = 0;
    GLuint image_texture_id;
    GLuint mask_texture_id;

   public:
    /**
     * Constructor: create an Image for displaying
     *
     * @param width             pixel width of image
     * @param height            pixel height of image
     * @param size_fraction     fraction of screen to take up, numerator
     * @param size_fraction_max fraction of screen to take up, denominator
     */
    Image(size_t width, size_t height, int size_fraction = 3,
          int size_fraction_max = 10)
        : width(width),
          height(height),
          size_fraction(size_fraction),
          size_fraction_max(size_fraction_max),
          data(width * height, 0),
          mask_data(4 * width * height, 0),
          texture_changed(true),
          mask_changed(true) {}

    /**
     * initialize the image. must be called after valid OpenGL context created
     */
    void initialize() {
        image_program_id =
            load_shaders(image_vertex_shader_code, image_fragment_shader_code);
        glGenBuffers(2, vertexbuffers.data());
        GLuint textures[2];
        glGenTextures(2, textures);
        image_texture_id = textures[0];
        mask_texture_id = textures[1];
    }

    /**
     * set the image
     *
     * @param image_data pointer to array of at least as many elements as there
     *                   are pixels in the image, in row-major format
     */
    void setImage(const GLfloat* image_data) {
        const size_t n = width * height;
        std::copy(image_data, image_data + n, data.begin());
        texture_changed = true;
    }

    /**
     * set the RGBA mask
     *
     * @param image_data pointer to array of at least 4x as many elements as
     *                   there are pixels in the image, in row-major format
     */
    void setMask(const GLfloat* msk_data) {
        const size_t n = width * height * 4;
        std::copy(msk_data, msk_data + n, mask_data.begin());
        mask_changed = true;
    }

    /**
     * render the monochrome image.
     *
     * @param cam modifies the camera to offset it so that it is centered on the
     *            region not covered by image.
     */
    void draw(Camera& cam);

    /**
     * set pixel dimensions of the image
     *
     * @param w pixel height of image
     * @param h pixel width of image
     */
    void resize(size_t w, size_t h) {
        if (w == width && h == height) return;
        width = w;
        height = h;
        data.resize(w * h);
        mask_data.resize(w * h * 4);
    }

    /**
     * increase or decrease size fraction. Sets member variable
     * size_fraction to be between 0 and size_fraction_max (inclusive)
     *
     * @param amount amount to increase by (can be negative)
     */
    void changeSizeFraction(int amount) {
        size_fraction = (size_fraction + amount + (size_fraction_max + 1)) %
                        (size_fraction_max + 1);
    }

    /**
     * set the display aspect ratio of the image
     *
     * @param a the aspect ratio, i.e. height divided by width
     */
    void setAspectRatio(GLfloat a) { aspect_ratio = a; }

    /**
     * destructor, delete program
     */
    ~Image() { glDeleteProgram(image_program_id); }
};

}  // namespace impl
}  // namespace viz
}  // namespace ouster
